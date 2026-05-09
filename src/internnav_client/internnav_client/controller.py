import json
import time
from enum import Enum
from typing import Optional, Tuple

# ros2
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State

# ros2 msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header

from unitree_api.msg import Request

import numpy as np

# User defined modules
from internnav_client.mpc import MPCController
from internnav_client.pd import PDController
from internnav_client import utils

class ControlMode(Enum):
    IDLE = 0
    PD   = 1
    MPC  = 2

class Controller(LifecycleNode):
    def __init__(self):
        super().__init__('internnav_controller')

        self.declare_parameter('hz', 100.0)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        hz = self.get_parameter('hz')\
            .get_parameter_value()\
            .double_value

        # ChannelFactoryInitialize(1, 'eth0')
        # self.sport_client = SportClient()
        # self.sport_client.SetTimeout(10.0)
        # self.sport_client.Init()
        # self.get_logger().info('Go2 sport client initialized.')

        self._mode = ControlMode.IDLE
        self._mpc: Optional[MPCController] = None
        self._pd = PDController(
            Kp_trans=2.0,
            Kd_trans=0.0,
            Kp_yaw=1.5,
            Kd_yaw=0.0,
            max_v=0.6,
            max_w=0.5
        )

        self._reset_state()

        self._sport_pub = self.create_lifecycle_publisher(
            Request,
            '/api/obstacles_avoid/request',
            1
        )

        self._timer = self.create_timer(1 / hz, self.control_loop)

        self.get_logger().info('Controller initialized')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._odom_sub = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            1
        )
        self._cmd_path_sub = self.create_subscription(
            Path,
            '/internnav/client/cmd_path',
            self.cmd_path_callback,
            1
        )
        self._cmd_pose_sub = self.create_subscription(
            PoseStamped,
            '/internnav/client/cmd_pose',
            self.cmd_pose_callback,
            1
        )
        self._cmd_stop_sub = self.create_subscription(
            Header,
            '/internnav/client/cmd_stop',
            self.cmd_stop_callback,
            1
        )

        self.get_logger().info('Controller activated')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self._odom_sub)
        self.destroy_subscription(self._cmd_path_sub)
        self.destroy_subscription(self._cmd_pose_sub)
        self.destroy_subscription(self._cmd_stop_sub)
        self._move(0.0, 0.0)
        self._mode = ControlMode.IDLE
        self._reset_state()
        self.get_logger().info('Controller deactivated')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _reset_state(self):
        self._odom: Optional[Tuple[float, float, float]] = None
        self._vel: Optional[Tuple[float, float]] = None
        self._target_pose: Optional[Tuple[float, float, float]] = None
        self._last_stop_time: int = 0
        self._mpc = None

    def _move(self, v: float, w: float):
        req = Request()
        req.header.identity.id = time.monotonic_ns()
        req.header.identity.api_id = 1003
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = True
        req.parameter = json.dumps({
            'x': float(v),
            'y': 0.0,
            'z': float(w),
            'mode': 0
        })
        req.binary = []
        self._sport_pub.publish(req)

    def control_loop(self):
        if self._mode == ControlMode.IDLE or self._odom is None:
            return

        elif self._mode == ControlMode.MPC:
            if self._mpc is None:
                return

            opt_u, _ = self._mpc.solve(np.array(self._odom))
            v, w = float(opt_u[0, 0]), float(opt_u[0, 1])
            self._move(v, w)

        elif self._mode == ControlMode.PD:
            if self._vel is None or self._target_pose is None:
                return

            v, w, e_p, e_r = self._pd.solve(
                utils.to_homo(*self._odom),
                utils.to_homo(*self._target_pose),
                self._vel
            )

            # ----------------------- Deadband -----------------------
            MIN_W = 0.5

            if abs(e_r) >= 0.05 and abs(w) < MIN_W:
                w = MIN_W if w > 0 else -MIN_W
            # --------------------------------------------------------

            self._move(max(v, 0.0), w)

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        yaw = utils.to_yaw(pose.orientation.z, pose.orientation.w)

        self._odom = (pose.position.x, pose.position.y, yaw)
        self._vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

        if self._target_pose is None:
            self._target_pose = self._odom

    def cmd_pose_callback(self, msg: PoseStamped):
        if msg.header.frame_id != 'odom':
            self.get_logger().error('cmd_pose_callback: frame_id != "odom"')
            return

        stamp = msg.header.stamp
        now = utils.to_nanosec(stamp.sec, stamp.nanosec)

        if self._last_stop_time >= now:
            self.get_logger().warn(f'Stale cmd_pose discarded (Msg: {now} <= Stop: {self._last_stop_time})')
            return

        target_yaw = utils.to_yaw(msg.pose.orientation.z, msg.pose.orientation.w)
        self._target_pose = (msg.pose.position.x, msg.pose.position.y, target_yaw)
        self._mode = ControlMode.PD

    def cmd_path_callback(self, msg: Path):
        if msg.header.frame_id != 'odom':
            self.get_logger().error('cmd_path_callback: frame_id != "odom"')
            return

        stamp = msg.header.stamp
        now = utils.to_nanosec(stamp.sec, stamp.nanosec)

        if self._last_stop_time >= now:
            self.get_logger().warn(f'Stale cmd_path discarded (Msg: {now} <= Stop: {self._last_stop_time})')
            return

        trajs = np.array([(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses])

        if self._mpc is None:
            self._mpc = MPCController(trajs)
        else:
            self._mpc.update_ref_traj(trajs)

        self._mode = ControlMode.MPC

    def cmd_stop_callback(self, header: Header):
        stamp = header.stamp
        self._last_stop_time = utils.to_nanosec(stamp.sec, stamp.nanosec)

        self._move(0.0, 0.0)
        self._mode = ControlMode.IDLE
        self.get_logger().info('`STOP` received, switching to IDLE')

def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
