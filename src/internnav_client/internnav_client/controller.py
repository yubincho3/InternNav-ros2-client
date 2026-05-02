import json
from enum import Enum
from typing import Optional, Tuple

# ros2
import rclpy
from rclpy.node import Node

# ros2 msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header

# from unitree_sdk2py.go2.sport.sport_client import SportClient
# from unitree_sdk2py.core.channel import ChannelFactoryInitialize
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

class Controller(Node):
    def __init__(self, hz=100.0):
        super().__init__('internnav_controller')

        # ChannelFactoryInitialize(1, 'eth0')
        # self.sport_client = SportClient()
        # self.sport_client.SetTimeout(10.0)
        # self.sport_client.Init()
        # self.get_logger().info('Go2 sport client initialized.')

        self.mode = ControlMode.IDLE
        self.mpc: Optional[MPCController] = None
        self.pd = PDController(
            Kp_trans=2.0,
            Kd_trans=0.0,
            Kp_yaw=1.5,
            Kd_yaw=0.0,
            max_v=0.6,
            max_w=0.5
        )

        self.create_timer(1/hz, self.control_loop)

        self.sport_pub = self.create_publisher(
            Request,
            '/api/sport/request',
            10
        )

        self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            1
        )
        self.create_subscription(
            Path,
            '/internnav/client/cmd_path',
            self.cmd_path_callback,
            1
        )
        self.create_subscription(
            PoseStamped,
            '/internnav/client/cmd_pose',
            self.cmd_pose_callback,
            1
        )
        self.create_subscription(
            Header,
            '/internnav/client/cmd_stop',
            self.cmd_stop_callback,
            1
        )

        self.odom: Optional[Tuple[float, float, float]] = None
        self.vel: Optional[Tuple[float, float]] = None
        self.target_pose: Optional[Tuple[float, float, float]] = None
        self.last_stop_time: int = 0

        self.get_logger().info('Controller initialized')

    def _move(self, v: float, w: float):
        req = Request()
        req.header.identity.api_id = 1008
        req.parameter = json.dumps({
            'x': float(v),
            'y': 0.0,
            'z': float(w),
        })
        self.sport_pub.publish(req)
        # self.sport_client.Move(v, 0.0, w)

    def control_loop(self):
        if self.mode == ControlMode.IDLE or self.odom is None:
            return

        elif self.mode == ControlMode.MPC:
            if self.mpc is None:
                return

            opt_u, _ = self.mpc.solve(np.array(self.odom))
            v, w = float(opt_u[0, 0]), float(opt_u[0, 1])
            self._move(v, w)

        elif self.mode == ControlMode.PD:
            if self.vel is None or self.target_pose is None:
                return

            v, w, e_p, e_r = self.pd.solve(
                utils.to_homo(*self.odom),
                utils.to_homo(*self.target_pose),
                self.vel
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

        self.odom = (pose.position.x, pose.position.y, yaw)
        self.vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

        if self.target_pose is None:
            self.target_pose = self.odom

    def cmd_pose_callback(self, msg: PoseStamped):
        if msg.header.frame_id != 'odom':
            self.get_logger().error('cmd_pose_callback: frame_id != "odom"')
            return

        stamp = msg.header.stamp
        now = utils.to_nanosec(stamp.sec, stamp.nanosec)

        if self.last_stop_time >= now:
            self.get_logger().warn(f'Stale cmd_pose discarded (Msg: {now} <= Stop: {self.last_stop_time})')
            return

        target_yaw = utils.to_yaw(msg.pose.orientation.z, msg.pose.orientation.w)
        self.target_pose = (msg.pose.position.x, msg.pose.position.y, target_yaw)
        self.mode = ControlMode.PD

    def cmd_path_callback(self, msg: Path):
        if msg.header.frame_id != 'odom':
            self.get_logger().error('cmd_path_callback: frame_id != "odom"')
            return

        stamp = msg.header.stamp
        now = utils.to_nanosec(stamp.sec, stamp.nanosec)

        if self.last_stop_time >= now:
            self.get_logger().warn(f'Stale cmd_path discarded (Msg: {now} <= Stop: {self.last_stop_time})')
            return

        trajs = np.array([(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses])

        if self.mpc is None:
            self.mpc = MPCController(trajs)
        else:
            self.mpc.update_ref_traj(trajs)

        self.mode = ControlMode.MPC

    def cmd_stop_callback(self, header: Header):
        stamp = header.stamp
        self.last_stop_time = utils.to_nanosec(stamp.sec, stamp.nanosec)

        self._move(0.0, 0.0)
        self.mode = ControlMode.IDLE
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
