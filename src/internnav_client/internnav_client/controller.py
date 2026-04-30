import json
from enum import Enum
from typing import Optional, Tuple

# ros2
import rclpy
from rclpy.node import Node

# ros2 msgs
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

# from unitree_sdk2py.go2.sport.sport_client import SportClient
# from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_api.msg import Request

import numpy as np

# User defined modules
from internnav_client.mpc import MPCController
from internnav_client.pd import PDController
from internnav_client import utils

# User defined msgs
from internnav_interfaces.msg import Trajectory

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
            Trajectory,
            '/internnav/client/cmd_traj',
            self.traj_callback,
            1
        )
        self.create_subscription(
            Pose,
            '/internnav/client/cmd_pose',
            self.cmd_pose_callback,
            1
        )
        self.create_subscription(
            Empty,
            '/internnav/client/stop',
            self.stop_callback,
            1
        )

        self.odom: Optional[Tuple[float, float, float]] = None
        self.vel: Optional[Tuple[float, float]] = None
        self.target_pose: Optional[Tuple[float, float, float]] = None

        self.get_logger().info('Controller initialized')

    def _move(self, v: float, w: float):
        req = Request()
        req.header.identity.api_id = 1008
        req.parameter = json.dumps({
            "x": float(v),
            "y": 0.0,
            "z": float(w),
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

    def cmd_pose_callback(self, msg: Pose):
        target_yaw = utils.to_yaw(msg.orientation.z, msg.orientation.w)
        self.target_pose = (msg.position.x, msg.position.y, target_yaw)
        self.mode = ControlMode.PD

    def traj_callback(self, msg: Trajectory):
        waypoints = np.array([[p.x, p.y] for p in msg.waypoints])

        if self.mpc is None:
            self.mpc = MPCController(waypoints)
        else:
            self.mpc.update_ref_traj(waypoints)

        self.mode = ControlMode.MPC

    def stop_callback(self, _):
        self.mode = ControlMode.IDLE
        self._move(0.0, 0.0)
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
