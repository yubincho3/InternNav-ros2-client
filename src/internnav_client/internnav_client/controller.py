from enum import Enum
from typing import Optional

# ros2
import rclpy
from rclpy.node import Node

# ros2 msgs
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from unitree_sdk2py.go2.sport.sport_client import SportClient

import numpy as np

# User defined modules
from mpc import MPCController
from pd import PDController
import utils

# User defined msgs
from internnav_interfaces.msg import Trajectory

class ControlMode(Enum):
    IDLE = 0
    PD   = 1
    MPC  = 2

class Controller(Node):
    def __init__(self, hz=100.0):
        super().__init__('internnav_controller')

        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        self.get_logger().info('Go2 sport client initialized.')

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

        self.odom_sub = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            1
        )
        self.traj_sub = self.create_subscription(
            Trajectory,
            '/internnav/client/cmd_traj',
            self.traj_callback,
            1
        )
        self.cmd_pose_sub = self.create_subscription(
            Pose,
            '/internnav/client/cmd_pose',
            self.cmd_pose_callback,
            1
        )

        self.odom: Optional[tuple[float, float, float]] = None
        self.vel: Optional[tuple[float, float]] = None
        self.target_pose: Optional[tuple[float, float, float]] = None

        self.get_logger().info('Controller initialized')

    def control_loop(self):
        if self.mode == ControlMode.MPC:
            if self.mpc is None or self.odom is None:
                return

            opt_u, _ = self.mpc.solve(np.array(self.odom))
            v, w = float(opt_u[0, 0]), float(opt_u[0, 1])
            self.sport_client.Move(v, 0.0, w)

        elif self.mode == ControlMode.PD:
            if self.odom is None or self.vel is None or self.target_pose is None:
                return

            v, w, e_p, e_r = self.pd.solve(
                utils.to_homo(*self.odom),
                utils.to_homo(*self.target_pose),
                self.vel
            )

            # --------------------------------------------------------
            # 정지 마찰력 극복을 위한 최소 각속도 보장 (Deadband 보상)
            MIN_W = 0.5

            if abs(e_r) >= 0.05 and abs(w) < MIN_W:
                w = MIN_W if w > 0 else -MIN_W
            # --------------------------------------------------------

            if v < 0.0:
                v = 0.0

            self.sport_client.Move(v, 0.0, w)

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

def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
