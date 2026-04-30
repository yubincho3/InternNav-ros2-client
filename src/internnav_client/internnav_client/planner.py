import math
from collections import deque
from typing import Deque, List, Tuple

# ros2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# ros2 msgs
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

import numpy as np

# User defined modules
from internnav_client import utils

# User defined msgs
from internnav_interfaces.msg import DiscreteStamped, Trajectory, TrajectoryStamped

class Planner(Node):
    def __init__(
        self,
        slowdown_factor: int = 1,
        odom_timeout_sec: float = 1.0,
        rotation_degree: int = 15
    ):
        super().__init__('internnav_planner')

        assert type(slowdown_factor) is int, 'slowdown factor must be an integer!'
        assert slowdown_factor >= 1, 'slowdown factor must be greater than or equal to 1!'
        self.slowdown_factor = slowdown_factor

        # pre-compute rotation matrix
        rad = math.radians(rotation_degree)
        c, s = math.cos(rad), math.sin(rad)
        self.rot_left  = np.array([[c, -s, 0], [ s, c, 0], [0, 0, 1]])
        self.rot_right = np.array([[c,  s, 0], [-s, c, 0], [0, 0, 1]])

        assert odom_timeout_sec >= 0, 'odom timeout must be non-negative!'
        self.odom_timeout_sec = odom_timeout_sec

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            qos
        )
        self.odom_queue: Deque[Tuple[float, float, float]] = deque(maxlen=100)

        self.create_subscription(
            TrajectoryStamped,
            '/internnav/server/trajectory',
            self.trajectory_callback,
            1
        )
        self.create_subscription(
            DiscreteStamped,
            '/internnav/server/discrete',
            self.discrete_callback,
            1
        )

        self.cmd_traj_pub = self.create_publisher(
            Trajectory,
            '/internnav/client/cmd_traj',
            1
        )
        self.cmd_pose_pub = self.create_publisher(
            Pose,
            '/internnav/client/cmd_pose',
            1
        )
        self.cmd_stop_pub = self.create_publisher(
            Empty,
            '/internnav/client/stop',
            1
        )

        self.get_logger().info(f'Planner initialized (slowdown_factor={slowdown_factor})')

    def odom_callback(self, msg: Odometry):
        yaw = utils.to_yaw(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        self.odom_queue.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))

    def trajectory_callback(self, msg: TrajectoryStamped):
        if not self.odom_queue:
            self.get_logger().warn('No odom available for trajectory')
            return
        odom_infer = self.odom_queue[-1]
        self.process_trajectory(msg.waypoints, odom_infer)

    def discrete_callback(self, msg: DiscreteStamped):
        actions = list(msg.actions)
        if actions == [DiscreteStamped.STOP]:
            self.cmd_stop_pub.publish(Empty())
            return

        if not self.odom_queue:
            self.get_logger().warn('No odom available for discrete')
            return
        odom_infer = self.odom_queue[-1]
        self.process_discrete(actions, odom_infer)

    def process_trajectory(
        self,
        raw_trajectory: List[Point],
        odom_infer: Tuple[float, float, float]
    ):
        x, y, yaw = odom_infer

        w_T_b = np.array([
            [np.cos(yaw), -np.sin(yaw), 0, x],
            [np.sin(yaw),  np.cos(yaw), 0, y],
            [0.0, 0.0, 1.0, 0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        # trajs_in_world = []
        # for i, traj in enumerate(raw_trajectory):
        #     if i < 3:
        #         continue
        #     body_pt = np.array([traj.x, traj.y, 0.0, 1.0])
        #     world_pt = (w_T_b @ body_pt)[:2]
        #     trajs_in_world.append(world_pt)
        #
        # trajs_in_world = np.array(trajs_in_world)
        # -> optimized
        pts = np.array([[pt.x, pt.y, 0.0, 1.0] for pt in raw_trajectory[3:]]).T
        trajs_in_world = (w_T_b @ pts)[:2, :].T

        if len(trajs_in_world) == 0:
            self.get_logger().warn('No waypoints after skipping first 3')
            return

        # --------------------------------------------------------
        # slowdown interpolate
        if len(trajs_in_world) > 1 and self.slowdown_factor > 1:
            old_indices = np.arange(len(trajs_in_world))
            num_new = (len(trajs_in_world) - 1) * self.slowdown_factor + 1
            new_indices = np.linspace(0, len(trajs_in_world) - 1, num_new)
            x_interp = np.interp(new_indices, old_indices, trajs_in_world[:, 0])
            y_interp = np.interp(new_indices, old_indices, trajs_in_world[:, 1])
            trajs_in_world = np.column_stack((x_interp, y_interp))
        # --------------------------------------------------------

        msg = Trajectory()
        msg.waypoints = [
            Point(x=float(x), y=float(y), z=0.0)
            for x, y in trajs_in_world
        ]
        self.cmd_traj_pub.publish(msg)

    def process_discrete(self, actions: List[int], odom_infer: Tuple[float, float, float]):
        if actions == [DiscreteStamped.LOOK_DOWN]:
            return

        x, y, yaw = odom_infer
        homo_goal = utils.to_homo(x, y, yaw)

        for action in actions:
            if action == DiscreteStamped.FORWARD:
                curr_yaw = math.atan2(homo_goal[1, 0], homo_goal[0, 0])
                homo_goal[0, 3] += 0.25 * np.cos(curr_yaw)
                homo_goal[1, 3] += 0.25 * np.sin(curr_yaw)
            elif action == DiscreteStamped.TURN_LEFT:
                homo_goal[:3, :3] = self.rot_left @ homo_goal[:3, :3]
            elif action == DiscreteStamped.TURN_RIGHT:
                homo_goal[:3, :3] = self.rot_right @ homo_goal[:3, :3]

        final_x = homo_goal[0, 3]
        final_y = homo_goal[1, 3]
        final_yaw = math.atan2(homo_goal[1, 0], homo_goal[0, 0])

        pose_msg = Pose()
        pose_msg.position.x = float(final_x)
        pose_msg.position.y = float(final_y)
        pose_msg.position.z = 0.0
        pose_msg.orientation.z = math.sin(final_yaw / 2)
        pose_msg.orientation.w = math.cos(final_yaw / 2)

        self.cmd_pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Planner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
