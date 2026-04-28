import math
from collections import deque
from typing import Optional

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
        slowdown_factor: int = 4,
        odom_timeout_sec: float = 0.5,
        rotation_degree: int = 12
    ):
        super().__init__('internnav_planner')

        assert type(slowdown_factor) is int, 'slowdown factor must be an integer!'
        assert slowdown_factor > 1, 'slowdown factor must be greater than 1!'
        self.slowdown_factor = slowdown_factor

        assert odom_timeout_sec >= 0, 'odom timeout must be non-negative!'
        self.odom_timeout_sec = odom_timeout_sec

        rad = math.radians(rotation_degree)
        c, s = math.cos(rad), math.sin(rad)
        self.rot_left  = np.array([[c, -s, 0], [ s, c, 0], [0, 0, 1]])
        self.rot_right = np.array([[c,  s, 0], [-s, c, 0], [0, 0, 1]])

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
        self.odom_queue: deque[tuple[float, tuple[float, float, float]]] = deque(maxlen=50)

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
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.odom_queue.append((timestamp, odom))

    def find_odom_at_time(self, target_sec: float) -> Optional[tuple[float, float, float]]:
        if not self.odom_queue:
            return None

        best_time, best_odom = min(self.odom_queue, key=lambda x: abs(x[0] - target_sec))
        t_diff = abs(best_time - target_sec)

        if t_diff > self.odom_timeout_sec:
            self.get_logger().warn(f'Odom match failed: time difference too big ({t_diff:.3f}s)')
            return None

        return best_odom

    def trajectory_callback(self, msg: TrajectoryStamped):
        image_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        odom_infer = self.find_odom_at_time(image_time)
        if odom_infer is not None:
            self.process_trajectory(msg.waypoints, odom_infer)

    def discrete_callback(self, msg: DiscreteStamped):
        actions = list(msg.actions)
        if actions == [DiscreteStamped.STOP]:
            self.cmd_stop_pub.publish(Empty())
            return

        image_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        odom_infer = self.find_odom_at_time(image_time)
        if odom_infer is not None:
            self.process_discrete(actions, odom_infer)

    def process_trajectory(
        self,
        raw_trajectory: list[Point],
        odom_infer: tuple[float, float, float]
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

    def process_discrete(self, actions: list[int], odom_infer: tuple[float, float, float]):
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
