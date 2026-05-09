import math
from collections import deque
from typing import Deque, List, Tuple

# ros2
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, State
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# ros2 msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header

import numpy as np

# User defined modules
from internnav_client import utils

# User defined msgs
from internnav_interfaces.msg import DiscreteStamped

class Planner(LifecycleNode):
    def __init__(self):
        super().__init__('internnav_planner')

        self.declare_parameter('slowdown_factor', 1)
        self.declare_parameter('rotation_degree', 15)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        rotation_degree = self.get_parameter('rotation_degree')\
            .get_parameter_value()\
            .integer_value
        self._slowdown_factor = self.get_parameter('slowdown_factor')\
            .get_parameter_value()\
            .integer_value

        assert type(self._slowdown_factor) is int, 'slowdown factor must be an integer!'
        assert self._slowdown_factor >= 1, 'slowdown factor must be greater than or equal to 1!'

        # pre-compute rotation matrix
        rad = math.radians(rotation_degree)
        c, s = math.cos(rad), math.sin(rad)
        self._rot_left  = np.array([[c, -s, 0], [ s, c, 0], [0, 0, 1]])
        self._rot_right = np.array([[c,  s, 0], [-s, c, 0], [0, 0, 1]])

        self._odom_queue: Deque[Tuple[float, float, float]] = deque(maxlen=100)

        self._cmd_path_pub = self.create_lifecycle_publisher(
            Path,
            '/internnav/client/cmd_path',
            1
        )
        self._cmd_pose_pub = self.create_lifecycle_publisher(
            PoseStamped,
            '/internnav/client/cmd_pose',
            1
        )
        self._cmd_stop_pub = self.create_lifecycle_publisher(
            Header,
            '/internnav/client/cmd_stop',
            1
        )

        self.get_logger().info(f'Planner configured (slowdown_factor={self._slowdown_factor})')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            qos
        )
        self._path_sub = self.create_subscription(
            Path,
            '/internnav/server/system1/output_path',
            self.path_callback,
            1
        )
        self._discretes_sub = self.create_subscription(
            DiscreteStamped,
            '/internnav/server/system2/output_discretes',
            self.discretes_callback,
            1
        )

        self.get_logger().info('Planner activated')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.destroy_subscription(self._odom_sub)
        self.destroy_subscription(self._path_sub)
        self.destroy_subscription(self._discretes_sub)
        self._odom_queue.clear()
        self.get_logger().info('Planner deactivated')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def odom_callback(self, msg: Odometry):
        yaw = utils.to_yaw(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        self._odom_queue.append((msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))

    def path_callback(self, msg: Path):
        if not self._odom_queue:
            self.get_logger().warn('No odom available for trajectory')
            return

        odom_infer = self._odom_queue[-1]
        self.process_path(msg, odom_infer)

    def discretes_callback(self, msg: DiscreteStamped):
        actions = list(msg.actions)

        if actions == [DiscreteStamped.STOP]:
            header = Header(stamp=msg.header.stamp, frame_id='')
            self._cmd_stop_pub.publish(header)
            return

        if not self._odom_queue:
            self.get_logger().warn('No odom available for discrete')
            return

        odom_infer = self._odom_queue[-1]
        self.process_discretes(actions, msg.header, odom_infer)

    def process_path(
        self,
        base_msg: Path,
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
        # for i, pose in enumerate(base_msg.poses):
        #     if i < 3:
        #         continue
        #     body_pt = np.array([pose.pose.position.x, pose.pose.position.y, 0.0, 1.0])
        #     world_pt = (w_T_b @ body_pt)[:2]
        #     trajs_in_world.append(world_pt)
        #
        # trajs_in_world = np.array(trajs_in_world)
        # -> optimized
        pts = np.array([[pose.pose.position.x, pose.pose.position.y, 0.0, 1.0] for pose in base_msg.poses[3:]]).T
        trajs_in_world = (w_T_b @ pts)[:2, :].T

        if len(trajs_in_world) == 0:
            self.get_logger().warn('No waypoints after skipping first 3')
            return

        # --------------------------------------------------------
        # slowdown interpolate
        if len(trajs_in_world) > 1 and self._slowdown_factor > 1:
            old_indices = np.arange(len(trajs_in_world))
            num_new = (len(trajs_in_world) - 1) * self._slowdown_factor + 1
            new_indices = np.linspace(0, len(trajs_in_world) - 1, num_new)
            x_interp = np.interp(new_indices, old_indices, trajs_in_world[:, 0])
            y_interp = np.interp(new_indices, old_indices, trajs_in_world[:, 1])
            trajs_in_world = np.column_stack((x_interp, y_interp))
        # --------------------------------------------------------

        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = base_msg.header.stamp

        for x, y in trajs_in_world:
            pose = PoseStamped()

            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y

            msg.poses.append(pose)

        self._cmd_path_pub.publish(msg)

    def process_discretes(self, actions: List[int], header: Header, odom_infer: Tuple[float, float, float]):
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
                homo_goal[:3, :3] = self._rot_left @ homo_goal[:3, :3]
            elif action == DiscreteStamped.TURN_RIGHT:
                homo_goal[:3, :3] = self._rot_right @ homo_goal[:3, :3]

        final_x = homo_goal[0, 3]
        final_y = homo_goal[1, 3]
        final_yaw = math.atan2(homo_goal[1, 0], homo_goal[0, 0])

        msg = PoseStamped()

        msg.header.frame_id = 'odom'
        msg.header.stamp = header.stamp
        msg.pose.position.x = float(final_x)
        msg.pose.position.y = float(final_y)
        msg.pose.orientation.z = math.sin(final_yaw / 2)
        msg.pose.orientation.w = math.cos(final_yaw / 2)

        self._cmd_pose_pub.publish(msg)

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
