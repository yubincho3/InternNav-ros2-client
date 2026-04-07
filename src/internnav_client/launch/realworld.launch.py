from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_node = Node(
        package='internnav_client',
        executable='controller',
        name='internnav_controller',
        output='screen',
    )

    planner_node = Node(
        package='internnav_client',
        executable='planner',
        name='internnav_planner',
        output='screen',
    )

    return LaunchDescription([
        controller_node,
        planner_node,
    ])
