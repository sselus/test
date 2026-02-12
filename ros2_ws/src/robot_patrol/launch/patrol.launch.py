from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("waypoint_file", default_value=""),
            Node(
                package="robot_patrol",
                executable="patrol_node",
                name="patrol_node",
                parameters=[{"waypoint_file": ""}],
            ),
        ]
    )
