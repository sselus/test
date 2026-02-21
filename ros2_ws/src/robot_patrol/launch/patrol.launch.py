from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    waypoint_file = LaunchConfiguration("waypoint_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("waypoint_file", default_value=""),
            Node(
                package="robot_patrol",
                executable="patrol_node",
                name="patrol_node",
                parameters=[{"waypoint_file": waypoint_file}],
            ),
        ]
    )
