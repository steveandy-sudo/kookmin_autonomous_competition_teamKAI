import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("track_drive")
    config_path = os.path.join(
        package_share, "config", "mission_manager.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="track_drive",
                executable="mission_manager",
                name="mission_manager",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
