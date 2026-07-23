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
                executable="mission_start_signal_adapter",
                name="mission_start_signal_adapter",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="track_drive",
                executable="mission_drive_policy_adapter",
                name="mission_drive_policy_adapter",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="track_drive",
                executable="mission_lane_fallback_adapter",
                name="mission_lane_fallback_adapter",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="track_drive",
                executable="mission_camera_cone_adapter",
                name="mission_camera_cone_adapter",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="track_drive",
                executable="mission_lidar_cone_adapter",
                name="mission_lidar_cone_adapter",
                output="screen",
                parameters=[config_path],
            ),
            Node(
                package="track_drive",
                executable="mission_manager",
                name="mission_manager",
                output="screen",
                parameters=[config_path],
            )
        ]
    )
