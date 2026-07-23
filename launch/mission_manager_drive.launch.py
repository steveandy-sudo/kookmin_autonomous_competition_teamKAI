"""Run Mission Manager V0.2 with the single 50 Hz Final Driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("track_drive")
    config_path = PathJoinSubstitution(
        [package_share, "config", "mission_manager.yaml"]
    )
    draft_launch_path = PathJoinSubstitution(
        [package_share, "launch", "mission_manager_draft.launch.py"]
    )
    fallback_speed = LaunchConfiguration("fallback_speed")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fallback_speed",
                description=(
                    "Vehicle-tested numeric speed used while an IL or "
                    "Lane Fallback steering value is held"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(draft_launch_path)
            ),
            Node(
                package="track_drive",
                executable="final_driver",
                name="final_driver",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "fallback_speed": ParameterValue(
                            fallback_speed,
                            value_type=float,
                        )
                    },
                ],
            ),
        ]
    )
