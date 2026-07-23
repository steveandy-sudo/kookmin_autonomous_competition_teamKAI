from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("xycar_rule_drive")
    return LaunchDescription(
        [
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            package_share,
                            "launch",
                            "canonical_stanley_pursuit.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "drive_enabled": LaunchConfiguration("drive_enabled"),
                }.items(),
            ),
        ]
    )
