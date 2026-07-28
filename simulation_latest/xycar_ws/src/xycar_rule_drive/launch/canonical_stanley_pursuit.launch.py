from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "config",
            "canonical_stanley_pursuit.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=config_file),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("drive_enabled", default_value="true"),
            DeclareLaunchArgument(
                "target_lateral_offset_m",
                default_value="0.10",
            ),
            Node(
                package="xycar_rule_drive",
                executable="canonical_stanley_pursuit_driver",
                name="canonical_stanley_pursuit_driver",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                        "drive_enabled": ParameterValue(
                            LaunchConfiguration("drive_enabled"),
                            value_type=bool,
                        ),
                        "target_lateral_offset_m": ParameterValue(
                            LaunchConfiguration("target_lateral_offset_m"),
                            value_type=float,
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
