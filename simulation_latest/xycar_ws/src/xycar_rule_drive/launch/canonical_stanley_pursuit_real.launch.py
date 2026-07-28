from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("xycar_rule_drive")
    base_config = PathJoinSubstitution(
        [
            package_share,
            "config",
            "canonical_stanley_pursuit.yaml",
        ]
    )
    real_config = PathJoinSubstitution(
        [
            package_share,
            "config",
            "canonical_stanley_pursuit_real.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument(
                "target_lateral_offset_m",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "cruise_speed_command",
                default_value="8.0",
            ),
            DeclareLaunchArgument(
                "minimum_speed_command",
                default_value="6.0",
            ),
            DeclareLaunchArgument(
                "control_latency_preview_sec",
                default_value="0.30",
            ),
            Node(
                package="xycar_rule_drive",
                executable="canonical_stanley_pursuit_driver",
                name="canonical_stanley_pursuit_driver",
                parameters=[
                    base_config,
                    real_config,
                    {
                        "use_sim_time": False,
                        "drive_enabled": ParameterValue(
                            LaunchConfiguration("drive_enabled"),
                            value_type=bool,
                        ),
                        "target_lateral_offset_m": ParameterValue(
                            LaunchConfiguration("target_lateral_offset_m"),
                            value_type=float,
                        ),
                        "cruise_speed_command": ParameterValue(
                            LaunchConfiguration("cruise_speed_command"),
                            value_type=float,
                        ),
                        "minimum_speed_command": ParameterValue(
                            LaunchConfiguration("minimum_speed_command"),
                            value_type=float,
                        ),
                        "control_latency_preview_sec": ParameterValue(
                            LaunchConfiguration(
                                "control_latency_preview_sec"
                            ),
                            value_type=float,
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
