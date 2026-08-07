"""Run only the final rule command adapter for isolated topic tests."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _as_float(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument(
                "straight_speed_command", default_value="10.0"
            ),
            DeclareLaunchArgument("turn_speed_command", default_value="8.0"),
            DeclareLaunchArgument(
                "slowdown_start_angle_command", default_value="20.0"
            ),
            DeclareLaunchArgument(
                "full_slowdown_angle_command", default_value="42.0"
            ),
            DeclareLaunchArgument("speed_curve_exponent", default_value="1.0"),
            DeclareLaunchArgument("obstacle_shift_m", default_value="0.20"),
            DeclareLaunchArgument(
                "obstacle_release_delay_sec", default_value="2.0"
            ),
            Node(
                package="xycar_rule_drive",
                executable="rule_command_adapter",
                name="rule_command_adapter_test",
                output="screen",
                parameters=[
                    {
                        "drive_enabled": _as_bool("drive_enabled"),
                        "straight_speed_command": _as_float(
                            "straight_speed_command"
                        ),
                        "turn_speed_command": _as_float(
                            "turn_speed_command"
                        ),
                        "slowdown_start_angle_command": _as_float(
                            "slowdown_start_angle_command"
                        ),
                        "full_slowdown_angle_command": _as_float(
                            "full_slowdown_angle_command"
                        ),
                        "speed_curve_exponent": _as_float(
                            "speed_curve_exponent"
                        ),
                        "obstacle_shift_m": _as_float("obstacle_shift_m"),
                        "obstacle_release_delay_sec": _as_float(
                            "obstacle_release_delay_sec"
                        ),
                    }
                ],
            ),
        ]
    )
