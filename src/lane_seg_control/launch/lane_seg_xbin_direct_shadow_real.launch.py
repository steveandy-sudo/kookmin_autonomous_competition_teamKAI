"""Run direct Xbin metric-path control in motor-free shadow mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    perception = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "lane_seg_far_centerline_extended_real.launch.py",
        ]
    )
    rule_base = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "config",
            "canonical_stanley_pursuit.yaml",
        ]
    )
    rule_real = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "config",
            "canonical_stanley_pursuit_real.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument("max_output_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("command_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("speed_command", default_value="8.0"),
            DeclareLaunchArgument("curve_speed_command", default_value="8.0"),
            DeclareLaunchArgument(
                "degraded_path_speed_command", default_value="8.0"
            ),
            DeclareLaunchArgument(
                "shadow_motor_topic",
                default_value="/hybrid/xbin_direct_rule_candidate",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception),
                launch_arguments={
                    "image_topic": LaunchConfiguration("image_topic"),
                    "max_output_rate_hz": LaunchConfiguration(
                        "max_output_rate_hz"
                    ),
                    "direct_centerline_enabled": "true",
                    "direct_centerline_topic": (
                        "/perception/xbin_direct_centerline"
                    ),
                    "direct_canonical_enabled": "false",
                    "start_canonical_adapter": "false",
                    "publish_intermediate_topics": "false",
                    "debug_rate_hz": "0.0",
                }.items(),
            ),
            Node(
                package="xycar_rule_drive",
                executable="canonical_stanley_pursuit_driver",
                name="xbin_direct_stanley_pursuit_shadow",
                namespace="/",
                output="screen",
                parameters=[
                    rule_base,
                    rule_real,
                    {
                        "external_path_enabled": True,
                        "external_path_topic": (
                            "/perception/xbin_direct_centerline"
                        ),
                        "external_path_timeout_sec": 0.50,
                        "external_path_previous_weight": 0.0,
                        "canonical_forward_range_m": 2.5,
                        "command_rate_hz": _float("command_rate_hz"),
                        "drive_enabled": False,
                        "steering_only": True,
                        "shadow_motor_topic": LaunchConfiguration(
                            "shadow_motor_topic"
                        ),
                        "target_path_topic": (
                            "/rule_drive/xbin_direct_connected_path"
                        ),
                        "diagnostics_topic": (
                            "/rule_drive/xbin_direct_diagnostics"
                        ),
                        "debug_image_topic": (
                            "/rule_drive/xbin_direct_debug_image"
                        ),
                        "debug_markers_topic": (
                            "/rule_drive/xbin_direct_debug_markers"
                        ),
                        "cruise_speed_command": _float("speed_command"),
                        "minimum_speed_command": _float(
                            "curve_speed_command"
                        ),
                        "curve_speed_command": _float(
                            "curve_speed_command"
                        ),
                        "degraded_path_speed_command": _float(
                            "degraded_path_speed_command"
                        ),
                    },
                ],
            ),
        ]
    )
