from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def _mode_is(mode, expected):
    return IfCondition(PythonExpression(["'", mode, "' == '", expected, "'"]))


def generate_launch_description():
    driver_mode = LaunchConfiguration("driver_mode")
    drive_enabled = LaunchConfiguration("drive_enabled")
    start_perception = LaunchConfiguration("start_perception")
    device = LaunchConfiguration("device")
    perception_cpu_threads = LaunchConfiguration("perception_cpu_threads")
    policy_cpu_threads = LaunchConfiguration("policy_cpu_threads")
    target_lateral_offset_m = LaunchConfiguration("target_lateral_offset_m")
    checkpoint_path = LaunchConfiguration("checkpoint_path")
    model_speed_cap = LaunchConfiguration("model_speed_cap")
    cone_speed_cap = LaunchConfiguration("cone_speed_cap")

    perception_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "lane_seg_lraspp_low_latency_real.launch.py",
        ]
    )
    rule_launch = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "launch",
            "canonical_stanley_pursuit_real.launch.py",
        ]
    )
    rl_launch = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rl"),
            "launch",
            "final_uncapped_avg17_real.launch.py",
        ]
    )
    hybrid_launch = PathJoinSubstitution(
        [
            FindPackageShare("xycar_hybrid_drive"),
            "launch",
            "hybrid_real.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "driver_mode",
                default_value="hybrid",
                description="Select one direct motor-command source: hybrid, rule, or rl.",
            ),
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument(
                "perception_cpu_threads", default_value="4"
            ),
            DeclareLaunchArgument("policy_cpu_threads", default_value="4"),
            DeclareLaunchArgument(
                "target_lateral_offset_m",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "checkpoint_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("xycar_rl"),
                        "models",
                        "final_rule_td3_bc_uncapped_avg17_20260723",
                        "camera_speed_td3_bc_best.pth",
                    ]
                ),
            ),
            DeclareLaunchArgument("model_speed_cap", default_value="10.0"),
            DeclareLaunchArgument("cone_speed_cap", default_value="10.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch),
                condition=IfCondition(start_perception),
                launch_arguments={
                    "cpu_threads": perception_cpu_threads,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rule_launch),
                condition=_mode_is(driver_mode, "rule"),
                launch_arguments={
                    "drive_enabled": drive_enabled,
                    "target_lateral_offset_m": target_lateral_offset_m,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rl_launch),
                condition=_mode_is(driver_mode, "rl"),
                launch_arguments={
                    "drive_enabled": drive_enabled,
                    "device": device,
                    "policy_cpu_threads": policy_cpu_threads,
                    "target_lateral_offset_m": target_lateral_offset_m,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hybrid_launch),
                condition=_mode_is(driver_mode, "hybrid"),
                launch_arguments={
                    "drive_enabled": drive_enabled,
                    "device": device,
                    "policy_cpu_threads": policy_cpu_threads,
                    "checkpoint_path": checkpoint_path,
                    "target_lateral_offset_m": target_lateral_offset_m,
                    "model_speed_cap": model_speed_cap,
                    "cone_speed_cap": cone_speed_cap,
                }.items(),
            ),
        ]
    )
