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

    perception_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "lane_seg_lraspp_canonical_only.launch.py",
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "driver_mode",
                default_value="rule",
                description="Select exactly one motor-command source: rule or rl.",
            ),
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument("device", default_value="cpu"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch),
                condition=IfCondition(start_perception),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rule_launch),
                condition=_mode_is(driver_mode, "rule"),
                launch_arguments={
                    "drive_enabled": drive_enabled,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rl_launch),
                condition=_mode_is(driver_mode, "rl"),
                launch_arguments={
                    "drive_enabled": drive_enabled,
                    "device": device,
                }.items(),
            ),
        ]
    )
