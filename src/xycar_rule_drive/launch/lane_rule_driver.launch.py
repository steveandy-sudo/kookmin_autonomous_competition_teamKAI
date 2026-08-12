from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("xycar_rule_drive"), "config", "lane_rule_driver.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=config_file,
                description="YAML parameters for rule-based lane driving.",
            ),
            Node(
                package="xycar_rule_drive",
                executable="lane_rule_driver",
                name="xycar_lane_rule_driver",
                parameters=[LaunchConfiguration("config_file")],
                output="screen",
            ),
        ]
    )
