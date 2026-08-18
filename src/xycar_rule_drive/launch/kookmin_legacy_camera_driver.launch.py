from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("xycar_rule_drive"), "config", "kookmin_legacy_camera_driver.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=config_file,
                description="YAML parameters for the Kookmin legacy camera driver.",
            ),
            Node(
                package="xycar_rule_drive",
                executable="kookmin_legacy_camera_driver",
                name="kookmin_legacy_camera_driver",
                parameters=[LaunchConfiguration("config_file")],
                output="screen",
            ),
        ]
    )
