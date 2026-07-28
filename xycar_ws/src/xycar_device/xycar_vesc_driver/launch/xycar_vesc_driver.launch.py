from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = Path(
        get_package_share_directory("xycar_vesc_driver")
    )
    default_config = package_share / "config" / "xycar_vesc_driver.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=str(default_config),
                description="Native VESC driver parameter file.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyMOTOR",
                description="VESC serial device.",
            ),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description="Allow non-zero motor output.",
            ),
            Node(
                package="xycar_vesc_driver",
                executable="xycar_vesc_driver",
                name="xycar_vesc_driver",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "port": LaunchConfiguration("port"),
                        "drive_enabled": ParameterValue(
                            LaunchConfiguration("drive_enabled"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
