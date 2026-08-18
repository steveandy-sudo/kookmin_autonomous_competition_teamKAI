"""Start only the real sensors and native VESC used by hybrid testing."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package: str, launch_file: str, *, condition: str, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package), "launch", launch_file]
            )
        ),
        condition=IfCondition(LaunchConfiguration(condition)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    from launch.actions import DeclareLaunchArgument

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_vesc", default_value="true"),
            DeclareLaunchArgument("vesc_drive_enabled", default_value="false"),
            DeclareLaunchArgument(
                "steering_center_trim_command", default_value="-5.0"
            ),
            DeclareLaunchArgument("camera_device", default_value=(
                "/dev/v4l/by-id/"
                "usb-HD_USB_Camera_HD_USB_Camera-video-index0"
            )),
            DeclareLaunchArgument("lidar_params_file", default_value=(
                PathJoinSubstitution(
                    [FindPackageShare("xycar_lidar"), "params", "ydlidar.yaml"]
                )
            )),
            include(
                "wide_camera",
                "wide_camera.launch.py",
                condition="start_camera",
                arguments={"device": LaunchConfiguration("camera_device")},
            ),
            include(
                "xycar_lidar",
                "xycar_lidar.launch.py",
                condition="start_lidar",
                arguments={
                    "params_file": LaunchConfiguration("lidar_params_file")
                },
            ),
            include(
                "xycar_vesc_driver",
                "xycar_vesc_driver.launch.py",
                condition="start_vesc",
                arguments={
                    "port": "/dev/ttyMOTOR",
                    "drive_enabled": LaunchConfiguration(
                        "vesc_drive_enabled"
                    ),
                    "steering_center_trim_command": LaunchConfiguration(
                        "steering_center_trim_command"
                    ),
                },
            ),
        ]
    )
