import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_calibration = os.path.join(
        get_package_share_directory("app_wide_camera_calib"),
        "config",
        "wide_camera_fisheye_1280x1024.yaml",
    )
    default_extrinsic = os.path.join(
        get_package_share_directory("app_wide_camera_calib"),
        "config",
        "lidar_camera_extrinsic_measured.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device",
                default_value=(
                    "/dev/v4l/by-id/"
                    "usb-HD_USB_Camera_HD_USB_Camera-video-index0"
                ),
            ),
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="1024"),
            DeclareLaunchArgument("fps", default_value="30"),
            DeclareLaunchArgument("auto_exposure", default_value="3"),
            DeclareLaunchArgument("exposure_time_absolute", default_value="157"),
            DeclareLaunchArgument("gain", default_value="0"),
            DeclareLaunchArgument("backlight_compensation", default_value="1"),
            DeclareLaunchArgument("brightness", default_value="0"),
            DeclareLaunchArgument("frame_id", default_value="wide_camera_optical_frame"),
            DeclareLaunchArgument(
                "compressed_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument("image_topic", default_value="/wide_camera/rect/image_raw"),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/wide_camera/rect/camera_info",
            ),
            DeclareLaunchArgument("camera_yaml", default_value=default_calibration),
            DeclareLaunchArgument("balance", default_value="0.3"),
            DeclareLaunchArgument("publish_extrinsic_tf", default_value="false"),
            DeclareLaunchArgument("extrinsic_yaml", default_value=default_extrinsic),
            Node(
                package="app_wide_camera",
                executable="mjpeg_passthrough_node",
                name="wide_camera_mjpeg",
                output="screen",
                parameters=[
                    {
                        "device": LaunchConfiguration("device"),
                        "width": LaunchConfiguration("width"),
                        "height": LaunchConfiguration("height"),
                        "fps": LaunchConfiguration("fps"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "topic": LaunchConfiguration("compressed_topic"),
                        "power_line_frequency": 2,
                        "exposure_dynamic_framerate": 0,
                        "auto_exposure": LaunchConfiguration("auto_exposure"),
                        "exposure_time_absolute": LaunchConfiguration(
                            "exposure_time_absolute"
                        ),
                        "gain": LaunchConfiguration("gain"),
                        "backlight_compensation": LaunchConfiguration(
                            "backlight_compensation"
                        ),
                        "brightness": LaunchConfiguration("brightness"),
                    }
                ],
            ),
            Node(
                package="app_wide_camera_calib",
                executable="rectify_wide_camera_node",
                name="wide_camera_rectifier",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("compressed_topic"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "camera_yaml": LaunchConfiguration("camera_yaml"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "balance": LaunchConfiguration("balance"),
                    }
                ],
            ),
            Node(
                package="app_wide_camera_calib",
                executable="publish_lidar_camera_static_tf",
                name="lidar_camera_static_tf",
                output="screen",
                parameters=[{"yaml_path": LaunchConfiguration("extrinsic_yaml")}],
                condition=IfCondition(LaunchConfiguration("publish_extrinsic_tf")),
            ),
        ]
    )
