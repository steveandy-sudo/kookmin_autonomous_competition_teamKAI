import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rule_share = get_package_share_directory("my_rule")
    camera_share = get_package_share_directory("app_wide_camera_calib")
    default_model = PathJoinSubstitution([FindPackageShare("my_rule"), "models", "best.pt"])
    default_extrinsic = os.path.join(
        camera_share,
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
            DeclareLaunchArgument("fps", default_value="30"),
            DeclareLaunchArgument("balance", default_value="0.3"),
            DeclareLaunchArgument("use_lidar_camera_projection", default_value="true"),
            DeclareLaunchArgument("publish_extrinsic_tf", default_value="true"),
            DeclareLaunchArgument("extrinsic_yaml", default_value=default_extrinsic),
            DeclareLaunchArgument("start_yolo", default_value="false"),
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("yolo_device", default_value="cpu"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(camera_share, "launch", "wide_camera_rectified.launch.py")
                ),
                launch_arguments={
                    "device": LaunchConfiguration("device"),
                    "fps": LaunchConfiguration("fps"),
                    "balance": LaunchConfiguration("balance"),
                    "publish_extrinsic_tf": LaunchConfiguration("publish_extrinsic_tf"),
                    "extrinsic_yaml": LaunchConfiguration("extrinsic_yaml"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rule_share, "launch", "my_rule.launch.py")
                ),
                launch_arguments={
                    "image_topic": "/wide_camera/rect/image_raw",
                    "camera_info_topic": "/wide_camera/rect/camera_info",
                    "detection_topic": "/yolo/detections",
                    "use_lidar_camera_projection": LaunchConfiguration(
                        "use_lidar_camera_projection"
                    ),
                    "lidar_camera_extrinsic_yaml": LaunchConfiguration("extrinsic_yaml"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rule_share, "launch", "my_rule_yolo.launch.py")
                ),
                launch_arguments={
                    "model": LaunchConfiguration("model"),
                    "image_topic": "/wide_camera/rect/image_raw",
                    "device": LaunchConfiguration("yolo_device"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_yolo")),
            ),
        ]
    )
