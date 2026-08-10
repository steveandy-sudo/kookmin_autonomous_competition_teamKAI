from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    preview_launch = PathJoinSubstitution(
        [
            FindPackageShare("my_control"),
            "launch",
            "real_lane_drive_rviz.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument("use_compressed_image", default_value="true"),
            DeclareLaunchArgument("enable_rectify", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(preview_launch),
                launch_arguments={
                    "image_topic": LaunchConfiguration("image_topic"),
                    "use_compressed_image": LaunchConfiguration(
                        "use_compressed_image"
                    ),
                    "enable_rectify": LaunchConfiguration("enable_rectify"),
                    "start_camera": LaunchConfiguration("start_camera"),
                }.items(),
            ),
            LogInfo(
                msg=(
                    "Calibrated real-camera 1.5 m BEV preview enabled. "
                    "Input is rectified once inside perception. This launch remains "
                    "in SHADOW mode and never publishes physical motor output."
                )
            ),
        ]
    )
