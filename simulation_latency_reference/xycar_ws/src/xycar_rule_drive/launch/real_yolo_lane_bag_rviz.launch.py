from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    playback_rate = LaunchConfiguration("playback_rate")
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    yolo_device = LaunchConfiguration("yolo_device")
    yolo_confidence = LaunchConfiguration("yolo_confidence")
    yolo_image_size = LaunchConfiguration("yolo_image_size")
    base_launch = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "launch",
            "real_temp_track_bag_rviz.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Absolute path to a ROS 2 bag directory.",
            ),
            DeclareLaunchArgument("playback_rate", default_value="0.25"),
            DeclareLaunchArgument(
                "yolo_model_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("xycar_perception"),
                        "models",
                        "kookmin_lane_yolo11n_512.pt",
                    ]
                ),
            ),
            DeclareLaunchArgument("yolo_device", default_value="0"),
            DeclareLaunchArgument("yolo_confidence", default_value="0.25"),
            DeclareLaunchArgument("yolo_image_size", default_value="512"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "bag_path": bag_path,
                    "playback_rate": playback_rate,
                    "bag_start_delay": "4.0",
                    "perception_launch_file": (
                        "real_canonical_perception.launch.py"
                    ),
                    "lane_segmentation_backend": "yolo",
                    "yolo_model_path": yolo_model_path,
                    "yolo_device": yolo_device,
                    "yolo_confidence": yolo_confidence,
                    "yolo_image_size": yolo_image_size,
                }.items(),
            ),
        ]
    )
