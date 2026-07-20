from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    real_launch = PathJoinSubstitution(
        [
            FindPackageShare("xycar_perception"),
            "launch",
            "real_canonical_perception.launch.py",
        ]
    )

    arguments = {
        "image_topic": LaunchConfiguration("image_topic"),
        "use_compressed_image": LaunchConfiguration("use_compressed_image"),
        "enable_rectify": LaunchConfiguration("enable_rectify"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "lane_segmentation_backend": "yolo",
        "yolo_model_path": LaunchConfiguration("yolo_model_path"),
        "yolo_device": "cpu",
        "yolo_confidence": LaunchConfiguration("yolo_confidence"),
        "yolo_image_size": LaunchConfiguration("yolo_image_size"),
        "yolo_cpu_threads": LaunchConfiguration("yolo_cpu_threads"),
        "publish_rate_limit_hz": LaunchConfiguration(
            "publish_rate_limit_hz"
        ),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic", default_value="/wide_camera/rect/image_raw"
            ),
            DeclareLaunchArgument("use_compressed_image", default_value="false"),
            DeclareLaunchArgument("enable_rectify", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
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
            DeclareLaunchArgument("yolo_confidence", default_value="0.25"),
            DeclareLaunchArgument("yolo_image_size", default_value="512"),
            DeclareLaunchArgument("yolo_cpu_threads", default_value="4"),
            DeclareLaunchArgument(
                "publish_rate_limit_hz",
                default_value="15.0",
                description=(
                    "Lane inference rate cap. Keep obstacle YOLO at 5 Hz or less "
                    "on the Ryzen 5 vehicle PC."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_launch),
                launch_arguments=arguments.items(),
            ),
        ]
    )
