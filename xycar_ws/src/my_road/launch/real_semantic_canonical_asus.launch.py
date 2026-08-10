from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    real_launch = PathJoinSubstitution(
        [
            FindPackageShare("my_road"),
            "launch",
            "real_canonical_perception.launch.py",
        ]
    )
    arguments = {
        "image_topic": LaunchConfiguration("image_topic"),
        "use_compressed_image": LaunchConfiguration("use_compressed_image"),
        "enable_rectify": LaunchConfiguration("enable_rectify"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "lane_segmentation_backend": "semantic",
        "semantic_model_path": LaunchConfiguration("semantic_model_path"),
        "semantic_device": "cpu",
        "semantic_input_width": "256",
        "semantic_input_height": "144",
        "semantic_cpu_threads": LaunchConfiguration("semantic_cpu_threads"),
        "publish_rate_limit_hz": LaunchConfiguration("publish_rate_limit_hz"),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument("use_compressed_image", default_value="true"),
            DeclareLaunchArgument("enable_rectify", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "semantic_model_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("my_road"),
                        "models",
                        "kookmin_lane_lraspp_mbv3s_256x144.pt",
                    ]
                ),
            ),
            DeclareLaunchArgument("semantic_cpu_threads", default_value="1"),
            DeclareLaunchArgument("publish_rate_limit_hz", default_value="20.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_launch),
                launch_arguments=arguments.items(),
            ),
        ]
    )
