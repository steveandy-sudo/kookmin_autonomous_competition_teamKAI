from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    canonical_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "lane_seg_lraspp_canonical_only.launch.py",
        ]
    )
    camera_yaml = PathJoinSubstitution(
        [
            FindPackageShare("xycar_perception"),
            "config",
            "wide_camera_fisheye_1280x1024_20260708.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("cpu_threads", default_value="4"),
            DeclareLaunchArgument("max_output_rate_hz", default_value="15.0"),
            DeclareLaunchArgument("debug_rate_hz", default_value="0.0"),
            DeclareLaunchArgument(
                "direct_model_rectify_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "direct_model_rectify_oversample", default_value="3"
            ),
            DeclareLaunchArgument(
                "publish_intermediate_topics", default_value="false"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(canonical_launch),
                launch_arguments={
                    "image_topic": (
                        "/wide_camera_mjpeg/image_raw/compressed"
                    ),
                    "use_compressed_image": "true",
                    "enable_rectify": "true",
                    "direct_model_rectify_enabled": LaunchConfiguration(
                        "direct_model_rectify_enabled"
                    ),
                    "direct_model_rectify_oversample": LaunchConfiguration(
                        "direct_model_rectify_oversample"
                    ),
                    "camera_yaml": camera_yaml,
                    "rect_balance": "0.3",
                    "max_input_age_sec": "0.35",
                    "direct_canonical_enabled": "true",
                    "publish_intermediate_topics": LaunchConfiguration(
                        "publish_intermediate_topics"
                    ),
                    "pipeline_qos_depth": "1",
                    "cpu_threads": LaunchConfiguration("cpu_threads"),
                    "max_output_rate_hz": LaunchConfiguration(
                        "max_output_rate_hz"
                    ),
                    "debug_rate_hz": LaunchConfiguration("debug_rate_hz"),
                }.items(),
            ),
        ]
    )
