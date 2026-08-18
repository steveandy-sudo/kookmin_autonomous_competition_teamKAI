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
    model_path = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "models",
            "kookmin_far_centerline_xbin_512x288.pt",
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
            DeclareLaunchArgument("max_output_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("debug_rate_hz", default_value="0.0"),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument("use_compressed_image", default_value="true"),
            DeclareLaunchArgument("enable_rectify", default_value="true"),
            DeclareLaunchArgument(
                "direct_model_rectify_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "direct_model_rectify_oversample", default_value="2"
            ),
            DeclareLaunchArgument(
                "publish_intermediate_topics", default_value="false"
            ),
            DeclareLaunchArgument("bev_height", default_value="660"),
            DeclareLaunchArgument("dst_top_y_ratio", default_value="0.0"),
            DeclareLaunchArgument(
                "dst_bottom_y_ratio", default_value="0.666666667"
            ),
            DeclareLaunchArgument(
                "forward_m_per_px", default_value="0.002272727273"
            ),
            DeclareLaunchArgument(
                "canonical_forward_range_m", default_value="1.5"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(canonical_launch),
                launch_arguments={
                    "model_path": model_path,
                    "input_width": "512",
                    "input_height": "288",
                    "image_topic": LaunchConfiguration("image_topic"),
                    "use_compressed_image": LaunchConfiguration(
                        "use_compressed_image"
                    ),
                    "enable_rectify": LaunchConfiguration("enable_rectify"),
                    "direct_model_rectify_enabled": LaunchConfiguration(
                        "direct_model_rectify_enabled"
                    ),
                    "direct_model_rectify_oversample": LaunchConfiguration(
                        "direct_model_rectify_oversample"
                    ),
                    "camera_yaml": camera_yaml,
                    "rect_balance": "0.3",
                    "max_input_age_sec": "0.25",
                    "direct_canonical_enabled": "true",
                    "publish_intermediate_topics": LaunchConfiguration(
                        "publish_intermediate_topics"
                    ),
                    "pipeline_qos_depth": "1",
                    "bev_height": LaunchConfiguration("bev_height"),
                    "dst_top_y_ratio": LaunchConfiguration(
                        "dst_top_y_ratio"
                    ),
                    "dst_bottom_y_ratio": LaunchConfiguration(
                        "dst_bottom_y_ratio"
                    ),
                    "forward_m_per_px": LaunchConfiguration(
                        "forward_m_per_px"
                    ),
                    "canonical_forward_range_m": LaunchConfiguration(
                        "canonical_forward_range_m"
                    ),
                    "white_confidence": "0.99",
                    "yellow_confidence": "0.50",
                    "cpu_threads": LaunchConfiguration("cpu_threads"),
                    "max_output_rate_hz": LaunchConfiguration(
                        "max_output_rate_hz"
                    ),
                    "debug_rate_hz": LaunchConfiguration("debug_rate_hz"),
                }.items(),
            ),
        ]
    )
