from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "lane_seg_far_centerline_low_latency_real.launch.py",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("cpu_threads", default_value="4"),
            DeclareLaunchArgument("max_output_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("debug_rate_hz", default_value="0.0"),
            DeclareLaunchArgument(
                "publish_intermediate_topics", default_value="false"
            ),
            DeclareLaunchArgument(
                "canonical_forward_range_m", default_value="2.5"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    "cpu_threads": LaunchConfiguration("cpu_threads"),
                    "max_output_rate_hz": LaunchConfiguration(
                        "max_output_rate_hz"
                    ),
                    "debug_rate_hz": LaunchConfiguration("debug_rate_hz"),
                    "publish_intermediate_topics": LaunchConfiguration(
                        "publish_intermediate_topics"
                    ),
                    # Preserve the measured 1.5 m homography and translate it
                    # down by 440 BEV rows. This exposes its flat-ground
                    # projective continuation from 1.5 m to 2.5 m without
                    # changing the lateral or longitudinal metres per pixel.
                    "bev_height": "1100",
                    "dst_top_y_ratio": "0.4",
                    "dst_bottom_y_ratio": "0.8",
                    "forward_m_per_px": "0.002272727273",
                    "canonical_forward_range_m": LaunchConfiguration(
                        "canonical_forward_range_m"
                    ),
                }.items(),
            ),
        ]
    )
