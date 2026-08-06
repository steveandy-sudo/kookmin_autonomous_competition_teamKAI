from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    canonical_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "lane_seg_lraspp_canonical_only.launch.py",
        ]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(canonical_launch),
                launch_arguments={
                    "image_topic": "/image_raw",
                    "debug_rate_hz": "7.0",
                    # Match the real vehicle's canonical runtime contract.
                    "max_output_rate_hz": "7.0",
                    # The inference node publishes source/white/yellow as a
                    # timestamp-matched triplet. Keep several triplets so the
                    # adapter cannot lose one member while processing a frame.
                    "pipeline_qos_depth": "5",
                    # Keep the established Gazebo camera projection. The real
                    # wrapper inherits the calibrated stationary-bag profile.
                    "src_tl_x_ratio": str(472.0 / 1280.0),
                    "src_tl_y_ratio": str(494.0 / 1024.0),
                    "src_tr_x_ratio": str(906.0 / 1280.0),
                    "src_tr_y_ratio": str(486.0 / 1024.0),
                    "src_br_x_ratio": str(1272.0 / 1280.0),
                    "src_br_y_ratio": str(612.0 / 1024.0),
                    "src_bl_x_ratio": str(46.0 / 1280.0),
                    "src_bl_y_ratio": str(622.0 / 1024.0),
                    "dst_left_ratio": str(80.0 / 640.0),
                    "dst_right_ratio": str(560.0 / 640.0),
                    "dst_bottom_y_ratio": str(479.0 / 660.0),
                    "canonical_yellow_normalize_enabled": "true",
                    "canonical_yellow_normalize_line_width_px": "3",
                }.items(),
            )
        ]
    )
