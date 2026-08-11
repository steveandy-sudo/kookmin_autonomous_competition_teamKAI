"""Run shortcut-only LR-ASPP behind the existing hybrid Bool gate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lane_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "lane_seg_lraspp_canonical_only.launch.py",
        ]
    )
    arguments = [
        DeclareLaunchArgument(
            "lane_model",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("xycar_perception"),
                    "models",
                    "kookmin_lane_lraspp_mbv3s_256x144.pt",
                ]
            ),
        ),
        DeclareLaunchArgument(
            "source_topic",
            default_value="/wide_camera_mjpeg/image_raw/compressed",
        ),
        DeclareLaunchArgument(
            "processing_enabled_topic",
            default_value="/hybrid/shortcut_processing_enabled",
        ),
    ]
    gate = Node(
        package="shortcut_entry_review",
        executable="compressed_image_gate",
        name="shortcut_compressed_image_gate",
        output="screen",
        parameters=[
            {
                "source_topic": LaunchConfiguration("source_topic"),
                "output_topic": "/shortcut/lraspp/input/compressed",
                "processing_enabled_topic": LaunchConfiguration(
                    "processing_enabled_topic"
                ),
                "default_enabled": False,
            }
        ],
    )
    lane = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lane_launch),
        launch_arguments={
            "model_path": LaunchConfiguration("lane_model"),
            "image_topic": "/shortcut/lraspp/input/compressed",
            "use_compressed_image": "true",
            "enable_rectify": "true",
            "direct_model_rectify_enabled": "true",
            "direct_model_rectify_oversample": "3",
            "max_input_age_sec": "0.0",
            "direct_canonical_enabled": "false",
            "publish_intermediate_topics": "true",
            "max_output_rate_hz": "15.0",
            "debug_rate_hz": "5.0",
            "pipeline_qos_depth": "1",
            "cpu_threads": "4",
        }.items(),
    )
    isolated_lane = GroupAction(
        [
            SetRemap(
                src="/lane_seg/source_image",
                dst="/shortcut/lraspp/source_image",
            ),
            SetRemap(
                src="/lane_seg/white_boundary_mask",
                dst="/shortcut/lraspp/white_mask",
            ),
            SetRemap(
                src="/lane_seg/yellow_centerline_mask",
                dst="/shortcut/lraspp/yellow_mask",
            ),
            SetRemap(
                src="/lane_seg/debug_image",
                dst="/shortcut/lraspp/debug_image",
            ),
            SetRemap(
                src="/lane_seg/diagnostics",
                dst="/shortcut/lraspp/diagnostics",
            ),
            SetRemap(
                src="/perception/yolo_debug_image",
                dst="/shortcut/lraspp/perception_debug_image",
            ),
            lane,
        ]
    )
    selector = Node(
        package="shortcut_entry_review",
        executable="white_yellow_entry_review",
        name="white_yellow_entry_review",
        output="screen",
        parameters=[
            {
                "white_mask_topic": "/shortcut/lraspp/white_mask",
                "yellow_mask_topic": "/shortcut/lraspp/yellow_mask",
                "processing_enabled_topic": LaunchConfiguration(
                    "processing_enabled_topic"
                ),
                "default_enabled": False,
                "debug_topic": "/shortcut/entry/debug_image",
                "status_topic": "/shortcut/entry/status",
                "diagnostics_topic": "/shortcut/entry/diagnostics",
            }
        ],
    )
    return LaunchDescription([*arguments, gate, isolated_lane, selector])
