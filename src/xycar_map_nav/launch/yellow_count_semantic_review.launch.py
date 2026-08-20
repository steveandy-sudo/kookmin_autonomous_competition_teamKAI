"""Run raw gated inference; the viewer applies the exact W1 input BEV warp."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_gate = Node(
        package="shortcut_entry_review",
        executable="compressed_image_gate",
        name="yellow_count_lraspp_camera_gate",
        output="screen",
        parameters=[
            {
                "source_topic": LaunchConfiguration("source_topic"),
                "output_topic": "/yellow_count/lraspp/input/compressed",
                "processing_enabled_topic": LaunchConfiguration(
                    "processing_enabled_topic"
                ),
                "default_enabled": False,
            }
        ],
    )

    # Same TorchScript model and rectified model input as the W1 shortcut.
    # This node publishes direct 256x144 class masks and does not build a
    # canonical road image.  The review viewer applies the same mask-only BEV
    # warp that sequence_entry_node uses immediately before W1/W2 selection.
    lane_model = Node(
        package="lane_seg_control",
        executable="lane_seg_lraspp_inference_node",
        name="yellow_count_lane_seg_lraspp_inference",
        output="screen",
        additional_env={
            "OMP_NUM_THREADS": "4",
            "OMP_WAIT_POLICY": "PASSIVE",
            "MKL_NUM_THREADS": "4",
            "OPENBLAS_NUM_THREADS": "1",
        },
        parameters=[
            {
                "model_path": LaunchConfiguration("lane_model"),
                "image_topic": "/yellow_count/lraspp/input/compressed",
                "use_compressed_image": True,
                "enable_rectify": True,
                "direct_model_rectify_enabled": True,
                "direct_model_rectify_oversample": 3,
                "camera_yaml": LaunchConfiguration("camera_yaml"),
                "rect_balance": 0.3,
                "max_input_age_sec": 0.0,
                "processed_image_topic": "/yellow_count/lraspp/source_image",
                "white_mask_topic": "/yellow_count/lraspp/unused_white_mask",
                "yellow_mask_topic": "/yellow_count/lraspp/yellow_mask",
                "debug_topic": "/yellow_count/lraspp/debug_image",
                "perception_debug_topic": (
                    "/yellow_count/lraspp/perception_debug_image"
                ),
                "diagnostics_topic": "/yellow_count/lraspp/diagnostics",
                "input_width": 256,
                "input_height": 144,
                "white_class_id": 1,
                "yellow_class_id": 2,
                "white_confidence": 0.50,
                "yellow_confidence": 0.50,
                "cpu_threads": 4,
                "opencv_threads": 1,
                "output_qos_depth": 1,
                "debug_rate_hz": 0.0,
                "max_output_rate_hz": 15.0,
                "output_native_resolution": False,
                "publish_intermediate_topics": True,
                "direct_canonical_enabled": False,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "processing_enabled_topic",
                default_value="/yellow_count/processing_enabled",
            ),
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
                "camera_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("xycar_perception"),
                        "config",
                        "wide_camera_fisheye_1280x1024_20260708.yaml",
                    ]
                ),
            ),
            camera_gate,
            lane_model,
        ]
    )
