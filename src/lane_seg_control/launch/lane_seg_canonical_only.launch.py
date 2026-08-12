from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    processed_image_topic = "/lane_seg/source_image"
    white_topic = "/lane_seg/white_boundary_mask"
    yellow_topic = "/lane_seg/yellow_centerline_mask"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lane_seg_control"), "models", "best_512.onnx"]
                ),
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/wide_camera/rect/image_raw"
            ),
            DeclareLaunchArgument("confidence", default_value="0.20"),
            DeclareLaunchArgument("yellow_confidence", default_value="0.40"),
            DeclareLaunchArgument("iou", default_value="0.60"),
            DeclareLaunchArgument("image_size", default_value="512"),
            DeclareLaunchArgument("cpu_threads", default_value="4"),
            DeclareLaunchArgument("debug_rate_hz", default_value="1.0"),
            Node(
                package="lane_seg_control",
                executable="lane_seg_inference_node",
                name="lane_seg_inference",
                output="screen",
                additional_env={"OMP_NUM_THREADS": "4", "OMP_WAIT_POLICY": "PASSIVE"},
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "image_topic": image_topic,
                        "processed_image_topic": processed_image_topic,
                        "white_mask_topic": white_topic,
                        "yellow_mask_topic": yellow_topic,
                        "confidence": ParameterValue(
                            LaunchConfiguration("confidence"), value_type=float
                        ),
                        "yellow_confidence": ParameterValue(
                            LaunchConfiguration("yellow_confidence"),
                            value_type=float,
                        ),
                        "iou": ParameterValue(
                            LaunchConfiguration("iou"), value_type=float
                        ),
                        "image_size": ParameterValue(
                            LaunchConfiguration("image_size"), value_type=int
                        ),
                        "cpu_threads": ParameterValue(
                            LaunchConfiguration("cpu_threads"), value_type=int
                        ),
                        "device": "cpu",
                        "debug_rate_hz": ParameterValue(
                            LaunchConfiguration("debug_rate_hz"), value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="lane_seg_control",
                executable="lane_seg_canonical_adapter",
                name="lane_seg_canonical_adapter",
                output="screen",
                parameters=[
                    {
                        "image_topic": processed_image_topic,
                        "white_mask_topic": white_topic,
                        "yellow_mask_topic": yellow_topic,
                        "input_is_bev": False,
                        "src_tl_x_ratio": 0.442578,
                        "src_tl_y_ratio": 0.480781,
                        "src_tr_x_ratio": 0.688281,
                        "src_tr_y_ratio": 0.480781,
                        "src_br_x_ratio": 0.919141,
                        "src_br_y_ratio": 0.614189,
                        "src_bl_x_ratio": 0.190625,
                        "src_bl_y_ratio": 0.614189,
                        "dst_left_ratio": 0.205714,
                        "dst_right_ratio": 0.794286,
                        "dst_top_y_ratio": 0.0,
                        "dst_bottom_y_ratio": 0.666666667,
                        "bev_width": 640,
                        "bev_height": 660,
                        "bev_valid_lateral_margin_px": 0,
                        "bev_valid_erode_px": 0,
                        "bev_clip_to_source_polygon": False,
                        "lateral_m_per_px": 1.4 / 640.0,
                        "forward_m_per_px": 1.5 / 660.0,
                        "canonical_lateral_range_m": 1.4,
                        "canonical_forward_range_m": 1.5,
                        "debug_rate_hz": ParameterValue(
                            LaunchConfiguration("debug_rate_hz"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
