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
                    [
                        FindPackageShare("lane_seg_control"),
                        "models",
                        "kookmin_lane_lraspp_mbv3s_256x144.pt",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/wide_camera/rect/image_raw"
            ),
            DeclareLaunchArgument("white_confidence", default_value="0.50"),
            DeclareLaunchArgument("yellow_confidence", default_value="0.50"),
            DeclareLaunchArgument("cpu_threads", default_value="4"),
            DeclareLaunchArgument("opencv_threads", default_value="1"),
            DeclareLaunchArgument("pipeline_qos_depth", default_value="1"),
            DeclareLaunchArgument("debug_rate_hz", default_value="1.0"),
            DeclareLaunchArgument(
                "canonical_white_fit_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "canonical_white_fit_window_count", default_value="9"
            ),
            DeclareLaunchArgument(
                "canonical_white_fit_margin_px", default_value="24"
            ),
            DeclareLaunchArgument(
                "canonical_white_fit_min_pixels", default_value="4"
            ),
            DeclareLaunchArgument(
                "canonical_white_fit_min_centers", default_value="2"
            ),
            DeclareLaunchArgument(
                "canonical_white_fit_min_span_px", default_value="8"
            ),
            DeclareLaunchArgument(
                "canonical_white_fit_residual_px", default_value="6.0"
            ),
            DeclareLaunchArgument(
                "canonical_white_fit_line_width_px", default_value="5"
            ),
            DeclareLaunchArgument(
                "canonical_yellow_divider_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "canonical_yellow_divider_min_pixels", default_value="3"
            ),
            DeclareLaunchArgument(
                "canonical_yellow_divider_residual_px", default_value="6.0"
            ),
            DeclareLaunchArgument(
                "canonical_yellow_divider_line_width_px", default_value="5"
            ),
            Node(
                package="lane_seg_control",
                executable="lane_seg_lraspp_inference_node",
                name="lane_seg_lraspp_inference",
                output="screen",
                additional_env={
                    "OMP_NUM_THREADS": "4",
                    "OMP_WAIT_POLICY": "PASSIVE",
                    "MKL_NUM_THREADS": "4",
                    "OPENBLAS_NUM_THREADS": "1",
                },
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "image_topic": image_topic,
                        "processed_image_topic": processed_image_topic,
                        "white_mask_topic": white_topic,
                        "yellow_mask_topic": yellow_topic,
                        "input_width": 256,
                        "input_height": 144,
                        "white_class_id": 1,
                        "yellow_class_id": 2,
                        "white_confidence": ParameterValue(
                            LaunchConfiguration("white_confidence"), value_type=float
                        ),
                        "yellow_confidence": ParameterValue(
                            LaunchConfiguration("yellow_confidence"), value_type=float
                        ),
                        "cpu_threads": ParameterValue(
                            LaunchConfiguration("cpu_threads"), value_type=int
                        ),
                        "opencv_threads": ParameterValue(
                            LaunchConfiguration("opencv_threads"), value_type=int
                        ),
                        "output_qos_depth": ParameterValue(
                            LaunchConfiguration("pipeline_qos_depth"), value_type=int
                        ),
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
                        "src_tl_x_ratio": 472.0 / 1280.0,
                        "src_tl_y_ratio": 494.0 / 1024.0,
                        "src_tr_x_ratio": 906.0 / 1280.0,
                        "src_tr_y_ratio": 486.0 / 1024.0,
                        "src_br_x_ratio": 1272.0 / 1280.0,
                        "src_br_y_ratio": 612.0 / 1024.0,
                        "src_bl_x_ratio": 46.0 / 1280.0,
                        "src_bl_y_ratio": 622.0 / 1024.0,
                        "dst_left_ratio": 80.0 / 640.0,
                        "dst_right_ratio": 560.0 / 640.0,
                        "dst_top_y_ratio": 0.0,
                        "dst_bottom_y_ratio": 479.0 / 660.0,
                        "bev_width": 640,
                        "bev_height": 660,
                        "bev_valid_lateral_margin_px": 0,
                        "bev_valid_erode_px": 0,
                        "bev_clip_to_source_polygon": False,
                        "lateral_m_per_px": 1.4 / 640.0,
                        "forward_m_per_px": 1.5 / 660.0,
                        "canonical_lateral_range_m": 1.4,
                        "canonical_forward_range_m": 1.5,
                        "canonical_white_fit_enabled": ParameterValue(
                            LaunchConfiguration("canonical_white_fit_enabled"),
                            value_type=bool,
                        ),
                        "canonical_white_fit_window_count": ParameterValue(
                            LaunchConfiguration(
                                "canonical_white_fit_window_count"
                            ),
                            value_type=int,
                        ),
                        "canonical_white_fit_margin_px": ParameterValue(
                            LaunchConfiguration("canonical_white_fit_margin_px"),
                            value_type=int,
                        ),
                        "canonical_white_fit_min_pixels": ParameterValue(
                            LaunchConfiguration("canonical_white_fit_min_pixels"),
                            value_type=int,
                        ),
                        "canonical_white_fit_min_centers": ParameterValue(
                            LaunchConfiguration("canonical_white_fit_min_centers"),
                            value_type=int,
                        ),
                        "canonical_white_fit_min_span_px": ParameterValue(
                            LaunchConfiguration("canonical_white_fit_min_span_px"),
                            value_type=int,
                        ),
                        "canonical_white_fit_residual_px": ParameterValue(
                            LaunchConfiguration("canonical_white_fit_residual_px"),
                            value_type=float,
                        ),
                        "canonical_white_fit_line_width_px": ParameterValue(
                            LaunchConfiguration(
                                "canonical_white_fit_line_width_px"
                            ),
                            value_type=int,
                        ),
                        "canonical_yellow_divider_enabled": ParameterValue(
                            LaunchConfiguration(
                                "canonical_yellow_divider_enabled"
                            ),
                            value_type=bool,
                        ),
                        "canonical_yellow_divider_min_pixels": ParameterValue(
                            LaunchConfiguration(
                                "canonical_yellow_divider_min_pixels"
                            ),
                            value_type=int,
                        ),
                        "canonical_yellow_divider_residual_px": ParameterValue(
                            LaunchConfiguration(
                                "canonical_yellow_divider_residual_px"
                            ),
                            value_type=float,
                        ),
                        "canonical_yellow_divider_line_width_px": ParameterValue(
                            LaunchConfiguration(
                                "canonical_yellow_divider_line_width_px"
                            ),
                            value_type=int,
                        ),
                        "sync_qos_depth": ParameterValue(
                            LaunchConfiguration("pipeline_qos_depth"), value_type=int
                        ),
                        "debug_rate_hz": ParameterValue(
                            LaunchConfiguration("debug_rate_hz"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
