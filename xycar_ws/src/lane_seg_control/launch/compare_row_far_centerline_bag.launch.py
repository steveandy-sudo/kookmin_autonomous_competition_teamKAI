from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _rule_parameters():
    share = Path(get_package_share_directory("xycar_rule_drive"))
    merged = {}
    for name in (
        "canonical_stanley_pursuit.yaml",
        "canonical_stanley_pursuit_real.yaml",
    ):
        with (share / "config" / name).open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        merged.update(
            document["canonical_stanley_pursuit_driver"]["ros__parameters"]
        )
    return merged


def _perception_parameters(
    *,
    model_path,
    input_width,
    input_height,
    rectify_oversample,
    topic_prefix,
    forward_range_m,
    bev_height,
    dst_top_y_ratio,
    dst_bottom_y_ratio,
    white_confidence,
    white_fit_enabled,
):
    camera_yaml = PathJoinSubstitution(
        [
            FindPackageShare("xycar_perception"),
            "config",
            "wide_camera_fisheye_1280x1024_20260708.yaml",
        ]
    )
    return {
        "model_path": model_path,
        "image_topic": "/wide_camera_mjpeg/image_raw/compressed",
        "use_compressed_image": True,
        "enable_rectify": True,
        "direct_model_rectify_enabled": True,
        "direct_model_rectify_oversample": rectify_oversample,
        "camera_yaml": camera_yaml,
        "rect_balance": 0.3,
        "max_input_age_sec": 0.25,
        "input_width": input_width,
        "input_height": input_height,
        "white_class_id": 1,
        "yellow_class_id": 2,
        "white_confidence": white_confidence,
        "yellow_confidence": 0.50,
        "cpu_threads": 4,
        "opencv_threads": 1,
        "output_qos_depth": 1,
        "debug_rate_hz": 0.0,
        "max_output_rate_hz": 20.0,
        "output_native_resolution": False,
        "publish_intermediate_topics": True,
        "processed_image_topic": f"{topic_prefix}/source_image",
        "white_mask_topic": f"{topic_prefix}/white_mask",
        "yellow_mask_topic": f"{topic_prefix}/yellow_mask",
        "direct_canonical_enabled": True,
        "canonical_topic": f"{topic_prefix}/canonical",
        "canonical_white_topic": f"{topic_prefix}/canonical_white",
        "canonical_yellow_topic": f"{topic_prefix}/canonical_yellow",
        "canonical_valid_topic": f"{topic_prefix}/canonical_valid",
        "diagnostics_topic": f"{topic_prefix}/perception_diagnostics",
        "debug_topic": f"{topic_prefix}/model_debug",
        "perception_debug_topic": f"{topic_prefix}/perception_debug",
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
        "dst_top_y_ratio": dst_top_y_ratio,
        "dst_bottom_y_ratio": dst_bottom_y_ratio,
        "bev_width": 640,
        "bev_height": bev_height,
        "bev_clip_to_source_polygon": False,
        "lateral_m_per_px": 1.4 / 640.0,
        "forward_m_per_px": 1.5 / 660.0,
        "canonical_width": 256,
        "canonical_height": 144,
        "canonical_lateral_range_m": 1.4,
        "canonical_forward_range_m": forward_range_m,
        "canonical_white_fit_enabled": white_fit_enabled,
        "canonical_yellow_divider_enabled": True,
        "canonical_yellow_normalize_enabled": False,
    }


def _controller_parameters(*, topic_prefix, forward_range_m, speed_command):
    parameters = _rule_parameters()
    parameters.update(
        {
            "use_sim_time": False,
            "canonical_topic": f"{topic_prefix}/canonical",
            "drive_enabled": False,
            "steering_only": False,
            "motor_topic": f"{topic_prefix}/disabled_motor",
            "shadow_motor_topic": f"{topic_prefix}/rule_candidate",
            "target_path_topic": f"{topic_prefix}/connected_path",
            "diagnostics_topic": f"{topic_prefix}/rule_diagnostics",
            "debug_image_topic": f"{topic_prefix}/controller_debug",
            "debug_markers_topic": f"{topic_prefix}/debug_markers",
            "action_trace_topic": f"{topic_prefix}/action",
            "canonical_forward_range_m": forward_range_m,
            "cruise_speed_command": ParameterValue(
                speed_command, value_type=float
            ),
            "minimum_speed_command": ParameterValue(
                speed_command, value_type=float
            ),
            "target_left_offset_m": 0.0,
            "target_right_offset_m": 0.0,
        }
    )
    return parameters


def generate_launch_description():
    package_share = FindPackageShare("lane_seg_control")
    segmentation_model = PathJoinSubstitution(
        [
            package_share,
            "models",
            "kookmin_lane_lraspp_mbv3s_256x144.pt",
        ]
    )
    row_model = PathJoinSubstitution(
        [
            package_share,
            "models",
            "kookmin_yellow_row_centerline_256x144.pt",
        ]
    )
    new_model = PathJoinSubstitution(
        [
            package_share,
            "models",
            "kookmin_far_centerline_xbin_512x288.pt",
        ]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "row_far_centerline_compare.rviz"]
    )
    bag_path = LaunchConfiguration("bag_path")
    rate = LaunchConfiguration("rate")
    speed_command = LaunchConfiguration("speed_command")

    seg_prefix = "/comparison/seg"
    row_prefix = "/comparison/row"
    new_prefix = "/comparison/new"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("HOME"),
                        "rosbags",
                        "integrated_drive",
                        "rule_speed8_run01_20260724_131500_playback",
                    ]
                ),
            ),
            DeclareLaunchArgument("rate", default_value="0.5"),
            DeclareLaunchArgument("speed_command", default_value="8.0"),
            DeclareLaunchArgument("start_bag", default_value="true"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument(
                "enable_steering_graph", default_value="true"
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    bag_path,
                    "--rate",
                    rate,
                    "--loop",
                    "--topics",
                    "/wide_camera_mjpeg/image_raw/compressed",
                ],
                condition=IfCondition(LaunchConfiguration("start_bag")),
                output="screen",
            ),
            Node(
                package="lane_seg_control",
                executable="lane_seg_lraspp_inference_node",
                name="baseline_segmentation_perception",
                output="screen",
                parameters=[
                    _perception_parameters(
                        model_path=segmentation_model,
                        input_width=256,
                        input_height=144,
                        rectify_oversample=3,
                        topic_prefix=seg_prefix,
                        forward_range_m=1.5,
                        bev_height=660,
                        dst_top_y_ratio=0.0,
                        dst_bottom_y_ratio=2.0 / 3.0,
                        white_confidence=0.50,
                        white_fit_enabled=True,
                    )
                ],
            ),
            Node(
                package="lane_seg_control",
                executable="lane_seg_lraspp_inference_node",
                name="row_centerline_perception",
                output="screen",
                parameters=[
                    _perception_parameters(
                        model_path=row_model,
                        input_width=256,
                        input_height=144,
                        rectify_oversample=3,
                        topic_prefix=row_prefix,
                        forward_range_m=1.5,
                        bev_height=660,
                        dst_top_y_ratio=0.0,
                        dst_bottom_y_ratio=2.0 / 3.0,
                        white_confidence=0.99,
                        white_fit_enabled=False,
                    )
                ],
            ),
            Node(
                package="lane_seg_control",
                executable="lane_seg_lraspp_inference_node",
                name="new_far_centerline_perception",
                output="screen",
                parameters=[
                    _perception_parameters(
                        model_path=new_model,
                        input_width=512,
                        input_height=288,
                        rectify_oversample=2,
                        topic_prefix=new_prefix,
                        forward_range_m=2.5,
                        bev_height=1100,
                        dst_top_y_ratio=0.4,
                        dst_bottom_y_ratio=0.8,
                        white_confidence=0.99,
                        white_fit_enabled=False,
                    )
                ],
            ),
            Node(
                package="xycar_rule_drive",
                executable="canonical_stanley_pursuit_driver",
                name="baseline_segmentation_rule_shadow",
                output="screen",
                parameters=[
                    _controller_parameters(
                        topic_prefix=seg_prefix,
                        forward_range_m=1.5,
                        speed_command=speed_command,
                    )
                ],
            ),
            Node(
                package="xycar_rule_drive",
                executable="canonical_stanley_pursuit_driver",
                name="row_rule_shadow",
                output="screen",
                parameters=[
                    _controller_parameters(
                        topic_prefix=row_prefix,
                        forward_range_m=1.5,
                        speed_command=speed_command,
                    )
                ],
            ),
            Node(
                package="xycar_rule_drive",
                executable="canonical_stanley_pursuit_driver",
                name="new_far_rule_shadow",
                output="screen",
                parameters=[
                    _controller_parameters(
                        topic_prefix=new_prefix,
                        forward_range_m=2.5,
                        speed_command=speed_command,
                    )
                ],
            ),
            Node(
                package="lane_seg_control",
                executable="steering_compare_viewer",
                name="old_new_steering_compare",
                condition=IfCondition(
                    LaunchConfiguration("enable_steering_graph")
                ),
                output="screen",
                parameters=[
                    {
                        "base_topic": f"{seg_prefix}/rule_candidate",
                        "current_topic": f"{row_prefix}/rule_candidate",
                        "third_topic": f"{new_prefix}/rule_candidate",
                        "base_label": "SEG 1.5m",
                        "current_label": "ROW 1.5m",
                        "third_label": "NEW 2.5m",
                        "window_name": "SEG vs ROW vs NEW CENTERLINE",
                        "history_sec": 20.0,
                    }
                ],
            ),
            Node(
                package="lane_seg_control",
                executable="camera_path_compare_viewer",
                name="camera_path_compare",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="row_far_centerline_compare_rviz",
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
                output="screen",
                arguments=["-d", rviz_config],
                additional_env={
                    "LIBGL_ALWAYS_SOFTWARE": "1",
                    "QT_XCB_GL_INTEGRATION": "none",
                },
            ),
        ]
    )
