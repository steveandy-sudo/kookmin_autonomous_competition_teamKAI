"""Start lane RULE, cone, vehicle avoidance, and final arbitration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            LaunchConfiguration("lane_perception_launch"),
        ]
    )
    direct_bev_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "direct_bev_stanley_pursuit.launch.py",
        ]
    )
    rule_base = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "config",
            "canonical_stanley_pursuit.yaml",
        ]
    )
    rule_real = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "config",
            "canonical_stanley_pursuit_real.yaml",
        ]
    )
    hybrid_config = PathJoinSubstitution(
        [
            FindPackageShare("xycar_map_nav"),
            "config",
            "sequential_hybrid_real.yaml",
        ]
    )
    cone_config = PathJoinSubstitution(
        [FindPackageShare("my_rule"), "config", "cone_control.yaml"]
    )
    object_config = PathJoinSubstitution(
        [FindPackageShare("my_rule"), "config", "object_detection.yaml"]
    )
    object_model = PathJoinSubstitution(
        [
            FindPackageShare("my_rule"),
            "models",
            "kookmin_objects_best_20260811.pt",
        ]
    )
    object_camera_yaml = PathJoinSubstitution(
        [
            FindPackageShare("xycar_perception"),
            "config",
            "wide_camera_fisheye_1280x1024_20260708.yaml",
        ]
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("xycar_map_nav"),
            "rviz",
            "avoidance_test.rviz",
        ]
    )
    drive_enabled = LaunchConfiguration("drive_enabled")
    speed_command = LaunchConfiguration("speed_command")
    scan_topic = LaunchConfiguration("scan_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("steering_only", default_value="false"),
            DeclareLaunchArgument("gate_arming_required", default_value="false"),
            DeclareLaunchArgument("force_rule_only", default_value="true"),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument(
                "lane_perception_launch",
                default_value="lane_seg_far_centerline_extended_real.launch.py",
                description=(
                    "lane_seg_control perception launch; defaults to the "
                    "2.5m yellow Xbin candidate"
                ),
            ),
            DeclareLaunchArgument("start_rule", default_value="true"),
            DeclareLaunchArgument(
                "start_direct_bev_rule", default_value="false"
            ),
            DeclareLaunchArgument(
                "direct_bev_model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("lane_seg_control"), "models", "best_512.onnx"]
                ),
            ),
            DeclareLaunchArgument("direct_bev_image_size", default_value="512"),
            DeclareLaunchArgument("direct_bev_confidence", default_value="0.20"),
            DeclareLaunchArgument(
                "direct_bev_yellow_confidence", default_value="0.40"
            ),
            DeclareLaunchArgument("direct_bev_cpu_threads", default_value="4"),
            DeclareLaunchArgument(
                "direct_bev_command_rate_hz", default_value="10.0"
            ),
            DeclareLaunchArgument(
                "direct_bev_path_timeout_sec", default_value="1.50"
            ),
            DeclareLaunchArgument("start_cone", default_value="true"),
            DeclareLaunchArgument(
                "start_object_detection", default_value="true"
            ),
            DeclareLaunchArgument(
                "vehicle_avoidance_enabled", default_value="true"
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("speed_command", default_value="18.0"),
            DeclareLaunchArgument(
                "curvature_speed_control_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "curve_speed_command", default_value="16.0"
            ),
            DeclareLaunchArgument(
                "degraded_path_speed_command", default_value="12.0"
            ),
            DeclareLaunchArgument(
                "curve_speed_exit_threshold_per_m", default_value="0.12"
            ),
            DeclareLaunchArgument(
                "curve_speed_confirmation_frames", default_value="2"
            ),
            DeclareLaunchArgument(
                "curve_speed_release_frames", default_value="3"
            ),
            DeclareLaunchArgument(
                "degraded_path_minimum_span_m", default_value="0.60"
            ),
            DeclareLaunchArgument(
                "selector_minimum_speed_command", default_value="3.0"
            ),
            DeclareLaunchArgument("cone_speed_command", default_value="8.0"),
            DeclareLaunchArgument(
                "cone_sensor_presence_timeout_sec", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "lookahead_distance_m", default_value="0.3"
            ),
            DeclareLaunchArgument(
                "pure_pursuit_weight", default_value="0.8"
            ),
            DeclareLaunchArgument(
                "pure_pursuit_control_x_m", default_value="-0.08"
            ),
            DeclareLaunchArgument(
                "stanley_control_x_m", default_value="0.16"
            ),
            DeclareLaunchArgument("stanley_gain", default_value="1.20"),
            DeclareLaunchArgument(
                "stanley_softening_mps", default_value="0.35"
            ),
            DeclareLaunchArgument(
                "straight_pure_pursuit_weight", default_value="0.10"
            ),
            DeclareLaunchArgument(
                "straight_stanley_gain", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "straight_stanley_softening_mps", default_value="0.65"
            ),
            DeclareLaunchArgument(
                "opposed_stanley_weight", default_value="0.70"
            ),
            DeclareLaunchArgument(
                "control_latency_preview_sec", default_value="0.35"
            ),
            DeclareLaunchArgument(
                "curve_detection_near_x_m", default_value="0.20"
            ),
            DeclareLaunchArgument(
                "curve_detection_far_x_m", default_value="1.20"
            ),
            DeclareLaunchArgument(
                "curve_detection_segment_count", default_value="1"
            ),
            DeclareLaunchArgument(
                "adaptive_curve_lookahead_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "adaptive_curve_lookahead_m", default_value="0.30"
            ),
            DeclareLaunchArgument(
                "adaptive_curve_minimum_path_reach_m", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "adaptive_curve_confirmation_frames", default_value="2"
            ),
            DeclareLaunchArgument(
                "adaptive_curve_release_frames", default_value="2"
            ),
            DeclareLaunchArgument(
                "straight_path_curvature_threshold", default_value="0.16"
            ),
            DeclareLaunchArgument(
                "steering_current_weight", default_value="0.40"
            ),
            DeclareLaunchArgument(
                "steering_curve_current_weight", default_value="0.70"
            ),
            DeclareLaunchArgument(
                "steering_rate_limit_cmd_per_sec", default_value="180.0"
            ),
            DeclareLaunchArgument(
                "steering_curve_rate_limit_cmd_per_sec", default_value="300.0"
            ),
            DeclareLaunchArgument(
                "steering_lead_time_sec", default_value="0.08"
            ),
            DeclareLaunchArgument(
                "steering_max_lead_command", default_value="6.0"
            ),
            DeclareLaunchArgument(
                "curve_steering_multiplier_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "curve_steering_multiplier_activation_command",
                default_value="20.0",
            ),
            DeclareLaunchArgument(
                "curve_steering_multiplier", default_value="1.5"
            ),
            DeclareLaunchArgument(
                "target_left_offset_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "straight_target_right_offset_m", default_value="0.05"
            ),
            DeclareLaunchArgument(
                "maximum_speed_command", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "perception_max_output_rate_hz", default_value="15.0"
            ),
            DeclareLaunchArgument(
                "canonical_forward_range_m", default_value="2.5"
            ),
            DeclareLaunchArgument(
                "direct_model_rectify_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "direct_model_rectify_oversample", default_value="3"
            ),
            DeclareLaunchArgument(
                "vehicle_yolo_min_confidence", default_value="0.45"
            ),
            DeclareLaunchArgument(
                "vehicle_side_decision_straight_only", default_value="true"
            ),
            DeclareLaunchArgument(
                "vehicle_side_decision_max_rule_angle_command",
                default_value="8.0",
            ),
            DeclareLaunchArgument(
                "vehicle_side_decision_rule_timeout_sec",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "yellow_straight_max_rmse_px", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "yellow_straight_min_span_ratio", default_value="0.20"
            ),
            DeclareLaunchArgument(
                "cone_as_vehicle_obstacle", default_value="false"
            ),
            DeclareLaunchArgument(
                "cone_as_vehicle_min_confidence", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "vehicle_yolo_required_frames", default_value="1"
            ),
            DeclareLaunchArgument(
                "vehicle_yolo_timeout_sec", default_value="1.00"
            ),
            DeclareLaunchArgument(
                "vehicle_camera_lidar_hfov_deg", default_value="60.0"
            ),
            DeclareLaunchArgument(
                "vehicle_camera_lidar_padding_deg", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "vehicle_preferred_side_required_frames", default_value="1"
            ),
            DeclareLaunchArgument(
                "vehicle_lidar_min_points", default_value="2"
            ),
            DeclareLaunchArgument(
                "vehicle_lidar_sector_memory_sec", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "vehicle_lidar_association_angle_margin_deg",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "vehicle_lidar_association_distance_tolerance_m",
                default_value="0.35",
            ),
            DeclareLaunchArgument(
                "vehicle_avoidance_immediate_on_yolo", default_value="true"
            ),
            DeclareLaunchArgument(
                "vehicle_avoidance_entry_distance_m", default_value="1.20"
            ),
            DeclareLaunchArgument(
                "vehicle_minimum_side_clearance_m", default_value="0.70"
            ),
            DeclareLaunchArgument("vehicle_left_offset_m", default_value="0.20"),
            DeclareLaunchArgument("vehicle_right_offset_m", default_value="0.20"),
            DeclareLaunchArgument(
                "vehicle_offset_rate_mps", default_value="0.50"
            ),
            DeclareLaunchArgument(
                "vehicle_avoidance_speed_limit_command", default_value="8.0"
            ),
            DeclareLaunchArgument(
                "vehicle_minimum_avoid_sec", default_value="0.50"
            ),
            DeclareLaunchArgument("vehicle_clear_hold_sec", default_value="0.50"),
            DeclareLaunchArgument("vehicle_return_hold_sec", default_value="0.30"),
            DeclareLaunchArgument(
                "vehicle_return_deadband_m", default_value="0.02"
            ),
            DeclareLaunchArgument("vehicle_body_length_m", default_value="0.55"),
            DeclareLaunchArgument("vehicle_body_width_m", default_value="0.28"),
            DeclareLaunchArgument(
                "lidar_obstacle_detect_distance_m", default_value="1.50"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_minimum_distance_m", default_value="0.18"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_path_half_width_m", default_value="0.18"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_minimum_cluster_points", default_value="3"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_maximum_scan_index_gap", default_value="2"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_maximum_cluster_gap_m", default_value="0.16"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_minimum_cluster_width_m", default_value="0.09"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_maximum_cluster_width_m", default_value="0.70"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_side_probe_inner_m", default_value="0.18"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_side_probe_outer_m", default_value="0.55"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_lidar_x_m", default_value="0.065"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_lidar_y_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "lidar_obstacle_lidar_yaw_deg", default_value="0.0"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch),
                condition=IfCondition(LaunchConfiguration("start_perception")),
                launch_arguments={
                    "max_output_rate_hz": LaunchConfiguration(
                        "perception_max_output_rate_hz"
                    ),
                    "direct_model_rectify_enabled": LaunchConfiguration(
                        "direct_model_rectify_enabled"
                    ),
                    "direct_model_rectify_oversample": LaunchConfiguration(
                        "direct_model_rectify_oversample"
                    ),
                    "debug_rate_hz": "0.0",
                    "publish_intermediate_topics": "true",
                    "canonical_forward_range_m": LaunchConfiguration(
                        "canonical_forward_range_m"
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(direct_bev_launch),
                condition=IfCondition(
                    LaunchConfiguration("start_direct_bev_rule")
                ),
                launch_arguments={
                    "start_camera": "false",
                    "start_vesc": "false",
                    "model_path": LaunchConfiguration(
                        "direct_bev_model_path"
                    ),
                    "image_size": LaunchConfiguration(
                        "direct_bev_image_size"
                    ),
                    "confidence": LaunchConfiguration(
                        "direct_bev_confidence"
                    ),
                    "yellow_confidence": LaunchConfiguration(
                        "direct_bev_yellow_confidence"
                    ),
                    "cpu_threads": LaunchConfiguration(
                        "direct_bev_cpu_threads"
                    ),
                    "display_mode": "off",
                    "publish_debug_images": "true",
                    "drive_enabled": "false",
                    "steering_only": LaunchConfiguration("steering_only"),
                    "shadow_motor_topic": "/hybrid/rule_candidate",
                    "external_lateral_offset_enabled": "true",
                    "external_lateral_offset_topic": (
                        "/hybrid/avoidance_lateral_offset"
                    ),
                    "cruise_speed_command": speed_command,
                    "minimum_speed_command": LaunchConfiguration(
                        "curve_speed_command"
                    ),
                    "curvature_speed_control_enabled": LaunchConfiguration(
                        "curvature_speed_control_enabled"
                    ),
                    "curve_speed_command": LaunchConfiguration(
                        "curve_speed_command"
                    ),
                    "degraded_path_speed_command": LaunchConfiguration(
                        "degraded_path_speed_command"
                    ),
                    "curve_speed_exit_threshold_per_m": LaunchConfiguration(
                        "curve_speed_exit_threshold_per_m"
                    ),
                    "curve_speed_confirmation_frames": LaunchConfiguration(
                        "curve_speed_confirmation_frames"
                    ),
                    "curve_speed_release_frames": LaunchConfiguration(
                        "curve_speed_release_frames"
                    ),
                    "degraded_path_minimum_span_m": LaunchConfiguration(
                        "degraded_path_minimum_span_m"
                    ),
                    "command_rate_hz": LaunchConfiguration(
                        "direct_bev_command_rate_hz"
                    ),
                    "external_path_timeout_sec": LaunchConfiguration(
                        "direct_bev_path_timeout_sec"
                    ),
                    "lookahead_distance_m": LaunchConfiguration(
                        "lookahead_distance_m"
                    ),
                    "pure_pursuit_weight": LaunchConfiguration(
                        "pure_pursuit_weight"
                    ),
                    "pure_pursuit_control_x_m": LaunchConfiguration(
                        "pure_pursuit_control_x_m"
                    ),
                    "stanley_control_x_m": LaunchConfiguration(
                        "stanley_control_x_m"
                    ),
                    "stanley_gain": LaunchConfiguration("stanley_gain"),
                    "stanley_softening_mps": LaunchConfiguration(
                        "stanley_softening_mps"
                    ),
                    "target_left_offset_m": LaunchConfiguration(
                        "target_left_offset_m"
                    ),
                    "straight_target_right_offset_m": LaunchConfiguration(
                        "straight_target_right_offset_m"
                    ),
                    "straight_path_curvature_threshold": LaunchConfiguration(
                        "straight_path_curvature_threshold"
                    ),
                    "curve_detection_near_x_m": LaunchConfiguration(
                        "curve_detection_near_x_m"
                    ),
                    "curve_detection_far_x_m": LaunchConfiguration(
                        "curve_detection_far_x_m"
                    ),
                    "curve_detection_segment_count": LaunchConfiguration(
                        "curve_detection_segment_count"
                    ),
                    "straight_pure_pursuit_weight": LaunchConfiguration(
                        "straight_pure_pursuit_weight"
                    ),
                    "straight_stanley_gain": LaunchConfiguration(
                        "straight_stanley_gain"
                    ),
                    "straight_stanley_softening_mps": LaunchConfiguration(
                        "straight_stanley_softening_mps"
                    ),
                    "opposed_stanley_weight": LaunchConfiguration(
                        "opposed_stanley_weight"
                    ),
                    "control_latency_preview_sec": LaunchConfiguration(
                        "control_latency_preview_sec"
                    ),
                    "steering_current_weight": LaunchConfiguration(
                        "steering_current_weight"
                    ),
                    "steering_curve_current_weight": LaunchConfiguration(
                        "steering_curve_current_weight"
                    ),
                    "steering_rate_limit_cmd_per_sec": LaunchConfiguration(
                        "steering_rate_limit_cmd_per_sec"
                    ),
                    "steering_curve_rate_limit_cmd_per_sec": LaunchConfiguration(
                        "steering_curve_rate_limit_cmd_per_sec"
                    ),
                    "curve_steering_multiplier_enabled": LaunchConfiguration(
                        "curve_steering_multiplier_enabled"
                    ),
                    "curve_steering_multiplier_activation_command": LaunchConfiguration(
                        "curve_steering_multiplier_activation_command"
                    ),
                    "curve_steering_multiplier": LaunchConfiguration(
                        "curve_steering_multiplier"
                    ),
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_avoidance_test",
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
                output="screen",
                arguments=["-d", rviz_config],
                additional_env={
                    "LIBGL_ALWAYS_SOFTWARE": "1",
                    "QT_XCB_GL_INTEGRATION": "none",
                },
            ),
            Node(
                package="xycar_rule_drive",
                executable="canonical_stanley_pursuit_driver",
                name="canonical_stanley_pursuit_driver",
                condition=IfCondition(LaunchConfiguration("start_rule")),
                output="screen",
                parameters=[
                    rule_base,
                    rule_real,
                    {
                        "use_sim_time": False,
                        "drive_enabled": False,
                        "steering_only": ParameterValue(
                            LaunchConfiguration("steering_only"),
                            value_type=bool,
                        ),
                        "base_frame_id": "laser_frame",
                        "shadow_motor_topic": "/hybrid/rule_candidate",
                        "cruise_speed_command": ParameterValue(
                            speed_command, value_type=float
                        ),
                        "minimum_speed_command": ParameterValue(
                            LaunchConfiguration("curve_speed_command"),
                            value_type=float,
                        ),
                        "curvature_speed_control_enabled": ParameterValue(
                            LaunchConfiguration(
                                "curvature_speed_control_enabled"
                            ),
                            value_type=bool,
                        ),
                        "curve_speed_command": ParameterValue(
                            LaunchConfiguration("curve_speed_command"),
                            value_type=float,
                        ),
                        "degraded_path_speed_command": ParameterValue(
                            LaunchConfiguration(
                                "degraded_path_speed_command"
                            ),
                            value_type=float,
                        ),
                        "curve_speed_exit_threshold_per_m": ParameterValue(
                            LaunchConfiguration(
                                "curve_speed_exit_threshold_per_m"
                            ),
                            value_type=float,
                        ),
                        "curve_speed_confirmation_frames": ParameterValue(
                            LaunchConfiguration(
                                "curve_speed_confirmation_frames"
                            ),
                            value_type=int,
                        ),
                        "curve_speed_release_frames": ParameterValue(
                            LaunchConfiguration(
                                "curve_speed_release_frames"
                            ),
                            value_type=int,
                        ),
                        "degraded_path_minimum_span_m": ParameterValue(
                            LaunchConfiguration(
                                "degraded_path_minimum_span_m"
                            ),
                            value_type=float,
                        ),
                        "canonical_forward_range_m": ParameterValue(
                            LaunchConfiguration(
                                "canonical_forward_range_m"
                            ),
                            value_type=float,
                        ),
                        "lookahead_distance_m": ParameterValue(
                            LaunchConfiguration("lookahead_distance_m"),
                            value_type=float,
                        ),
                        "pure_pursuit_weight": ParameterValue(
                            LaunchConfiguration("pure_pursuit_weight"),
                            value_type=float,
                        ),
                        "pure_pursuit_control_x_m": ParameterValue(
                            LaunchConfiguration("pure_pursuit_control_x_m"),
                            value_type=float,
                        ),
                        "stanley_control_x_m": ParameterValue(
                            LaunchConfiguration("stanley_control_x_m"),
                            value_type=float,
                        ),
                        "stanley_gain": ParameterValue(
                            LaunchConfiguration("stanley_gain"),
                            value_type=float,
                        ),
                        "stanley_softening_mps": ParameterValue(
                            LaunchConfiguration("stanley_softening_mps"),
                            value_type=float,
                        ),
                        "straight_pure_pursuit_weight": ParameterValue(
                            LaunchConfiguration(
                                "straight_pure_pursuit_weight"
                            ),
                            value_type=float,
                        ),
                        "straight_stanley_gain": ParameterValue(
                            LaunchConfiguration("straight_stanley_gain"),
                            value_type=float,
                        ),
                        "straight_stanley_softening_mps": ParameterValue(
                            LaunchConfiguration(
                                "straight_stanley_softening_mps"
                            ),
                            value_type=float,
                        ),
                        "opposed_stanley_weight": ParameterValue(
                            LaunchConfiguration("opposed_stanley_weight"),
                            value_type=float,
                        ),
                        "control_latency_preview_sec": ParameterValue(
                            LaunchConfiguration(
                                "control_latency_preview_sec"
                            ),
                            value_type=float,
                        ),
                        "curve_detection_near_x_m": ParameterValue(
                            LaunchConfiguration("curve_detection_near_x_m"),
                            value_type=float,
                        ),
                        "curve_detection_far_x_m": ParameterValue(
                            LaunchConfiguration("curve_detection_far_x_m"),
                            value_type=float,
                        ),
                        "curve_detection_segment_count": ParameterValue(
                            LaunchConfiguration(
                                "curve_detection_segment_count"
                            ),
                            value_type=int,
                        ),
                        "adaptive_curve_lookahead_enabled": ParameterValue(
                            LaunchConfiguration(
                                "adaptive_curve_lookahead_enabled"
                            ),
                            value_type=bool,
                        ),
                        "adaptive_curve_lookahead_m": ParameterValue(
                            LaunchConfiguration(
                                "adaptive_curve_lookahead_m"
                            ),
                            value_type=float,
                        ),
                        "adaptive_curve_minimum_path_reach_m": ParameterValue(
                            LaunchConfiguration(
                                "adaptive_curve_minimum_path_reach_m"
                            ),
                            value_type=float,
                        ),
                        "adaptive_curve_confirmation_frames": ParameterValue(
                            LaunchConfiguration(
                                "adaptive_curve_confirmation_frames"
                            ),
                            value_type=int,
                        ),
                        "adaptive_curve_release_frames": ParameterValue(
                            LaunchConfiguration(
                                "adaptive_curve_release_frames"
                            ),
                            value_type=int,
                        ),
                        "straight_path_curvature_threshold": ParameterValue(
                            LaunchConfiguration(
                                "straight_path_curvature_threshold"
                            ),
                            value_type=float,
                        ),
                        "steering_current_weight": ParameterValue(
                            LaunchConfiguration("steering_current_weight"),
                            value_type=float,
                        ),
                        "steering_curve_current_weight": ParameterValue(
                            LaunchConfiguration(
                                "steering_curve_current_weight"
                            ),
                            value_type=float,
                        ),
                        "steering_rate_limit_cmd_per_sec": ParameterValue(
                            LaunchConfiguration(
                                "steering_rate_limit_cmd_per_sec"
                            ),
                            value_type=float,
                        ),
                        "steering_curve_rate_limit_cmd_per_sec": ParameterValue(
                            LaunchConfiguration(
                                "steering_curve_rate_limit_cmd_per_sec"
                            ),
                            value_type=float,
                        ),
                        "steering_lead_time_sec": ParameterValue(
                            LaunchConfiguration("steering_lead_time_sec"),
                            value_type=float,
                        ),
                        "steering_max_lead_command": ParameterValue(
                            LaunchConfiguration("steering_max_lead_command"),
                            value_type=float,
                        ),
                        "curve_steering_multiplier_enabled": ParameterValue(
                            LaunchConfiguration(
                                "curve_steering_multiplier_enabled"
                            ),
                            value_type=bool,
                        ),
                        "curve_steering_multiplier_activation_command": ParameterValue(
                            LaunchConfiguration(
                                "curve_steering_multiplier_activation_command"
                            ),
                            value_type=float,
                        ),
                        "curve_steering_multiplier": ParameterValue(
                            LaunchConfiguration(
                                "curve_steering_multiplier"
                            ),
                            value_type=float,
                        ),
                        # Keep the validated static calibration. Vehicle
                        # avoidance now shapes the full target path with the
                        # gazebo_sitl entry/hold/return planner.
                        "target_right_offset_m": 0.0,
                        "target_left_offset_m": ParameterValue(
                            LaunchConfiguration("target_left_offset_m"),
                            value_type=float,
                        ),
                        "straight_target_right_offset_m": ParameterValue(
                            LaunchConfiguration(
                                "straight_target_right_offset_m"
                            ),
                            value_type=float,
                        ),
                        "external_lateral_offset_enabled": True,
                        "external_lateral_offset_topic": (
                            "/hybrid/avoidance_lateral_offset"
                        ),
                        "sitl_bypass_path_enabled": False,
                        "sitl_bypass_path_request_topic": (
                            "/hybrid/avoidance_path_request"
                        ),
                        "vehicle_body_length_m": ParameterValue(
                            LaunchConfiguration("vehicle_body_length_m"),
                            value_type=float,
                        ),
                        "vehicle_body_width_m": ParameterValue(
                            LaunchConfiguration("vehicle_body_width_m"),
                            value_type=float,
                        ),
                        "lane_center_separation_m": 0.40,
                    },
                ],
            ),
            Node(
                package="my_rule",
                executable="object_detection_node",
                name="my_rule_object_detection_node",
                condition=IfCondition(
                    LaunchConfiguration("start_object_detection")
                ),
                output="screen",
                additional_env={
                    "OMP_NUM_THREADS": "2",
                    "OMP_WAIT_POLICY": "PASSIVE",
                    "MKL_NUM_THREADS": "2",
                    "OPENBLAS_NUM_THREADS": "1",
                    "MPLCONFIGDIR": "/tmp/my_rule_matplotlib",
                    "YOLO_CONFIG_DIR": "/tmp/my_rule_ultralytics",
                },
                parameters=[
                    object_config,
                    {
                        "model_path": object_model,
                        "camera_yaml": object_camera_yaml,
                        "startup_signal_hsv_enabled": False,
                        "inference_rate_hz": 3.0,
                        "cone_confidence": ParameterValue(
                            LaunchConfiguration(
                                "cone_as_vehicle_min_confidence"
                            ),
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package="my_rule",
                executable="cone_node",
                name="my_rule_cone_node",
                condition=IfCondition(LaunchConfiguration("start_cone")),
                output="screen",
                parameters=[
                    cone_config,
                    {
                        "processing_gate_enabled": True,
                        "cone_speed": ParameterValue(
                            LaunchConfiguration("cone_speed_command"),
                            value_type=float,
                        ),
                        "cone_min_drive_speed": ParameterValue(
                            LaunchConfiguration("cone_speed_command"),
                            value_type=float,
                        ),
                        "cone_straight_boost_speed": ParameterValue(
                            LaunchConfiguration("cone_speed_command"),
                            value_type=float,
                        ),
                        "single_boundary_max_speed": ParameterValue(
                            LaunchConfiguration("cone_speed_command"),
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package="xycar_map_nav",
                executable="sequential_hybrid_driver",
                name="sequential_hybrid_driver",
                output="screen",
                parameters=[
                    hybrid_config,
                    {
                        "drive_enabled": ParameterValue(
                            drive_enabled, value_type=bool
                        ),
                        "gate_arming_required": ParameterValue(
                            LaunchConfiguration("gate_arming_required"),
                            value_type=bool,
                        ),
                        "force_rule_only": ParameterValue(
                            LaunchConfiguration("force_rule_only"),
                            value_type=bool,
                        ),
                        "scan_topic": scan_topic,
                        "minimum_speed_command": ParameterValue(
                            LaunchConfiguration(
                                "selector_minimum_speed_command"
                            ),
                            value_type=float,
                        ),
                        "maximum_speed_command": ParameterValue(
                            LaunchConfiguration("maximum_speed_command"),
                            value_type=float,
                        ),
                        "cone_sensor_presence_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "cone_sensor_presence_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "vehicle_avoidance_enabled": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_avoidance_enabled"
                            ),
                            value_type=bool,
                        ),
                        "vehicle_yolo_min_confidence": ParameterValue(
                            LaunchConfiguration("vehicle_yolo_min_confidence"),
                            value_type=float,
                        ),
                        "cone_as_vehicle_obstacle": ParameterValue(
                            LaunchConfiguration("cone_as_vehicle_obstacle"),
                            value_type=bool,
                        ),
                        "cone_as_vehicle_min_confidence": ParameterValue(
                            LaunchConfiguration(
                                "cone_as_vehicle_min_confidence"
                            ),
                            value_type=float,
                        ),
                        "vehicle_yolo_required_frames": ParameterValue(
                            LaunchConfiguration("vehicle_yolo_required_frames"),
                            value_type=int,
                        ),
                        "vehicle_yolo_timeout_sec": ParameterValue(
                            LaunchConfiguration("vehicle_yolo_timeout_sec"),
                            value_type=float,
                        ),
                        "vehicle_camera_lidar_hfov_deg": ParameterValue(
                            LaunchConfiguration("vehicle_camera_lidar_hfov_deg"),
                            value_type=float,
                        ),
                        "vehicle_camera_lidar_padding_deg": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_camera_lidar_padding_deg"
                            ),
                            value_type=float,
                        ),
                        "vehicle_preferred_side_required_frames": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_preferred_side_required_frames"
                            ),
                            value_type=int,
                        ),
                        "vehicle_side_decision_straight_only": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_side_decision_straight_only"
                            ),
                            value_type=bool,
                        ),
                        "vehicle_side_decision_max_rule_angle_command": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_side_decision_max_rule_angle_command"
                            ),
                            value_type=float,
                        ),
                        "vehicle_side_decision_rule_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_side_decision_rule_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "yellow_straight_max_rmse_px": ParameterValue(
                            LaunchConfiguration(
                                "yellow_straight_max_rmse_px"
                            ),
                            value_type=float,
                        ),
                        "yellow_straight_min_span_ratio": ParameterValue(
                            LaunchConfiguration(
                                "yellow_straight_min_span_ratio"
                            ),
                            value_type=float,
                        ),
                        "vehicle_lidar_min_points": ParameterValue(
                            LaunchConfiguration("vehicle_lidar_min_points"),
                            value_type=int,
                        ),
                        "vehicle_lidar_sector_memory_sec": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_lidar_sector_memory_sec"
                            ),
                            value_type=float,
                        ),
                        "vehicle_lidar_association_angle_margin_deg": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_lidar_association_angle_margin_deg"
                            ),
                            value_type=float,
                        ),
                        "vehicle_lidar_association_distance_tolerance_m": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_lidar_association_distance_tolerance_m"
                            ),
                            value_type=float,
                        ),
                        "vehicle_avoidance_immediate_on_yolo": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_avoidance_immediate_on_yolo"
                            ),
                            value_type=bool,
                        ),
                        "vehicle_avoidance_entry_distance_m": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_avoidance_entry_distance_m"
                            ),
                            value_type=float,
                        ),
                        "vehicle_minimum_side_clearance_m": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_minimum_side_clearance_m"
                            ),
                            value_type=float,
                        ),
                        "vehicle_left_offset_m": ParameterValue(
                            LaunchConfiguration("vehicle_left_offset_m"),
                            value_type=float,
                        ),
                        "vehicle_right_offset_m": ParameterValue(
                            LaunchConfiguration("vehicle_right_offset_m"),
                            value_type=float,
                        ),
                        "vehicle_offset_rate_mps": ParameterValue(
                            LaunchConfiguration("vehicle_offset_rate_mps"),
                            value_type=float,
                        ),
                        "vehicle_avoidance_speed_limit_command": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_avoidance_speed_limit_command"
                            ),
                            value_type=float,
                        ),
                        "vehicle_minimum_avoid_sec": ParameterValue(
                            LaunchConfiguration("vehicle_minimum_avoid_sec"),
                            value_type=float,
                        ),
                        "vehicle_clear_hold_sec": ParameterValue(
                            LaunchConfiguration("vehicle_clear_hold_sec"),
                            value_type=float,
                        ),
                        "vehicle_return_hold_sec": ParameterValue(
                            LaunchConfiguration("vehicle_return_hold_sec"),
                            value_type=float,
                        ),
                        "vehicle_return_deadband_m": ParameterValue(
                            LaunchConfiguration("vehicle_return_deadband_m"),
                            value_type=float,
                        ),
                        "vehicle_body_length_m": ParameterValue(
                            LaunchConfiguration("vehicle_body_length_m"),
                            value_type=float,
                        ),
                        "vehicle_body_width_m": ParameterValue(
                            LaunchConfiguration("vehicle_body_width_m"),
                            value_type=float,
                        ),
                        "lidar_obstacle_detect_distance_m": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_detect_distance_m"
                            ),
                            value_type=float,
                        ),
                        "lidar_obstacle_minimum_distance_m": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_minimum_distance_m"
                            ),
                            value_type=float,
                        ),
                        "lidar_obstacle_path_half_width_m": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_path_half_width_m"
                            ),
                            value_type=float,
                        ),
                        "lidar_obstacle_minimum_cluster_points": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_minimum_cluster_points"
                            ),
                            value_type=int,
                        ),
                        "lidar_obstacle_maximum_scan_index_gap": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_maximum_scan_index_gap"
                            ),
                            value_type=int,
                        ),
                        "lidar_obstacle_maximum_cluster_gap_m": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_maximum_cluster_gap_m"
                            ),
                            value_type=float,
                        ),
                        "lidar_obstacle_minimum_cluster_width_m": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_minimum_cluster_width_m"
                            ),
                            value_type=float,
                        ),
                        "lidar_obstacle_maximum_cluster_width_m": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_maximum_cluster_width_m"
                            ),
                            value_type=float,
                        ),
                        "lidar_obstacle_side_probe_inner_m": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_side_probe_inner_m"
                            ),
                            value_type=float,
                        ),
                        "lidar_obstacle_side_probe_outer_m": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_side_probe_outer_m"
                            ),
                            value_type=float,
                        ),
                        "lidar_obstacle_lidar_x_m": ParameterValue(
                            LaunchConfiguration("lidar_obstacle_lidar_x_m"),
                            value_type=float,
                        ),
                        "lidar_obstacle_lidar_y_m": ParameterValue(
                            LaunchConfiguration("lidar_obstacle_lidar_y_m"),
                            value_type=float,
                        ),
                        "lidar_obstacle_lidar_yaw_deg": ParameterValue(
                            LaunchConfiguration(
                                "lidar_obstacle_lidar_yaw_deg"
                            ),
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
