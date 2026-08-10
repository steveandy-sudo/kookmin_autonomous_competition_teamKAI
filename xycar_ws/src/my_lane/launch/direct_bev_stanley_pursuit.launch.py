from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def bool_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def float_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def int_param(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def generate_launch_description():
    lane_share = FindPackageShare("my_lane")
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_vesc", default_value="true"),
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyMOTOR"),
            DeclareLaunchArgument(
                "acceleration_slew_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "steering_center_trim_command", default_value="-5.0"
            ),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument(
                "camera_device",
                default_value=(
                    "/dev/v4l/by-id/"
                    "usb-HD_USB_Camera_HD_USB_Camera-video-index0"
                ),
            ),
            DeclareLaunchArgument("camera_width", default_value="1280"),
            DeclareLaunchArgument("camera_height", default_value="1024"),
            DeclareLaunchArgument("camera_fps", default_value="30"),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [lane_share, "models", "best_512.onnx"]
                ),
            ),
            DeclareLaunchArgument(
                "compressed_image_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "calib_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("my_road"),
                        "config",
                        "wide_camera_fisheye_1280x1024_20260708.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("rect_balance", default_value="0.30"),
            DeclareLaunchArgument("image_size", default_value="512"),
            DeclareLaunchArgument("confidence", default_value="0.20"),
            DeclareLaunchArgument("yellow_confidence", default_value="0.40"),
            DeclareLaunchArgument("cpu_threads", default_value="4"),
            DeclareLaunchArgument("display_mode", default_value="off"),
            DeclareLaunchArgument("publish_debug_images", default_value="false"),
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("steering_only", default_value="true"),
            DeclareLaunchArgument(
                "shadow_motor_topic", default_value="/xycar_motor_shadow"
            ),
            DeclareLaunchArgument(
                "external_lateral_offset_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "external_lateral_offset_topic",
                default_value="/hybrid/avoidance_lateral_offset",
            ),
            DeclareLaunchArgument("cruise_speed_command", default_value="3.0"),
            DeclareLaunchArgument("minimum_speed_command", default_value="3.0"),
            DeclareLaunchArgument(
                "curvature_speed_control_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "curve_speed_command", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "degraded_path_speed_command", default_value="3.0"
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
            DeclareLaunchArgument("command_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("lookahead_distance_m", default_value="0.30"),
            DeclareLaunchArgument("pure_pursuit_weight", default_value="0.80"),
            DeclareLaunchArgument("stanley_gain", default_value="1.20"),
            DeclareLaunchArgument("stanley_softening_mps", default_value="0.35"),
            DeclareLaunchArgument("target_left_offset_m", default_value="0.0"),
            DeclareLaunchArgument(
                "straight_target_right_offset_m", default_value="0.0"
            ),
            DeclareLaunchArgument("angle_command_min", default_value="-42.0"),
            DeclareLaunchArgument("angle_command_max", default_value="42.0"),
            DeclareLaunchArgument("pure_pursuit_control_x_m", default_value="-0.08"),
            DeclareLaunchArgument("stanley_control_x_m", default_value="0.16"),
            DeclareLaunchArgument("control_latency_preview_sec", default_value="0.10"),
            DeclareLaunchArgument("curve_slowdown_angle_command", default_value="24.0"),
            DeclareLaunchArgument("straight_stanley_enabled", default_value="true"),
            DeclareLaunchArgument(
                "straight_path_curvature_threshold", default_value="0.16"
            ),
            DeclareLaunchArgument("curve_detection_near_x_m", default_value="0.16"),
            DeclareLaunchArgument("curve_detection_far_x_m", default_value="0.70"),
            DeclareLaunchArgument("curve_detection_segment_count", default_value="3"),
            DeclareLaunchArgument("straight_pure_pursuit_weight", default_value="0.10"),
            DeclareLaunchArgument("straight_stanley_gain", default_value="0.50"),
            DeclareLaunchArgument("straight_stanley_softening_mps", default_value="0.65"),
            DeclareLaunchArgument("opposed_stanley_weight", default_value="0.70"),
            DeclareLaunchArgument("steering_current_weight", default_value="0.40"),
            DeclareLaunchArgument("steering_curve_current_weight", default_value="0.70"),
            DeclareLaunchArgument("steering_rate_limit_cmd_per_sec", default_value="180.0"),
            DeclareLaunchArgument(
                "steering_curve_rate_limit_cmd_per_sec", default_value="300.0"
            ),
            DeclareLaunchArgument("curve_steering_multiplier_enabled", default_value="false"),
            DeclareLaunchArgument(
                "curve_steering_multiplier_activation_command", default_value="20.0"
            ),
            DeclareLaunchArgument("curve_steering_multiplier", default_value="1.0"),
            DeclareLaunchArgument("external_path_timeout_sec", default_value="1.50"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("xycar_vesc_driver"),
                            "launch",
                            "xycar_vesc_driver.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_vesc")),
                launch_arguments={
                    "port": LaunchConfiguration("vesc_port"),
                    "drive_enabled": LaunchConfiguration("drive_enabled"),
                    "acceleration_slew_enabled": LaunchConfiguration(
                        "acceleration_slew_enabled"
                    ),
                    "steering_center_trim_command": LaunchConfiguration(
                        "steering_center_trim_command"
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("xycar_camera"),
                            "launch",
                            "xycar_camera.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("start_camera")),
                launch_arguments={
                    "device": LaunchConfiguration("camera_device"),
                    "width": LaunchConfiguration("camera_width"),
                    "height": LaunchConfiguration("camera_height"),
                    "fps": LaunchConfiguration("camera_fps"),
                    "topic": LaunchConfiguration("compressed_image_topic"),
                }.items(),
            ),
            Node(
                package="my_lane",
                executable="compressed_rectifier",
                name="lane_seg_compressed_rectifier",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration(
                            "compressed_image_topic"
                        ),
                        "output_topic": "/lane_seg/rectified_image",
                        "calib_yaml": LaunchConfiguration("calib_yaml"),
                        "balance": float_param("rect_balance"),
                    }
                ],
            ),
            Node(
                package="my_lane",
                executable="lane_seg_inference_node",
                name="lane_seg_inference",
                output="screen",
                additional_env={
                    "OMP_NUM_THREADS": "4",
                    "OMP_WAIT_POLICY": "PASSIVE",
                },
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "image_topic": "/lane_seg/rectified_image",
                        "processed_image_topic": "/lane_seg/source_image",
                        "white_mask_topic": "/lane_seg/white_boundary_mask",
                        "yellow_mask_topic": "/lane_seg/yellow_centerline_mask",
                        "confidence": float_param("confidence"),
                        "yellow_confidence": float_param("yellow_confidence"),
                        "image_size": int_param("image_size"),
                        "cpu_threads": int_param("cpu_threads"),
                        "device": "cpu",
                        "debug_rate_hz": 2.0,
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "my_lane",
                    "lane_seg_unified_viewer",
                    "--display-mode",
                    LaunchConfiguration("display_mode"),
                    "--publish-debug-images",
                    LaunchConfiguration("publish_debug_images"),
                    "--publish-canonical",
                    "false",
                    "--image-topic",
                    "/lane_seg/source_image",
                    "--white-mask-topic",
                    "/lane_seg/white_boundary_mask",
                    "--yellow-mask-topic",
                    "/lane_seg/yellow_centerline_mask",
                    "--bev-config",
                    PathJoinSubstitution(
                        [lane_share, "config", "bev_latest.json"]
                    ),
                    "--params",
                    PathJoinSubstitution(
                        [lane_share, "config", "lane_seg_path_params.yaml"]
                    ),
                    "--bag-name",
                    "direct_bev_live",
                ],
                output="screen",
            ),
            Node(
                package="my_lane",
                executable="bev_path_centerline",
                name="bev_path_centerline",
                output="screen",
            ),
            Node(
                package="my_control",
                executable="canonical_stanley_pursuit_driver",
                name="direct_bev_stanley_pursuit_driver",
                output="screen",
                parameters=[
                    {
                        "external_path_enabled": True,
                        "external_path_topic": "/perception/bev_direct_centerline",
                        "external_path_timeout_sec": float_param(
                            "external_path_timeout_sec"
                        ),
                        "external_path_previous_weight": 0.0,
                        "command_on_canonical": False,
                        "drive_enabled": bool_param("drive_enabled"),
                        "steering_only": bool_param("steering_only"),
                        "shadow_motor_topic": LaunchConfiguration(
                            "shadow_motor_topic"
                        ),
                        "cruise_speed_command": float_param(
                            "cruise_speed_command"
                        ),
                        "minimum_speed_command": float_param(
                            "minimum_speed_command"
                        ),
                        "curvature_speed_control_enabled": bool_param(
                            "curvature_speed_control_enabled"
                        ),
                        "curve_speed_command": float_param(
                            "curve_speed_command"
                        ),
                        "degraded_path_speed_command": float_param(
                            "degraded_path_speed_command"
                        ),
                        "curve_speed_exit_threshold_per_m": float_param(
                            "curve_speed_exit_threshold_per_m"
                        ),
                        "curve_speed_confirmation_frames": int_param(
                            "curve_speed_confirmation_frames"
                        ),
                        "curve_speed_release_frames": int_param(
                            "curve_speed_release_frames"
                        ),
                        "degraded_path_minimum_span_m": float_param(
                            "degraded_path_minimum_span_m"
                        ),
                        "lane_loss_speed_command": 0.0,
                        "hold_last_steering_on_lane_loss": False,
                        "hold_last_speed_on_lane_loss": False,
                        "command_rate_hz": float_param("command_rate_hz"),
                        "lookahead_distance_m": float_param(
                            "lookahead_distance_m"
                        ),
                        "pure_pursuit_control_x_m": float_param(
                            "pure_pursuit_control_x_m"
                        ),
                        "stanley_control_x_m": float_param(
                            "stanley_control_x_m"
                        ),
                        "control_latency_preview_sec": float_param(
                            "control_latency_preview_sec"
                        ),
                        "pure_pursuit_weight": float_param(
                            "pure_pursuit_weight"
                        ),
                        "stanley_gain": float_param("stanley_gain"),
                        "stanley_softening_mps": float_param(
                            "stanley_softening_mps"
                        ),
                        "target_left_offset_m": float_param(
                            "target_left_offset_m"
                        ),
                        "straight_target_right_offset_m": float_param(
                            "straight_target_right_offset_m"
                        ),
                        "target_right_offset_m": 0.0,
                        "external_lateral_offset_enabled": bool_param(
                            "external_lateral_offset_enabled"
                        ),
                        "external_lateral_offset_topic": LaunchConfiguration(
                            "external_lateral_offset_topic"
                        ),
                        "angle_command_min": float_param("angle_command_min"),
                        "angle_command_max": float_param("angle_command_max"),
                        "curve_slowdown_angle_command": float_param(
                            "curve_slowdown_angle_command"
                        ),
                        "straight_stanley_enabled": bool_param(
                            "straight_stanley_enabled"
                        ),
                        "straight_path_curvature_threshold": float_param(
                            "straight_path_curvature_threshold"
                        ),
                        "curve_detection_near_x_m": float_param(
                            "curve_detection_near_x_m"
                        ),
                        "curve_detection_far_x_m": float_param(
                            "curve_detection_far_x_m"
                        ),
                        "curve_detection_segment_count": int_param(
                            "curve_detection_segment_count"
                        ),
                        "straight_pure_pursuit_weight": float_param(
                            "straight_pure_pursuit_weight"
                        ),
                        "straight_stanley_gain": float_param(
                            "straight_stanley_gain"
                        ),
                        "straight_stanley_softening_mps": float_param(
                            "straight_stanley_softening_mps"
                        ),
                        "opposed_stanley_weight": float_param(
                            "opposed_stanley_weight"
                        ),
                        "steering_current_weight": float_param(
                            "steering_current_weight"
                        ),
                        "steering_curve_current_weight": float_param(
                            "steering_curve_current_weight"
                        ),
                        "steering_rate_limit_cmd_per_sec": float_param(
                            "steering_rate_limit_cmd_per_sec"
                        ),
                        "steering_curve_rate_limit_cmd_per_sec": float_param(
                            "steering_curve_rate_limit_cmd_per_sec"
                        ),
                        "curve_steering_multiplier_enabled": bool_param(
                            "curve_steering_multiplier_enabled"
                        ),
                        "curve_steering_multiplier_activation_command": float_param(
                            "curve_steering_multiplier_activation_command"
                        ),
                        "curve_steering_multiplier": float_param(
                            "curve_steering_multiplier"
                        ),
                    }
                ],
            ),
        ]
    )
