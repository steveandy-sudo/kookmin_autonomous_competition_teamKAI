"""Run the low-latency LR-ASPP perception with real-car rule control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _as_bool(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _as_float(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    perception_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "lane_seg_lraspp_low_latency_real.launch.py",
        ]
    )
    camera_launch = PathJoinSubstitution(
        [FindPackageShare("wide_camera"), "launch", "wide_camera.launch.py"]
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
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "rviz",
            "real_yolo_camera_canonical.rviz",
        ]
    )
    object_config = PathJoinSubstitution(
        [FindPackageShare("my_rule"), "config", "object_detection.yaml"]
    )
    object_model = PathJoinSubstitution(
        [
            FindPackageShare("my_rule"),
            "models",
            "kookmin_objects_best_20260804.pt",
        ]
    )
    object_camera_yaml = PathJoinSubstitution(
        [
            FindPackageShare("xycar_perception"),
            "config",
            "wide_camera_fisheye_1280x1024_20260708.yaml",
        ]
    )

    arguments = [
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_perception", default_value="true"),
        DeclareLaunchArgument(
            "start_object_detection", default_value="true"
        ),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        DeclareLaunchArgument("drive_enabled", default_value="false"),
        DeclareLaunchArgument("steering_only", default_value="false"),
        DeclareLaunchArgument("speed_command", default_value="16.0"),
        DeclareLaunchArgument(
            "straight_speed_command",
            default_value=LaunchConfiguration("speed_command"),
        ),
        DeclareLaunchArgument("turn_speed_command", default_value="8.0"),
        DeclareLaunchArgument(
            "slowdown_start_angle_command", default_value="20.0"
        ),
        DeclareLaunchArgument(
            "full_slowdown_angle_command", default_value="42.0"
        ),
        DeclareLaunchArgument("speed_curve_exponent", default_value="1.0"),
        DeclareLaunchArgument("target_left_offset_m", default_value="0.0"),
        DeclareLaunchArgument("lookahead_distance_m", default_value="0.30"),
        DeclareLaunchArgument("pure_pursuit_weight", default_value="0.80"),
        DeclareLaunchArgument(
            "pure_pursuit_control_x_m", default_value="-0.08"
        ),
        DeclareLaunchArgument("stanley_control_x_m", default_value="0.16"),
        DeclareLaunchArgument("stanley_gain", default_value="1.30"),
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
            "control_latency_preview_sec", default_value="0.20"
        ),
        DeclareLaunchArgument(
            "curve_control_latency_preview_sec", default_value="0.35"
        ),
        DeclareLaunchArgument(
            "curve_control_latency_minimum_hold_sec", default_value="0.50"
        ),
        DeclareLaunchArgument(
            "curve_detection_near_x_m", default_value="0.15"
        ),
        DeclareLaunchArgument(
            "curve_detection_far_x_m", default_value="0.60"
        ),
        DeclareLaunchArgument(
            "curve_detection_segment_count", default_value="3"
        ),
        DeclareLaunchArgument(
            "curve_steering_multiplier_enabled", default_value="false"
        ),
        DeclareLaunchArgument(
            "curve_steering_multiplier_activation_command",
            default_value="20.0",
        ),
        DeclareLaunchArgument(
            "curve_steering_multiplier", default_value="1.0"
        ),
        DeclareLaunchArgument("angle_command_min", default_value="-42.0"),
        DeclareLaunchArgument("angle_command_max", default_value="42.0"),
        DeclareLaunchArgument("perception_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("rule_command_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("perception_debug_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("cpu_threads", default_value="4"),
        DeclareLaunchArgument(
            "obstacle_avoidance_enabled", default_value="true"
        ),
        DeclareLaunchArgument("obstacle_shift_m", default_value="0.20"),
        DeclareLaunchArgument(
            "obstacle_release_delay_sec", default_value="2.0"
        ),
        DeclareLaunchArgument(
            "vehicle_min_confidence", default_value="0.45"
        ),
        DeclareLaunchArgument(
            "object_inference_rate_hz", default_value="3.0"
        ),
        DeclareLaunchArgument(
            "direct_model_rectify_enabled", default_value="true"
        ),
        DeclareLaunchArgument(
            "direct_model_rectify_oversample", default_value="3"
        ),
        DeclareLaunchArgument(
            "camera_device",
            default_value=(
                "/dev/v4l/by-id/"
                "usb-HD_USB_Camera_HD_USB_Camera-video-index0"
            ),
        ),
        DeclareLaunchArgument("motor_topic", default_value="/xycar_motor"),
        DeclareLaunchArgument(
            "shadow_motor_topic", default_value="/xycar_motor_shadow"
        ),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch),
        condition=IfCondition(LaunchConfiguration("start_camera")),
        launch_arguments={
            "device": LaunchConfiguration("camera_device"),
        }.items(),
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch),
        condition=IfCondition(LaunchConfiguration("start_perception")),
        launch_arguments={
            "cpu_threads": LaunchConfiguration("cpu_threads"),
            "max_output_rate_hz": LaunchConfiguration(
                "perception_rate_hz"
            ),
            "debug_rate_hz": LaunchConfiguration(
                "perception_debug_rate_hz"
            ),
            "direct_model_rectify_enabled": LaunchConfiguration(
                "direct_model_rectify_enabled"
            ),
            "direct_model_rectify_oversample": LaunchConfiguration(
                "direct_model_rectify_oversample"
            ),
            "publish_intermediate_topics": "true",
        }.items(),
    )
    object_detection = Node(
        package="my_rule",
        executable="object_detection_node",
        name="my_rule_object_detection_node",
        condition=IfCondition(LaunchConfiguration("start_object_detection")),
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
                "inference_rate_hz": _as_float(
                    "object_inference_rate_hz"
                ),
                "car_confidence": _as_float("vehicle_min_confidence"),
                "obstacle_vehicle_confidence": _as_float(
                    "vehicle_min_confidence"
                ),
            },
        ],
    )
    rule_driver = Node(
        package="xycar_rule_drive",
        executable="canonical_stanley_pursuit_driver",
        name="canonical_stanley_pursuit_driver",
        output="screen",
        parameters=[
            rule_base,
            rule_real,
            {
                "use_sim_time": False,
                "canonical_topic": "/perception/canonical_road_image",
                "motor_topic": LaunchConfiguration("motor_topic"),
                "shadow_motor_topic": "/rule_drive/base_motor_shadow",
                "drive_enabled": False,
                "steering_only": _as_bool("steering_only"),
                "cruise_speed_command": _as_float(
                    "straight_speed_command"
                ),
                "minimum_speed_command": _as_float(
                    "turn_speed_command"
                ),
                "lane_loss_speed_command": _as_float(
                    "turn_speed_command"
                ),
                "curve_slowdown_angle_command": _as_float(
                    "full_slowdown_angle_command"
                ),
                "target_right_offset_m": 0.0,
                "target_left_offset_m": _as_float(
                    "target_left_offset_m"
                ),
                "lookahead_distance_m": _as_float(
                    "lookahead_distance_m"
                ),
                "pure_pursuit_weight": _as_float(
                    "pure_pursuit_weight"
                ),
                "pure_pursuit_control_x_m": _as_float(
                    "pure_pursuit_control_x_m"
                ),
                "stanley_control_x_m": _as_float(
                    "stanley_control_x_m"
                ),
                "stanley_gain": _as_float("stanley_gain"),
                "stanley_softening_mps": _as_float(
                    "stanley_softening_mps"
                ),
                "straight_pure_pursuit_weight": _as_float(
                    "straight_pure_pursuit_weight"
                ),
                "straight_stanley_gain": _as_float(
                    "straight_stanley_gain"
                ),
                "straight_stanley_softening_mps": _as_float(
                    "straight_stanley_softening_mps"
                ),
                "opposed_stanley_weight": _as_float(
                    "opposed_stanley_weight"
                ),
                "control_latency_preview_sec": _as_float(
                    "control_latency_preview_sec"
                ),
                "curve_control_latency_preview_sec": _as_float(
                    "curve_control_latency_preview_sec"
                ),
                "curve_control_latency_minimum_hold_sec": _as_float(
                    "curve_control_latency_minimum_hold_sec"
                ),
                "curve_detection_near_x_m": _as_float(
                    "curve_detection_near_x_m"
                ),
                "curve_detection_far_x_m": _as_float(
                    "curve_detection_far_x_m"
                ),
                "curve_detection_segment_count": ParameterValue(
                    LaunchConfiguration("curve_detection_segment_count"),
                    value_type=int,
                ),
                "curve_steering_multiplier_enabled": _as_bool(
                    "curve_steering_multiplier_enabled"
                ),
                "curve_steering_multiplier_activation_command": _as_float(
                    "curve_steering_multiplier_activation_command"
                ),
                "curve_steering_multiplier": _as_float(
                    "curve_steering_multiplier"
                ),
                "angle_command_min": _as_float("angle_command_min"),
                "angle_command_max": _as_float("angle_command_max"),
                "command_rate_hz": _as_float("rule_command_rate_hz"),
                "command_on_canonical": True,
                "external_lateral_offset_enabled": True,
                "external_lateral_offset_topic": (
                    "/rule_drive/obstacle_lateral_offset"
                ),
                "external_lateral_offset_timeout_sec": 0.25,
                "sitl_bypass_path_enabled": False,
            },
        ],
    )
    command_adapter = Node(
        package="xycar_rule_drive",
        executable="rule_command_adapter",
        name="rule_command_adapter",
        output="screen",
        parameters=[
            {
                "drive_enabled": _as_bool("drive_enabled"),
                "obstacle_avoidance_enabled": _as_bool(
                    "obstacle_avoidance_enabled"
                ),
                "motor_topic": LaunchConfiguration("motor_topic"),
                "shadow_motor_topic": LaunchConfiguration(
                    "shadow_motor_topic"
                ),
                "straight_speed_command": _as_float(
                    "straight_speed_command"
                ),
                "turn_speed_command": _as_float(
                    "turn_speed_command"
                ),
                "slowdown_start_angle_command": _as_float(
                    "slowdown_start_angle_command"
                ),
                "full_slowdown_angle_command": _as_float(
                    "full_slowdown_angle_command"
                ),
                "speed_curve_exponent": _as_float(
                    "speed_curve_exponent"
                ),
                "obstacle_shift_m": _as_float("obstacle_shift_m"),
                "obstacle_release_delay_sec": _as_float(
                    "obstacle_release_delay_sec"
                ),
                "vehicle_min_confidence": _as_float(
                    "vehicle_min_confidence"
                ),
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_lraspp_rule_drive",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        output="screen",
        arguments=["-d", rviz_config],
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "QT_XCB_GL_INTEGRATION": "none",
        },
    )

    return LaunchDescription(
        [
            *arguments,
            camera,
            perception,
            object_detection,
            rule_driver,
            command_adapter,
            rviz,
        ]
    )
