"""Start perception, RL/rule candidates, and sequential gate arbitration."""

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
            "lane_seg_lraspp_low_latency_real.launch.py",
        ]
    )
    rl_launch = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rl"),
            "launch",
            "real_shadow.launch.py",
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
    checkpoint_path = LaunchConfiguration("checkpoint_path")
    drive_enabled = LaunchConfiguration("drive_enabled")
    speed_command = LaunchConfiguration("speed_command")
    scan_topic = LaunchConfiguration("scan_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("gate_arming_required", default_value="false"),
            DeclareLaunchArgument("force_rule_only", default_value="false"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument("start_rule", default_value="true"),
            DeclareLaunchArgument("start_rl", default_value="true"),
            DeclareLaunchArgument("start_cone", default_value="true"),
            DeclareLaunchArgument(
                "start_object_detection", default_value="true"
            ),
            DeclareLaunchArgument(
                "vehicle_avoidance_enabled", default_value="true"
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("speed_command", default_value="3.0"),
            DeclareLaunchArgument("cone_speed_command", default_value="6.0"),
            DeclareLaunchArgument(
                "lookahead_distance_m", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "pure_pursuit_weight", default_value="0.9"
            ),
            DeclareLaunchArgument(
                "target_left_offset_m", default_value="0.15"
            ),
            DeclareLaunchArgument(
                "start_waypoint_number", default_value="1"
            ),
            DeclareLaunchArgument(
                "maximum_speed_command", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "perception_max_output_rate_hz", default_value="10.0"
            ),
            DeclareLaunchArgument(
                "checkpoint_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("xycar_rl"),
                        "models",
                        "lap_time_speed_only_round02_20260723",
                        "camera_speed_lap_time_speed_only_actor.pth",
                    ]
                ),
            ),
            DeclareLaunchArgument("model_speed_cap", default_value="30.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch),
                condition=IfCondition(LaunchConfiguration("start_perception")),
                launch_arguments={
                    "max_output_rate_hz": LaunchConfiguration(
                        "perception_max_output_rate_hz"
                    ),
                    "debug_rate_hz": "0.0",
                }.items(),
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
                        "shadow_motor_topic": "/hybrid/rule_candidate",
                        "cruise_speed_command": ParameterValue(
                            speed_command, value_type=float
                        ),
                        "minimum_speed_command": ParameterValue(
                            speed_command, value_type=float
                        ),
                        "lookahead_distance_m": ParameterValue(
                            LaunchConfiguration("lookahead_distance_m"),
                            value_type=float,
                        ),
                        "pure_pursuit_weight": ParameterValue(
                            LaunchConfiguration("pure_pursuit_weight"),
                            value_type=float,
                        ),
                        # Apply the same configurable left correction to RULE
                        # and learned-policy targets.
                        "target_right_offset_m": 0.0,
                        "target_left_offset_m": ParameterValue(
                            LaunchConfiguration("target_left_offset_m"),
                            value_type=float,
                        ),
                        "external_lateral_offset_enabled": True,
                        "external_lateral_offset_topic": (
                            "/hybrid/avoidance_lateral_offset"
                        ),
                    },
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rl_launch),
                condition=IfCondition(LaunchConfiguration("start_rl")),
                launch_arguments={
                    "policy_kind": "camera_speed_td3_bc",
                    "checkpoint_path": checkpoint_path,
                    "image_topic": "/perception/canonical_road_image",
                    # Shift the learned target left without a constant
                    # steering bias that would make straight driving curve.
                    "model_target_left_offset_m": LaunchConfiguration(
                        "target_left_offset_m"
                    ),
                    "canonical_lateral_range_m": "1.4",
                    "canonical_background_gray": "36",
                    "scan_topic": scan_topic,
                    "drive_enabled": "false",
                    "min_speed_command": speed_command,
                    "max_speed_command": LaunchConfiguration(
                        "maximum_speed_command"
                    ),
                    "deployment_speed_cap": LaunchConfiguration(
                        "model_speed_cap"
                    ),
                    "speed_temporal_alpha": "1.0",
                    "max_steering_command": "42.0",
                    "steering_gain": "1.0",
                    "max_inference_rate_hz": "0.0",
                    "lidar_safety_enabled": "false",
                    "adaptive_steering_enabled": "false",
                    "straight_steering_temporal_alpha": "1.0",
                    "steering_temporal_alpha": "1.0",
                    "steering_straight_threshold": "1.0",
                    "steering_curve_threshold": "1.0",
                    "straight_steering_rate_limit": "1.0",
                    "curve_steering_rate_limit": "1.0",
                    "steering_deadband": "0.0",
                    "turn_in_anticipation_gain": "0.0",
                    "turn_in_anticipation_threshold": "1.0",
                    "preview_steering_enabled": "false",
                    "device": "cpu",
                }.items(),
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
                        "start_waypoint_number": ParameterValue(
                            LaunchConfiguration("start_waypoint_number"),
                            value_type=int,
                        ),
                        "minimum_speed_command": ParameterValue(
                            speed_command, value_type=float
                        ),
                        "maximum_speed_command": ParameterValue(
                            LaunchConfiguration("maximum_speed_command"),
                            value_type=float,
                        ),
                        "vehicle_avoidance_enabled": ParameterValue(
                            LaunchConfiguration(
                                "vehicle_avoidance_enabled"
                            ),
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
