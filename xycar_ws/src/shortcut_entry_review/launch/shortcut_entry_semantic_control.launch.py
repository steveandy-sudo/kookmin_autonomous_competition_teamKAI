"""W1/Y1 semantic entry, existing Stanley/Pursuit, and ShortcutCore handoff."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lane_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "lane_seg_lraspp_canonical_only.launch.py",
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
    processing_topic = LaunchConfiguration("processing_enabled_topic")
    default_enabled = LaunchConfiguration("default_enabled")

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
            "camera_yaml",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("xycar_perception"),
                    "config",
                    "wide_camera_fisheye_1280x1024_20260708.yaml",
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
        DeclareLaunchArgument("default_enabled", default_value="false"),
        DeclareLaunchArgument(
            "controller_reference_speed_command", default_value="20.0"
        ),
        DeclareLaunchArgument("w1_path_weight", default_value="0.60"),
        DeclareLaunchArgument(
            "rule_candidate_topic", default_value="/hybrid/rule_candidate"
        ),
        DeclareLaunchArgument("rule_angle_index", default_value="0"),
        DeclareLaunchArgument("rule_speed_index", default_value="1"),
        DeclareLaunchArgument(
            "vehicle_state_topic", default_value="/vehicle/vesc_state"
        ),
        DeclareLaunchArgument(
            "full_control_distance_m", default_value="0.15"
        ),
        DeclareLaunchArgument(
            "minimum_start_distance_m", default_value="0.55"
        ),
        DeclareLaunchArgument(
            "maximum_start_distance_m", default_value="1.20"
        ),
        DeclareLaunchArgument("control_latency_sec", default_value="0.25"),
        DeclareLaunchArgument("distance_margin_m", default_value="0.08"),
        DeclareLaunchArgument("minimum_control_blend", default_value="0.05"),
        DeclareLaunchArgument("speed_command_to_mps", default_value="0.08"),
        DeclareLaunchArgument(
            "candidate_topic", default_value="/hybrid/shortcut_candidate"
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("show_opencv_windows", default_value="false"),
    ]

    camera_gate = Node(
        package="shortcut_entry_review",
        executable="compressed_image_gate",
        name="shortcut_lraspp_camera_gate",
        output="screen",
        parameters=[
            {
                "source_topic": LaunchConfiguration("source_topic"),
                "output_topic": "/shortcut/lraspp/input/compressed",
                "processing_enabled_topic": processing_topic,
                "default_enabled": ParameterValue(default_enabled, value_type=bool),
            }
        ],
    )
    lane = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lane_launch),
        launch_arguments={
            "model_path": LaunchConfiguration("lane_model"),
            "camera_yaml": LaunchConfiguration("camera_yaml"),
            "image_topic": "/shortcut/lraspp/input/compressed",
            "use_compressed_image": "true",
            "enable_rectify": "true",
            "direct_model_rectify_enabled": "true",
            "direct_model_rectify_oversample": "3",
            "max_input_age_sec": "0.0",
            "direct_canonical_enabled": "true",
            # Keep the white fork branches.  The sequence selector must see
            # the same raw 256x144 canonical masks used by the hand-labelled
            # W1/W2 evaluation; the generic single-white-line fitter erases
            # the fork before W1 can be selected.
            "canonical_white_fit_enabled": "false",
            "publish_intermediate_topics": "true",
            "max_output_rate_hz": "15.0",
            "debug_rate_hz": "10.0",
            "pipeline_qos_depth": "1",
            "cpu_threads": "4",
        }.items(),
    )
    isolated_lane = GroupAction(
        [
            # The normal RULE perception launch may run at the same time.  A
            # namespace keeps node names and parameter services unambiguous;
            # the absolute remaps below keep the documented shortcut topics.
            PushRosNamespace("shortcut_entry"),
            SetRemap(src="/lane_seg/source_image", dst="/shortcut/lraspp/source_image"),
            SetRemap(src="/lane_seg/white_boundary_mask", dst="/shortcut/lraspp/white_mask"),
            SetRemap(src="/lane_seg/yellow_centerline_mask", dst="/shortcut/lraspp/yellow_mask"),
            SetRemap(src="/lane_seg/debug_image", dst="/shortcut/lraspp/debug_image"),
            SetRemap(src="/lane_seg/diagnostics", dst="/shortcut/lraspp/diagnostics"),
            SetRemap(src="/perception/yolo_debug_image", dst="/shortcut/lraspp/perception_debug_image"),
            SetRemap(src="/perception/canonical_road_image", dst="/shortcut/lraspp/canonical_road_image"),
            SetRemap(src="/perception/canonical_white_mask", dst="/shortcut/lraspp/canonical_white_mask"),
            SetRemap(src="/perception/canonical_yellow_mask", dst="/shortcut/lraspp/canonical_yellow_mask"),
            SetRemap(src="/perception/canonical_valid_mask", dst="/shortcut/lraspp/canonical_valid_mask"),
            lane,
        ]
    )
    selector = Node(
        package="shortcut_entry_review",
        executable="sequence_entry",
        name="shortcut_sequence_entry",
        output="screen",
        parameters=[
            {
                "white_mask_topic": "/shortcut/lraspp/canonical_white_mask",
                "yellow_mask_topic": "/shortcut/lraspp/canonical_yellow_mask",
                "processing_enabled_topic": processing_topic,
                "default_enabled": ParameterValue(default_enabled, value_type=bool),
                "input_is_bev": True,
                "w1_path_weight": ParameterValue(
                    LaunchConfiguration("w1_path_weight"), value_type=float
                ),
                "show_opencv_windows": ParameterValue(
                    LaunchConfiguration("show_opencv_windows"), value_type=bool
                ),
            }
        ],
    )
    entry_controller = Node(
        package="xycar_rule_drive",
        executable="canonical_stanley_pursuit_driver",
        name="shortcut_entry_stanley_pursuit",
        output="screen",
        parameters=[
            rule_base,
            rule_real,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "drive_enabled": False,
                "external_path_enabled": True,
                "external_path_topic": "/shortcut/entry/selected_centerline",
                "shadow_motor_topic": "/shortcut/entry/controller_candidate",
                "motor_topic": "/shortcut/entry/UNUSED_motor",
                "target_right_offset_m": 0.0,
                "target_left_offset_m": 0.0,
                "cruise_speed_command": ParameterValue(
                    LaunchConfiguration("controller_reference_speed_command"),
                    value_type=float,
                ),
                "minimum_speed_command": ParameterValue(
                    LaunchConfiguration("controller_reference_speed_command"),
                    value_type=float,
                ),
                "lane_loss_speed_command": 0.0,
                "hold_last_steering_on_lane_loss": False,
                "hold_last_speed_on_lane_loss": False,
                "external_path_timeout_sec": 0.35,
                "command_rate_hz": 20.0,
                "target_path_topic": "/shortcut/entry/controller_path",
                "debug_markers_topic": "/shortcut/entry/controller_markers",
                "diagnostics_topic": "/shortcut/entry/controller_diagnostics",
            },
        ],
    )
    legacy_cruise = Node(
        package="track_drive_sve",
        executable="shortcut_candidate_node",
        name="shortcut_legacy_cruise_candidate",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("source_topic"),
                "processing_enabled_topic": "/shortcut/entry/cruise_enabled",
                "candidate_topic": "/shortcut/legacy_cruise_candidate",
                "start_in_cruise": True,
                "maximum_abs_angle_command": 42.0,
            }
        ],
    )
    mux = Node(
        package="shortcut_entry_review",
        executable="shortcut_candidate_mux",
        name="shortcut_entry_candidate_mux",
        output="screen",
        parameters=[
            {
                "processing_enabled_topic": processing_topic,
                "default_enabled": ParameterValue(default_enabled, value_type=bool),
                "candidate_topic": LaunchConfiguration("candidate_topic"),
                "rule_candidate_topic": LaunchConfiguration(
                    "rule_candidate_topic"
                ),
                "rule_angle_index": ParameterValue(
                    LaunchConfiguration("rule_angle_index"), value_type=int
                ),
                "rule_speed_index": ParameterValue(
                    LaunchConfiguration("rule_speed_index"), value_type=int
                ),
                "vehicle_state_topic": LaunchConfiguration(
                    "vehicle_state_topic"
                ),
                "full_control_distance_m": ParameterValue(
                    LaunchConfiguration("full_control_distance_m"),
                    value_type=float,
                ),
                "minimum_start_distance_m": ParameterValue(
                    LaunchConfiguration("minimum_start_distance_m"),
                    value_type=float,
                ),
                "maximum_start_distance_m": ParameterValue(
                    LaunchConfiguration("maximum_start_distance_m"),
                    value_type=float,
                ),
                "control_latency_sec": ParameterValue(
                    LaunchConfiguration("control_latency_sec"),
                    value_type=float,
                ),
                "distance_margin_m": ParameterValue(
                    LaunchConfiguration("distance_margin_m"),
                    value_type=float,
                ),
                "minimum_control_blend": ParameterValue(
                    LaunchConfiguration("minimum_control_blend"),
                    value_type=float,
                ),
                "speed_command_to_mps": ParameterValue(
                    LaunchConfiguration("speed_command_to_mps"),
                    value_type=float,
                ),
            }
        ],
    )
    return LaunchDescription(
        [
            *arguments,
            camera_gate,
            isolated_lane,
            selector,
            entry_controller,
            legacy_cruise,
            mux,
        ]
    )
