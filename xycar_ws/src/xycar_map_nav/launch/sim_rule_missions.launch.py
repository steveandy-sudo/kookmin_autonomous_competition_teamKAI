"""Run CAD-v4 Gazebo cone and vehicle-avoidance tuning missions."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_root = LaunchConfiguration("project_root")
    detections_topic = LaunchConfiguration("object_detections_topic")
    rule_config = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "config",
            "canonical_stanley_pursuit.yaml",
        ]
    )
    rule_real_config = PathJoinSubstitution(
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
    mission_config = PathJoinSubstitution(
        [
            FindPackageShare("xycar_map_nav"),
            "config",
            "sim_mission_layout.yaml",
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
    simulation_launch = PathJoinSubstitution(
        [FindPackageShare("xycar_rl"), "launch", "rl_sim.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "xycar_kookmin_gazebo_track"]
                ),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        project_root,
                        "worlds",
                        "kookmin_xycar_track_cad_v4_missions.sdf",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="-s",
                description="Set to 'gui' to open the Gazebo window.",
            ),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description=(
                    "Keep false when using the SPACE drive gate. False "
                    "publishes only the safe shadow command."
                ),
            ),
            DeclareLaunchArgument(
                "gate_arming_required",
                default_value="true",
                description=(
                    "Hold mission state until /hybrid_gate/drive_armed is true."
                ),
            ),
            DeclareLaunchArgument("start_ground_truth", default_value="true"),
            DeclareLaunchArgument(
                "start_yolo",
                default_value="false",
                description=(
                    "Run the trained detector against the Gazebo image."
                ),
            ),
            DeclareLaunchArgument(
                "object_detections_topic",
                default_value="/my_rule/sim_ground_truth_detections",
            ),
            DeclareLaunchArgument("speed_command", default_value="3.0"),
            DeclareLaunchArgument("cone_speed_command", default_value="6.0"),
            DeclareLaunchArgument(
                "target_left_offset_m", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "lookahead_distance_m", default_value="0.30"
            ),
            DeclareLaunchArgument("pure_pursuit_weight", default_value="0.80"),
            DeclareLaunchArgument(
                "maximum_speed_command", default_value="30.0"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={
                    "project_root": project_root,
                    "world": LaunchConfiguration("world"),
                    "headless": LaunchConfiguration("headless"),
                    "enable_rviz": LaunchConfiguration("enable_rviz"),
                    "auto_start": "true",
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="mission_vehicle_command_bridge",
                arguments=[
                    "/model/mission_moving_vehicle/cmd_vel"
                    "@geometry_msgs/msg/Twist@gz.msgs.Twist"
                ],
                output="screen",
            ),
            Node(
                package="xycar_map_nav",
                executable="sim_moving_vehicle_controller",
                name="sim_moving_vehicle_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "layout_config": mission_config,
                    }
                ],
            ),
            Node(
                package="xycar_rule_drive",
                executable="canonical_stanley_pursuit_driver",
                name="canonical_stanley_pursuit_driver",
                output="screen",
                parameters=[
                    rule_config,
                    rule_real_config,
                    {
                        "use_sim_time": True,
                        "drive_enabled": False,
                        "shadow_motor_topic": "/hybrid/rule_candidate",
                        "cruise_speed_command": ParameterValue(
                            LaunchConfiguration("speed_command"),
                            value_type=float,
                        ),
                        "minimum_speed_command": ParameterValue(
                            LaunchConfiguration("speed_command"),
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
                        "target_right_offset_m": 0.0,
                        "target_left_offset_m": ParameterValue(
                            LaunchConfiguration("target_left_offset_m"),
                            value_type=float,
                        ),
                        "external_lateral_offset_enabled": False,
                        "sitl_bypass_path_enabled": True,
                        "sitl_bypass_path_request_topic": (
                            "/hybrid/avoidance_path_request"
                        ),
                        "vehicle_body_length_m": 0.55,
                        "vehicle_body_width_m": 0.28,
                        "lane_center_separation_m": 0.40,
                    },
                ],
            ),
            Node(
                package="xycar_map_nav",
                executable="sim_mission_ground_truth",
                name="sim_mission_ground_truth",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("start_ground_truth")
                ),
                parameters=[
                    {
                        "use_sim_time": True,
                        "layout_config": mission_config,
                        "detections_topic": detections_topic,
                    }
                ],
            ),
            Node(
                package="my_rule",
                executable="object_detection_node",
                name="my_rule_sim_object_detection_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_yolo")),
                additional_env={
                    "OMP_NUM_THREADS": "2",
                    "OMP_WAIT_POLICY": "PASSIVE",
                    "MKL_NUM_THREADS": "2",
                    "OPENBLAS_NUM_THREADS": "1",
                    "MPLCONFIGDIR": "/tmp/my_rule_sim_matplotlib",
                    "YOLO_CONFIG_DIR": "/tmp/my_rule_sim_ultralytics",
                },
                parameters=[
                    object_config,
                    {
                        "use_sim_time": True,
                        "model_path": object_model,
                        "image_topic": "/image_raw",
                        "use_compressed_image": False,
                        "detections_topic": detections_topic,
                        "enable_rectify": False,
                        "startup_signal_hsv_enabled": False,
                        "inference_rate_hz": 5.0,
                    },
                ],
            ),
            Node(
                package="my_rule",
                executable="cone_node",
                name="my_rule_cone_node",
                output="screen",
                parameters=[
                    cone_config,
                    {
                        "use_sim_time": True,
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
                        "use_sim_time": True,
                        "drive_enabled": ParameterValue(
                            LaunchConfiguration("drive_enabled"),
                            value_type=bool,
                        ),
                        "gate_arming_required": ParameterValue(
                            LaunchConfiguration("gate_arming_required"),
                            value_type=bool,
                        ),
                        "force_rule_only": True,
                        "object_detections_topic": detections_topic,
                        "minimum_speed_command": ParameterValue(
                            LaunchConfiguration("speed_command"),
                            value_type=float,
                        ),
                        "maximum_speed_command": ParameterValue(
                            LaunchConfiguration("maximum_speed_command"),
                            value_type=float,
                        ),
                        "vehicle_avoidance_enabled": True,
                    },
                ],
            ),
        ]
    )
