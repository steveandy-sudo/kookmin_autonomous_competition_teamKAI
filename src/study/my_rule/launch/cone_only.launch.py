"""Launch only the camera-YOLO + LiDAR cone-driving stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _cone_node_for_speed_profile(context, *, my_rule_share, perception_share):
    profile = LaunchConfiguration("cone_speed_profile").perform(context).strip().lower()
    # Adaptive cone-speed execution is intentionally disabled.  The validated
    # fixed profile keeps every normal-path speed input at 10 while preserving
    # the final-left path-loss recovery speed of 8.
    if profile != "fixed":
        raise RuntimeError(
            "adaptive cone speed is disabled; cone_speed_profile must be 'fixed', "
            f"got {profile!r}"
        )

    final_left_recovery_speed = float(
        LaunchConfiguration("cone_final_left_recovery_speed").perform(context)
    )
    fixed_speed = float(LaunchConfiguration("cone_drive_speed").perform(context))
    speeds = {
        "cone_speed": fixed_speed,
        "cone_min_drive_speed": fixed_speed,
        "cone_straight_boost_speed": fixed_speed,
        "single_boundary_max_speed": fixed_speed,
        "cone_final_left_recovery_speed": final_left_recovery_speed,
    }

    return [
        Node(
            package="my_rule",
            executable="cone_node",
            name="my_rule_cone_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([my_rule_share, "config", "cone_control.yaml"]),
                {
                    "cone_yolo_association_enabled": True,
                    **speeds,
                    "camera_yaml": PathJoinSubstitution(
                        [
                            perception_share,
                            "config",
                            "wide_camera_fisheye_1280x1024.yaml",
                        ]
                    ),
                },
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    my_rule_share = FindPackageShare("my_rule")
    perception_share = FindPackageShare("xycar_perception")
    start_camera = LaunchConfiguration("start_camera")
    start_lidar = LaunchConfiguration("start_lidar")
    start_object_detection = LaunchConfiguration("start_object_detection")
    force_cone_mode = LaunchConfiguration("force_cone_mode")
    start_motor_driver = LaunchConfiguration("start_motor_driver")
    motor_drive_enabled = LaunchConfiguration("motor_drive_enabled")
    motor_port = LaunchConfiguration("motor_port")
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wide_camera"), "launch", "wide_camera.launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("xycar_lidar"), "launch", "xycar_lidar.launch.py"]
            )
        ),
        condition=IfCondition(start_lidar),
    )
    motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("xycar_vesc_driver"),
                    "launch",
                    "xycar_vesc_driver.launch.py",
                ]
            )
        ),
        launch_arguments={
            "drive_enabled": motor_drive_enabled,
            "port": motor_port,
        }.items(),
        condition=IfCondition(start_motor_driver),
    )
    object_detection = Node(
        package="my_rule",
        executable="object_detection_node",
        name="my_rule_object_detection_node",
        output="screen",
        condition=IfCondition(start_object_detection),
        additional_env={
            "OMP_NUM_THREADS": "2",
            "OMP_WAIT_POLICY": "PASSIVE",
            "MKL_NUM_THREADS": "2",
            "OPENBLAS_NUM_THREADS": "1",
            "MPLCONFIGDIR": "/tmp/my_rule_matplotlib",
            "YOLO_CONFIG_DIR": "/tmp/my_rule_ultralytics",
        },
        parameters=[
            PathJoinSubstitution(
                [my_rule_share, "config", "object_detection.yaml"]
            ),
            {
                "model_path": PathJoinSubstitution(
                    [my_rule_share, "models", "final.pt"]
                ),
                "startup_signal_hsv_enabled": False,
                "camera_yaml": PathJoinSubstitution(
                    [
                        perception_share,
                        "config",
                        "wide_camera_fisheye_1280x1024.yaml",
                    ]
                ),
            },
        ],
    )
    cone = OpaqueFunction(
        function=_cone_node_for_speed_profile,
        kwargs={
            "my_rule_share": my_rule_share,
            "perception_share": perception_share,
        },
    )
    manager = Node(
        package="my_rule",
        executable="drive_manager",
        name="my_rule_drive_manager",
        output="screen",
        parameters=[
            PathJoinSubstitution([my_rule_share, "config", "lane_control.yaml"]),
            PathJoinSubstitution([my_rule_share, "config", "drive_manager.yaml"]),
            {
                "force_cone_mode": ParameterValue(
                    force_cone_mode, value_type=bool
                ),
                # Cone-only means that non-cone mission gates must never
                # interrupt the slalom. Object YOLO remains active solely for
                # YOLO+LiDAR cone-entry confirmation.
                "require_yolo_cone_for_entry": True,
                "enable_traffic_light_control": False,
                "wait_for_green_at_start": False,
                "enable_static_obstacle_handling": False,
                "enable_dynamic_obstacle_handling": False,
                "cone_manager_recovery_enabled": False,
                "cone_fresh_stop_is_authoritative": True,
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument(
                "start_object_detection",
                default_value="true",
                description=(
                    "Run object YOLO only for cone-entry confirmation."
                ),
            ),
            DeclareLaunchArgument(
                "force_cone_mode",
                default_value="false",
                description=(
                    "Unsafe test override: bypass YOLO+LiDAR entry confirmation."
                ),
            ),
            DeclareLaunchArgument(
                "start_motor_driver",
                default_value="true",
                description="Start the native ROS 2 VESC serial driver.",
            ),
            DeclareLaunchArgument(
                "motor_drive_enabled",
                default_value="true",
                description="Allow the VESC driver to apply non-zero output.",
            ),
            DeclareLaunchArgument(
                "motor_port",
                default_value="/dev/ttyMOTOR",
                description="VESC serial device owned by the native ROS 2 driver.",
            ),
            DeclareLaunchArgument(
                "cone_speed_profile",
                default_value="fixed",
                description="Cone-only speed profile; only fixed is enabled.",
            ),
            DeclareLaunchArgument(
                "cone_drive_speed",
                default_value="10.0",
                description=(
                    "Motor command used by the fixed cone-only speed profile."
                ),
            ),
            DeclareLaunchArgument(
                "cone_final_left_recovery_speed",
                default_value="8.0",
                description=(
                    "Reduced motor command used only when the final-left "
                    "path is temporarily lost or rejected."
                ),
            ),
            camera,
            lidar,
            motor,
            object_detection,
            cone,
            manager,
        ]
    )
