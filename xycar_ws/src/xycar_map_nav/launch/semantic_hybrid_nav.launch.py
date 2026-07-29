"""Run SLAM global-path control with sensor-gated mission overrides."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    map_nav_share = FindPackageShare("xycar_map_nav")
    my_rule_share = FindPackageShare("my_rule")
    perception_share = FindPackageShare("xycar_perception")
    waypoint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    map_nav_share,
                    "launch",
                    "real_waypoint_nav.launch.py",
                ]
            )
        ),
        launch_arguments={
            "map_yaml": LaunchConfiguration("map_yaml"),
            "waypoints_yaml": LaunchConfiguration("waypoints_yaml"),
            "params_file": LaunchConfiguration("params_file"),
            "drive_enabled": LaunchConfiguration("drive_enabled"),
            "auto_localize_on_route": LaunchConfiguration(
                "auto_localize_on_route"
            ),
            "cruise_speed_command": LaunchConfiguration(
                "cruise_speed_command"
            ),
            "minimum_speed_command": LaunchConfiguration(
                "minimum_speed_command"
            ),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    object_detection = Node(
        package="my_rule",
        executable="object_detection_node",
        name="my_rule_object_detection_node",
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("start_object_detection")
        ),
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
                "model_path": LaunchConfiguration("object_model_path"),
                "image_topic": LaunchConfiguration("object_image_topic"),
                "use_compressed_image": ParameterValue(
                    LaunchConfiguration(
                        "object_use_compressed_image"
                    ),
                    value_type=bool,
                ),
                "camera_yaml": LaunchConfiguration(
                    "object_camera_yaml"
                ),
                "enable_rectify": ParameterValue(
                    LaunchConfiguration("object_enable_rectify"),
                    value_type=bool,
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
            },
        ],
    )
    cone_controller = Node(
        package="my_rule",
        executable="cone_node",
        name="my_rule_cone_node",
        output="screen",
        condition=IfCondition(
            LaunchConfiguration("start_cone_controller")
        ),
        parameters=[
            PathJoinSubstitution(
                [my_rule_share, "config", "cone_control.yaml"]
            ),
            {
                "scan_topic": LaunchConfiguration("scan_topic"),
                "processing_gate_enabled": ParameterValue(
                    LaunchConfiguration("gate_cone_processing"),
                    value_type=bool,
                ),
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"),
                    value_type=bool,
                ),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_yaml",
                description=(
                    "Absolute YAML path of the map used by the active SLAM "
                    "localization session."
                ),
            ),
            DeclareLaunchArgument(
                "waypoints_yaml",
                description=(
                    "Absolute waypoint YAML captured on the same saved map."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        map_nav_share,
                        "config",
                        "waypoint_nav_real.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument(
                "auto_localize_on_route", default_value="false"
            ),
            DeclareLaunchArgument(
                "cruise_speed_command", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "minimum_speed_command", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "start_object_detection", default_value="true"
            ),
            DeclareLaunchArgument(
                "start_cone_controller", default_value="true"
            ),
            DeclareLaunchArgument(
                "gate_cone_processing", default_value="true"
            ),
            DeclareLaunchArgument(
                "object_model_path",
                default_value=PathJoinSubstitution(
                    [
                        my_rule_share,
                        "models",
                        "my_rule_objects.pt",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "object_image_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "object_use_compressed_image", default_value="true"
            ),
            DeclareLaunchArgument(
                "object_camera_yaml",
                default_value=PathJoinSubstitution(
                    [
                        perception_share,
                        "config",
                        "wide_camera_fisheye_1280x1024.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "object_enable_rectify", default_value="true"
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            waypoint_launch,
            object_detection,
            cone_controller,
        ]
    )
