"""Launch the complete real-car mission stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    my_rule_share = FindPackageShare("my_rule")
    perception_share = FindPackageShare("xycar_perception")
    start_camera = LaunchConfiguration("start_camera")
    start_lidar = LaunchConfiguration("start_lidar")
    start_ultrasonic = LaunchConfiguration("start_ultrasonic")
    start_object_detection = LaunchConfiguration("start_object_detection")
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
    ultrasonic = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("xycar_ultrasonic"),
                    "launch",
                    "xycar_ultrasonic.launch.py",
                ]
            )
        ),
        condition=IfCondition(start_ultrasonic),
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
    perception = Node(
        package="my_rule",
        executable="lane_perception_node",
        name="my_rule_lane_perception_node",
        output="screen",
        additional_env={
            "OMP_NUM_THREADS": "4",
            "OMP_WAIT_POLICY": "PASSIVE",
            "MKL_NUM_THREADS": "4",
            "OPENBLAS_NUM_THREADS": "1",
        },
        parameters=[
            PathJoinSubstitution(
                [my_rule_share, "config", "lane_perception.yaml"]
            ),
            {
                "model_path": PathJoinSubstitution(
                    [my_rule_share, "models", "my_rule_lane.pt"]
                ),
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
                    [my_rule_share, "models", "my_rule_objects.pt"]
                ),
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
    cone = Node(
        package="my_rule",
        executable="cone_node",
        name="my_rule_cone_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([my_rule_share, "config", "cone_control.yaml"])
        ],
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
                "enable_traffic_light_control": True,
                "wait_for_green_at_start": True,
                "enable_static_obstacle_handling": True,
                # Implemented with the obstacle_vehicle contract, but kept
                # off until its detector class and real-car pass are verified.
                "enable_dynamic_obstacle_handling": False,
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument(
                "start_ultrasonic",
                default_value="true",
                description=(
                    "Start the 8-channel ultrasonic driver used by "
                    "static-obstacle side/rear clearance checks."
                ),
            ),
            DeclareLaunchArgument(
                "start_object_detection",
                default_value="true",
                description="Run low-rate mission YOLO in a separate process.",
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
            camera,
            lidar,
            ultrasonic,
            motor,
            object_detection,
            perception,
            cone,
            manager,
        ]
    )
