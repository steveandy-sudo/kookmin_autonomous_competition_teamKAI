from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("xycar_gazebo_bridge")
    params_file = PathJoinSubstitution(
        [package_share, "config", "xycar_gazebo_bridge.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "xycar_gazebo_sensors.rviz"]
    )
    perception_config = PathJoinSubstitution(
        [
            FindPackageShare("xycar_perception"),
            "config",
            "camera_perception.yaml",
        ]
    )
    perception_calib = PathJoinSubstitution(
        [
            FindPackageShare("xycar_perception"),
            "config",
            "wide_camera_fisheye_1280x1024.yaml",
        ]
    )

    bridge_args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/model/xycar_ackermann/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
        "/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        "/model/xycar_ackermann/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/world/kookmin_xycar_track/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/xycar_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        "/world/kookmin_xycar_track/control@ros_gz_interfaces/srv/ControlWorld",
    ]
    rviz_clean_env = (
        "import os, sys; "
        "[os.environ.pop(k, None) for k, v in list(os.environ.items()) "
        "if k.startswith('SNAP') or '/snap' in v "
        "or k in {'QT_PLUGIN_PATH', 'QT_QPA_PLATFORM_PLUGIN_PATH', "
        "'QT_QPA_FONTDIR'}]; "
        "os.execv('/opt/ros/humble/bin/rviz2', "
        "['/opt/ros/humble/bin/rviz2', '-d', sys.argv[1], "
        "'--ros-args', '-r', '__node:=xycar_gazebo_rviz', "
        "'-p', 'use_sim_time:=true'])"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "auto_start",
                default_value="true",
                description="Unpause Gazebo after startup. RL owns stepping when false.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="YAML parameters for xycar_motor_bridge.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=rviz_config,
                description="RViz config that shows Gazebo camera and lidar topics.",
            ),
            DeclareLaunchArgument(
                "perception_config",
                default_value=perception_config,
                description="YAML parameters for camera-based perception overlays.",
            ),
            DeclareLaunchArgument(
                "perception_calib",
                default_value=perception_calib,
                description="Fisheye calibration YAML for the 1280x1024 real wide camera model.",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Start RViz; disable for unattended dataset batches.",
            ),
            DeclareLaunchArgument(
                "enable_legacy_perception",
                default_value="true",
                description=(
                    "Start the legacy camera perception node. Set false when "
                    "another canonical perception backend is included."
                ),
            ),
            TimerAction(
                period=1.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "gz",
                            "service",
                            "-s",
                            "/world/kookmin_xycar_track/control",
                            "--reqtype",
                            "gz.msgs.WorldControl",
                            "--reptype",
                            "gz.msgs.Boolean",
                            "--timeout",
                            "3000",
                            "--req",
                            "pause: false",
                        ],
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("auto_start")),
                    )
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="xycar_ros_gz_parameter_bridge",
                arguments=bridge_args,
                output="screen",
            ),
            Node(
                package="xycar_gazebo_bridge",
                executable="xycar_motor_bridge",
                name="xycar_motor_bridge",
                parameters=[LaunchConfiguration("params_file")],
                output="screen",
            ),
            Node(
                package="xycar_perception",
                executable="camera_perception_node",
                name="xycar_camera_perception",
                parameters=[
                    LaunchConfiguration("perception_config"),
                    {"calib_yaml": LaunchConfiguration("perception_calib")},
                ],
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("enable_legacy_perception")
                ),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xycar_base_footprint_to_base_link_tf",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_footprint",
                    "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xycar_base_to_laser_tf",
                arguments=[
                    "0.065",
                    "0",
                    "0.08",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "xycar_ackermann/chassis/lidar",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="xycar_base_to_camera_tf",
                arguments=[
                    "-0.04",
                    "0",
                    "0.17",
                    "0",
                    "0",
                    "0",
                    "1",
                    "base_link",
                    "xycar_ackermann/chassis/front_camera",
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "python3",
                    "-c",
                    rviz_clean_env,
                    LaunchConfiguration("rviz_config"),
                ],
                name="xycar_gazebo_rviz",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_rviz")),
            ),
        ]
    )
