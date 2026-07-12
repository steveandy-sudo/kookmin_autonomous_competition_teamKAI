from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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
    ]
    rviz_clean_env = (
        "import os, sys; "
        "[os.environ.pop(k, None) for k, v in list(os.environ.items()) "
        "if k.startswith('SNAP') or '/snap' in v]; "
        "os.execv('/opt/ros/humble/bin/rviz2', "
        "['/opt/ros/humble/bin/rviz2', '-d', sys.argv[1], "
        "'--ros-args', '-r', '__node:=xycar_gazebo_rviz', "
        "'-p', 'use_sim_time:=true'])"
    )

    return LaunchDescription(
        [
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
                    "0.08",
                    "0",
                    "0.06",
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
            ),
        ]
    )
