from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("xycar_gazebo_bridge")
    bridge_params = PathJoinSubstitution(
        [package_share, "config", "xycar_gazebo_bridge.yaml"]
    )
    legacy_driver_params = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "config",
            "kookmin_legacy_camera_driver.yaml",
        ]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "xycar_legacy_camera_drive.rviz"]
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
        "'--ros-args', '-r', '__node:=xycar_legacy_camera_drive_rviz', "
        "'-p', 'use_sim_time:=true'])"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bridge_params",
                default_value=bridge_params,
                description="YAML parameters for xycar_motor_bridge.",
            ),
            DeclareLaunchArgument(
                "legacy_driver_params",
                default_value=legacy_driver_params,
                description="YAML parameters for the Kookmin legacy camera driver.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=rviz_config,
                description="RViz config for the legacy camera rule driver.",
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
                parameters=[LaunchConfiguration("bridge_params")],
                output="screen",
            ),
            Node(
                package="xycar_rule_drive",
                executable="kookmin_legacy_camera_driver",
                name="kookmin_legacy_camera_driver",
                parameters=[LaunchConfiguration("legacy_driver_params")],
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
                name="xycar_legacy_camera_drive_rviz",
                output="screen",
            ),
        ]
    )
