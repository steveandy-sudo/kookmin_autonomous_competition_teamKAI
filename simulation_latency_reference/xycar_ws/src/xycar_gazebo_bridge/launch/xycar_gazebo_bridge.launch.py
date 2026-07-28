from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("xycar_gazebo_bridge"), "config", "xycar_gazebo_bridge.yaml"]
    )

    bridge_args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/model/xycar_ackermann/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
        "/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
        "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=config_file,
                description="YAML parameters for xycar_motor_bridge.",
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
        ]
    )
