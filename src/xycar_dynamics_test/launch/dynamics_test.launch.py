from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("xycar_dynamics_test"), "config", "dynamics_test.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=config_file,
                description="YAML parameters for real Xycar dynamics tests.",
            ),
            DeclareLaunchArgument(
                "test_name",
                default_value="speed_step",
                description="speed_step, steer_step, turn_radius, or all.",
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                description="If true, do not publish motor commands.",
            ),
            DeclareLaunchArgument(
                "motor_topic",
                default_value="/xycar_motor",
                description="Xycar Float32MultiArray [angle, speed] command topic.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu",
                description="IMU topic used for yaw-rate and acceleration.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Optional odometry topic.",
            ),
            DeclareLaunchArgument(
                "use_odom",
                default_value="false",
                description="Use odometry speed/yaw-rate when available.",
            ),
            DeclareLaunchArgument(
                "speed_commands",
                default_value="5,10,15,20",
                description="Comma-separated speed command sequence.",
            ),
            DeclareLaunchArgument(
                "angle_commands",
                default_value="10,-10,20,-20,30,-30,40,-40",
                description="Comma-separated steering command sequence.",
            ),
            DeclareLaunchArgument(
                "turn_angle_commands",
                default_value="20,-20,30,-30,40,-40",
                description="Comma-separated steering commands for turn-radius tests.",
            ),
            DeclareLaunchArgument(
                "steer_step_speed_cmd",
                default_value="6.0",
                description="Speed command during steering step tests.",
            ),
            DeclareLaunchArgument(
                "turn_speed_cmd",
                default_value="8.0",
                description="Speed command during turn-radius tests.",
            ),
            Node(
                package="xycar_dynamics_test",
                executable="dynamics_test_runner",
                name="xycar_dynamics_test_runner",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "test_name": LaunchConfiguration("test_name"),
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "motor_topic": LaunchConfiguration("motor_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "use_odom": ParameterValue(
                            LaunchConfiguration("use_odom"), value_type=bool
                        ),
                        "speed_commands": LaunchConfiguration("speed_commands"),
                        "angle_commands": LaunchConfiguration("angle_commands"),
                        "turn_angle_commands": LaunchConfiguration("turn_angle_commands"),
                        "steer_step_speed_cmd": ParameterValue(
                            LaunchConfiguration("steer_step_speed_cmd"), value_type=float
                        ),
                        "turn_speed_cmd": ParameterValue(
                            LaunchConfiguration("turn_speed_cmd"), value_type=float
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
