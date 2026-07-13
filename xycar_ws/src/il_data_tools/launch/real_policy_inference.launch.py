from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    motor_topic = LaunchConfiguration("motor_topic")
    shadow_topic = LaunchConfiguration("shadow_topic")
    drive_enabled = LaunchConfiguration("drive_enabled")
    speed_command = LaunchConfiguration("speed_command")
    min_speed_command = LaunchConfiguration("min_speed_command")
    device = LaunchConfiguration("device")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("il_data_tools"),
                        "models",
                        "drive_policy_scripted.pt",
                    ]
                ),
                description="Packaged TorchScript drive policy.",
            ),
            DeclareLaunchArgument("image_topic", default_value="/image_raw"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("motor_topic", default_value="/xycar_motor"),
            DeclareLaunchArgument(
                "shadow_topic",
                default_value="/il/policy_motor_shadow",
            ),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description="Publish to the physical motor only when explicitly enabled.",
            ),
            DeclareLaunchArgument(
                "speed_command",
                default_value="1.0",
                description="Conservative first-test straight speed command.",
            ),
            DeclareLaunchArgument(
                "min_speed_command",
                default_value="0.8",
                description="Conservative first-test curve speed command.",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="cpu",
                description="Use CPU on the AMD real-car mini PC unless CUDA is available.",
            ),
            Node(
                package="il_data_tools",
                executable="il_policy_inference",
                name="il_policy_inference",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "model_path": model_path,
                        "device": device,
                        "image_topic": image_topic,
                        "scan_topic": scan_topic,
                        "motor_topic": motor_topic,
                        "shadow_topic": shadow_topic,
                        "drive_enabled": ParameterValue(
                            drive_enabled, value_type=bool
                        ),
                        "speed_command": ParameterValue(
                            speed_command, value_type=float
                        ),
                        "min_speed_command": ParameterValue(
                            min_speed_command, value_type=float
                        ),
                        "sensor_timeout_sec": 0.5,
                    }
                ],
            ),
        ]
    )
