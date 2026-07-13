from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_launch = PathJoinSubstitution(
        [
            FindPackageShare("xycar_perception"),
            "launch",
            "real_canonical_perception.launch.py",
        ]
    )
    inference_launch = PathJoinSubstitution(
        [FindPackageShare("il_data_tools"), "launch", "real_policy_inference.launch.py"]
    )
    model_path = LaunchConfiguration("model_path")
    source_image_topic = LaunchConfiguration("source_image_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    motor_topic = LaunchConfiguration("motor_topic")
    drive_enabled = LaunchConfiguration("drive_enabled")
    speed_command = LaunchConfiguration("speed_command")
    steering_output_sign = LaunchConfiguration("steering_output_sign")
    max_steer_scale = LaunchConfiguration("max_steer_scale")
    steering_temporal_alpha = LaunchConfiguration("steering_temporal_alpha")
    device = LaunchConfiguration("device")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("il_data_tools"),
                        "models",
                        "drive_canonical_policy_scripted.pt",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "source_image_topic", default_value="/wide_camera/rect/image_raw"
            ),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("motor_topic", default_value="/xycar_motor"),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description="Shadow mode is mandatory for the first real-car run.",
            ),
            DeclareLaunchArgument("speed_command", default_value="3.0"),
            DeclareLaunchArgument("steering_output_sign", default_value="1.0"),
            DeclareLaunchArgument("max_steer_scale", default_value="100.0"),
            DeclareLaunchArgument("steering_temporal_alpha", default_value="0.55"),
            DeclareLaunchArgument("device", default_value="cpu"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch),
                launch_arguments={
                    "image_topic": source_image_topic,
                    "enable_rectify": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(inference_launch),
                launch_arguments={
                    "model_path": model_path,
                    "image_topic": "/perception/canonical_road_image",
                    "scan_topic": scan_topic,
                    "motor_topic": motor_topic,
                    "drive_enabled": drive_enabled,
                    "speed_command": speed_command,
                    "min_speed_command": speed_command,
                    "steering_output_sign": steering_output_sign,
                    "max_steer_scale": max_steer_scale,
                    "steering_temporal_alpha": steering_temporal_alpha,
                    "device": device,
                }.items(),
            ),
        ]
    )
