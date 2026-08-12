"""Start the complete Jetson real-car stack with output disarmed by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(package, launch_file, condition, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package), "launch", launch_file]
            )
        ),
        condition=IfCondition(LaunchConfiguration(condition)),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    camera_device = LaunchConfiguration("camera_device")
    lidar_params_file = LaunchConfiguration("lidar_params_file")
    vesc_port = LaunchConfiguration("vesc_port")
    command_drive_enabled = LaunchConfiguration("command_drive_enabled")
    vesc_drive_enabled = LaunchConfiguration("vesc_drive_enabled")
    steering_center_trim = LaunchConfiguration(
        "steering_center_trim_command"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_vesc", default_value="true"),
            DeclareLaunchArgument("start_drive_stack", default_value="true"),
            DeclareLaunchArgument("start_perception", default_value="true"),
            DeclareLaunchArgument(
                "command_drive_enabled",
                default_value="false",
                description=(
                    "Allow the selected driver to publish /xycar_motor."
                ),
            ),
            DeclareLaunchArgument(
                "vesc_drive_enabled",
                default_value="false",
                description=(
                    "Allow the native VESC driver to apply non-zero output."
                ),
            ),
            DeclareLaunchArgument("driver_mode", default_value="hybrid"),
            DeclareLaunchArgument(
                "device",
                default_value="cuda",
                description="Torch device shared by perception and policy.",
            ),
            DeclareLaunchArgument(
                "perception_cpu_threads", default_value="4"
            ),
            DeclareLaunchArgument("policy_cpu_threads", default_value="4"),
            DeclareLaunchArgument("model_speed_cap", default_value="20.0"),
            DeclareLaunchArgument("cone_speed_cap", default_value="9.5"),
            DeclareLaunchArgument(
                "camera_device",
                default_value=(
                    "/dev/v4l/by-id/"
                    "usb-HD_USB_Camera_HD_USB_Camera-video-index0"
                ),
            ),
            DeclareLaunchArgument("camera_width", default_value="1280"),
            DeclareLaunchArgument("camera_height", default_value="1024"),
            DeclareLaunchArgument("camera_fps", default_value="30"),
            DeclareLaunchArgument(
                "lidar_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("xycar_lidar"), "params", "ydlidar.yaml"]
                ),
            ),
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyMOTOR"),
            DeclareLaunchArgument(
                "acceleration_slew_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "steering_center_trim_command", default_value="-5.0"
            ),
            _include(
                "wide_camera",
                "wide_camera.launch.py",
                "start_camera",
                {
                    "device": camera_device,
                    "width": LaunchConfiguration("camera_width"),
                    "height": LaunchConfiguration("camera_height"),
                    "fps": LaunchConfiguration("camera_fps"),
                },
            ),
            _include(
                "xycar_lidar",
                "xycar_lidar.launch.py",
                "start_lidar",
                {"params_file": lidar_params_file},
            ),
            _include(
                "xycar_vesc_driver",
                "xycar_vesc_driver.launch.py",
                "start_vesc",
                {
                    "port": vesc_port,
                    "drive_enabled": vesc_drive_enabled,
                    "acceleration_slew_enabled": LaunchConfiguration(
                        "acceleration_slew_enabled"
                    ),
                    "steering_center_trim_command": steering_center_trim,
                },
            ),
            _include(
                "xycar_final_drive",
                "final_real_stack.launch.py",
                "start_drive_stack",
                {
                    "driver_mode": LaunchConfiguration("driver_mode"),
                    "drive_enabled": command_drive_enabled,
                    "start_perception": LaunchConfiguration(
                        "start_perception"
                    ),
                    "device": LaunchConfiguration("device"),
                    "perception_cpu_threads": LaunchConfiguration(
                        "perception_cpu_threads"
                    ),
                    "policy_cpu_threads": LaunchConfiguration(
                        "policy_cpu_threads"
                    ),
                    "model_speed_cap": LaunchConfiguration(
                        "model_speed_cap"
                    ),
                    "cone_speed_cap": LaunchConfiguration("cone_speed_cap"),
                },
            ),
        ]
    )
