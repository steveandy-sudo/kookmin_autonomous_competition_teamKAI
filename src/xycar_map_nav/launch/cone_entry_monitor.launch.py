"""Integrated cone-entry telemetry with the physical motor path removed."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def package_launch(package: str, filename: str):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), "launch", filename])
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_device",
                default_value=(
                    "/dev/v4l/by-id/"
                    "usb-HD_USB_Camera_HD_USB_Camera-video-index0"
                ),
            ),
            # Deliberately include only camera and LiDAR.  The VESC launch
            # file is not part of this launch graph at all.
            IncludeLaunchDescription(
                package_launch(
                    "wide_camera", "wide_camera.launch.py"
                ),
                launch_arguments={
                    "device": LaunchConfiguration("camera_device"),
                }.items(),
            ),
            IncludeLaunchDescription(
                package_launch("xycar_lidar", "xycar_lidar.launch.py"),
                launch_arguments={
                    "params_file": PathJoinSubstitution(
                        [
                            FindPackageShare("xycar_lidar"),
                            "params",
                            "ydlidar.yaml",
                        ]
                    )
                }.items(),
            ),
            IncludeLaunchDescription(
                package_launch(
                    "xycar_map_nav",
                    "real_sequential_hybrid_drive.launch.py",
                ),
                launch_arguments={
                    # The selector still computes normal integrated commands,
                    # but never creates a /xycar_motor publisher.
                    "drive_enabled": "false",
                    "steering_only": "false",
                    "gate_arming_required": "false",
                    "force_rule_only": "true",
                    "enable_rviz": "false",
                    "start_shortcut": "false",
                    "traffic_light_control_enabled": "false",
                    "vehicle_avoidance_enabled": "false",
                    "speed_command": "25.0",
                    "curve_speed_command": "16.0",
                    "degraded_path_speed_command": "15.0",
                    "cone_speed_command": "10.0",
                }.items(),
            ),
            Node(
                package="xycar_map_nav",
                executable="cone_entry_monitor",
                name="cone_entry_monitor",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "yolo_confidence": 0.50,
                        "yolo_required_frames": 2,
                        "yolo_timeout_sec": 0.75,
                        "entry_distance_m": 3.0,
                        "cluster_timeout_sec": 0.50,
                        "path_confidence": 0.35,
                        "command_required_frames": 3,
                        "command_timeout_sec": 0.35,
                        # One stable screen per second is slow enough for the
                        # operator to read while pushing the vehicle by hand.
                        "display_rate_hz": 1.0,
                        "clear_terminal": True,
                    }
                ],
            ),
        ]
    )
