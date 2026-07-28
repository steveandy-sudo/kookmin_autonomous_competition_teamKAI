"""Launch the optional read-only lane/cone 2x2 visualization window."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    my_rule_share = FindPackageShare("my_rule")
    perception_share = FindPackageShare("xycar_perception")
    mode = LaunchConfiguration("mode")
    view_rate_hz = LaunchConfiguration("view_rate_hz")
    show_window = LaunchConfiguration("show_window")
    start_camera = LaunchConfiguration("start_camera")
    lidar_camera_extrinsic_yaml = LaunchConfiguration(
        "lidar_camera_extrinsic_yaml"
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("wide_camera"), "launch", "wide_camera.launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="auto"),
            DeclareLaunchArgument("view_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("show_window", default_value="true"),
            DeclareLaunchArgument("start_camera", default_value="false"),
            DeclareLaunchArgument(
                "lidar_camera_extrinsic_yaml",
                default_value=PathJoinSubstitution(
                    [
                        perception_share,
                        "config",
                        "lidar_camera_extrinsic_measured.yaml",
                    ]
                ),
                description=(
                    "laser_frame to rectified-camera extrinsic YAML; the "
                    "default uses the measured 9 cm LiDAR plane."
                ),
            ),
            camera,
            Node(
                package="my_rule",
                executable="perception_view_node",
                name="my_rule_perception_view",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [my_rule_share, "config", "perception_view.yaml"]
                    ),
                    {
                        "camera_yaml": PathJoinSubstitution(
                            [
                                perception_share,
                                "config",
                                "wide_camera_fisheye_1280x1024.yaml",
                            ]
                        ),
                        "lidar_camera_extrinsic_yaml": (
                            lidar_camera_extrinsic_yaml
                        ),
                        "mode": mode,
                        "view_rate_hz": ParameterValue(
                            view_rate_hz, value_type=float
                        ),
                        "show_window": ParameterValue(
                            show_window, value_type=bool
                        ),
                    },
                ],
            )
        ]
    )
