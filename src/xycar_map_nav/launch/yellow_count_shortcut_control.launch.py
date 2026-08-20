"""Yellow-count shortcut perception and time-bounded forced-left candidate."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_launch = PathJoinSubstitution(
        [
            FindPackageShare("xycar_map_nav"),
            "launch",
            "yellow_count_semantic_review.launch.py",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "processing_enabled_topic",
                default_value="/hybrid/shortcut_processing_enabled",
            ),
            DeclareLaunchArgument(
                "rule_command_topic", default_value="/hybrid/rule_candidate"
            ),
            DeclareLaunchArgument(
                "candidate_topic", default_value="/hybrid/shortcut_candidate"
            ),
            DeclareLaunchArgument(
                "ready_topic", default_value="/shortcut/entry/ready"
            ),
            DeclareLaunchArgument(
                "lane_model",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("xycar_perception"),
                        "models",
                        "kookmin_lane_lraspp_mbv3s_256x144.pt",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "camera_yaml",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("xycar_perception"),
                        "config",
                        "wide_camera_fisheye_1280x1024_20260708.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "forced_steering_command", default_value="-42.0"
            ),
            DeclareLaunchArgument("forced_steering_sec", default_value="0.7"),
            DeclareLaunchArgument("entry_speed_command", default_value="9.0"),
            DeclareLaunchArgument("yellow_pass_target", default_value="1"),
            DeclareLaunchArgument("yellow_visible_frames", default_value="2"),
            DeclareLaunchArgument("yellow_absent_frames", default_value="1"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch),
                launch_arguments={
                    "source_topic": LaunchConfiguration("source_topic"),
                    "processing_enabled_topic": LaunchConfiguration(
                        "processing_enabled_topic"
                    ),
                    "lane_model": LaunchConfiguration("lane_model"),
                    "camera_yaml": LaunchConfiguration("camera_yaml"),
                }.items(),
            ),
            Node(
                package="xycar_map_nav",
                executable="yellow_count_shortcut_controller",
                name="yellow_count_shortcut_controller",
                output="screen",
                parameters=[
                    {
                        "processing_enabled_topic": LaunchConfiguration(
                            "processing_enabled_topic"
                        ),
                        "yellow_mask_topic": (
                            "/yellow_count/lraspp/yellow_mask"
                        ),
                        "rule_command_topic": LaunchConfiguration(
                            "rule_command_topic"
                        ),
                        "candidate_topic": LaunchConfiguration(
                            "candidate_topic"
                        ),
                        "ready_topic": LaunchConfiguration("ready_topic"),
                        "forced_steering_command": ParameterValue(
                            LaunchConfiguration("forced_steering_command"),
                            value_type=float,
                        ),
                        "forced_steering_sec": ParameterValue(
                            LaunchConfiguration("forced_steering_sec"),
                            value_type=float,
                        ),
                        "entry_speed_command": ParameterValue(
                            LaunchConfiguration("entry_speed_command"),
                            value_type=float,
                        ),
                        "yellow_pass_target": ParameterValue(
                            LaunchConfiguration("yellow_pass_target"),
                            value_type=int,
                        ),
                        "yellow_visible_frames": ParameterValue(
                            LaunchConfiguration("yellow_visible_frames"),
                            value_type=int,
                        ),
                        "yellow_absent_frames": ParameterValue(
                            LaunchConfiguration("yellow_absent_frames"),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
