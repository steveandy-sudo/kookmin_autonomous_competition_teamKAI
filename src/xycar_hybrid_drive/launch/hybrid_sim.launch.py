from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _parameters(package: str, relative_path: str, node_name: str) -> dict:
    path = Path(get_package_share_directory(package)) / relative_path
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)[node_name]["ros__parameters"]


def generate_launch_description():
    rl_share = FindPackageShare("xycar_rl")
    rule_base = _parameters(
        "xycar_rule_drive",
        "config/canonical_stanley_pursuit.yaml",
        "canonical_stanley_pursuit_driver",
    )
    hybrid_sim = _parameters(
        "xycar_hybrid_drive",
        "config/hybrid_sim.yaml",
        "xycar_hybrid_drive",
    )
    project_root = LaunchConfiguration("project_root")
    checkpoint = PathJoinSubstitution(
        [
            rl_share,
            "models",
            "straight_speed25_recovery_v3_20260805",
            "camera_speed_td3_bc_best.pth",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("HOME"),
                        "xycar_kookmin_gazebo_track",
                    ]
                ),
            ),
            DeclareLaunchArgument("headless", default_value="gui"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("drive_enabled", default_value="true"),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument(
                "checkpoint_path",
                default_value=checkpoint,
            ),
            DeclareLaunchArgument("model_speed_cap", default_value="25.0"),
            DeclareLaunchArgument("cone_speed_cap", default_value="9.5"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [rl_share, "launch", "rl_sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "project_root": project_root,
                    "headless": LaunchConfiguration("headless"),
                    "enable_rviz": LaunchConfiguration("enable_rviz"),
                    "auto_start": "true",
                }.items(),
            ),
            Node(
                package="xycar_hybrid_drive",
                executable="hybrid_drive_node",
                name="xycar_hybrid_drive",
                parameters=[
                    rule_base,
                    hybrid_sim,
                    {
                        "use_sim_time": True,
                        "drive_enabled": ParameterValue(
                            LaunchConfiguration("drive_enabled"),
                            value_type=bool,
                        ),
                        "hybrid_device": LaunchConfiguration("device"),
                        "hybrid_checkpoint_path": LaunchConfiguration(
                            "checkpoint_path"
                        ),
                        "hybrid_model_speed_cap": ParameterValue(
                            LaunchConfiguration("model_speed_cap"),
                            value_type=float,
                        ),
                        "hybrid_cone_speed_cap": ParameterValue(
                            LaunchConfiguration("cone_speed_cap"),
                            value_type=float,
                        ),
                        "hybrid_sim_track_mode_override": True,
                        "hybrid_sim_track_curve_feedback_enabled": True,
                        "hybrid_sim_track_world_path": ParameterValue(
                            PathJoinSubstitution(
                                [
                                    project_root,
                                    "worlds",
                                    "kookmin_xycar_track_final.sdf",
                                ]
                            ),
                            value_type=str,
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
