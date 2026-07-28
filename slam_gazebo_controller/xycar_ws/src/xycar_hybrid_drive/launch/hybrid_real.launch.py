from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    rule_real = _parameters(
        "xycar_rule_drive",
        "config/canonical_stanley_pursuit_real.yaml",
        "canonical_stanley_pursuit_driver",
    )
    hybrid_real = _parameters(
        "xycar_hybrid_drive",
        "config/hybrid_real.yaml",
        "xycar_hybrid_drive",
    )
    checkpoint = PathJoinSubstitution(
        [
            rl_share,
            "models",
            "final_rule_td3_bc_uncapped_avg17_20260723",
            "camera_speed_td3_bc_best.pth",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("policy_cpu_threads", default_value="4"),
            DeclareLaunchArgument(
                "checkpoint_path",
                default_value=checkpoint,
            ),
            DeclareLaunchArgument("model_speed_cap", default_value="8.0"),
            DeclareLaunchArgument("cone_speed_cap", default_value="9.5"),
            Node(
                package="xycar_hybrid_drive",
                executable="hybrid_drive_node",
                name="xycar_hybrid_drive",
                parameters=[
                    rule_base,
                    rule_real,
                    hybrid_real,
                    {
                        "use_sim_time": False,
                        "drive_enabled": ParameterValue(
                            LaunchConfiguration("drive_enabled"),
                            value_type=bool,
                        ),
                        "hybrid_device": LaunchConfiguration("device"),
                        "hybrid_policy_cpu_threads": ParameterValue(
                            LaunchConfiguration("policy_cpu_threads"),
                            value_type=int,
                        ),
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
                    },
                ],
                output="screen",
            ),
        ]
    )
