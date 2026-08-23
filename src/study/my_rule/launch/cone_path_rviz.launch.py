"""Launch the read-only cone-path visualizer and its RViz layout."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("my_rule")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_rviz = LaunchConfiguration("enable_rviz")
    start_cone_node = LaunchConfiguration("start_cone_node")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument(
                "start_cone_node",
                default_value="false",
                description=(
                    "Start cone_node for raw /scan-only bag replay. Leave false "
                    "when the driving stack already publishes /my_rule/cone_path."
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "cone_path.rviz"]
                ),
            ),
            Node(
                package="my_rule",
                executable="cone_node",
                name="my_rule_cone_node",
                output="screen",
                condition=IfCondition(start_cone_node),
                parameters=[
                    PathJoinSubstitution(
                        [package_share, "config", "cone_control.yaml"]
                    ),
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        # This standalone view has no hybrid controller mode
                        # publisher, so retain its always-visible cone path.
                        "gate_paths_by_control_mode": False,
                    },
                ],
            ),
            Node(
                package="my_rule",
                executable="cone_path_visualizer",
                name="my_rule_cone_path_visualizer",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [package_share, "config", "cone_path_visualizer.yaml"]
                    ),
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        )
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="my_rule_cone_path_rviz",
                output="screen",
                condition=IfCondition(enable_rviz),
                arguments=["-d", rviz_config],
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        )
                    }
                ],
            ),
        ]
    )
