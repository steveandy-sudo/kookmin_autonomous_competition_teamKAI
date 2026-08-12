"""Launch the read-only integrated lane, path, cone, and YOLO RViz view."""

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
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [package_share, "rviz", "integrated_drive.rviz"]
                ),
            ),
            Node(
                package="my_rule",
                executable="drive_path_visualizer",
                # Keep the historical node name so the shared YAML section
                # applies to both the cone-only and integrated launch files.
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
                name="my_rule_integrated_drive_rviz",
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
                additional_env={
                    "LIBGL_ALWAYS_SOFTWARE": "1",
                    "QT_XCB_GL_INTEGRATION": "none",
                },
            ),
        ]
    )
