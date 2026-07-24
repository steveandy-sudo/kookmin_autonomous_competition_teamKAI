"""Localize the real vehicle against a serialized slam_toolbox map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("xycar_map_nav")
    localization_params = PathJoinSubstitution(
        [package_share, "config", "slam_toolbox_localization.yaml"]
    )
    odom_params = PathJoinSubstitution(
        [package_share, "config", "command_odom_real.yaml"]
    )
    use_command_odom = LaunchConfiguration("use_command_odom")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pose_graph",
                description=(
                    "Serialized slam_toolbox map basename or .posegraph path"
                ),
            ),
            DeclareLaunchArgument(
                "localization_params_file",
                default_value=localization_params,
            ),
            DeclareLaunchArgument(
                "command_odom_params_file", default_value=odom_params
            ),
            DeclareLaunchArgument("use_command_odom", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("laser_x", default_value="0.0"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_z", default_value="0.02"),
            DeclareLaunchArgument("laser_yaw", default_value="0.0"),
            Node(
                package="xycar_map_nav",
                executable="command_odom_node",
                name="xycar_command_odom",
                output="screen",
                condition=IfCondition(use_command_odom),
                parameters=[
                    LaunchConfiguration("command_odom_params_file"),
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        )
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser",
                arguments=[
                    "--x",
                    LaunchConfiguration("laser_x"),
                    "--y",
                    LaunchConfiguration("laser_y"),
                    "--z",
                    LaunchConfiguration("laser_z"),
                    "--yaw",
                    LaunchConfiguration("laser_yaw"),
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "base_footprint",
                    "--child-frame-id",
                    "laser_frame",
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="localization_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    LaunchConfiguration("localization_params_file"),
                    {
                        "map_file_name": ParameterValue(
                            LaunchConfiguration("pose_graph"),
                            value_type=str,
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(
                    LaunchConfiguration("enable_rviz")
                ),
            ),
        ]
    )
