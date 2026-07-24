from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_default = PathJoinSubstitution(
        [
            FindPackageShare("xycar_gazebo_bridge"),
            "maps",
            "slam_glass_balanced",
            "slam_glass_balanced.yaml",
        ]
    )
    waypoint_default = PathJoinSubstitution(
        [
            FindPackageShare("xycar_map_nav"),
            "config",
            "slam_glass_balanced_example_waypoints.yaml",
        ]
    )
    params_default = PathJoinSubstitution(
        [
            FindPackageShare("xycar_map_nav"),
            "config",
            "waypoint_nav_real.yaml",
        ]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_yaml", default_value=map_default),
            DeclareLaunchArgument(
                "waypoints_yaml", default_value=waypoint_default
            ),
            DeclareLaunchArgument("params_file", default_value=params_default),
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("cruise_speed_command", default_value="5.0"),
            DeclareLaunchArgument("minimum_speed_command", default_value="3.0"),
            DeclareLaunchArgument("capture_output_yaml", default_value=""),
            Node(
                package="xycar_map_nav",
                executable="waypoint_nav_node",
                name="xycar_waypoint_nav",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "map_yaml": LaunchConfiguration("map_yaml"),
                        "waypoints_yaml": LaunchConfiguration(
                            "waypoints_yaml"
                        ),
                        "capture_output_yaml": LaunchConfiguration(
                            "capture_output_yaml"
                        ),
                        "drive_enabled": ParameterValue(
                            LaunchConfiguration("drive_enabled"),
                            value_type=bool,
                        ),
                        "cruise_speed_command": ParameterValue(
                            LaunchConfiguration("cruise_speed_command"),
                            value_type=float,
                        ),
                        "minimum_speed_command": ParameterValue(
                            LaunchConfiguration("minimum_speed_command"),
                            value_type=float,
                        ),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
