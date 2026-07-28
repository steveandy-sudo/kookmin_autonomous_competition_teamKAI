from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    route_localizer_params_default = PathJoinSubstitution(
        [
            FindPackageShare("xycar_map_nav"),
            "config",
            "route_scan_localizer.yaml",
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
            DeclareLaunchArgument(
                "drive_start_delay_sec",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "initialize_pose_from_first_waypoint",
                default_value="false",
                description=(
                    "Set localization to the first waypoint and its outgoing "
                    "heading; use only when the vehicle is physically there"
                ),
            ),
            DeclareLaunchArgument(
                "auto_localize_on_route",
                default_value="false",
                description=(
                    "Match LiDAR against route candidates and gate all drive "
                    "output until a unique, stable pose is found"
                ),
            ),
            DeclareLaunchArgument(
                "route_localizer_params_file",
                default_value=route_localizer_params_default,
            ),
            DeclareLaunchArgument(
                "route_localization_scan_topic",
                default_value="/slam/scan_filtered",
            ),
            DeclareLaunchArgument("closed_route", default_value="true"),
            DeclareLaunchArgument("laser_x", default_value="0.065"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_yaw", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("cruise_speed_command", default_value="5.0"),
            DeclareLaunchArgument("minimum_speed_command", default_value="3.0"),
            DeclareLaunchArgument("fixed_speed_command", default_value="-1.0"),
            DeclareLaunchArgument(
                "speed_planner_mode",
                default_value="local",
                description="local or forward_backward",
            ),
            DeclareLaunchArgument(
                "curve_controller",
                default_value="stanley",
                description="stanley or pure_pursuit on curved segments",
            ),
            DeclareLaunchArgument(
                "curve_pure_pursuit_lookahead_m",
                default_value="0.30",
            ),
            DeclareLaunchArgument(
                "curve_pure_pursuit_speed_preview_sec",
                default_value="0.12",
            ),
            DeclareLaunchArgument("capture_output_yaml", default_value=""),
            Node(
                package="xycar_map_nav",
                executable="route_start_pose_publisher",
                name="route_start_pose_publisher",
                output="screen",
                parameters=[
                    {
                        "waypoints_yaml": LaunchConfiguration(
                            "waypoints_yaml"
                        )
                    }
                ],
                condition=IfCondition(
                    LaunchConfiguration(
                        "initialize_pose_from_first_waypoint"
                    )
                ),
            ),
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
                        "closed_route": ParameterValue(
                            LaunchConfiguration("closed_route"),
                            value_type=bool,
                        ),
                        "require_route_localization": ParameterValue(
                            LaunchConfiguration("auto_localize_on_route"),
                            value_type=bool,
                        ),
                        "drive_start_delay_sec": ParameterValue(
                            LaunchConfiguration("drive_start_delay_sec"),
                            value_type=float,
                        ),
                        "cruise_speed_command": ParameterValue(
                            LaunchConfiguration("cruise_speed_command"),
                            value_type=float,
                        ),
                        "minimum_speed_command": ParameterValue(
                            LaunchConfiguration("minimum_speed_command"),
                            value_type=float,
                        ),
                        "fixed_speed_command": ParameterValue(
                            LaunchConfiguration("fixed_speed_command"),
                            value_type=float,
                        ),
                        "speed_planner_mode": LaunchConfiguration(
                            "speed_planner_mode"
                        ),
                        "curve_controller": LaunchConfiguration(
                            "curve_controller"
                        ),
                        "curve_pure_pursuit_lookahead_m": ParameterValue(
                            LaunchConfiguration(
                                "curve_pure_pursuit_lookahead_m"
                            ),
                            value_type=float,
                        ),
                        (
                            "curve_pure_pursuit_speed_preview_sec"
                        ): ParameterValue(
                            LaunchConfiguration(
                                "curve_pure_pursuit_speed_preview_sec"
                            ),
                            value_type=float,
                        ),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="xycar_map_nav",
                executable="route_scan_localizer",
                name="route_scan_localizer",
                output="screen",
                parameters=[
                    LaunchConfiguration("route_localizer_params_file"),
                    {
                        "map_yaml": LaunchConfiguration("map_yaml"),
                        "scan_topic": LaunchConfiguration(
                            "route_localization_scan_topic"
                        ),
                        "closed_route": ParameterValue(
                            LaunchConfiguration("closed_route"),
                            value_type=bool,
                        ),
                        "laser_x": ParameterValue(
                            LaunchConfiguration("laser_x"),
                            value_type=float,
                        ),
                        "laser_y": ParameterValue(
                            LaunchConfiguration("laser_y"),
                            value_type=float,
                        ),
                        "laser_yaw": ParameterValue(
                            LaunchConfiguration("laser_yaw"),
                            value_type=float,
                        ),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                    },
                ],
                condition=IfCondition(
                    LaunchConfiguration("auto_localize_on_route")
                ),
            ),
        ]
    )
