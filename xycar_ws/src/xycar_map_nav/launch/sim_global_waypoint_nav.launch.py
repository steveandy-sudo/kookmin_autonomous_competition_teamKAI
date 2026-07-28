"""Run the real-parameter global path controller in the SLAM Gazebo world."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_nav_share = FindPackageShare("xycar_map_nav")
    gazebo_share = FindPackageShare("xycar_gazebo_bridge")
    gazebo_launch = PathJoinSubstitution(
        [gazebo_share, "launch", "slam_map_gazebo.launch.py"]
    )
    params_file = PathJoinSubstitution(
        [map_nav_share, "config", "waypoint_nav_real.yaml"]
    )
    map_yaml = PathJoinSubstitution(
        [
            gazebo_share,
            "maps",
            "slam_glass_balanced",
            "slam_glass_balanced.yaml",
        ]
    )
    default_waypoints_yaml = PathJoinSubstitution(
        [
            map_nav_share,
            "config",
            "slam_glass_balanced_global_test_waypoints.yaml",
        ]
    )
    gazebo_odom_topic = "/model/xycar_ackermann/odometry"

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument(
                "waypoints_yaml",
                default_value=default_waypoints_yaml,
            ),
            DeclareLaunchArgument(
                "path_csv",
                default_value="",
                description=(
                    "Optional explicit regression path. No CSV is selected "
                    "implicitly."
                ),
            ),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description=(
                    "Safety default: publish shadow commands until a route is "
                    "explicitly selected."
                ),
            ),
            DeclareLaunchArgument(
                "cruise_speed_command", default_value="7.0"
            ),
            DeclareLaunchArgument(
                "minimum_speed_command", default_value="3.0"
            ),
            DeclareLaunchArgument(
                "emergency_stop_distance_m",
                default_value="0.05",
                description=(
                    "Simulation regression value; real vehicle keeps 0.38 m."
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "headless": LaunchConfiguration("headless"),
                    "start_coordinate_gui": "false",
                    "enable_rviz": LaunchConfiguration("enable_rviz"),
                }.items(),
            ),
            TimerAction(
                period=2.5,
                actions=[
                    Node(
                        package="xycar_map_nav",
                        executable="odom_tf_republisher",
                        name="gazebo_map_tf",
                        parameters=[
                            {
                                "odom_topic": gazebo_odom_topic,
                                "odom_frame": "map",
                                "base_frame": "base_footprint",
                                "origin_x": 8.64926958348203,
                                "origin_y": 7.420009550920498,
                                "origin_yaw": -2.852326823557933,
                                "use_sim_time": True,
                            }
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="xycar_map_nav",
                        executable="waypoint_nav_node",
                        name="xycar_waypoint_nav",
                        parameters=[
                            params_file,
                            {
                                "map_yaml": map_yaml,
                                "waypoints_yaml": LaunchConfiguration(
                                    "waypoints_yaml"
                                ),
                                "path_csv": LaunchConfiguration("path_csv"),
                                "drive_enabled": ParameterValue(
                                    LaunchConfiguration("drive_enabled"),
                                    value_type=bool,
                                ),
                                "use_sim_time": True,
                                "odom_topic": gazebo_odom_topic,
                                "cruise_speed_command": ParameterValue(
                                    LaunchConfiguration(
                                        "cruise_speed_command"
                                    ),
                                    value_type=float,
                                ),
                                "minimum_speed_command": ParameterValue(
                                    LaunchConfiguration(
                                        "minimum_speed_command"
                                    ),
                                    value_type=float,
                                ),
                                "emergency_stop_distance_m": ParameterValue(
                                    LaunchConfiguration(
                                        "emergency_stop_distance_m"
                                    ),
                                    value_type=float,
                                ),
                            },
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
