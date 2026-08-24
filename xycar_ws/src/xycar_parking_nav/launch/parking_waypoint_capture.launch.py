"""Run the real stack in shadow mode and save RViz Publish Point clicks."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("xycar_parking_nav"))
    default_waypoints = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "parking_waypoints.yaml"]
    )
    default_mission = PathJoinSubstitution(
        [EnvironmentVariable("HOME"), "parking_waypoint_mission.yaml"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("start_vesc", default_value="true"),
            DeclareLaunchArgument("start_odometry", default_value="true"),
            DeclareLaunchArgument("enable_preflight", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("closed_route", default_value="false"),
            DeclareLaunchArgument(
                "reverse_waypoint_ranges",
                default_value="",
                description=(
                    "Inclusive waypoint ranges driven in reverse, e.g. 4-5,8-9"
                ),
            ),
            DeclareLaunchArgument("waypoints_yaml", default_value=default_waypoints),
            DeclareLaunchArgument("mission_output_yaml", default_value=default_mission),
            DeclareLaunchArgument(
                "map",
                default_value=str(share / "maps" / "parking_map.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch" / "parking_real.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "drive_enabled": "false",
                    "autostart_mission": "false",
                    "start_mission_manager": "false",
                    "start_cmd_vel_adapter": "false",
                    "start_lidar": LaunchConfiguration("start_lidar"),
                    "start_imu": LaunchConfiguration("start_imu"),
                    "start_vesc": LaunchConfiguration("start_vesc"),
                    "start_odometry": LaunchConfiguration("start_odometry"),
                    "enable_preflight": LaunchConfiguration("enable_preflight"),
                    "enable_rviz": LaunchConfiguration("enable_rviz"),
                    "map": LaunchConfiguration("map"),
                }.items(),
            ),
            Node(
                package="xycar_parking_nav",
                executable="waypoint_route_manager",
                name="parking_waypoint_route",
                output="screen",
                parameters=[
                    str(share / "config" / "waypoint_route.yaml"),
                    {
                        "map_yaml": LaunchConfiguration("map"),
                        "waypoints_yaml": LaunchConfiguration("waypoints_yaml"),
                        "mission_output_yaml": LaunchConfiguration(
                            "mission_output_yaml"
                        ),
                        "reference_mission_yaml": str(
                            share / "config" / "parking_mission.yaml"
                        ),
                        "capture_enabled": True,
                        "reverse_waypoint_ranges": LaunchConfiguration(
                            "reverse_waypoint_ranges"
                        ),
                        "closed_route": ParameterValue(
                            LaunchConfiguration("closed_route"), value_type=bool
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
        ]
    )
