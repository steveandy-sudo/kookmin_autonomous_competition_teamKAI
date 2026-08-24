"""LiDAR-route localization followed by the captured Nav2 waypoint mission."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("xycar_parking_nav"))
    # The repository contains the last field-validated route.  Operators may
    # still override both paths with $HOME files while capturing new points.
    default_waypoints = str(share / "config" / "parking_waypoints.yaml")
    default_mission = str(
        share / "config" / "parking_waypoint_mission.yaml"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("start_vesc", default_value="true"),
            DeclareLaunchArgument("start_odometry", default_value="true"),
            DeclareLaunchArgument("enable_preflight", default_value="true"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("closed_route", default_value="false"),
            DeclareLaunchArgument("waypoints_yaml", default_value=default_waypoints),
            DeclareLaunchArgument("mission_config", default_value=default_mission),
            DeclareLaunchArgument("laser_x", default_value="0.065"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_yaw", default_value="0.0"),
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
                    "drive_enabled": LaunchConfiguration("drive_enabled"),
                    "autostart_mission": "false",
                    "initial_pose_publish_count": "0",
                    "require_route_localization": "true",
                    "start_lidar": LaunchConfiguration("start_lidar"),
                    "start_imu": LaunchConfiguration("start_imu"),
                    "start_vesc": LaunchConfiguration("start_vesc"),
                    "start_odometry": LaunchConfiguration("start_odometry"),
                    "enable_preflight": LaunchConfiguration("enable_preflight"),
                    "enable_rviz": LaunchConfiguration("enable_rviz"),
                    "map": LaunchConfiguration("map"),
                    "mission_config": LaunchConfiguration("mission_config"),
                    "laser_x": LaunchConfiguration("laser_x"),
                    "laser_y": LaunchConfiguration("laser_y"),
                    "laser_yaw": LaunchConfiguration("laser_yaw"),
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
                        "mission_output_yaml": LaunchConfiguration("mission_config"),
                        "reference_mission_yaml": str(
                            share / "config" / "parking_mission.yaml"
                        ),
                        "capture_enabled": False,
                        "closed_route": ParameterValue(
                            LaunchConfiguration("closed_route"), value_type=bool
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
            Node(
                package="xycar_parking_nav",
                executable="route_scan_localizer",
                name="parking_route_scan_localizer",
                output="screen",
                parameters=[
                    str(share / "config" / "route_scan_localizer.yaml"),
                    {
                        "map_yaml": LaunchConfiguration("map"),
                        "closed_route": ParameterValue(
                            LaunchConfiguration("closed_route"), value_type=bool
                        ),
                        "laser_x": ParameterValue(
                            LaunchConfiguration("laser_x"), value_type=float
                        ),
                        "laser_y": ParameterValue(
                            LaunchConfiguration("laser_y"), value_type=float
                        ),
                        "laser_yaw": ParameterValue(
                            LaunchConfiguration("laser_yaw"), value_type=float
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
        ]
    )
