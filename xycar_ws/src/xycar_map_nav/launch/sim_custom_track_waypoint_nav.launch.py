"""Capture and follow a user-defined route in the original custom track."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_nav_share = FindPackageShare("xycar_map_nav")
    gazebo_share = FindPackageShare("xycar_gazebo_bridge")
    project_root = LaunchConfiguration("project_root")
    map_yaml = PathJoinSubstitution(
        [
            gazebo_share,
            "maps",
            "kookmin_custom_track",
            "kookmin_custom_track.yaml",
        ]
    )
    world = PathJoinSubstitution(
        [project_root, "worlds", "kookmin_xycar_track_final.sdf"]
    )
    route_yaml = PathJoinSubstitution(
        [project_root, "routes", "kookmin_custom_user_waypoints.yaml"]
    )
    gazebo_launch = PathJoinSubstitution(
        [gazebo_share, "launch", "slam_map_gazebo.launch.py"]
    )
    rviz_config = PathJoinSubstitution(
        [map_nav_share, "rviz", "custom_track_waypoint.rviz"]
    )
    params_file = PathJoinSubstitution(
        [map_nav_share, "config", "waypoint_nav_real.yaml"]
    )
    gazebo_odom_topic = "/model/xycar_ackermann/odometry"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=os.getcwd(),
                description=(
                    "Repository root containing worlds/, media/, and routes/."
                ),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description=(
                    "false=capture route without motor output; "
                    "true=follow the saved route"
                ),
            ),
            DeclareLaunchArgument(
                "drive_start_delay_sec",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "path_smoothing_data_weight",
                default_value="0.005",
            ),
            DeclareLaunchArgument(
                "path_smoothing_weight",
                default_value="0.43",
            ),
            DeclareLaunchArgument(
                "path_smoothing_iterations",
                default_value="1500",
            ),
            DeclareLaunchArgument(
                "path_smoothing_anchor_weight",
                default_value="0.09",
            ),
            DeclareLaunchArgument(
                "path_smoothing_maximum_deviation_m",
                default_value="0.22",
            ),
            DeclareLaunchArgument(
                "cruise_speed_command",
                default_value="40.0",
            ),
            DeclareLaunchArgument(
                "minimum_speed_command",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "fixed_speed_command",
                default_value="-1.0",
                description=(
                    "Non-negative values bypass curve and alignment speed "
                    "planning and publish this exact speed command."
                ),
            ),
            DeclareLaunchArgument(
                "speed_planner_mode",
                default_value="forward_backward",
                description="local or forward_backward",
            ),
            DeclareLaunchArgument(
                "speed_profile_max_accel_mps2",
                default_value="1.40",
            ),
            DeclareLaunchArgument(
                "speed_profile_max_decel_mps2",
                default_value="2.00",
            ),
            DeclareLaunchArgument(
                "speed_profile_curvature_window_m",
                default_value="0.55",
            ),
            DeclareLaunchArgument(
                "speed_profile_curvature_smoothing_points",
                default_value="2",
            ),
            DeclareLaunchArgument(
                "speed_profile_braking_preview_sec",
                default_value="0.35",
            ),
            DeclareLaunchArgument(
                "maximum_lateral_accel_mps2",
                default_value="0.54",
            ),
            DeclareLaunchArgument(
                "speed_acceleration_rate_command_per_sec",
                default_value="25.0",
            ),
            DeclareLaunchArgument(
                "speed_deceleration_rate_command_per_sec",
                default_value="40.0",
            ),
            DeclareLaunchArgument(
                "speed_alignment_cross_track_soft_m",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "speed_alignment_cross_track_hard_m",
                default_value="0.60",
            ),
            DeclareLaunchArgument(
                "speed_alignment_heading_soft_rad",
                default_value="0.15",
            ),
            DeclareLaunchArgument(
                "speed_alignment_heading_hard_rad",
                default_value="1.20",
            ),
            DeclareLaunchArgument(
                "path_heading_preview_m",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "steering_delay_sec",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "stanley_gain",
                default_value="1.20",
            ),
            DeclareLaunchArgument(
                "stanley_heading_gain",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "curvature_feedforward_gain",
                default_value="0.35",
            ),
            DeclareLaunchArgument(
                "path_curvature_preview_m",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "speed_curvature_preview_m",
                default_value="-1.0",
            ),
            DeclareLaunchArgument(
                "straight_stanley_gain",
                default_value="0.45",
            ),
            DeclareLaunchArgument(
                "straight_stanley_heading_gain",
                default_value="0.55",
            ),
            DeclareLaunchArgument(
                "straight_steering_rate_command_per_sec",
                default_value="90.0",
            ),
            DeclareLaunchArgument(
                "straight_steering_filter_sec",
                default_value="0.16",
            ),
            DeclareLaunchArgument(
                "curve_controller",
                default_value="stanley",
            ),
            DeclareLaunchArgument(
                "curve_pure_pursuit_lookahead_m",
                default_value="0.30",
            ),
            DeclareLaunchArgument(
                "curve_pure_pursuit_speed_preview_sec",
                default_value="0.12",
            ),
            DeclareLaunchArgument(
                "route_yaml",
                default_value=route_yaml,
            ),
            DeclareLaunchArgument(
                "reposition_vehicle",
                default_value="false",
                description=(
                    "Move the vehicle to an explicit route start before "
                    "enabling the controller."
                ),
            ),
            DeclareLaunchArgument("start_x", default_value="-2.725"),
            DeclareLaunchArgument("start_y", default_value="2.4456"),
            DeclareLaunchArgument(
                "start_yaw_deg",
                default_value="-173.257623",
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [
                    project_root,
                    os.pathsep,
                    EnvironmentVariable(
                        "GZ_SIM_RESOURCE_PATH",
                        default_value="",
                    ),
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "world": world,
                    "headless": LaunchConfiguration("headless"),
                    "start_coordinate_gui": "false",
                    "enable_rviz": LaunchConfiguration("enable_rviz"),
                    "rviz_config": rviz_config,
                }.items(),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="xycar_map_nav",
                        executable="static_track_map_node",
                        name="kookmin_custom_track_map",
                        parameters=[
                            {
                                "map_yaml": map_yaml,
                                "use_sim_time": True,
                            }
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=2.5,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "xycar_gazebo_bridge",
                            "xycar_sim_control",
                            "set-pose",
                            "xycar_ackermann",
                            "--x",
                            LaunchConfiguration("start_x"),
                            "--y",
                            LaunchConfiguration("start_y"),
                            "--z",
                            "0.05",
                            "--yaw-deg",
                            LaunchConfiguration("start_yaw_deg"),
                        ],
                        output="screen",
                        condition=IfCondition(
                            LaunchConfiguration("reposition_vehicle")
                        ),
                    ),
                    Node(
                        package="xycar_map_nav",
                        executable="gazebo_pose_tf_republisher",
                        name="gazebo_custom_track_world_tf",
                        parameters=[
                            {
                                "pose_topic": (
                                    "/world/kookmin_xycar_track/"
                                    "dynamic_pose/info"
                                ),
                                "map_frame": "map",
                                "base_frame": "base_footprint",
                                "pose_index": 0,
                                "use_sim_time": True,
                            }
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=4.0,
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
                                    "route_yaml"
                                ),
                                "capture_output_yaml": LaunchConfiguration(
                                    "route_yaml"
                                ),
                                "path_csv": "",
                                "ignore_map_occupancy": True,
                                "clearance_cost_weight": 0.0,
                                "path_smoothing_data_weight": ParameterValue(
                                    LaunchConfiguration(
                                        "path_smoothing_data_weight"
                                    ),
                                    value_type=float,
                                ),
                                "path_smoothing_weight": ParameterValue(
                                    LaunchConfiguration(
                                        "path_smoothing_weight"
                                    ),
                                    value_type=float,
                                ),
                                "path_smoothing_iterations": ParameterValue(
                                    LaunchConfiguration(
                                        "path_smoothing_iterations"
                                    ),
                                    value_type=int,
                                ),
                                "path_smoothing_anchor_weight": ParameterValue(
                                    LaunchConfiguration(
                                        "path_smoothing_anchor_weight"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "path_smoothing_maximum_deviation_m"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        (
                                            "path_smoothing_maximum_"
                                            "deviation_m"
                                        )
                                    ),
                                    value_type=float,
                                ),
                                "drive_enabled": ParameterValue(
                                    LaunchConfiguration("drive_enabled"),
                                    value_type=bool,
                                ),
                                "drive_start_delay_sec": ParameterValue(
                                    LaunchConfiguration(
                                        "drive_start_delay_sec"
                                    ),
                                    value_type=float,
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
                                "fixed_speed_command": ParameterValue(
                                    LaunchConfiguration(
                                        "fixed_speed_command"
                                    ),
                                    value_type=float,
                                ),
                                "speed_planner_mode": LaunchConfiguration(
                                    "speed_planner_mode"
                                ),
                                (
                                    "speed_profile_max_accel_mps2"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "speed_profile_max_accel_mps2"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "speed_profile_max_decel_mps2"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "speed_profile_max_decel_mps2"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "speed_profile_curvature_window_m"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "speed_profile_curvature_window_m"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "speed_profile_curvature_smoothing_points"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        (
                                            "speed_profile_curvature_"
                                            "smoothing_points"
                                        )
                                    ),
                                    value_type=int,
                                ),
                                (
                                    "speed_profile_braking_preview_sec"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "speed_profile_braking_preview_sec"
                                    ),
                                    value_type=float,
                                ),
                                "maximum_lateral_accel_mps2": ParameterValue(
                                    LaunchConfiguration(
                                        "maximum_lateral_accel_mps2"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "speed_alignment_cross_track_soft_m"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "speed_alignment_cross_track_soft_m"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "speed_alignment_cross_track_hard_m"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "speed_alignment_cross_track_hard_m"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "speed_alignment_heading_soft_rad"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "speed_alignment_heading_soft_rad"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "speed_alignment_heading_hard_rad"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "speed_alignment_heading_hard_rad"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "speed_acceleration_rate_command_per_sec"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "speed_acceleration_rate_command_per_sec"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "speed_deceleration_rate_command_per_sec"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "speed_deceleration_rate_command_per_sec"
                                    ),
                                    value_type=float,
                                ),
                                "path_heading_preview_m": ParameterValue(
                                    LaunchConfiguration(
                                        "path_heading_preview_m"
                                    ),
                                    value_type=float,
                                ),
                                "steering_delay_sec": ParameterValue(
                                    LaunchConfiguration(
                                        "steering_delay_sec"
                                    ),
                                    value_type=float,
                                ),
                                "stanley_gain": ParameterValue(
                                    LaunchConfiguration("stanley_gain"),
                                    value_type=float,
                                ),
                                "stanley_heading_gain": ParameterValue(
                                    LaunchConfiguration(
                                        "stanley_heading_gain"
                                    ),
                                    value_type=float,
                                ),
                                "curvature_feedforward_gain": ParameterValue(
                                    LaunchConfiguration(
                                        "curvature_feedforward_gain"
                                    ),
                                    value_type=float,
                                ),
                                "path_curvature_preview_m": ParameterValue(
                                    LaunchConfiguration(
                                        "path_curvature_preview_m"
                                    ),
                                    value_type=float,
                                ),
                                "speed_curvature_preview_m": ParameterValue(
                                    LaunchConfiguration(
                                        "speed_curvature_preview_m"
                                    ),
                                    value_type=float,
                                ),
                                "straight_stanley_gain": ParameterValue(
                                    LaunchConfiguration(
                                        "straight_stanley_gain"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "straight_stanley_heading_gain"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "straight_stanley_heading_gain"
                                    ),
                                    value_type=float,
                                ),
                                (
                                    "straight_steering_rate_command_per_sec"
                                ): ParameterValue(
                                    LaunchConfiguration(
                                        "straight_steering_rate_command_per_sec"
                                    ),
                                    value_type=float,
                                ),
                                "straight_steering_filter_sec": ParameterValue(
                                    LaunchConfiguration(
                                        "straight_steering_filter_sec"
                                    ),
                                    value_type=float,
                                ),
                                "curve_controller": LaunchConfiguration(
                                    "curve_controller"
                                ),
                                (
                                    "curve_pure_pursuit_lookahead_m"
                                ): ParameterValue(
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
                                "emergency_stop_distance_m": 0.05,
                                "dynamic_detector_required": False,
                            },
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
