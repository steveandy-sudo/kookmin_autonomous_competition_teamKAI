"""Run the glass-balanced SLAM map in Gazebo with coordinate controls."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("xycar_gazebo_bridge")
    default_world = PathJoinSubstitution(
        [package_share, "worlds", "slam_glass_balanced.sdf"]
    )
    bridge_launch = PathJoinSubstitution(
        [
            package_share,
            "launch",
            "xycar_gazebo_rviz.launch.py",
        ]
    )
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    start_gui = LaunchConfiguration("start_coordinate_gui")
    enable_rviz = LaunchConfiguration("enable_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Generated SLAM Gazebo SDF world",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo server without its 3D GUI",
            ),
            DeclareLaunchArgument(
                "start_coordinate_gui",
                default_value="true",
                description="Open the Tk map-click coordinate controller",
            ),
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="false",
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [
                    package_share,
                    os.pathsep,
                    EnvironmentVariable(
                        "GZ_SIM_RESOURCE_PATH",
                        default_value="",
                    ),
                ],
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", world],
                name="slam_map_gazebo",
                output="screen",
                condition=UnlessCondition(headless),
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-s", "-r", world],
                name="slam_map_gazebo_server",
                output="screen",
                condition=IfCondition(headless),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(bridge_launch),
                        launch_arguments={
                            "auto_start": "false",
                            "enable_rviz": enable_rviz,
                            "enable_legacy_perception": "false",
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "xycar_gazebo_bridge",
                            "xycar_sim_control_gui",
                        ],
                        name="slam_map_coordinate_gui",
                        output="screen",
                        condition=IfCondition(start_gui),
                    )
                ],
            ),
        ]
    )
