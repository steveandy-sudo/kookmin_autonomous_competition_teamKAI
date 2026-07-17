from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_root = LaunchConfiguration("project_root")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    enable_rviz = LaunchConfiguration("enable_rviz")
    gui_requested = PythonExpression(["'", headless, "' == 'gui'"])
    gazebo_server = ExecuteProcess(
        cmd=["bash", "-lc", ["gz sim ", headless, " ", world]],
        name="kookmin_rl_gazebo",
        output="screen",
        condition=UnlessCondition(gui_requested),
    )
    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", world],
        name="kookmin_rl_gazebo_gui",
        output="screen",
        condition=IfCondition(gui_requested),
    )
    bridge_launch = PathJoinSubstitution(
        [
            FindPackageShare("xycar_gazebo_bridge"),
            "launch",
            "xycar_gazebo_rviz.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "project_root",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("HOME"), "xycar_kookmin_gazebo_track"]
                ),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [project_root, "worlds", "kookmin_xycar_track_final.sdf"]
                ),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="-s",
                description="Use '-s' for server-only data collection or 'gui' for Gazebo GUI.",
            ),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    project_root,
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            gazebo_server,
            gazebo_gui,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
                launch_arguments={
                    "auto_start": "false",
                    "enable_rviz": enable_rviz,
                }.items(),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gazebo_server,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="Gazebo RL server exited"))
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gazebo_gui,
                    on_exit=[
                        EmitEvent(event=Shutdown(reason="Gazebo RL GUI exited"))
                    ],
                )
            ),
        ]
    )
