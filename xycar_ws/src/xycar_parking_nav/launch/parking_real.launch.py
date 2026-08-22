"""Bring up the complete real-car parking stack fail-closed by default."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    parking_share = Path(get_package_share_directory("xycar_parking_nav"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    imu_share = Path(get_package_share_directory("xycar_imu"))
    lidar_share = Path(get_package_share_directory("xycar_lidar"))
    vesc_share = Path(get_package_share_directory("xycar_vesc_driver"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    drive_enabled = LaunchConfiguration("drive_enabled")
    start_lidar = LaunchConfiguration("start_lidar")
    start_imu = LaunchConfiguration("start_imu")
    start_vesc = LaunchConfiguration("start_vesc")
    start_odometry = LaunchConfiguration("start_odometry")
    enable_rviz = LaunchConfiguration("enable_rviz")

    lidar_node = LifecycleNode(
        package="xycar_lidar",
        executable="xycar_lidar_node",
        name="xycar_lidar_node",
        namespace="/",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("lidar_params")],
        condition=IfCondition(start_lidar),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description=(
                    "Enable both the final adapter and native VESC output. "
                    "Keep false for RViz/shadow validation."
                ),
            ),
            DeclareLaunchArgument("autostart_mission", default_value="false"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_imu", default_value="true"),
            DeclareLaunchArgument("start_vesc", default_value="true"),
            DeclareLaunchArgument("start_odometry", default_value="true"),
            DeclareLaunchArgument("enable_preflight", default_value="true"),
            DeclareLaunchArgument("preflight_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyMOTOR"),
            DeclareLaunchArgument("laser_x", default_value="0.065"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_z", default_value="0.080"),
            DeclareLaunchArgument("laser_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "map",
                default_value=str(parking_share / "maps" / "parking_map.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=str(parking_share / "config" / "nav2_parking.yaml"),
            ),
            DeclareLaunchArgument(
                "mission_config",
                default_value=str(parking_share / "config" / "parking_mission.yaml"),
            ),
            DeclareLaunchArgument(
                "manager_params",
                default_value=str(parking_share / "config" / "mission_manager.yaml"),
            ),
            DeclareLaunchArgument(
                "odom_params",
                default_value=str(parking_share / "config" / "vesc_imu_odom.yaml"),
            ),
            DeclareLaunchArgument(
                "adapter_params",
                default_value=str(parking_share / "config" / "cmd_vel_adapter.yaml"),
            ),
            DeclareLaunchArgument(
                "lidar_params",
                default_value=str(lidar_share / "params" / "ydlidar.yaml"),
            ),
            DeclareLaunchArgument(
                "vesc_params",
                default_value=str(vesc_share / "config" / "xycar_vesc_driver.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(imu_share / "launch" / "xycar_imu.launch.py")
                ),
                condition=IfCondition(start_imu),
            ),
            lidar_node,
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_lidar",
                output="screen",
                parameters=[
                    LaunchConfiguration("manager_params"),
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "autostart": True,
                        "node_names": ["xycar_lidar_node"],
                    }
                ],
                condition=IfCondition(start_lidar),
            ),
            Node(
                package="xycar_vesc_driver",
                executable="xycar_vesc_driver",
                name="xycar_vesc_driver",
                output="screen",
                parameters=[
                    LaunchConfiguration("vesc_params"),
                    {
                        "port": LaunchConfiguration("vesc_port"),
                        "drive_enabled": ParameterValue(
                            drive_enabled, value_type=bool
                        ),
                        "publish_tf": False,
                        "odom_topic": "/vehicle/raw_odom",
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    },
                ],
                condition=IfCondition(start_vesc),
            ),
            Node(
                package="xycar_parking_nav",
                executable="vesc_imu_odom",
                name="vesc_imu_odom",
                output="screen",
                parameters=[
                    LaunchConfiguration("odom_params"),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                condition=IfCondition(start_odometry),
            ),
            Node(
                package="xycar_parking_nav",
                executable="scan_filter",
                name="parking_scan_filter",
                output="screen",
                parameters=[
                    LaunchConfiguration("odom_params"),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="parking_base_to_laser",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2_share / "launch" / "bringup_launch.py")
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("nav2_params"),
                    "use_sim_time": use_sim_time,
                    "slam": "False",
                    "autostart": "True",
                    # Humble's bringup launch embeds use_composition in a
                    # PythonExpression.  Python boolean spelling is required.
                    "use_composition": "False",
                    "use_respawn": "False",
                }.items(),
            ),
            Node(
                package="xycar_parking_nav",
                executable="mission_manager",
                name="parking_mission_manager",
                output="screen",
                parameters=[
                    {
                        "mission_config": LaunchConfiguration("mission_config"),
                        "behavior_tree": str(
                            parking_share
                            / "behavior_trees"
                            / "ackermann_navigate_to_pose.xml"
                        ),
                        "autostart_mission": ParameterValue(
                            LaunchConfiguration("autostart_mission"),
                            value_type=bool,
                        ),
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
            Node(
                package="xycar_parking_nav",
                executable="cmd_vel_adapter",
                name="parking_cmd_vel_adapter",
                output="screen",
                parameters=[
                    LaunchConfiguration("adapter_params"),
                    {
                        "drive_enabled": ParameterValue(
                            drive_enabled, value_type=bool
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
            Node(
                package="xycar_parking_nav",
                executable="parking_preflight",
                name="parking_preflight",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "timeout_sec": ParameterValue(
                            LaunchConfiguration("preflight_timeout_sec"),
                            value_type=float,
                        ),
                        "drive_enabled": ParameterValue(
                            drive_enabled, value_type=bool
                        ),
                        "lidar_device": "/dev/ttyLIDAR",
                        "imu_device": "/dev/ttyIMU",
                        "vesc_device": LaunchConfiguration("vesc_port"),
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("enable_preflight")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="parking_rviz",
                output="screen",
                arguments=["-d", str(parking_share / "rviz" / "parking_nav.rviz")],
                condition=IfCondition(enable_rviz),
            ),
        ]
    )
