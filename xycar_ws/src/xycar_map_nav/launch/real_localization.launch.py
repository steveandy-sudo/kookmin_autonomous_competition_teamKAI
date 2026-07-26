"""Localize the real vehicle against a serialized slam_toolbox map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from xycar_map_nav.real_odom_launch import make_real_odom_nodes


def generate_launch_description():
    package_share = FindPackageShare("xycar_map_nav")
    localization_params = PathJoinSubstitution(
        [package_share, "config", "slam_toolbox_localization.yaml"]
    )
    odom_params = PathJoinSubstitution(
        [package_share, "config", "command_odom_real.yaml"]
    )
    vesc_imu_odom_params = PathJoinSubstitution(
        [package_share, "config", "vesc_imu_odom_real.yaml"]
    )
    vesc_driver_config = PathJoinSubstitution(
        [
            FindPackageShare("xycar_vesc_driver"),
            "config",
            "xycar_vesc_driver.yaml",
        ]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "real_mapping.rviz"]
    )
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
            DeclareLaunchArgument(
                "vesc_imu_odom_params_file",
                default_value=vesc_imu_odom_params,
            ),
            DeclareLaunchArgument(
                "odom_source",
                default_value="vesc_imu",
                description="vesc_imu, command, or external",
            ),
            DeclareLaunchArgument(
                "use_command_odom",
                default_value="false",
                description=(
                    "Backward-compatible override; true selects command odom"
                ),
            ),
            DeclareLaunchArgument(
                "vesc_driver_config",
                default_value=vesc_driver_config,
            ),
            DeclareLaunchArgument(
                "start_native_vesc_driver",
                default_value="true",
                description=(
                    "Start the native ROS 2 owner of /dev/ttyMOTOR; set false "
                    "when one native driver is already running"
                ),
            ),
            DeclareLaunchArgument(
                "vesc_port",
                default_value="/dev/ttyMOTOR",
            ),
            DeclareLaunchArgument(
                "vesc_drive_enabled",
                default_value="false",
                description=(
                    "Explicitly allow non-zero native VESC output"
                ),
            ),
            DeclareLaunchArgument(
                "vesc_topic",
                default_value="/vehicle/vesc_state",
            ),
            DeclareLaunchArgument("use_imu_yaw", default_value="true"),
            DeclareLaunchArgument("imu_topic", default_value="/imu"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config", default_value=rviz_config
            ),
            DeclareLaunchArgument("laser_x", default_value="0.065"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_z", default_value="0.080"),
            DeclareLaunchArgument("laser_yaw", default_value="0.0"),
            OpaqueFunction(function=make_real_odom_nodes),
            Node(
                package="xycar_map_nav",
                executable="scan_filter_node",
                name="slam_scan_filter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        )
                    }
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
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(
                    LaunchConfiguration("enable_rviz")
                ),
            ),
        ]
    )
