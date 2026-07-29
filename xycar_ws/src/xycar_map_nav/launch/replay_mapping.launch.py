"""Re-run SLAM from recorded scan and odometry topics without motor access."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("xycar_map_nav")
    slam_params = PathJoinSubstitution(
        [
            package_share,
            "config",
            "slam_toolbox_mapping.yaml",
        ]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "real_mapping.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=slam_params,
            ),
            DeclareLaunchArgument("enable_rviz", default_value="false"),
            DeclareLaunchArgument("laser_x", default_value="0.065"),
            DeclareLaunchArgument("laser_y", default_value="0.0"),
            DeclareLaunchArgument("laser_z", default_value="0.080"),
            DeclareLaunchArgument("laser_yaw", default_value="0.0"),
            Node(
                package="xycar_map_nav",
                executable="odom_tf_republisher",
                name="odom_tf_republisher",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="replay_base_to_laser",
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
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    LaunchConfiguration("slam_params_file"),
                    {
                        "use_sim_time": True,
                        "map_file_name": "",
                        "map_start_at_dock": False,
                    },
                ],
            ),
            Node(
                package="xycar_map_nav",
                executable="occupancy_grid_to_cloud",
                name="occupancy_grid_to_cloud",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(
                    LaunchConfiguration("enable_rviz")
                ),
            ),
        ]
    )
