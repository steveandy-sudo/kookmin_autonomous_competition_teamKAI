"""Start real-vehicle SLAM mapping without taking motor authority."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_bool(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _make_slam_toolbox_node(context):
    pose_graph = LaunchConfiguration("pose_graph").perform(context)
    resume_from_pose = _launch_bool(context, "resume_from_pose")

    runtime_parameters = {
        "map_file_name": pose_graph,
        "map_start_at_dock": _launch_bool(context, "map_start_at_dock"),
        "loop_search_maximum_distance": float(
            LaunchConfiguration("loop_search_maximum_distance").perform(context)
        ),
        "use_sim_time": _launch_bool(context, "use_sim_time"),
    }

    if resume_from_pose:
        if not pose_graph:
            raise RuntimeError(
                "resume_from_pose requires a non-empty pose_graph"
            )
        runtime_parameters["map_start_at_dock"] = False
        runtime_parameters["map_start_pose"] = [
            float(LaunchConfiguration("map_start_pose_x").perform(context)),
            float(LaunchConfiguration("map_start_pose_y").perform(context)),
            float(LaunchConfiguration("map_start_pose_yaw").perform(context)),
        ]

    return [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                LaunchConfiguration("slam_params_file"),
                runtime_parameters,
            ],
        )
    ]


def generate_launch_description():
    package_share = FindPackageShare("xycar_map_nav")
    slam_params = PathJoinSubstitution(
        [package_share, "config", "slam_toolbox_mapping.yaml"]
    )
    odom_params = PathJoinSubstitution(
        [package_share, "config", "command_odom_real.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "real_mapping.rviz"]
    )
    use_command_odom = LaunchConfiguration("use_command_odom")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "slam_params_file", default_value=slam_params
            ),
            DeclareLaunchArgument(
                "command_odom_params_file", default_value=odom_params
            ),
            DeclareLaunchArgument("use_command_odom", default_value="true"),
            DeclareLaunchArgument("use_imu_yaw", default_value="true"),
            DeclareLaunchArgument("imu_topic", default_value="/imu"),
            DeclareLaunchArgument(
                "pose_graph",
                default_value="",
                description=(
                    "Serialized pose graph basename to resume; leave empty "
                    "for a new map"
                ),
            ),
            DeclareLaunchArgument(
                "map_start_at_dock",
                default_value="true",
                description=(
                    "Match the first resumed scan against the first map node"
                ),
            ),
            DeclareLaunchArgument(
                "resume_from_pose",
                default_value="false",
                description=(
                    "Resume a serialized graph near map_start_pose instead "
                    "of the first node; this does not repair accumulated "
                    "pose-graph drift"
                ),
            ),
            DeclareLaunchArgument("map_start_pose_x", default_value="0.0"),
            DeclareLaunchArgument("map_start_pose_y", default_value="0.0"),
            DeclareLaunchArgument("map_start_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "loop_search_maximum_distance",
                default_value="5.0",
                description=(
                    "Maximum pose-graph distance in meters for loop search; "
                    "large values can create false closures in similar "
                    "corridors"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config", default_value=rviz_config
            ),
            DeclareLaunchArgument(
                "laser_x",
                default_value="0.065",
                description=(
                    "Laser x offset from the front-wheel-center vehicle frame"
                ),
            ),
            DeclareLaunchArgument(
                "laser_y",
                default_value="0.0",
                description="Measured laser-frame y offset from base_footprint",
            ),
            DeclareLaunchArgument(
                "laser_z",
                default_value="0.080",
                description=(
                    "Laser z offset from the front-wheel-center vehicle frame"
                ),
            ),
            DeclareLaunchArgument(
                "laser_yaw",
                default_value="0.0",
                description="Measured laser yaw in radians",
            ),
            Node(
                package="xycar_map_nav",
                executable="command_odom_node",
                name="xycar_command_odom",
                output="screen",
                condition=IfCondition(use_command_odom),
                parameters=[
                    LaunchConfiguration("command_odom_params_file"),
                    {
                        "use_imu_yaw": ParameterValue(
                            LaunchConfiguration("use_imu_yaw"),
                            value_type=bool,
                        ),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        )
                    },
                ],
            ),
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
            OpaqueFunction(function=_make_slam_toolbox_node),
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
