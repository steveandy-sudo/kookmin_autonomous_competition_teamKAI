"""Launch helpers that guarantee one local-odometry publisher."""

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_bool(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def make_real_odom_nodes(context):
    """Select measured, command fallback, or externally provided odometry."""
    source = (
        LaunchConfiguration("odom_source")
        .perform(context)
        .strip()
        .lower()
    )
    if _launch_bool(context, "use_command_odom"):
        source = "command"
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"),
        value_type=bool,
    )

    nodes = []
    if _launch_bool(context, "start_native_vesc_driver"):
        nodes.append(
            Node(
                package="xycar_vesc_driver",
                executable="xycar_vesc_driver",
                name="xycar_vesc_driver",
                output="screen",
                parameters=[
                    LaunchConfiguration("vesc_driver_config"),
                    {
                        "port": LaunchConfiguration("vesc_port"),
                        "drive_enabled": ParameterValue(
                            LaunchConfiguration("vesc_drive_enabled"),
                            value_type=bool,
                        ),
                        "vesc_state_topic": LaunchConfiguration(
                            "vesc_topic"
                        ),
                        "publish_tf": False,
                        "use_sim_time": use_sim_time,
                    },
                ],
            )
        )

    if source == "external":
        return nodes
    if source == "command":
        nodes.append(
            Node(
                package="xycar_map_nav",
                executable="command_odom_node",
                name="xycar_command_odom",
                output="screen",
                parameters=[
                    LaunchConfiguration("command_odom_params_file"),
                    {
                        "use_imu_yaw": ParameterValue(
                            LaunchConfiguration("use_imu_yaw"),
                            value_type=bool,
                        ),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "use_sim_time": use_sim_time,
                    },
                ],
            )
        )
        return nodes
    if source != "vesc_imu":
        raise RuntimeError(
            "odom_source must be one of: vesc_imu, command, external"
        )

    nodes.append(
        Node(
            package="xycar_map_nav",
            executable="vesc_imu_odom_node",
            name="vesc_imu_odom",
            output="screen",
            parameters=[
                LaunchConfiguration("vesc_imu_odom_params_file"),
                {
                    "vesc_topic": LaunchConfiguration("vesc_topic"),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "use_sim_time": use_sim_time,
                },
            ],
        )
    )
    return nodes
