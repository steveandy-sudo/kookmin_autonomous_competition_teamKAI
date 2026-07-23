import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config_file = os.path.join(
        get_package_share_directory("my_rule"),
        "config",
        "rule_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config_file),
            DeclareLaunchArgument("image_topic", default_value="/wide_camera/rect/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/wide_camera/rect/camera_info"),
            DeclareLaunchArgument("detection_topic", default_value="/yolo/detections"),
            DeclareLaunchArgument("use_lidar_camera_projection", default_value="false"),
            DeclareLaunchArgument("lidar_camera_extrinsic_yaml", default_value=""),
            Node(
                package="my_rule",
                executable="centerline_tracer",
                name="my_rule_centerline_tracer",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "detection_topic": LaunchConfiguration("detection_topic"),
                    },
                ],
            ),
            Node(
                package="my_rule",
                executable="lane_node",
                name="my_rule_lane_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "detection_topic": LaunchConfiguration("detection_topic"),
                    },
                ],
            ),
            Node(
                package="my_rule",
                executable="cone_node",
                name="my_rule_cone_node",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
            Node(
                package="my_rule",
                executable="rule_driver",
                name="my_rule_driver",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "detection_topic": LaunchConfiguration("detection_topic"),
                        "use_lidar_camera_projection": ParameterValue(
                            LaunchConfiguration("use_lidar_camera_projection"),
                            value_type=bool,
                        ),
                        "lidar_camera_extrinsic_yaml": LaunchConfiguration(
                            "lidar_camera_extrinsic_yaml"
                        ),
                    },
                ],
            ),
        ]
    )
