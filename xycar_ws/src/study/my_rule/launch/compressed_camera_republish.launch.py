"""Launch one shared JPEG decoder/rectifier for camera perception nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "input_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "lane_output_topic",
                default_value="/wide_camera/lane_rect/image_raw",
            ),
            DeclareLaunchArgument(
                "object_output_topic",
                default_value="/wide_camera/object_rect/image_raw",
            ),
            DeclareLaunchArgument("enable_rectify", default_value="true"),
            DeclareLaunchArgument("lane_rectify_width", default_value="1024"),
            DeclareLaunchArgument("lane_rectify_height", default_value="576"),
            DeclareLaunchArgument("lane_output_width", default_value="512"),
            DeclareLaunchArgument("lane_output_height", default_value="288"),
            DeclareLaunchArgument("lane_output_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("object_output_width", default_value="640"),
            DeclareLaunchArgument("object_output_height", default_value="512"),
            DeclareLaunchArgument("object_output_rate_hz", default_value="5.0"),
            Node(
                package="my_rule",
                executable="compressed_camera_republisher",
                name="my_rule_compressed_camera_republisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "lane_output_topic": LaunchConfiguration(
                            "lane_output_topic"
                        ),
                        "object_output_topic": LaunchConfiguration(
                            "object_output_topic"
                        ),
                        "enable_rectify": ParameterValue(
                            LaunchConfiguration("enable_rectify"),
                            value_type=bool,
                        ),
                        "lane_rectify_width": ParameterValue(
                            LaunchConfiguration("lane_rectify_width"),
                            value_type=int,
                        ),
                        "lane_rectify_height": ParameterValue(
                            LaunchConfiguration("lane_rectify_height"),
                            value_type=int,
                        ),
                        "lane_output_width": ParameterValue(
                            LaunchConfiguration("lane_output_width"),
                            value_type=int,
                        ),
                        "lane_output_height": ParameterValue(
                            LaunchConfiguration("lane_output_height"),
                            value_type=int,
                        ),
                        "lane_output_rate_hz": ParameterValue(
                            LaunchConfiguration("lane_output_rate_hz"),
                            value_type=float,
                        ),
                        "object_output_width": ParameterValue(
                            LaunchConfiguration("object_output_width"),
                            value_type=int,
                        ),
                        "object_output_height": ParameterValue(
                            LaunchConfiguration("object_output_height"),
                            value_type=int,
                        ),
                        "object_output_rate_hz": ParameterValue(
                            LaunchConfiguration("object_output_rate_hz"),
                            value_type=float,
                        ),
                        "camera_yaml": PathJoinSubstitution(
                            [
                                FindPackageShare("xycar_perception"),
                                "config",
                                "wide_camera_fisheye_1280x1024_20260708.yaml",
                            ]
                        ),
                    }
                ],
            ),
        ]
    )
