from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("xycar_perception")
    config_file = PathJoinSubstitution(
        [package_share, "config", "camera_perception_real.yaml"]
    )
    calib_file = PathJoinSubstitution(
        [package_share, "config", "wide_camera_fisheye_1280x1024.yaml"]
    )
    image_topic = LaunchConfiguration("image_topic")
    enable_rectify = LaunchConfiguration("enable_rectify")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/wide_camera/rect/image_raw",
                description="Rectified real camera topic, or /image_raw with enable_rectify=true.",
            ),
            DeclareLaunchArgument(
                "enable_rectify",
                default_value="false",
                description="Rectify an unrectified fisheye input using the packaged calibration.",
            ),
            Node(
                package="xycar_perception",
                executable="camera_perception_node",
                name="xycar_camera_perception",
                parameters=[
                    config_file,
                    {
                        "calib_yaml": calib_file,
                        "image_topic": image_topic,
                        "enable_rectify": ParameterValue(
                            enable_rectify, value_type=bool
                        ),
                        "use_sim_time": False,
                    },
                ],
                output="screen",
            ),
        ]
    )
