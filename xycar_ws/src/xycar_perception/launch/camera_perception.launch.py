from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("xycar_perception"), "config", "camera_perception.yaml"]
    )
    calib_file = PathJoinSubstitution(
        [FindPackageShare("xycar_perception"), "config", "wide_camera_fisheye_1280x1024.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=config_file,
                description="YAML parameters for camera-based Xycar perception.",
            ),
            DeclareLaunchArgument(
                "calib_file",
                default_value=calib_file,
                description="Fisheye calibration YAML for the 1280x1024 real wide camera model.",
            ),
            Node(
                package="xycar_perception",
                executable="camera_perception_node",
                name="xycar_camera_perception",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"calib_yaml": LaunchConfiguration("calib_file")},
                ],
                output="screen",
            ),
        ]
    )
