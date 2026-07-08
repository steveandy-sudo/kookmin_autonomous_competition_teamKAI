from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("xycar_perception"), "config", "camera_perception.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=config_file,
                description="YAML parameters for camera-based Xycar perception.",
            ),
            Node(
                package="xycar_perception",
                executable="camera_perception_node",
                name="xycar_camera_perception",
                parameters=[LaunchConfiguration("config_file")],
                output="screen",
            ),
        ]
    )
