from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    display_mode = LaunchConfiguration("display_mode")
    publish_debug_images = LaunchConfiguration("publish_debug_images")
    return LaunchDescription([
        DeclareLaunchArgument("display_mode", default_value="minimal"),
        DeclareLaunchArgument("publish_debug_images", default_value="false"),
        Node(
            package="my_lane",
            executable="lane_seg_unified_viewer",
            name="lane_seg_unified_viewer",
            output="screen",
            arguments=[
                "--controller-preview",
                "--display-mode",
                display_mode,
                "--publish-debug-images",
                publish_debug_images,
            ],
        )
    ])
