from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("lane_bev_tools"), "config", "sim_gazebo_bev.yaml"]
    )
    calib_file = PathJoinSubstitution(
        [FindPackageShare("lane_bev_tools"), "config", "wide_camera_fisheye_1280x1024.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="lane_bev_tools",
                executable="bev_preview_homography",
                name="bev_preview_homography",
                parameters=[params_file, {"calib_yaml": calib_file}],
                output="screen",
            )
        ]
    )
