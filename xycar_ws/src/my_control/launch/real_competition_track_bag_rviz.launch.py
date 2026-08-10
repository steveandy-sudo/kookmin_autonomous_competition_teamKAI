from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    playback_rate = LaunchConfiguration("playback_rate")
    bag_rviz_launch = PathJoinSubstitution(
        [
            FindPackageShare("my_control"),
            "launch",
            "real_temp_track_bag_rviz.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value=(
                    "/home/as/kookmin_ros_bag/"
                    "fusion_01_20260714_144924_330669 (copy)"
                ),
                description="Kookmin competition-track rosbag directory.",
            ),
            DeclareLaunchArgument("playback_rate", default_value="1.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bag_rviz_launch),
                launch_arguments={
                    "bag_path": bag_path,
                    "playback_rate": playback_rate,
                    "perception_launch_file": (
                        "real_canonical_perception.launch.py"
                    ),
                }.items(),
            ),
        ]
    )
