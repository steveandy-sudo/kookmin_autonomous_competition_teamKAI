from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    selected_preview = PathJoinSubstitution(
        [
            FindPackageShare("my_control"),
            "launch",
            "real_lane_drive_1p5m_preview_rviz.launch.py",
        ]
    )
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "The historical 2m preview command now uses the selected "
                    "1.5m canonical profile."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(selected_preview)
            ),
        ]
    )
