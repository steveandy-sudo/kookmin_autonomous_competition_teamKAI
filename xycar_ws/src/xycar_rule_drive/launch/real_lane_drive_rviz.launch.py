from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    use_compressed_image = LaunchConfiguration('use_compressed_image')
    start_camera = LaunchConfiguration('start_camera')
    rviz_config = LaunchConfiguration('rviz_config')

    camera_launch = PathJoinSubstitution(
        [FindPackageShare('xycar_cam'), 'launch', 'xycar_cam.launch.py']
    )
    shadow_launch = PathJoinSubstitution(
        [
            FindPackageShare('xycar_rule_drive'),
            'launch',
            'real_lane_drive.launch.py',
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'image_topic',
                default_value='/wide_camera/rect/image_raw',
            ),
            DeclareLaunchArgument('use_compressed_image', default_value='false'),
            DeclareLaunchArgument(
                'start_camera',
                default_value='false',
                description=(
                    'Start xycar_cam in this launch when no camera node is running.'
                ),
            ),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('xycar_rule_drive'),
                        'rviz',
                        'real_lane_drive.rviz',
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                condition=IfCondition(start_camera),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(shadow_launch),
                launch_arguments={
                    'image_topic': image_topic,
                    'use_compressed_image': use_compressed_image,
                    'drive_enabled': 'false',
                }.items(),
            ),
            LogInfo(
                msg=(
                    'RViz lane inspection is running in SHADOW mode; '
                    'physical motor output is disabled.'
                )
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_real_lane_drive',
                arguments=['-d', rviz_config],
                output='screen',
            ),
        ]
    )
