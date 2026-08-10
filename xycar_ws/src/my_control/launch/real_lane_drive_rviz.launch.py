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
    enable_rectify = LaunchConfiguration('enable_rectify')
    start_camera = LaunchConfiguration('start_camera')
    rviz_config = LaunchConfiguration('rviz_config')
    src_tl_x_ratio = LaunchConfiguration('src_tl_x_ratio')
    src_tr_x_ratio = LaunchConfiguration('src_tr_x_ratio')
    src_bl_x_ratio = LaunchConfiguration('src_bl_x_ratio')
    src_br_x_ratio = LaunchConfiguration('src_br_x_ratio')
    src_top_y_ratio = LaunchConfiguration('src_top_y_ratio')
    src_bottom_y_ratio = LaunchConfiguration('src_bottom_y_ratio')
    dst_left_ratio = LaunchConfiguration('dst_left_ratio')
    dst_right_ratio = LaunchConfiguration('dst_right_ratio')
    dst_top_y_ratio = LaunchConfiguration('dst_top_y_ratio')
    dst_bottom_y_ratio = LaunchConfiguration('dst_bottom_y_ratio')
    lateral_m_per_px = LaunchConfiguration('lateral_m_per_px')
    forward_m_per_px = LaunchConfiguration('forward_m_per_px')
    canonical_forward_range_m = LaunchConfiguration('canonical_forward_range_m')
    canonical_top_ignore_m = LaunchConfiguration('canonical_top_ignore_m')

    camera_launch = PathJoinSubstitution(
        [FindPackageShare('xycar_cam'), 'launch', 'xycar_cam.launch.py']
    )
    shadow_launch = PathJoinSubstitution(
        [
            FindPackageShare('my_control'),
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
            DeclareLaunchArgument('enable_rectify', default_value='false'),
            DeclareLaunchArgument('src_tl_x_ratio', default_value='0.442578'),
            DeclareLaunchArgument('src_tr_x_ratio', default_value='0.688281'),
            DeclareLaunchArgument('src_bl_x_ratio', default_value='0.190625'),
            DeclareLaunchArgument('src_br_x_ratio', default_value='0.919141'),
            DeclareLaunchArgument('src_top_y_ratio', default_value='0.480781'),
            DeclareLaunchArgument('src_bottom_y_ratio', default_value='0.614189'),
            DeclareLaunchArgument('dst_left_ratio', default_value='0.205714'),
            DeclareLaunchArgument('dst_right_ratio', default_value='0.794286'),
            DeclareLaunchArgument('dst_top_y_ratio', default_value='0.0'),
            DeclareLaunchArgument(
                'dst_bottom_y_ratio', default_value='0.666666667'
            ),
            DeclareLaunchArgument('lateral_m_per_px', default_value='0.0021875'),
            DeclareLaunchArgument('forward_m_per_px', default_value='0.006818182'),
            DeclareLaunchArgument(
                'canonical_forward_range_m', default_value='1.5'
            ),
            DeclareLaunchArgument('canonical_top_ignore_m', default_value='0.0'),
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
                        FindPackageShare('my_control'),
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
                    'enable_rectify': enable_rectify,
                    'src_tl_x_ratio': src_tl_x_ratio,
                    'src_tr_x_ratio': src_tr_x_ratio,
                    'src_bl_x_ratio': src_bl_x_ratio,
                    'src_br_x_ratio': src_br_x_ratio,
                    'src_top_y_ratio': src_top_y_ratio,
                    'src_bottom_y_ratio': src_bottom_y_ratio,
                    'dst_left_ratio': dst_left_ratio,
                    'dst_right_ratio': dst_right_ratio,
                    'dst_top_y_ratio': dst_top_y_ratio,
                    'dst_bottom_y_ratio': dst_bottom_y_ratio,
                    'lateral_m_per_px': lateral_m_per_px,
                    'forward_m_per_px': forward_m_per_px,
                    'canonical_forward_range_m': canonical_forward_range_m,
                    'canonical_top_ignore_m': canonical_top_ignore_m,
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
