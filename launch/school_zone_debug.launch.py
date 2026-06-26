from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# 설명: ROS 2 launch 실행에 필요한 노드와 파라미터 구성을 만든다.
def generate_launch_description():
    camera_topic = LaunchConfiguration('camera_topic')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    src_top = LaunchConfiguration('school_zone_bev_src_top_ratio')
    src_bottom = LaunchConfiguration('school_zone_bev_src_bottom_ratio')
    top_half = LaunchConfiguration('school_zone_bev_src_top_half_width_ratio')
    bottom_half = LaunchConfiguration('school_zone_bev_src_bottom_half_width_ratio')
    left_edge = LaunchConfiguration('school_zone_bev_left_edge_max_ratio')
    right_edge = LaunchConfiguration('school_zone_bev_right_edge_min_ratio')
    min_pair = LaunchConfiguration('school_zone_bev_min_pair_row_ratio')
    min_bottom_pair = LaunchConfiguration('school_zone_bev_min_bottom_pair_row_ratio')
    min_pair_rows = LaunchConfiguration('school_zone_bev_min_pair_rows')
    min_sep = LaunchConfiguration('school_zone_bev_min_separation_ratio')
    hold_sec = LaunchConfiguration('school_zone_hold_sec')

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_raw/front'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_drive'),
                'rviz',
                'school_zone_debug.rviz',
            ]),
        ),
        DeclareLaunchArgument('school_zone_bev_src_top_ratio', default_value='0.50'),
        DeclareLaunchArgument('school_zone_bev_src_bottom_ratio', default_value='0.90'),
        DeclareLaunchArgument('school_zone_bev_src_top_half_width_ratio', default_value='0.12'),
        DeclareLaunchArgument('school_zone_bev_src_bottom_half_width_ratio', default_value='0.50'),
        DeclareLaunchArgument('school_zone_bev_left_edge_max_ratio', default_value='0.42'),
        DeclareLaunchArgument('school_zone_bev_right_edge_min_ratio', default_value='0.58'),
        DeclareLaunchArgument('school_zone_bev_min_pair_row_ratio', default_value='0.085'),
        DeclareLaunchArgument('school_zone_bev_min_bottom_pair_row_ratio', default_value='0.05'),
        DeclareLaunchArgument('school_zone_bev_min_pair_rows', default_value='4'),
        DeclareLaunchArgument('school_zone_bev_min_separation_ratio', default_value='0.38'),
        DeclareLaunchArgument('school_zone_hold_sec', default_value='1.5'),
        Node(
            package='track_drive',
            executable='school_zone_debug',
            name='school_zone_debug',
            output='screen',
            parameters=[{
                'camera_topic': camera_topic,
                'school_zone_bev_src_top_ratio': ParameterValue(src_top, value_type=float),
                'school_zone_bev_src_bottom_ratio': ParameterValue(src_bottom, value_type=float),
                'school_zone_bev_src_top_half_width_ratio': ParameterValue(top_half, value_type=float),
                'school_zone_bev_src_bottom_half_width_ratio': ParameterValue(bottom_half, value_type=float),
                'school_zone_bev_left_edge_max_ratio': ParameterValue(left_edge, value_type=float),
                'school_zone_bev_right_edge_min_ratio': ParameterValue(right_edge, value_type=float),
                'school_zone_bev_min_pair_row_ratio': ParameterValue(min_pair, value_type=float),
                'school_zone_bev_min_bottom_pair_row_ratio': ParameterValue(min_bottom_pair, value_type=float),
                'school_zone_bev_min_pair_rows': ParameterValue(min_pair_rows, value_type=int),
                'school_zone_bev_min_separation_ratio': ParameterValue(min_sep, value_type=float),
                'school_zone_hold_sec': ParameterValue(hold_sec, value_type=float),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_school_zone_debug',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
