from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_topic = LaunchConfiguration('camera_topic')
    debug_topic_prefix = LaunchConfiguration('debug_topic_prefix')
    src_top = LaunchConfiguration('stop_line_bev_src_top_ratio')
    src_bottom = LaunchConfiguration('stop_line_bev_src_bottom_ratio')
    src_top_half = LaunchConfiguration('stop_line_bev_src_top_half_width_ratio')
    src_bottom_half = LaunchConfiguration('stop_line_bev_src_bottom_half_width_ratio')
    front_top = LaunchConfiguration('stop_line_bev_front_top_ratio')
    front_bottom = LaunchConfiguration('stop_line_bev_front_bottom_ratio')
    min_width = LaunchConfiguration('stop_line_bev_min_width_ratio')
    min_aspect = LaunchConfiguration('stop_line_bev_min_aspect_ratio')
    min_fill = LaunchConfiguration('stop_line_bev_min_fill_ratio')
    min_row_run = LaunchConfiguration('stop_line_bev_min_row_run')
    min_solid_run = LaunchConfiguration('stop_line_bev_min_solid_run_ratio')
    solid_col_fill = LaunchConfiguration('stop_line_bev_solid_col_min_fill_ratio')
    detect_min_row = LaunchConfiguration('stop_line_detect_min_row_ratio')
    detect_max_distance = LaunchConfiguration('stop_line_detect_max_distance_m')
    value_min = LaunchConfiguration('stop_line_white_value_min')
    sat_max = LaunchConfiguration('stop_line_white_sat_max')
    show_windows = LaunchConfiguration('show_windows')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_raw/front'),
        DeclareLaunchArgument('debug_topic_prefix', default_value='/track_drive/stop_line_debug'),
        DeclareLaunchArgument('stop_line_bev_src_top_ratio', default_value='0.50'),
        DeclareLaunchArgument('stop_line_bev_src_bottom_ratio', default_value='0.80'),
        DeclareLaunchArgument('stop_line_bev_src_top_half_width_ratio', default_value='0.075'),
        DeclareLaunchArgument('stop_line_bev_src_bottom_half_width_ratio', default_value='0.475'),
        DeclareLaunchArgument('stop_line_bev_front_top_ratio', default_value='0.25'),
        DeclareLaunchArgument('stop_line_bev_front_bottom_ratio', default_value='1.00'),
        DeclareLaunchArgument('stop_line_bev_min_width_ratio', default_value='0.40'),
        DeclareLaunchArgument('stop_line_bev_min_aspect_ratio', default_value='5.0'),
        DeclareLaunchArgument('stop_line_bev_min_fill_ratio', default_value='0.22'),
        DeclareLaunchArgument('stop_line_bev_min_row_run', default_value='3'),
        DeclareLaunchArgument('stop_line_bev_min_solid_run_ratio', default_value='0.58'),
        DeclareLaunchArgument('stop_line_bev_solid_col_min_fill_ratio', default_value='0.43'),
        DeclareLaunchArgument('stop_line_detect_min_row_ratio', default_value='0.25'),
        DeclareLaunchArgument('stop_line_detect_max_distance_m', default_value='5.25'),
        DeclareLaunchArgument('stop_line_white_value_min', default_value='200'),
        DeclareLaunchArgument('stop_line_white_sat_max', default_value='80'),
        DeclareLaunchArgument('show_windows', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_drive'),
                'rviz',
                'stop_line_bev_debug.rviz',
            ]),
        ),
        Node(
            package='track_drive',
            executable='stop_line_bev_debug',
            name='stop_line_bev_debug',
            output='screen',
            parameters=[{
                'camera_topic': camera_topic,
                'debug_topic_prefix': debug_topic_prefix,
                'stop_line_bev_src_top_ratio': ParameterValue(src_top, value_type=float),
                'stop_line_bev_src_bottom_ratio': ParameterValue(src_bottom, value_type=float),
                'stop_line_bev_src_top_half_width_ratio': ParameterValue(src_top_half, value_type=float),
                'stop_line_bev_src_bottom_half_width_ratio': ParameterValue(src_bottom_half, value_type=float),
                'stop_line_bev_front_top_ratio': ParameterValue(front_top, value_type=float),
                'stop_line_bev_front_bottom_ratio': ParameterValue(front_bottom, value_type=float),
                'stop_line_bev_min_width_ratio': ParameterValue(min_width, value_type=float),
                'stop_line_bev_min_aspect_ratio': ParameterValue(min_aspect, value_type=float),
                'stop_line_bev_min_fill_ratio': ParameterValue(min_fill, value_type=float),
                'stop_line_bev_min_row_run': ParameterValue(min_row_run, value_type=int),
                'stop_line_bev_min_solid_run_ratio': ParameterValue(min_solid_run, value_type=float),
                'stop_line_bev_solid_col_min_fill_ratio': ParameterValue(solid_col_fill, value_type=float),
                'stop_line_detect_min_row_ratio': ParameterValue(detect_min_row, value_type=float),
                'stop_line_detect_max_distance_m': ParameterValue(detect_max_distance, value_type=float),
                'stop_line_white_value_min': ParameterValue(value_min, value_type=int),
                'stop_line_white_sat_max': ParameterValue(sat_max, value_type=int),
                'show_windows': ParameterValue(show_windows, value_type=bool),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_stop_line_debug',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
