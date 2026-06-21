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
    reject_repeating = LaunchConfiguration('stop_line_bev_reject_repeating_bands')
    repeating_min_bands = LaunchConfiguration('stop_line_bev_repeating_min_bands')
    repeating_min_gap = LaunchConfiguration('stop_line_bev_repeating_min_gap_ratio')
    reject_fragmented = LaunchConfiguration('stop_line_bev_reject_fragmented_band')
    fragment_min_runs = LaunchConfiguration('stop_line_bev_fragment_min_runs')
    fragment_max_solid = LaunchConfiguration('stop_line_bev_fragment_max_solid_run_ratio')
    fragment_col_fill = LaunchConfiguration('stop_line_bev_fragment_col_min_fill_ratio')
    detect_min_row = LaunchConfiguration('stop_line_detect_min_row_ratio')
    detect_max_distance = LaunchConfiguration('stop_line_detect_max_distance_m')
    original_min_y = LaunchConfiguration('stop_line_original_min_y_ratio')
    value_min = LaunchConfiguration('stop_line_white_value_min')
    sat_max = LaunchConfiguration('stop_line_white_sat_max')
    show_windows = LaunchConfiguration('show_windows')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_raw/front'),
        DeclareLaunchArgument('debug_topic_prefix', default_value='/track_drive/stop_line_debug'),
        DeclareLaunchArgument('stop_line_bev_src_top_ratio', default_value='0.46'),
        DeclareLaunchArgument('stop_line_bev_src_bottom_ratio', default_value='0.98'),
        DeclareLaunchArgument('stop_line_bev_src_top_half_width_ratio', default_value='0.080'),
        DeclareLaunchArgument('stop_line_bev_src_bottom_half_width_ratio', default_value='0.475'),
        DeclareLaunchArgument('stop_line_bev_front_top_ratio', default_value='0.14'),
        DeclareLaunchArgument('stop_line_bev_front_bottom_ratio', default_value='1.00'),
        DeclareLaunchArgument('stop_line_bev_min_width_ratio', default_value='0.30'),
        DeclareLaunchArgument('stop_line_bev_min_aspect_ratio', default_value='5.0'),
        DeclareLaunchArgument('stop_line_bev_min_fill_ratio', default_value='0.14'),
        DeclareLaunchArgument('stop_line_bev_min_row_run', default_value='1'),
        DeclareLaunchArgument('stop_line_bev_min_solid_run_ratio', default_value='0.52'),
        DeclareLaunchArgument('stop_line_bev_solid_col_min_fill_ratio', default_value='0.30'),
        DeclareLaunchArgument('stop_line_bev_reject_repeating_bands', default_value='true'),
        DeclareLaunchArgument('stop_line_bev_repeating_min_bands', default_value='3'),
        DeclareLaunchArgument('stop_line_bev_repeating_min_gap_ratio', default_value='0.030'),
        DeclareLaunchArgument('stop_line_bev_reject_fragmented_band', default_value='true'),
        DeclareLaunchArgument('stop_line_bev_fragment_min_runs', default_value='4'),
        DeclareLaunchArgument('stop_line_bev_fragment_max_solid_run_ratio', default_value='0.35'),
        DeclareLaunchArgument('stop_line_bev_fragment_col_min_fill_ratio', default_value='0.25'),
        DeclareLaunchArgument('stop_line_detect_min_row_ratio', default_value='0.10'),
        DeclareLaunchArgument('stop_line_detect_max_distance_m', default_value='8.50'),
        DeclareLaunchArgument('stop_line_original_min_y_ratio', default_value='0.52'),
        DeclareLaunchArgument('stop_line_white_value_min', default_value='185'),
        DeclareLaunchArgument('stop_line_white_sat_max', default_value='95'),
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
                'stop_line_bev_reject_repeating_bands': ParameterValue(reject_repeating, value_type=bool),
                'stop_line_bev_repeating_min_bands': ParameterValue(repeating_min_bands, value_type=int),
                'stop_line_bev_repeating_min_gap_ratio': ParameterValue(repeating_min_gap, value_type=float),
                'stop_line_bev_reject_fragmented_band': ParameterValue(reject_fragmented, value_type=bool),
                'stop_line_bev_fragment_min_runs': ParameterValue(fragment_min_runs, value_type=int),
                'stop_line_bev_fragment_max_solid_run_ratio': ParameterValue(fragment_max_solid, value_type=float),
                'stop_line_bev_fragment_col_min_fill_ratio': ParameterValue(fragment_col_fill, value_type=float),
                'stop_line_detect_min_row_ratio': ParameterValue(detect_min_row, value_type=float),
                'stop_line_detect_max_distance_m': ParameterValue(detect_max_distance, value_type=float),
                'stop_line_original_min_y_ratio': ParameterValue(original_min_y, value_type=float),
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
