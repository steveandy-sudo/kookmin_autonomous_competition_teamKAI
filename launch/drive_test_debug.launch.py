from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('track_drive')
    camera_topic = LaunchConfiguration('camera_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    motor_topic = LaunchConfiguration('motor_topic')
    model_path = LaunchConfiguration('model_path')
    speed = LaunchConfiguration('speed')
    intersection_left_turn_enabled = LaunchConfiguration('intersection_left_turn_enabled')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_raw/front'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('motor_topic', default_value='xycar_motor'),
        DeclareLaunchArgument(
            'model_path',
            default_value=PathJoinSubstitution([
                package_share,
                'assets',
                'models',
                'cone_bc_scripted_4.pt',
            ]),
        ),
        DeclareLaunchArgument('speed', default_value='30.0'),
        DeclareLaunchArgument('intersection_left_turn_enabled', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                package_share,
                'rviz',
                'drive_test_debug.rviz',
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                package_share,
                'launch',
                'ai_direct_hybrid.launch.py',
            ])),
            launch_arguments={
                'camera_topic': camera_topic,
                'scan_topic': scan_topic,
                'motor_topic': motor_topic,
                'model_path': model_path,
                'speed': speed,
                'intersection_left_turn_enabled': intersection_left_turn_enabled,
                'publish_drive_debug_image': 'false',
                'drive_debug_image_topic': '/track_drive/drive_debug_image',
                'publish_light_debug_image': 'false',
                'drive_debug_publish_rate_hz': '4.0',
                'stop_line_update_period_sec': '0.01',
                'school_zone_update_period_sec': '0.10',
                'startup_light_check_enabled': 'true',
                'startup_light_ignore_stop_line': 'true',
                'startup_light_require_signal': 'true',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_drive_test_debug',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
