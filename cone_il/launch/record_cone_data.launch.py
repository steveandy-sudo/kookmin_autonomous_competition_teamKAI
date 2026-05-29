from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dataset_dir = LaunchConfiguration('dataset_dir')

    return LaunchDescription([
        DeclareLaunchArgument('dataset_dir', default_value='~/cone_il_dataset'),

        Node(
            package='cone_il',
            executable='cone_data_recorder',
            name='cone_data_recorder',
            output='screen',
            parameters=[{
                'dataset_dir': dataset_dir,
                'image_topic': '/usb_cam/image_raw/front',
                'scan_topic': '/scan',
                'motor_topic': '/xycar_motor',
                'fallback_motor_topic': '',
                'save_rate_hz': 10.0,
                'resize_width': 160,
                'resize_height': 90,
                'roi_top_ratio': 0.45,
                'require_motion': True,
                'min_abs_speed': 0.1,
                'ignore_zero_commands': True,
                'flush_every_n': 1,
            }],
        ),
    ])
