from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    motor_topic = LaunchConfiguration('motor_topic')
    speed = LaunchConfiguration('speed')
    max_steer_deg = LaunchConfiguration('max_steer_deg')
    invert_steering = LaunchConfiguration('invert_steering')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='~/cone_il_model/cone_bc_scripted.pt'),
        DeclareLaunchArgument('motor_topic', default_value='/xycar_motor'),
        DeclareLaunchArgument('speed', default_value='4.0'),
        DeclareLaunchArgument('max_steer_deg', default_value='70.0'),
        DeclareLaunchArgument('invert_steering', default_value='false'),

        Node(
            package='cone_il',
            executable='cone_ai_driver',
            name='cone_ai_driver',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'image_topic': '/usb_cam/image_raw/front',
                'scan_topic': '/scan',
                'motor_topic': motor_topic,
                'control_rate_hz': 20.0,
                'speed': speed,
                'max_steer_deg': max_steer_deg,
                'invert_steering': invert_steering,
                'steer_smoothing': 0.20,
                'resize_width': 160,
                'resize_height': 90,
                'roi_top_ratio': 0.45,
                'require_orange_gate': False,
                'use_lidar_emergency_stop': True,
                'emergency_stop_distance': 0.45,
            }],
        ),
    ])
