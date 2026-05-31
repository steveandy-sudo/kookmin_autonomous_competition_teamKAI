from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_topic = LaunchConfiguration('camera_topic')
    yolo_light_model_path = LaunchConfiguration('yolo_light_model_path')
    light_debug_image_topic = LaunchConfiguration('light_debug_image_topic')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    yolo_light_input_size = LaunchConfiguration('yolo_light_input_size')
    yolo_light_conf_threshold = LaunchConfiguration('yolo_light_conf_threshold')
    yolo_stop_light_conf_threshold = LaunchConfiguration('yolo_stop_light_conf_threshold')
    yolo_red_light_period_sec = LaunchConfiguration('yolo_red_light_period_sec')

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_raw/front'),
        DeclareLaunchArgument('yolo_light_model_path', default_value='/home/xytron/model/final.onnx'),
        DeclareLaunchArgument('light_debug_image_topic', default_value='/track_drive/light_debug_image'),
        DeclareLaunchArgument('control_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('yolo_light_input_size', default_value='640'),
        DeclareLaunchArgument('yolo_light_conf_threshold', default_value='0.10'),
        DeclareLaunchArgument('yolo_stop_light_conf_threshold', default_value='0.80'),
        DeclareLaunchArgument('yolo_red_light_period_sec', default_value='0.10'),

        Node(
            package='track_drive',
            executable='track_drive',
            name='light_debug_driver',
            output='screen',
            parameters=[{
                'camera_topic': camera_topic,
                'motor_topic': '/track_drive/debug_motor',
                'control_rate_hz': ParameterValue(control_rate_hz, value_type=float),
                'publish_debug_visualization': False,
                'publish_light_debug_image': True,
                'light_debug_image_topic': light_debug_image_topic,
                'yolo_safety_enabled': True,
                'yolo_light_model_path': yolo_light_model_path,
                'yolo_light_input_size': ParameterValue(yolo_light_input_size, value_type=int),
                'yolo_light_conf_threshold': ParameterValue(yolo_light_conf_threshold, value_type=float),
                'yolo_stop_light_conf_threshold': ParameterValue(
                    yolo_stop_light_conf_threshold, value_type=float),
                'yolo_red_light_period_sec': ParameterValue(yolo_red_light_period_sec, value_type=float),
                'stop_on_red_light_enabled': True,
                'red_light_confirm_frames': 1,
                'yolo_light_class_count': 6,
                'yolo_light_class_ids': [0, 1, 2, 3, 4, 5],
                'yolo_red_light_class_ids': [4, 5],
                'yolo_go_light_class_ids': [1],
                'stop_on_person_enabled': False,
                'stop_on_vehicle_enabled': False,
                'vehicle_overtake_enabled': False,
                'vehicle_follow_enabled': False,
                'vehicle_camera_fallback_enabled': False,
                'school_zone_enabled': False,
                'hybrid_standby_enabled': True,
                'hybrid_on_obstacle_enabled': False,
            }],
        ),
    ])
