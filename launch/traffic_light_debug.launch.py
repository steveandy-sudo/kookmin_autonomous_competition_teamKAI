from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('track_drive')
    camera_topic = LaunchConfiguration('camera_topic')
    model_path = LaunchConfiguration('yolo_light_model_path')
    dnn_backend = LaunchConfiguration('yolo_dnn_backend')
    dnn_target = LaunchConfiguration('yolo_dnn_target')
    conf_threshold = LaunchConfiguration('yolo_light_conf_threshold')
    stop_threshold = LaunchConfiguration('yolo_stop_light_conf_threshold')
    left_threshold = LaunchConfiguration('yolo_left_light_conf_threshold')
    min_height = LaunchConfiguration('yolo_light_min_box_height_ratio')
    min_width = LaunchConfiguration('yolo_light_min_box_width_ratio')
    min_area = LaunchConfiguration('yolo_light_min_box_area_ratio')
    max_height = LaunchConfiguration('yolo_light_max_box_height_ratio')
    max_area = LaunchConfiguration('yolo_light_max_box_area_ratio')
    max_bottom = LaunchConfiguration('yolo_light_max_box_bottom_ratio')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/usb_cam/image_raw/front'),
        DeclareLaunchArgument(
            'yolo_light_model_path',
            default_value=PathJoinSubstitution([
                package_share,
                'assets',
                'models',
                'final.onnx',
            ]),
        ),
        DeclareLaunchArgument('yolo_dnn_backend', default_value='auto'),
        DeclareLaunchArgument('yolo_dnn_target', default_value='auto'),
        DeclareLaunchArgument('yolo_light_conf_threshold', default_value='0.35'),
        DeclareLaunchArgument('yolo_stop_light_conf_threshold', default_value='0.55'),
        DeclareLaunchArgument('yolo_left_light_conf_threshold', default_value='0.28'),
        DeclareLaunchArgument('yolo_light_min_box_height_ratio', default_value='0.025'),
        DeclareLaunchArgument('yolo_light_min_box_width_ratio', default_value='0.015'),
        DeclareLaunchArgument('yolo_light_min_box_area_ratio', default_value='0.00012'),
        DeclareLaunchArgument('yolo_light_max_box_height_ratio', default_value='0.65'),
        DeclareLaunchArgument('yolo_light_max_box_area_ratio', default_value='0.20'),
        DeclareLaunchArgument('yolo_light_max_box_bottom_ratio', default_value='0.98'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                package_share,
                'rviz',
                'traffic_light_debug.rviz',
            ]),
        ),
        Node(
            package='track_drive',
            executable='traffic_light_debug',
            name='traffic_light_debug',
            output='screen',
            parameters=[{
                'camera_topic': camera_topic,
                'yolo_light_model_path': model_path,
                'yolo_dnn_backend': dnn_backend,
                'yolo_dnn_target': dnn_target,
                'yolo_light_conf_threshold': ParameterValue(conf_threshold, value_type=float),
                'yolo_stop_light_conf_threshold': ParameterValue(stop_threshold, value_type=float),
                'yolo_left_light_conf_threshold': ParameterValue(left_threshold, value_type=float),
                'yolo_light_min_box_height_ratio': ParameterValue(min_height, value_type=float),
                'yolo_light_min_box_width_ratio': ParameterValue(min_width, value_type=float),
                'yolo_light_min_box_area_ratio': ParameterValue(min_area, value_type=float),
                'yolo_light_max_box_height_ratio': ParameterValue(max_height, value_type=float),
                'yolo_light_max_box_area_ratio': ParameterValue(max_area, value_type=float),
                'yolo_light_max_box_bottom_ratio': ParameterValue(max_bottom, value_type=float),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_traffic_light_debug',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
