from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# 설명: ROS 2 launch 실행에 필요한 노드와 파라미터 구성을 만든다.
def generate_launch_description():
    package_share = FindPackageShare('track_drive')
    camera_topic = LaunchConfiguration('camera_topic')
    model_path = LaunchConfiguration('yolo_light_model_path')
    dnn_backend = LaunchConfiguration('yolo_dnn_backend')
    dnn_target = LaunchConfiguration('yolo_dnn_target')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    light_conf = LaunchConfiguration('yolo_light_conf_threshold')
    stop_conf = LaunchConfiguration('yolo_stop_light_conf_threshold')
    left_conf = LaunchConfiguration('yolo_left_light_conf_threshold')
    cone_score = LaunchConfiguration('intersection_camera_cone_min_score')
    cone_left_max = LaunchConfiguration('intersection_camera_cone_left_max_ratio')
    stop_row = LaunchConfiguration('stop_line_detect_min_row_ratio')
    stop_distance = LaunchConfiguration('stop_line_detect_max_distance_m')
    max_height = LaunchConfiguration('yolo_light_max_box_height_ratio')
    max_area = LaunchConfiguration('yolo_light_max_box_area_ratio')
    max_bottom = LaunchConfiguration('yolo_light_max_box_bottom_ratio')

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
        DeclareLaunchArgument('yolo_light_max_box_height_ratio', default_value='0.65'),
        DeclareLaunchArgument('yolo_light_max_box_area_ratio', default_value='0.20'),
        DeclareLaunchArgument('yolo_light_max_box_bottom_ratio', default_value='0.98'),
        DeclareLaunchArgument('intersection_camera_cone_min_score', default_value='0.30'),
        DeclareLaunchArgument('intersection_camera_cone_left_max_ratio', default_value='0.68'),
        DeclareLaunchArgument('stop_line_detect_min_row_ratio', default_value='0.10'),
        DeclareLaunchArgument('stop_line_detect_max_distance_m', default_value='8.50'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                package_share,
                'rviz',
                'intersection_debug.rviz',
            ]),
        ),
        Node(
            package='track_drive',
            executable='intersection_debug',
            name='intersection_debug',
            output='screen',
            parameters=[{
                'camera_topic': camera_topic,
                'yolo_light_model_path': model_path,
                'yolo_dnn_backend': dnn_backend,
                'yolo_dnn_target': dnn_target,
                'yolo_light_conf_threshold': ParameterValue(light_conf, value_type=float),
                'yolo_stop_light_conf_threshold': ParameterValue(stop_conf, value_type=float),
                'yolo_left_light_conf_threshold': ParameterValue(left_conf, value_type=float),
                'yolo_light_max_box_height_ratio': ParameterValue(max_height, value_type=float),
                'yolo_light_max_box_area_ratio': ParameterValue(max_area, value_type=float),
                'yolo_light_max_box_bottom_ratio': ParameterValue(max_bottom, value_type=float),
                'intersection_camera_cone_min_score': ParameterValue(cone_score, value_type=float),
                'intersection_camera_cone_left_max_ratio': ParameterValue(cone_left_max, value_type=float),
                'stop_line_detect_min_row_ratio': ParameterValue(stop_row, value_type=float),
                'stop_line_detect_max_distance_m': ParameterValue(stop_distance, value_type=float),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_intersection_debug',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
