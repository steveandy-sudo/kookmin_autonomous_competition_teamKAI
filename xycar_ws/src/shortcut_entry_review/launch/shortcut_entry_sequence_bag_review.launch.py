"""Motor-free, S-gated rosbag visualization of W1/Y1 entry control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    control_launch = PathJoinSubstitution(
        [
            FindPackageShare("shortcut_entry_review"),
            "launch",
            "shortcut_entry_semantic_control.launch.py",
        ]
    )
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("shortcut_entry_review"),
            "rviz",
            "shortcut_entry_sequence_review.rviz",
        ]
    )
    processing_topic = "/shortcut/review/processing_enabled"
    detections_topic = "/shortcut/review/object_detections"
    arguments = [
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_object_detection", default_value="true"),
        DeclareLaunchArgument("show_opencv_windows", default_value="true"),
        DeclareLaunchArgument(
            "lane_model",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("xycar_perception"),
                    "models",
                    "kookmin_lane_lraspp_mbv3s_256x144.pt",
                ]
            ),
        ),
        DeclareLaunchArgument(
            "object_model",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("my_rule"),
                    "models",
                    "kookmin_objects_best_20260804.pt",
                ]
            ),
        ),
        DeclareLaunchArgument(
            "object_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("my_rule"), "config", "object_detection.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "camera_yaml",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("xycar_perception"),
                    "config",
                    "wide_camera_fisheye_1280x1024_20260708.yaml",
                ]
            ),
        ),
    ]
    semantic_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch),
        launch_arguments={
            "lane_model": LaunchConfiguration("lane_model"),
            "camera_yaml": LaunchConfiguration("camera_yaml"),
            "processing_enabled_topic": processing_topic,
            "candidate_topic": "/shortcut/review/candidate",
            "default_enabled": "false",
            "use_sim_time": "true",
            "show_opencv_windows": LaunchConfiguration("show_opencv_windows"),
        }.items(),
    )
    object_detection = Node(
        package="my_rule",
        executable="object_detection_node",
        name="my_rule_object_detection_node",
        condition=IfCondition(LaunchConfiguration("start_object_detection")),
        output="screen",
        parameters=[
            LaunchConfiguration("object_config"),
            {
                "model_path": LaunchConfiguration("object_model"),
                "camera_yaml": LaunchConfiguration("camera_yaml"),
                "max_input_age_sec": 0.0,
                "startup_signal_hsv_enabled": False,
                "detections_topic": detections_topic,
                "debug_topic": "/shortcut/review/object_debug_image",
                "startup_green_topic": "/shortcut/review/start_signal_green",
            },
        ],
    )
    left4_gate = Node(
        package="shortcut_entry_review",
        executable="left4_processing_gate",
        name="shortcut_left4_processing_gate",
        condition=IfCondition(LaunchConfiguration("start_object_detection")),
        output="screen",
        parameters=[
            {
                "detections_topic": detections_topic,
                "processing_enabled_topic": processing_topic,
                "minimum_confidence": 0.50,
                "required_visible_frames": 2,
                "required_absent_frames": 2,
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="shortcut_entry_sequence_rviz",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        output="screen",
        arguments=["-d", rviz_config],
        additional_env={
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "QT_XCB_GL_INTEGRATION": "none",
        },
    )
    return LaunchDescription(
        [*arguments, semantic_control, object_detection, left4_gate, rviz]
    )
