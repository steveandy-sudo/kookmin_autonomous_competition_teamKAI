"""Launch motor-free left_4 and semantic-lane rosbag visualization."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lane_launch = PathJoinSubstitution(
        [
            FindPackageShare("lane_seg_control"),
            "launch",
            "lane_seg_lraspp_canonical_only.launch.py",
        ]
    )
    camera_yaml = LaunchConfiguration("camera_yaml")
    lane_model = LaunchConfiguration("lane_model")
    object_model = LaunchConfiguration("object_model")
    rviz_config = LaunchConfiguration("rviz_config")

    arguments = [
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
            "camera_yaml",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("xycar_perception"),
                    "config",
                    "wide_camera_fisheye_1280x1024_20260708.yaml",
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
            "rviz_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("shortcut_entry_review"),
                    "rviz",
                    "shortcut_entry_bag_review.rviz",
                ]
            ),
        ),
        DeclareLaunchArgument("start_object_detection", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
    ]

    lane_perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lane_launch),
        launch_arguments={
            "model_path": lane_model,
            "image_topic": "/wide_camera_mjpeg/image_raw/compressed",
            "use_compressed_image": "true",
            "enable_rectify": "true",
            "direct_model_rectify_enabled": "true",
            "direct_model_rectify_oversample": "3",
            "camera_yaml": camera_yaml,
            "rect_balance": "0.3",
            "max_input_age_sec": "0.0",
            "direct_canonical_enabled": "true",
            "publish_intermediate_topics": "true",
            "pipeline_qos_depth": "1",
            "cpu_threads": "4",
            "max_output_rate_hz": "15.0",
            "debug_rate_hz": "15.0",
        }.items(),
    )
    isolated_lane_perception = GroupAction(
        [
            SetRemap(
                src="/lane_seg/debug_image",
                dst="/shortcut_review/lane_seg/debug_image",
            ),
            SetRemap(
                src="/lane_seg/diagnostics",
                dst="/shortcut_review/lane_seg/diagnostics",
            ),
            SetRemap(
                src="/lane_seg/source_image",
                dst="/shortcut_review/lane_seg/source_image",
            ),
            SetRemap(
                src="/lane_seg/white_boundary_mask",
                dst="/shortcut_review/lane_seg/white_boundary_mask",
            ),
            SetRemap(
                src="/lane_seg/yellow_centerline_mask",
                dst="/shortcut_review/lane_seg/yellow_centerline_mask",
            ),
            SetRemap(
                src="/perception/yolo_debug_image",
                dst="/shortcut_review/lane_seg/perception_debug_image",
            ),
            SetRemap(
                src="/perception/canonical_road_image",
                dst="/shortcut_review/canonical_road_image",
            ),
            SetRemap(
                src="/perception/canonical_white_mask",
                dst="/shortcut_review/canonical_white_mask",
            ),
            SetRemap(
                src="/perception/canonical_yellow_mask",
                dst="/shortcut_review/canonical_yellow_mask",
            ),
            SetRemap(
                src="/perception/canonical_valid_mask",
                dst="/shortcut_review/canonical_valid_mask",
            ),
            lane_perception,
        ]
    )

    object_detection = Node(
        package="my_rule",
        executable="object_detection_node",
        # Keep the production name so object_detection.yaml applies its
        # class aliases (green_3->green, red_car->car, and so on).
        name="my_rule_object_detection_node",
        condition=IfCondition(LaunchConfiguration("start_object_detection")),
        output="screen",
        parameters=[
            LaunchConfiguration("object_config"),
            {
                "model_path": object_model,
                "camera_yaml": camera_yaml,
                "max_input_age_sec": 0.0,
                "detections_topic": "/shortcut_review/object_detections",
                "debug_topic": "/shortcut_review/object_detection/debug_image",
                "startup_green_topic": "/shortcut_review/start_signal_green",
            },
        ],
    )
    entry_review = Node(
        package="shortcut_entry_review",
        executable="white_yellow_entry_review",
        name="white_yellow_entry_review",
        output="screen",
        parameters=[
            {
                "white_mask_topic": (
                    "/shortcut_review/lane_seg/white_boundary_mask"
                ),
                "yellow_mask_topic": (
                    "/shortcut_review/lane_seg/yellow_centerline_mask"
                ),
                # This launch is an offline, motor-free inspection tool. The
                # production shadow launch uses the real hybrid Bool gate.
                "default_enabled": True,
                "debug_topic": "/shortcut_review/entry/debug_image",
                "status_topic": "/shortcut_review/entry/status",
                "diagnostics_topic": (
                    "/shortcut_review/entry/diagnostics"
                ),
                "bag_start_timestamp_ns": 1786345371165916137,
            }
        ],
    )
    bev_annotation = Node(
        package="shortcut_entry_review",
        executable="bev_line_annotation",
        name="shortcut_bev_line_annotation",
        output="screen",
        parameters=[
            {
                "white_mask_topic": (
                    "/shortcut_review/lane_seg/white_boundary_mask"
                ),
                "yellow_mask_topic": (
                    "/shortcut_review/lane_seg/yellow_centerline_mask"
                ),
                "bag_start_timestamp_ns": 1786345371165916137,
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="shortcut_entry_review_rviz",
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        output="screen",
        arguments=["-d", rviz_config],
    )
    return LaunchDescription(
        [
            *arguments,
            isolated_lane_perception,
            object_detection,
            entry_review,
            bev_annotation,
            rviz,
        ]
    )
