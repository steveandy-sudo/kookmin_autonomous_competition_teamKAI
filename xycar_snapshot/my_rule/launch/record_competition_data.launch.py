import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    session_name = datetime.now().strftime("kookmin_%Y%m%d_%H%M%S")
    default_output = os.path.join(os.getcwd(), "bags", session_name)
    topics = [
        "/wide_camera_mjpeg/image_raw/compressed",
        "/wide_camera/rect/camera_info",
        "/scan",
        "/xycar_ultrasonic",
        "/xycar_motor",
        "/tf",
        "/tf_static",
        "/yolo/detections",
        "/center_curve",
        "/my_rule/state",
        "/my_rule/lane_cmd",
        "/my_rule/lane_override",
        "/my_rule/cone_cmd",
        "/my_rule/cone_clusters",
        "/my_rule/cone_path",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("output", default_value=default_output),
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", LaunchConfiguration("output"), *topics],
                output="screen",
            ),
        ]
    )
