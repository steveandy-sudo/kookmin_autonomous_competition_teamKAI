from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_model = PathJoinSubstitution([FindPackageShare("my_rule"), "models", "best.pt"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("image_topic", default_value="/wide_camera/rect/image_raw"),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("threshold", default_value="0.35"),
            DeclareLaunchArgument("iou", default_value="0.7"),
            DeclareLaunchArgument("imgsz_height", default_value="512"),
            DeclareLaunchArgument("imgsz_width", default_value="640"),
            DeclareLaunchArgument("image_reliability", default_value="2"),
            Node(
                package="yolo_ros",
                executable="yolo_node",
                name="yolo_node",
                namespace="yolo",
                output="screen",
                parameters=[
                    {
                        "model_type": "YOLO",
                        "model": LaunchConfiguration("model"),
                        "device": LaunchConfiguration("device"),
                        "enable": True,
                        "threshold": LaunchConfiguration("threshold"),
                        "iou": LaunchConfiguration("iou"),
                        "imgsz_height": LaunchConfiguration("imgsz_height"),
                        "imgsz_width": LaunchConfiguration("imgsz_width"),
                        "half": False,
                        "max_det": 100,
                        "augment": False,
                        "agnostic_nms": False,
                        "retina_masks": False,
                        "image_reliability": LaunchConfiguration("image_reliability"),
                    }
                ],
                remappings=[("image_raw", LaunchConfiguration("image_topic"))],
            ),
        ]
    )
