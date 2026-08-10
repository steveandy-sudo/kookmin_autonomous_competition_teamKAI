from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration("image_topic")
    processed_image_topic = "/lane_seg/source_image"
    white_topic = "/lane_seg/white_boundary_mask"
    yellow_topic = "/lane_seg/yellow_centerline_mask"
    package_share = FindPackageShare("my_lane")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("my_lane"), "models", "best_512.onnx"]
                ),
            ),
            DeclareLaunchArgument(
                "image_topic", default_value="/wide_camera/rect/image_raw"
            ),
            DeclareLaunchArgument("confidence", default_value="0.20"),
            DeclareLaunchArgument("yellow_confidence", default_value="0.40"),
            DeclareLaunchArgument("iou", default_value="0.60"),
            DeclareLaunchArgument("image_size", default_value="512"),
            DeclareLaunchArgument("cpu_threads", default_value="4"),
            DeclareLaunchArgument("debug_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("display_mode", default_value="minimal"),
            Node(
                package="my_lane",
                executable="lane_seg_inference_node",
                name="lane_seg_inference",
                output="screen",
                additional_env={"OMP_NUM_THREADS": "4", "OMP_WAIT_POLICY": "PASSIVE"},
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "image_topic": image_topic,
                        "processed_image_topic": processed_image_topic,
                        "white_mask_topic": white_topic,
                        "yellow_mask_topic": yellow_topic,
                        "confidence": ParameterValue(
                            LaunchConfiguration("confidence"), value_type=float
                        ),
                        "yellow_confidence": ParameterValue(
                            LaunchConfiguration("yellow_confidence"),
                            value_type=float,
                        ),
                        "iou": ParameterValue(
                            LaunchConfiguration("iou"), value_type=float
                        ),
                        "image_size": ParameterValue(
                            LaunchConfiguration("image_size"), value_type=int
                        ),
                        "cpu_threads": ParameterValue(
                            LaunchConfiguration("cpu_threads"), value_type=int
                        ),
                        "device": "cpu",
                        "debug_rate_hz": ParameterValue(
                            LaunchConfiguration("debug_rate_hz"), value_type=float
                        ),
                    }
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "my_lane",
                    "lane_seg_unified_viewer",
                    "--controller-preview",
                    "--display-mode",
                    LaunchConfiguration("display_mode"),
                    "--show-bev-detection",
                    "--publish-debug-images",
                    "false",
                    "--image-topic",
                    processed_image_topic,
                    "--white-mask-topic",
                    white_topic,
                    "--yellow-mask-topic",
                    yellow_topic,
                    "--bev-config",
                    PathJoinSubstitution([package_share, "config", "bev_latest.json"]),
                    "--params",
                    PathJoinSubstitution(
                        [package_share, "config", "lane_seg_path_params.yaml"]
                    ),
                    "--controller-preview-config",
                    PathJoinSubstitution(
                        [
                            package_share,
                            "config",
                            "lane_path_controller_preview.yaml",
                        ]
                    ),
                    "--bag-name",
                    "live_xycar",
                ],
                output="screen",
            ),
        ]
    )
