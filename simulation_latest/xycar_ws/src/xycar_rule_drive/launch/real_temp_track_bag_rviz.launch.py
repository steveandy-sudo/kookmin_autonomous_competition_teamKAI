from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    image_topic = LaunchConfiguration("image_topic")
    perception_launch_file = LaunchConfiguration(
        "perception_launch_file"
    )
    playback_rate = LaunchConfiguration("playback_rate")
    bag_start_delay = LaunchConfiguration("bag_start_delay")
    play_bag = LaunchConfiguration("play_bag")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")
    lane_segmentation_backend = LaunchConfiguration(
        "lane_segmentation_backend"
    )
    yolo_model_path = LaunchConfiguration("yolo_model_path")
    yolo_device = LaunchConfiguration("yolo_device")
    yolo_confidence = LaunchConfiguration("yolo_confidence")
    yolo_image_size = LaunchConfiguration("yolo_image_size")

    perception_launch = PathJoinSubstitution(
        [
            FindPackageShare("xycar_perception"),
            "launch",
            perception_launch_file,
        ]
    )
    driver_config = PathJoinSubstitution(
        [
            FindPackageShare("xycar_rule_drive"),
            "config",
            "lane_rule_driver_real.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path",
                default_value=(
                    "/home/as/Downloads/track_run_02_extract/"
                    "track_run_02_20260715_143636"
                ),
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            DeclareLaunchArgument(
                "perception_launch_file",
                default_value=(
                    "real_temp_track_canonical_perception.launch.py"
                ),
                description="Canonical perception profile launch file.",
            ),
            DeclareLaunchArgument("playback_rate", default_value="1.0"),
            DeclareLaunchArgument("bag_start_delay", default_value="2.0"),
            DeclareLaunchArgument("play_bag", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "lane_segmentation_backend", default_value="color"
            ),
            DeclareLaunchArgument("yolo_model_path", default_value=""),
            DeclareLaunchArgument("yolo_device", default_value="cpu"),
            DeclareLaunchArgument("yolo_confidence", default_value="0.25"),
            DeclareLaunchArgument("yolo_image_size", default_value="640"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("xycar_rule_drive"),
                        "rviz",
                        "real_lane_drive.rviz",
                    ]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(perception_launch),
                launch_arguments={
                    "image_topic": image_topic,
                    "use_compressed_image": "true",
                    "enable_rectify": "true",
                    "use_sim_time": use_sim_time,
                    "lane_segmentation_backend": (
                        lane_segmentation_backend
                    ),
                    "yolo_model_path": yolo_model_path,
                    "yolo_device": yolo_device,
                    "yolo_confidence": yolo_confidence,
                    "yolo_image_size": yolo_image_size,
                }.items(),
            ),
            Node(
                package="xycar_rule_drive",
                executable="lane_rule_driver",
                name="xycar_lane_rule_driver_bag_shadow",
                parameters=[
                    driver_config,
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "drive_enabled": False,
                        "motor_topic": "/xycar_motor_bag_disabled",
                        "shadow_motor_topic": "/xycar_motor_shadow",
                    },
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "env",
                    "-u",
                    "GTK_PATH",
                    "-u",
                    "GTK_EXE_PREFIX",
                    "-u",
                    "GIO_MODULE_DIR",
                    "-u",
                    "GTK_IM_MODULE_FILE",
                    "-u",
                    "SNAP",
                    "-u",
                    "SNAP_LIBRARY_PATH",
                    "rviz2",
                    "-d",
                    rviz_config,
                    "--ros-args",
                    "-r",
                    "__node:=rviz2_temp_track_bag",
                    "-p",
                    "use_sim_time:=true",
                ],
                output="screen",
            ),
            TimerAction(
                period=bag_start_delay,
                actions=[
                    ExecuteProcess(
                        condition=IfCondition(play_bag),
                        cmd=[
                            "ros2",
                            "bag",
                            "play",
                            bag_path,
                            "--clock",
                            "--rate",
                            playback_rate,
                            "--topics",
                            image_topic,
                        ],
                        output="screen",
                    )
                ],
            ),
            LogInfo(
                msg=(
                    "Rosbag RViz inspection is SHADOW-only; "
                    "physical motor output is disabled."
                )
            ),
        ]
    )
