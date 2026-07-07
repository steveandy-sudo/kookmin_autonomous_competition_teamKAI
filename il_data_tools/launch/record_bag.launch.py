from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def declare(name, default, description):
    return DeclareLaunchArgument(name, default_value=str(default), description=description)


def generate_launch_description():
    args = [
        declare("output_dir", "/home/xytron/xycar_ws/bags/il/run_from_launch", "rosbag2 output directory."),
        declare("camera_front_topic", "/usb_cam/image_raw/front", "Front camera topic."),
        declare("camera_left_topic", "/usb_cam/image_raw/left", "Left camera topic."),
        declare("camera_right_topic", "/usb_cam/image_raw/right", "Right camera topic."),
        declare("camera_rear_topic", "/usb_cam/image_raw/behind", "Rear camera topic."),
        declare("scan_topic", "/scan", "LaserScan topic."),
        declare("imu_topic", "/imu", "IMU topic."),
        declare("odom_topic", "/odom", "Odometry topic."),
        declare("motor_topic", "/xycar_motor", "Expert motor command topic."),
        declare("mission_label_topic", "/il/mission_label", "Mission label topic."),
    ]

    recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            LaunchConfiguration("output_dir"),
            LaunchConfiguration("camera_front_topic"),
            LaunchConfiguration("camera_left_topic"),
            LaunchConfiguration("camera_right_topic"),
            LaunchConfiguration("camera_rear_topic"),
            LaunchConfiguration("scan_topic"),
            LaunchConfiguration("imu_topic"),
            LaunchConfiguration("odom_topic"),
            LaunchConfiguration("motor_topic"),
            LaunchConfiguration("mission_label_topic"),
        ],
        output="screen",
    )

    return LaunchDescription(
        args
        + [
            LogInfo(
                msg=(
                    "record_bag.launch.py runs ros2 bag record directly. "
                    "For preflight checks, prefer scripts/record_bag.sh."
                )
            ),
            recorder,
        ]
    )
