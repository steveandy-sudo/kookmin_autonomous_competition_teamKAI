from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def declare(name, default, description):
    return DeclareLaunchArgument(name, default_value=str(default), description=description)


def generate_launch_description():
    args = [
        declare("output_root", "~/xycar_ws/datasets/il", "Dataset output root."),
        declare("session_name", "cone_session", "Session name suffix."),
        declare("camera_front_topic", "/usb_cam/image_raw/front", "Front camera topic."),
        declare("camera_left_topic", "/usb_cam/image_raw/left", "Left camera topic."),
        declare("camera_right_topic", "/usb_cam/image_raw/right", "Right camera topic."),
        declare("camera_rear_topic", "/usb_cam/image_raw/behind", "Rear camera topic."),
        declare("scan_topic", "/scan", "LaserScan topic."),
        declare("imu_topic", "/imu", "IMU topic."),
        declare("odom_topic", "/odom", "Odometry topic."),
        declare("motor_topic", "/xycar_motor", "Xycar motor command topic to subscribe."),
        declare("motor_msg_type", "auto", "auto, xycar, or float32_multi_array."),
        declare("mission_label_topic", "/il/mission_label", "Mission label topic."),
        declare("save_side_images", "false", "Save left/right/rear images."),
        declare("save_scan_npz", "true", "Cone profile default: save scan npz."),
        declare("max_save_rate_hz", "10.0", "Maximum sample save rate."),
        declare("image_format", "jpg", "jpg or png."),
        declare("jpeg_quality", "90", "JPEG quality."),
        declare("enable_recording_on_start", "true", "Start recording immediately."),
    ]
    node = Node(
        package="il_data_tools",
        executable="il_common_recorder",
        name="il_common_recorder_cone",
        output="screen",
        parameters=[
            {
                "output_root": LaunchConfiguration("output_root"),
                "session_name": LaunchConfiguration("session_name"),
                "dataset_profile": "cone",
                "allowed_labels": "cone_drive,recovery",
                "camera_front_topic": LaunchConfiguration("camera_front_topic"),
                "camera_left_topic": LaunchConfiguration("camera_left_topic"),
                "camera_right_topic": LaunchConfiguration("camera_right_topic"),
                "camera_rear_topic": LaunchConfiguration("camera_rear_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "motor_topic": LaunchConfiguration("motor_topic"),
                "motor_msg_type": LaunchConfiguration("motor_msg_type"),
                "mission_label_topic": LaunchConfiguration("mission_label_topic"),
                "save_front_image": True,
                "save_side_images": ParameterValue(LaunchConfiguration("save_side_images"), value_type=bool),
                "save_scan_npz": ParameterValue(LaunchConfiguration("save_scan_npz"), value_type=bool),
                "image_format": LaunchConfiguration("image_format"),
                "jpeg_quality": ParameterValue(LaunchConfiguration("jpeg_quality"), value_type=int),
                "max_save_rate_hz": ParameterValue(LaunchConfiguration("max_save_rate_hz"), value_type=float),
                "enable_recording_on_start": ParameterValue(LaunchConfiguration("enable_recording_on_start"), value_type=bool),
                "exclude_bad_data": True,
                "exclude_idle": True,
            }
        ],
    )
    return LaunchDescription(args + [node])
