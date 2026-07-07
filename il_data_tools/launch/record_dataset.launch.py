from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def declare(name, default, description):
    return DeclareLaunchArgument(name, default_value=str(default), description=description)


def generate_launch_description():
    args = [
        declare("output_dir", "~/xycar_ws/datasets/il", "Base dataset output directory."),
        declare("session_name", "session", "Session name suffix."),
        declare("camera_front_topic", "/usb_cam/image_raw/front", "Front camera topic."),
        declare("camera_left_topic", "/usb_cam/image_raw/left", "Left camera topic."),
        declare("camera_right_topic", "/usb_cam/image_raw/right", "Right camera topic."),
        declare("camera_rear_topic", "/usb_cam/image_raw/behind", "Rear camera topic."),
        declare("scan_topic", "/scan", "LaserScan topic."),
        declare("imu_topic", "/imu", "IMU topic."),
        declare("odom_topic", "/odom", "Odometry topic."),
        declare("motor_topic", "/xycar_motor", "Expert motor command topic."),
        declare("mission_label_topic", "/il/mission_label", "Mission label topic."),
        declare("motor_msg_type", "xycar_msgs/XycarMotor", "xycar_msgs/XycarMotor or std_msgs/Float32MultiArray."),
        declare("save_side_images", "false", "Save left/right/rear camera images."),
        declare("save_scan_npz", "true", "Save LaserScan arrays as npz files."),
        declare("save_imu", "false", "Write IMU fields to samples.csv."),
        declare("save_odom", "false", "Write odometry fields to samples.csv."),
        declare("save_rate_hz", "10.0", "Maximum saved sample rate."),
        declare("image_format", "jpg", "Image format: jpg or png."),
        declare("jpeg_quality", "90", "JPEG quality."),
        declare("enable_recording_on_start", "true", "Start recording immediately."),
        declare("write_metadata", "true", "Write one small metadata.json per session."),
        declare("write_session_readme", "false", "Write README_session.md inside each dataset session."),
        declare("metadata_update_every_n_samples", "0", "0 means only write metadata at start/shutdown."),
    ]

    recorder = Node(
        package="il_data_tools",
        executable="dataset_recorder_node",
        name="il_dataset_recorder",
        output="screen",
        parameters=[
            {
                "output_dir": LaunchConfiguration("output_dir"),
                "session_name": LaunchConfiguration("session_name"),
                "camera_front_topic": LaunchConfiguration("camera_front_topic"),
                "camera_left_topic": LaunchConfiguration("camera_left_topic"),
                "camera_right_topic": LaunchConfiguration("camera_right_topic"),
                "camera_rear_topic": LaunchConfiguration("camera_rear_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "motor_topic": LaunchConfiguration("motor_topic"),
                "mission_label_topic": LaunchConfiguration("mission_label_topic"),
                "motor_msg_type": LaunchConfiguration("motor_msg_type"),
                "save_side_images": ParameterValue(
                    LaunchConfiguration("save_side_images"), value_type=bool
                ),
                "save_scan_npz": ParameterValue(
                    LaunchConfiguration("save_scan_npz"), value_type=bool
                ),
                "save_imu": ParameterValue(LaunchConfiguration("save_imu"), value_type=bool),
                "save_odom": ParameterValue(LaunchConfiguration("save_odom"), value_type=bool),
                "max_save_rate_hz": ParameterValue(
                    LaunchConfiguration("save_rate_hz"), value_type=float
                ),
                "image_format": LaunchConfiguration("image_format"),
                "jpeg_quality": ParameterValue(
                    LaunchConfiguration("jpeg_quality"), value_type=int
                ),
                "enable_recording_on_start": ParameterValue(
                    LaunchConfiguration("enable_recording_on_start"), value_type=bool
                ),
                "write_metadata": ParameterValue(
                    LaunchConfiguration("write_metadata"), value_type=bool
                ),
                "write_session_readme": ParameterValue(
                    LaunchConfiguration("write_session_readme"), value_type=bool
                ),
                "metadata_update_every_n_samples": ParameterValue(
                    LaunchConfiguration("metadata_update_every_n_samples"),
                    value_type=int,
                ),
            }
        ],
    )

    return LaunchDescription(args + [recorder])
