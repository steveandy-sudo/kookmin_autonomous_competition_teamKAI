from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_config = PathJoinSubstitution(
        [FindPackageShare("xycar_perception"), "config", "camera_perception_real.yaml"]
    )
    driver_config = PathJoinSubstitution(
        [FindPackageShare("xycar_rule_drive"), "config", "lane_rule_driver_real.yaml"]
    )
    calibration_file = PathJoinSubstitution(
        [
            FindPackageShare("xycar_perception"),
            "config",
            "wide_camera_fisheye_1280x1024.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "perception_config",
                default_value=perception_config,
                description="Real-car camera perception parameters.",
            ),
            DeclareLaunchArgument(
                "driver_config",
                default_value=driver_config,
                description="Real-car lane controller parameters.",
            ),
            DeclareLaunchArgument(
                "calib_file",
                default_value=calibration_file,
                description="Calibration for the physical wide camera.",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/wide_camera/rect/image_raw",
                description="Rectified physical wide-camera topic.",
            ),
            DeclareLaunchArgument(
                "use_compressed_image",
                default_value="false",
                description="Subscribe with sensor_msgs/CompressedImage.",
            ),
            DeclareLaunchArgument(
                "motor_topic",
                default_value="/xycar_motor",
                description="Float32MultiArray [angle, speed] motor command topic.",
            ),
            DeclareLaunchArgument(
                "shadow_motor_topic",
                default_value="/xycar_motor_shadow",
                description="Always-published command topic for dry-run inspection.",
            ),
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description="Publish to the physical motor topic only when true.",
            ),
            DeclareLaunchArgument(
                "steering_only",
                default_value="false",
                description="Publish steering while forcing propulsion speed to zero.",
            ),
            DeclareLaunchArgument(
                "speed_command",
                default_value="1.0",
                description="Initial real-car speed command; raise only after shadow checks.",
            ),
            DeclareLaunchArgument(
                "min_speed_command",
                default_value="0.8",
                description="Minimum speed command while cornering.",
            ),
            DeclareLaunchArgument(
                "angle_command_min",
                default_value="-42.0",
                description="Minimum steering command allowed by the controller.",
            ),
            DeclareLaunchArgument(
                "angle_command_max",
                default_value="42.0",
                description="Maximum steering command allowed by the controller.",
            ),
            DeclareLaunchArgument(
                "prediction_enabled",
                default_value="true",
                description="Continue briefly from the last path after camera loss.",
            ),
            DeclareLaunchArgument(
                "hold_last_path_sec",
                default_value="1.0",
                description="Maximum last-path hold duration.",
            ),
            Node(
                package="xycar_perception",
                executable="camera_perception_node",
                name="xycar_camera_perception",
                parameters=[
                    LaunchConfiguration("perception_config"),
                    {
                        "use_sim_time": False,
                        "calib_yaml": LaunchConfiguration("calib_file"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "use_compressed_image": ParameterValue(
                            LaunchConfiguration("use_compressed_image"),
                            value_type=bool,
                        ),
                    },
                ],
                output="screen",
            ),
            Node(
                package="xycar_rule_drive",
                executable="lane_rule_driver",
                name="xycar_lane_rule_driver",
                parameters=[
                    LaunchConfiguration("driver_config"),
                    {
                        "use_sim_time": False,
                        "motor_topic": LaunchConfiguration("motor_topic"),
                        "shadow_motor_topic": LaunchConfiguration("shadow_motor_topic"),
                        "drive_enabled": ParameterValue(
                            LaunchConfiguration("drive_enabled"),
                            value_type=bool,
                        ),
                        "steering_only": ParameterValue(
                            LaunchConfiguration("steering_only"),
                            value_type=bool,
                        ),
                        "speed_command": ParameterValue(
                            LaunchConfiguration("speed_command"),
                            value_type=float,
                        ),
                        "min_speed_command": ParameterValue(
                            LaunchConfiguration("min_speed_command"),
                            value_type=float,
                        ),
                        "angle_command_min": ParameterValue(
                            LaunchConfiguration("angle_command_min"),
                            value_type=float,
                        ),
                        "angle_command_max": ParameterValue(
                            LaunchConfiguration("angle_command_max"),
                            value_type=float,
                        ),
                        "prediction_enabled": ParameterValue(
                            LaunchConfiguration("prediction_enabled"),
                            value_type=bool,
                        ),
                        "hold_last_path_sec": ParameterValue(
                            LaunchConfiguration("hold_last_path_sec"),
                            value_type=float,
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
