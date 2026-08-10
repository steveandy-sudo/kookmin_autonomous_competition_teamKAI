from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    perception_config = PathJoinSubstitution(
        [FindPackageShare("my_road"), "config", "camera_perception_real.yaml"]
    )
    driver_config = PathJoinSubstitution(
        [FindPackageShare("my_control"), "config", "lane_rule_driver_real.yaml"]
    )
    calibration_file = PathJoinSubstitution(
        [
            FindPackageShare("my_road"),
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
                "enable_rectify",
                default_value="false",
                description="Rectify an unrectified fisheye camera input.",
            ),
            DeclareLaunchArgument(
                "src_tl_x_ratio",
                default_value="0.442578",
                description="Top-left source x ratio used by the BEV warp.",
            ),
            DeclareLaunchArgument(
                "src_tr_x_ratio",
                default_value="0.688281",
                description="Top-right source x ratio used by the BEV warp.",
            ),
            DeclareLaunchArgument(
                "src_bl_x_ratio",
                default_value="0.190625",
                description="Bottom-left source x ratio used by the BEV warp.",
            ),
            DeclareLaunchArgument(
                "src_br_x_ratio",
                default_value="0.919141",
                description="Bottom-right source x ratio used by the BEV warp.",
            ),
            DeclareLaunchArgument(
                "src_top_y_ratio",
                default_value="0.480781",
                description="Top source-row ratio used by the real-camera BEV warp.",
            ),
            DeclareLaunchArgument("src_bottom_y_ratio", default_value="0.614189"),
            DeclareLaunchArgument("dst_left_ratio", default_value="0.205714"),
            DeclareLaunchArgument("dst_right_ratio", default_value="0.794286"),
            DeclareLaunchArgument("dst_top_y_ratio", default_value="0.0"),
            DeclareLaunchArgument(
                "dst_bottom_y_ratio", default_value="0.666666667"
            ),
            DeclareLaunchArgument("lateral_m_per_px", default_value="0.0021875"),
            DeclareLaunchArgument(
                "forward_m_per_px",
                default_value="0.006818182",
                description="Forward metres represented by one canonical BEV pixel.",
            ),
            DeclareLaunchArgument(
                "canonical_forward_range_m",
                default_value="1.5",
                description="Forward distance represented by the canonical output.",
            ),
            DeclareLaunchArgument(
                "canonical_top_ignore_m",
                default_value="0.0",
                description="Far-end strip ignored after canonical conversion.",
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
                default_value="3.0",
                description="Validated real-car straight speed command.",
            ),
            DeclareLaunchArgument(
                "min_speed_command",
                default_value="3.0",
                description="Validated real-car cornering speed command.",
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
                package="my_road",
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
                        "enable_rectify": ParameterValue(
                            LaunchConfiguration("enable_rectify"),
                            value_type=bool,
                        ),
                        "src_tl_x_ratio": ParameterValue(
                            LaunchConfiguration("src_tl_x_ratio"),
                            value_type=float,
                        ),
                        "src_tr_x_ratio": ParameterValue(
                            LaunchConfiguration("src_tr_x_ratio"),
                            value_type=float,
                        ),
                        "src_bl_x_ratio": ParameterValue(
                            LaunchConfiguration("src_bl_x_ratio"),
                            value_type=float,
                        ),
                        "src_br_x_ratio": ParameterValue(
                            LaunchConfiguration("src_br_x_ratio"),
                            value_type=float,
                        ),
                        "src_top_y_ratio": ParameterValue(
                            LaunchConfiguration("src_top_y_ratio"),
                            value_type=float,
                        ),
                        "src_bottom_y_ratio": ParameterValue(
                            LaunchConfiguration("src_bottom_y_ratio"),
                            value_type=float,
                        ),
                        "dst_left_ratio": ParameterValue(
                            LaunchConfiguration("dst_left_ratio"),
                            value_type=float,
                        ),
                        "dst_right_ratio": ParameterValue(
                            LaunchConfiguration("dst_right_ratio"),
                            value_type=float,
                        ),
                        "dst_top_y_ratio": ParameterValue(
                            LaunchConfiguration("dst_top_y_ratio"),
                            value_type=float,
                        ),
                        "dst_bottom_y_ratio": ParameterValue(
                            LaunchConfiguration("dst_bottom_y_ratio"),
                            value_type=float,
                        ),
                        "lateral_m_per_px": ParameterValue(
                            LaunchConfiguration("lateral_m_per_px"),
                            value_type=float,
                        ),
                        "forward_m_per_px": ParameterValue(
                            LaunchConfiguration("forward_m_per_px"),
                            value_type=float,
                        ),
                        "canonical_forward_range_m": ParameterValue(
                            LaunchConfiguration("canonical_forward_range_m"),
                            value_type=float,
                        ),
                        "canonical_top_ignore_m": ParameterValue(
                            LaunchConfiguration("canonical_top_ignore_m"),
                            value_type=float,
                        ),
                    },
                ],
                output="screen",
            ),
            Node(
                package="my_control",
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
