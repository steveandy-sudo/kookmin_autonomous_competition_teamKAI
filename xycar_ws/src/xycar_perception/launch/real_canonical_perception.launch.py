from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("xycar_perception")
    config_file = PathJoinSubstitution(
        [package_share, "config", "camera_perception_real.yaml"]
    )
    calib_file = PathJoinSubstitution(
        [package_share, "config", "wide_camera_fisheye_1280x1024.yaml"]
    )
    image_topic = LaunchConfiguration("image_topic")
    enable_rectify = LaunchConfiguration("enable_rectify")
    use_compressed_image = LaunchConfiguration("use_compressed_image")
    use_sim_time = LaunchConfiguration("use_sim_time")
    src_tl_x_ratio = LaunchConfiguration("src_tl_x_ratio")
    src_tr_x_ratio = LaunchConfiguration("src_tr_x_ratio")
    src_bl_x_ratio = LaunchConfiguration("src_bl_x_ratio")
    src_br_x_ratio = LaunchConfiguration("src_br_x_ratio")
    src_top_y_ratio = LaunchConfiguration("src_top_y_ratio")
    src_bottom_y_ratio = LaunchConfiguration("src_bottom_y_ratio")
    dst_left_ratio = LaunchConfiguration("dst_left_ratio")
    dst_right_ratio = LaunchConfiguration("dst_right_ratio")
    dst_top_y_ratio = LaunchConfiguration("dst_top_y_ratio")
    dst_bottom_y_ratio = LaunchConfiguration("dst_bottom_y_ratio")
    lateral_m_per_px = LaunchConfiguration("lateral_m_per_px")
    forward_m_per_px = LaunchConfiguration("forward_m_per_px")
    canonical_forward_range_m = LaunchConfiguration("canonical_forward_range_m")
    canonical_top_ignore_m = LaunchConfiguration("canonical_top_ignore_m")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/wide_camera/rect/image_raw",
                description=(
                    "Rectified real camera topic, or /image_raw with "
                    "enable_rectify=true."
                ),
            ),
            DeclareLaunchArgument(
                "enable_rectify",
                default_value="false",
                description=(
                    "Rectify an unrectified fisheye input using the "
                    "packaged calibration."
                ),
            ),
            DeclareLaunchArgument(
                "use_compressed_image",
                default_value="false",
                description=(
                    "Subscribe as sensor_msgs/CompressedImage for MJPEG "
                    "bags/cameras."
                ),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock when the bag is played with --clock.",
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
            DeclareLaunchArgument(
                "src_bottom_y_ratio",
                default_value="0.614189",
                description="0.5m source-row ratio from the lens calibration.",
            ),
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
            Node(
                package="xycar_perception",
                executable="camera_perception_node",
                name="xycar_camera_perception",
                parameters=[
                    config_file,
                    {
                        "calib_yaml": calib_file,
                        "image_topic": image_topic,
                        "enable_rectify": ParameterValue(
                            enable_rectify, value_type=bool
                        ),
                        "use_compressed_image": ParameterValue(
                            use_compressed_image, value_type=bool
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                        "src_tl_x_ratio": ParameterValue(
                            src_tl_x_ratio, value_type=float
                        ),
                        "src_tr_x_ratio": ParameterValue(
                            src_tr_x_ratio, value_type=float
                        ),
                        "src_bl_x_ratio": ParameterValue(
                            src_bl_x_ratio, value_type=float
                        ),
                        "src_br_x_ratio": ParameterValue(
                            src_br_x_ratio, value_type=float
                        ),
                        "src_top_y_ratio": ParameterValue(
                            src_top_y_ratio, value_type=float
                        ),
                        "src_bottom_y_ratio": ParameterValue(
                            src_bottom_y_ratio, value_type=float
                        ),
                        "dst_left_ratio": ParameterValue(
                            dst_left_ratio, value_type=float
                        ),
                        "dst_right_ratio": ParameterValue(
                            dst_right_ratio, value_type=float
                        ),
                        "dst_top_y_ratio": ParameterValue(
                            dst_top_y_ratio, value_type=float
                        ),
                        "dst_bottom_y_ratio": ParameterValue(
                            dst_bottom_y_ratio, value_type=float
                        ),
                        "lateral_m_per_px": ParameterValue(
                            lateral_m_per_px, value_type=float
                        ),
                        "forward_m_per_px": ParameterValue(
                            forward_m_per_px, value_type=float
                        ),
                        "canonical_forward_range_m": ParameterValue(
                            canonical_forward_range_m, value_type=float
                        ),
                        "canonical_top_ignore_m": ParameterValue(
                            canonical_top_ignore_m, value_type=float
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
