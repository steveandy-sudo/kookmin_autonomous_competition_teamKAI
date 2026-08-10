from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device",
                default_value=(
                    "/dev/v4l/by-id/"
                    "usb-HD_USB_Camera_HD_USB_Camera-video-index0"
                ),
            ),
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="1024"),
            DeclareLaunchArgument("fps", default_value="30"),
            DeclareLaunchArgument(
                "frame_id", default_value="wide_camera_optical_frame"
            ),
            DeclareLaunchArgument(
                "topic",
                default_value="/wide_camera_mjpeg/image_raw/compressed",
            ),
            Node(
                package="xycar_camera",
                executable="xycar_camera_node",
                # Preserve the deployed node name for runtime compatibility.
                name="xycar_wide_camera",
                output="screen",
                parameters=[
                    {
                        "device": LaunchConfiguration("device"),
                        "width": ParameterValue(
                            LaunchConfiguration("width"), value_type=int
                        ),
                        "height": ParameterValue(
                            LaunchConfiguration("height"), value_type=int
                        ),
                        "fps": ParameterValue(
                            LaunchConfiguration("fps"), value_type=int
                        ),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "topic": LaunchConfiguration("topic"),
                        "power_line_frequency": 2,
                        "exposure_dynamic_framerate": 0,
                    }
                ],
            ),
        ]
    )
