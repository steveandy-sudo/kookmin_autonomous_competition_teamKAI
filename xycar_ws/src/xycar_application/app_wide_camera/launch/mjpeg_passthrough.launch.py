from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0',
        ),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='1024'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('auto_exposure', default_value='3'),
        DeclareLaunchArgument('exposure_time_absolute', default_value='157'),
        DeclareLaunchArgument('gain', default_value='0'),
        DeclareLaunchArgument('backlight_compensation', default_value='1'),
        DeclareLaunchArgument('brightness', default_value='0'),
        DeclareLaunchArgument('frame_id', default_value='wide_camera_optical_frame'),
        DeclareLaunchArgument('topic', default_value='/wide_camera_mjpeg/image_raw/compressed'),

        Node(
            package='app_wide_camera',
            executable='mjpeg_passthrough_node',
            name='mjpeg_passthrough_node',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'frame_id': LaunchConfiguration('frame_id'),
                'topic': LaunchConfiguration('topic'),
                'power_line_frequency': 2,
                'exposure_dynamic_framerate': 0,
                'auto_exposure': LaunchConfiguration('auto_exposure'),
                'exposure_time_absolute': LaunchConfiguration(
                    'exposure_time_absolute'
                ),
                'gain': LaunchConfiguration('gain'),
                'backlight_compensation': LaunchConfiguration(
                    'backlight_compensation'
                ),
                'brightness': LaunchConfiguration('brightness'),
            }],
        ),
    ])
