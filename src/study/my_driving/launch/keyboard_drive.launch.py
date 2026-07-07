from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_driving',
            executable='keyboard_drive',
            name='keyboard_driver',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'motor_topic': '/xycar_motor',
                'publish_rate_hz': 50.0,
                'auto_forward_speed': 3.0,
                'start_delay_sec': 3.0,
                'max_turn_angle': 30.0,
                'steering_step_angle': 1.0,
                'accel_speed_per_sec': 4.0,
                'decel_speed_per_sec': 16.0,
                'steering_rate_deg_per_sec': 150.0,
                'steering_return_rate_deg_per_sec': 200.0,
                'invert_steering': False,
                'steering_relax_delay_sec': 0.7,
                'steering_relax_rate_deg_per_sec': 4.0,
                'stop_burst_count': 10,
                'stop_burst_dt_sec': 0.05,
            }],
        ),
    ])
