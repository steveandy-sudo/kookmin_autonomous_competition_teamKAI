from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_serial_imu = LaunchConfiguration("use_serial_imu")
    imu_port = LaunchConfiguration("imu_port")
    imu_baudrate = LaunchConfiguration("imu_baudrate")
    imu_topic = LaunchConfiguration("imu_topic")
    motor_topic = LaunchConfiguration("motor_topic")
    bridge_debug_topic = LaunchConfiguration("bridge_debug_topic")
    use_bridge_debug = LaunchConfiguration("use_bridge_debug")
    speed_gain_mps_per_cmd = LaunchConfiguration("speed_gain_mps_per_cmd")
    steering_gain_rad_per_cmd = LaunchConfiguration("steering_gain_rad_per_cmd")
    known_wheel_base_m = LaunchConfiguration("known_wheel_base_m")
    csv_path = LaunchConfiguration("csv_path")

    return LaunchDescription([
        DeclareLaunchArgument("use_serial_imu", default_value="false"),
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("imu_baudrate", default_value="460800"),
        DeclareLaunchArgument("imu_topic", default_value="/imu"),
        DeclareLaunchArgument("motor_topic", default_value="/xycar_motor"),
        DeclareLaunchArgument("bridge_debug_topic", default_value="/xycar_motor_bridge/debug"),
        DeclareLaunchArgument("use_bridge_debug", default_value="true"),
        DeclareLaunchArgument("speed_gain_mps_per_cmd", default_value="0.08"),
        DeclareLaunchArgument("steering_gain_rad_per_cmd", default_value="-0.0068"),
        DeclareLaunchArgument("known_wheel_base_m", default_value="0.32"),
        DeclareLaunchArgument("csv_path", default_value=""),
        Node(
            package="xycar_imu_tools",
            executable="ebimu_serial_publisher",
            name="ebimu_serial_publisher",
            output="screen",
            condition=IfCondition(use_serial_imu),
            parameters=[{
                "port": imu_port,
                "baudrate": ParameterValue(imu_baudrate, value_type=int),
                "topic": imu_topic,
                "raw_topic": "/imu/raw_data",
                "frame_id": "imu_link",
            }],
        ),
        Node(
            package="xycar_imu_tools",
            executable="imu_vehicle_spec_calibrator",
            name="imu_vehicle_spec_calibrator",
            output="screen",
            parameters=[{
                "imu_topic": imu_topic,
                "motor_topic": motor_topic,
                "bridge_debug_topic": bridge_debug_topic,
                "use_bridge_debug": ParameterValue(use_bridge_debug, value_type=bool),
                "speed_gain_mps_per_cmd": ParameterValue(speed_gain_mps_per_cmd, value_type=float),
                "steering_gain_rad_per_cmd": ParameterValue(steering_gain_rad_per_cmd, value_type=float),
                "known_wheel_base_m": ParameterValue(known_wheel_base_m, value_type=float),
                "csv_path": csv_path,
            }],
        ),
    ])
