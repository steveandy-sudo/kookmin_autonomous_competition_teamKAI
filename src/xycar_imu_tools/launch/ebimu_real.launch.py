from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu_port = LaunchConfiguration("imu_port")
    imu_baudrate = LaunchConfiguration("imu_baudrate")
    imu_topic = LaunchConfiguration("imu_topic")
    imu_raw_topic = LaunchConfiguration("imu_raw_topic")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    publish_tf = LaunchConfiguration("publish_tf")
    tf_parent_frame = LaunchConfiguration("tf_parent_frame")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    package_share = FindPackageShare("xycar_imu_tools")

    return LaunchDescription([
        DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("imu_baudrate", default_value="460800"),
        DeclareLaunchArgument("imu_topic", default_value="/imu"),
        DeclareLaunchArgument("imu_raw_topic", default_value="/imu/raw_data"),
        DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("tf_parent_frame", default_value="world"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([package_share, "rviz", "imu_tf.rviz"]),
        ),
        Node(
            package="xycar_imu_tools",
            executable="ebimu_serial_publisher",
            name="ebimu_serial_publisher",
            output="screen",
            parameters=[{
                "port": imu_port,
                "baudrate": ParameterValue(imu_baudrate, value_type=int),
                "topic": imu_topic,
                "raw_topic": imu_raw_topic,
                "frame_id": imu_frame_id,
            }],
        ),
        Node(
            package="xycar_imu_tools",
            executable="imu_tf_broadcaster",
            name="imu_tf_broadcaster",
            output="screen",
            condition=IfCondition(publish_tf),
            parameters=[{
                "imu_topic": imu_topic,
                "parent_frame": tf_parent_frame,
                "child_frame": imu_frame_id,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_imu",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
        ),
    ])
