"""Republish recorded odometry poses as TF for motor-free bag replay."""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def compose_planar_origin(
    x: float,
    y: float,
    quaternion: tuple[float, float, float, float],
    *,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
) -> tuple[float, float, tuple[float, float, float, float]]:
    """Place a relative odometry pose in an absolute planar frame."""
    cosine = math.cos(float(origin_yaw))
    sine = math.sin(float(origin_yaw))
    transformed_x = (
        float(origin_x) + cosine * float(x) - sine * float(y)
    )
    transformed_y = (
        float(origin_y) + sine * float(x) + cosine * float(y)
    )
    qx, qy, qz, qw = (float(value) for value in quaternion)
    half_sine = math.sin(float(origin_yaw) * 0.5)
    half_cosine = math.cos(float(origin_yaw) * 0.5)
    transformed_quaternion = (
        half_cosine * qx - half_sine * qy,
        half_cosine * qy + half_sine * qx,
        half_cosine * qz + half_sine * qw,
        half_cosine * qw - half_sine * qz,
    )
    return transformed_x, transformed_y, transformed_quaternion


class OdomTfRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_republisher")
        self.declare_parameter("odom_topic", "/slam/odom")
        self.declare_parameter("odom_frame", "slam_odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        self.declare_parameter("origin_yaw", 0.0)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._origin_x = float(self.get_parameter("origin_x").value)
        self._origin_y = float(self.get_parameter("origin_y").value)
        self._origin_yaw = float(self.get_parameter("origin_yaw").value)
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            qos_profile_sensor_data,
        )

    def _on_odom(self, message: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        x, y, quaternion = compose_planar_origin(
            position.x,
            position.y,
            (
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
            origin_x=self._origin_x,
            origin_y=self._origin_y,
            origin_yaw=self._origin_yaw,
        )
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
