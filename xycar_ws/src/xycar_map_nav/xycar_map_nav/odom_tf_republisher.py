"""Republish recorded odometry poses as TF for motor-free bag replay."""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomTfRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_republisher")
        self.declare_parameter("odom_topic", "/slam/odom")
        self.declare_parameter("odom_frame", "slam_odom")
        self.declare_parameter("base_frame", "base_footprint")
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
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
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
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
