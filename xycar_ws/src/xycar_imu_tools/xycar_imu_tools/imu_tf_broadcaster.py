from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class ImuTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("imu_tf_broadcaster")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "imu_link")

        imu_topic = str(self.get_parameter("imu_topic").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.default_child_frame = str(self.get_parameter("child_frame").value)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.get_logger().info(
            f"Broadcasting IMU TF from {imu_topic}: {self.parent_frame} -> {self.default_child_frame}"
        )

    def imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 1e-6:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = msg.header.frame_id or self.default_child_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q.x / norm
        tf_msg.transform.rotation.y = q.y / norm
        tf_msg.transform.rotation.z = q.z / norm
        tf_msg.transform.rotation.w = q.w / norm
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
