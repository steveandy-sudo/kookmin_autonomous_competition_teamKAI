#!/usr/bin/env python3
"""Forward camera frames only while the shortcut-processing gate is true."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


class CompressedImageGateNode(Node):
    """Keep LR-ASPP disconnected from camera frames outside shortcut mode."""

    def __init__(self) -> None:
        super().__init__("shortcut_compressed_image_gate")
        self.declare_parameter(
            "source_topic", "/wide_camera_mjpeg/image_raw/compressed"
        )
        self.declare_parameter(
            "output_topic", "/shortcut/lraspp/input/compressed"
        )
        self.declare_parameter(
            "processing_enabled_topic", "/hybrid/shortcut_processing_enabled"
        )
        self.declare_parameter("default_enabled", False)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        gate_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.enabled = bool(self.get_parameter("default_enabled").value)
        self.forwarded_frames = 0
        self.dropped_frames = 0
        self.publisher = self.create_publisher(
            CompressedImage,
            str(self.get_parameter("output_topic").value),
            image_qos,
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("source_topic").value),
            self.on_image,
            image_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("processing_enabled_topic").value),
            self.on_gate,
            gate_qos,
        )
        self.get_logger().info(
            "shortcut camera gate ready; LR-ASPP input is disabled until "
            "/hybrid/shortcut_processing_enabled is true"
        )

    def on_gate(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested == self.enabled:
            return
        self.enabled = requested
        state = "enabled" if self.enabled else "disabled"
        self.get_logger().info(f"shortcut LR-ASPP camera input {state}")

    def on_image(self, message: CompressedImage) -> None:
        if not self.enabled:
            self.dropped_frames += 1
            return
        self.publisher.publish(message)
        self.forwarded_frames += 1


def main() -> None:
    rclpy.init()
    node = CompressedImageGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
