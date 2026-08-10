#!/usr/bin/env python3
"""Expose the verified ShortcutCore as an integrated-drive candidate."""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32MultiArray, String

from .shortcut_core import ShortcutCore


PHASE_CODES = {
    "idle": 0.0,
    "enter": 1.0,
    "cruise": 2.0,
    "exit": 3.0,
    "done": 4.0,
}


def limit_shortcut_angle(angle: float, maximum_abs_angle: float) -> float:
    limit = max(0.0, float(maximum_abs_angle))
    return float(np.clip(float(angle), -limit, limit))


class ShortcutCandidateNode(Node):
    """Run ShortcutCore only while the integrated selector requests it."""

    def __init__(self) -> None:
        super().__init__("shortcut_candidate")
        self.declare_parameter(
            "image_topic", "/wide_camera_mjpeg/image_raw/compressed"
        )
        self.declare_parameter(
            "processing_enabled_topic", "/hybrid/shortcut_processing_enabled"
        )
        self.declare_parameter(
            "candidate_topic", "/hybrid/shortcut_candidate"
        )
        self.declare_parameter("status_topic", "/hybrid/shortcut_status")
        self.declare_parameter(
            "debug_topic", "/hybrid/shortcut_debug_image"
        )
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("maximum_input_age_sec", 0.35)
        self.declare_parameter("maximum_abs_angle_command", 42.0)

        self.bridge = CvBridge()
        self.core = ShortcutCore(
            logger=lambda message: self.get_logger().info(message)
        )
        self.enabled = False
        self.completed = False
        self.latest_image = None
        self.latest_image_time = float("-inf")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        gate_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("processing_enabled_topic").value),
            self._on_enabled,
            gate_qos,
        )
        self.candidate_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("candidate_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter("debug_topic").value),
            sensor_qos,
        )
        rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate_hz, self._step)

    def _on_image(self, message: CompressedImage) -> None:
        raw = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if frame is None:
            return
        self.latest_image = frame
        self.latest_image_time = time.monotonic()

    def _on_enabled(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested and not self.enabled:
            self.core.reset()
            self.completed = False
            self.enabled = True
            self.get_logger().warning("SHORTCUT candidate enabled")
        elif not requested and self.enabled:
            self.enabled = False
            self.completed = False
            self.core.reset()
            self.get_logger().info("SHORTCUT candidate disabled")

    def _publish_status(self, reason: str) -> None:
        self.status_pub.publish(
            String(
                data=(
                    f"enabled={self.enabled} phase={self.core.phase} "
                    f"reason={reason}"
                )
            )
        )

    def _step(self) -> None:
        if not self.enabled or self.completed:
            return
        now = time.monotonic()
        if (
            self.latest_image is None
            or now - self.latest_image_time
            > float(self.get_parameter("maximum_input_age_sec").value)
        ):
            self._publish_status("camera stale")
            return

        angle, speed, done = self.core.compute(self.latest_image, now)
        angle = limit_shortcut_angle(
            angle,
            float(
                self.get_parameter("maximum_abs_angle_command").value
            ),
        )
        phase_code = PHASE_CODES.get(self.core.phase, -1.0)
        self.candidate_pub.publish(
            Float32MultiArray(
                data=[angle, float(speed), 1.0 if done else 0.0, phase_code]
            )
        )
        self._publish_status("done" if done else "running")
        if self.debug_pub.get_subscription_count() > 0:
            debug = self.core.debug_img
            if debug is not None:
                self.debug_pub.publish(
                    self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                )
        if done:
            self.completed = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ShortcutCandidateNode()
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
