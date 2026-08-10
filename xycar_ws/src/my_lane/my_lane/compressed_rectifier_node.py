#!/usr/bin/env python3
"""Decode and rectify the real camera MJPEG stream inside this package."""

from __future__ import annotations

from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from my_lane.camera_input import CameraRectifier, decode_compressed_bgr


class CompressedRectifierNode(Node):
    def __init__(self) -> None:
        super().__init__("lane_seg_compressed_rectifier")
        default_calibration = (
            Path(get_package_share_directory("my_road"))
            / "config"
            / "wide_camera_fisheye_1280x1024_20260708.yaml"
        )
        self.declare_parameter(
            "input_topic", "/wide_camera_mjpeg/image_raw/compressed"
        )
        self.declare_parameter("output_topic", "/lane_seg/rectified_image")
        self.declare_parameter("calib_yaml", str(default_calibration))
        self.declare_parameter("balance", 0.3)

        calibration = str(self.get_parameter("calib_yaml").value)
        balance = float(self.get_parameter("balance").value)
        self.rectifier = CameraRectifier(calibration, balance)
        self.bridge = CvBridge()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(
            Image,
            str(self.get_parameter("output_topic").value),
            qos,
        )
        self.create_subscription(
            CompressedImage,
            str(self.get_parameter("input_topic").value),
            self.on_compressed_image,
            qos,
        )
        self.frame_count = 0
        self.last_log_time = time.monotonic()
        self.get_logger().info(
            "compressed fisheye rectifier ready: "
            f"calibration={calibration}, balance={balance:.2f}"
        )

    def on_compressed_image(self, message: CompressedImage) -> None:
        frame = decode_compressed_bgr(message.data)
        if frame is None:
            self.get_logger().warning(
                "failed to decode compressed camera frame",
                throttle_duration_sec=2.0,
            )
            return
        rectified = self.rectifier.rectify(frame)
        output = self.bridge.cv2_to_imgmsg(rectified, encoding="bgr8")
        output.header = message.header
        self.publisher.publish(output)
        self.frame_count += 1
        now = time.monotonic()
        elapsed = now - self.last_log_time
        if elapsed >= 5.0:
            self.get_logger().info(
                f"rectified camera rate: {self.frame_count / elapsed:.1f}Hz"
            )
            self.frame_count = 0
            self.last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompressedRectifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
