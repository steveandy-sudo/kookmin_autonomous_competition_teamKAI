#!/usr/bin/env python3
"""Decode once and publish small rectified lane and object camera streams."""

from __future__ import annotations

from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from my_rule.perception.camera_input import (
    CameraRectifier,
    decode_compressed_bgr,
)


def message_stamp_ns(message: CompressedImage) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


class CompressedCameraRepublisher(Node):
    """Publish bandwidth-bounded raw streams for lane and object models."""

    def __init__(self) -> None:
        super().__init__("my_rule_compressed_camera_republisher")
        perception_share = Path(
            get_package_share_directory("xycar_perception")
        )
        self.declare_parameter(
            "input_topic", "/wide_camera_mjpeg/image_raw/compressed"
        )
        self.declare_parameter(
            "lane_output_topic", "/wide_camera/lane_rect/image_raw"
        )
        self.declare_parameter(
            "object_output_topic", "/wide_camera/object_rect/image_raw"
        )
        self.declare_parameter(
            "camera_yaml",
            str(
                perception_share
                / "config"
                / "wide_camera_fisheye_1280x1024_20260708.yaml"
            ),
        )
        self.declare_parameter("enable_rectify", True)
        self.declare_parameter("rect_balance", 0.3)
        self.declare_parameter("lane_rectify_width", 1024)
        self.declare_parameter("lane_rectify_height", 576)
        self.declare_parameter("lane_output_width", 512)
        self.declare_parameter("lane_output_height", 288)
        self.declare_parameter("lane_output_rate_hz", 20.0)
        self.declare_parameter("object_output_width", 640)
        self.declare_parameter("object_output_height", 512)
        self.declare_parameter("object_output_rate_hz", 5.0)
        self.declare_parameter("opencv_threads", 2)

        cv2.setNumThreads(
            max(1, int(self.get_parameter("opencv_threads").value))
        )
        self.bridge = CvBridge()
        self.rectifier = None
        if bool(self.get_parameter("enable_rectify").value):
            self.rectifier = CameraRectifier(
                str(self.get_parameter("camera_yaml").value),
                float(self.get_parameter("rect_balance").value),
            )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        object_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.lane_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("lane_output_topic").value),
            qos,
        )
        self.object_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("object_output_topic").value),
            object_qos,
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            str(self.get_parameter("input_topic").value),
            self.on_image,
            qos,
        )
        self.lane_rate_hz = max(
            0.0, float(self.get_parameter("lane_output_rate_hz").value)
        )
        self.object_rate_hz = max(
            0.0, float(self.get_parameter("object_output_rate_hz").value)
        )
        self.lane_rectify_size = self.positive_size(
            "lane_rectify_width", "lane_rectify_height"
        )
        self.lane_output_size = self.positive_size(
            "lane_output_width", "lane_output_height"
        )
        self.object_output_size = self.positive_size(
            "object_output_width", "object_output_height"
        )
        self.last_lane_bucket = None
        self.last_object_bucket = None
        self.received_count = 0
        self.decoded_count = 0
        self.lane_published_count = 0
        self.object_published_count = 0
        self.decode_time_sum_ms = 0.0
        self.last_log_wall = time.monotonic()
        self.get_logger().info(
            "shared camera decoder ready: "
            f"input={self.get_parameter('input_topic').value}, "
            f"lane={self.get_parameter('lane_output_topic').value} "
            f"{self.lane_output_size[0]}x{self.lane_output_size[1]}"
            f"@{self.lane_rate_hz:.1f}Hz, "
            f"object={self.get_parameter('object_output_topic').value} "
            f"{self.object_output_size[0]}x{self.object_output_size[1]}"
            f"@{self.object_rate_hz:.1f}Hz, "
            f"rectify={self.rectifier is not None}, "
            f"lane_map={self.lane_rectify_size[0]}x"
            f"{self.lane_rectify_size[1]}"
        )

    def positive_size(self, width_name: str, height_name: str) -> tuple[int, int]:
        width = int(self.get_parameter(width_name).value)
        height = int(self.get_parameter(height_name).value)
        if width <= 0 or height <= 0:
            raise ValueError(
                f"{width_name} and {height_name} must both be positive"
            )
        return width, height

    @staticmethod
    def bucket_for_rate(stamp_ns: int, rate_hz: float) -> int:
        if stamp_ns > 0:
            return int(stamp_ns * rate_hz / 1_000_000_000)
        return int(time.monotonic() * rate_hz)

    def stream_due(
        self,
        stamp_ns: int,
        rate_hz: float,
        last_bucket: int | None,
    ) -> tuple[bool, int | None]:
        if rate_hz <= 0.0:
            return True, last_bucket
        bucket = self.bucket_for_rate(stamp_ns, rate_hz)
        return bucket != last_bucket, bucket

    def rectified_frame(
        self,
        frame,
        output_size: tuple[int, int],
        intermediate_size: tuple[int, int] | None = None,
    ):
        target_size = intermediate_size or output_size
        if self.rectifier is not None:
            output = self.rectifier.rectify_to_size(frame, *target_size)
        else:
            output = cv2.resize(frame, target_size, interpolation=cv2.INTER_AREA)
        if target_size != output_size:
            output = cv2.resize(
                output, output_size, interpolation=cv2.INTER_AREA
            )
        return output

    def publish_frame(self, publisher, frame, header) -> None:
        output = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        output.header = header
        publisher.publish(output)

    def on_image(self, message: CompressedImage) -> None:
        self.received_count += 1
        stamp_ns = message_stamp_ns(message)
        lane_due, lane_bucket = self.stream_due(
            stamp_ns, self.lane_rate_hz, self.last_lane_bucket
        )
        object_due, object_bucket = self.stream_due(
            stamp_ns, self.object_rate_hz, self.last_object_bucket
        )
        if not lane_due and not object_due:
            return
        started = time.perf_counter()
        frame = decode_compressed_bgr(message.data)
        if frame is None:
            self.get_logger().warning("received an invalid compressed image")
            return
        self.decoded_count += 1
        if lane_due:
            lane_frame = self.rectified_frame(
                frame,
                self.lane_output_size,
                self.lane_rectify_size,
            )
            self.publish_frame(
                self.lane_publisher, lane_frame, message.header
            )
            self.last_lane_bucket = lane_bucket
            self.lane_published_count += 1
        if object_due:
            object_frame = self.rectified_frame(
                frame,
                self.object_output_size,
            )
            self.publish_frame(
                self.object_publisher, object_frame, message.header
            )
            self.last_object_bucket = object_bucket
            self.object_published_count += 1
        self.decode_time_sum_ms += (time.perf_counter() - started) * 1000.0

        now = time.monotonic()
        if now - self.last_log_wall >= 5.0:
            average_ms = self.decode_time_sum_ms / max(1, self.decoded_count)
            self.get_logger().info(
                f"camera pipeline: received={self.received_count}, "
                f"decoded={self.decoded_count}, "
                f"lane={self.lane_published_count}, "
                f"object={self.object_published_count}, "
                f"avg={average_ms:.1f}ms"
            )
            self.last_log_wall = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompressedCameraRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
