from __future__ import annotations

from functools import partial
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


PERCEPTION_STREAMS = (
    (
        "/lane_seg/white_boundary_mask",
        "/recording/lane_seg_white_mask/compressed",
    ),
    (
        "/lane_seg/yellow_centerline_mask",
        "/recording/lane_seg_yellow_mask/compressed",
    ),
    (
        "/lane_seg_bev/color",
        "/recording/bev_before_canonical/compressed",
    ),
    (
        "/perception/canonical_white_mask",
        "/recording/canonical_white_mask/compressed",
    ),
    (
        "/perception/canonical_yellow_mask",
        "/recording/canonical_yellow_mask/compressed",
    ),
)


def perception_streams(include_intermediate_streams: bool):
    return PERCEPTION_STREAMS if include_intermediate_streams else ()


class CanonicalCompressor(Node):
    def __init__(self) -> None:
        super().__init__("canonical_recording_compressor")
        self.declare_parameter(
            "input_topic",
            "/perception/canonical_road_image",
        )
        self.declare_parameter(
            "output_topic",
            "/recording/canonical_road_image/compressed",
        )
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("png_compression", 3)
        self.declare_parameter("include_intermediate_streams", False)

        self.bridge = CvBridge()
        self.minimum_period_sec = 1.0 / max(
            0.1,
            float(self.get_parameter("max_rate_hz").value),
        )
        self.png_compression = max(
            0,
            min(9, int(self.get_parameter("png_compression").value)),
        )
        canonical_stream = (
            str(self.get_parameter("input_topic").value),
            str(self.get_parameter("output_topic").value),
        )
        include_intermediate_streams = bool(
            self.get_parameter("include_intermediate_streams").value
        )
        self.last_publish_wall_sec: dict[str, float] = {}
        self.stream_publishers = {}
        self.stream_subscriptions = []
        streams = (canonical_stream,) + perception_streams(
            include_intermediate_streams
        )
        for input_topic, output_topic in streams:
            self.last_publish_wall_sec[input_topic] = 0.0
            self.stream_publishers[input_topic] = self.create_publisher(
                CompressedImage,
                output_topic,
                qos_profile_sensor_data,
            )
            self.stream_subscriptions.append(
                self.create_subscription(
                    Image,
                    input_topic,
                    partial(self._on_image, input_topic=input_topic),
                    qos_profile_sensor_data,
                )
            )
        self.get_logger().info(
            "recording canonical input only"
            if not include_intermediate_streams
            else "recording canonical input and intermediate perception streams"
        )

    def _on_image(self, message: Image, *, input_topic: str) -> None:
        now = time.monotonic()
        if (
            now - self.last_publish_wall_sec[input_topic]
            < self.minimum_period_sec
        ):
            return
        image = self.bridge.imgmsg_to_cv2(
            message,
            desired_encoding="passthrough",
        )
        success, encoded = cv2.imencode(
            ".png",
            image,
            [cv2.IMWRITE_PNG_COMPRESSION, self.png_compression],
        )
        if not success:
            self.get_logger().warning("failed to encode canonical image")
            return
        compressed = CompressedImage()
        compressed.header = message.header
        compressed.format = (
            f"{message.encoding}; png compressed {message.encoding}"
        )
        compressed.data = encoded.tobytes()
        self.stream_publishers[input_topic].publish(compressed)
        self.last_publish_wall_sec[input_topic] = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CanonicalCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
