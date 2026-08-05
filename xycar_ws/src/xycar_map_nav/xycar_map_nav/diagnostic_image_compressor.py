"""Publish low-rate compressed copies of real-drive perception images."""

from __future__ import annotations

from dataclasses import dataclass
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


@dataclass(frozen=True)
class ImageStream:
    source: str
    output: str
    lossless: bool
    debug_only: bool = False


STREAMS = (
    ImageStream(
        "/lane_seg/white_boundary_mask",
        "/recording/lane_seg_white_mask/compressed",
        True,
    ),
    ImageStream(
        "/lane_seg/yellow_centerline_mask",
        "/recording/lane_seg_yellow_mask/compressed",
        True,
    ),
    ImageStream(
        "/perception/canonical_road_image",
        "/recording/canonical_road_image/compressed",
        True,
    ),
    ImageStream(
        "/perception/canonical_white_mask",
        "/recording/canonical_white_mask/compressed",
        True,
    ),
    ImageStream(
        "/perception/canonical_yellow_mask",
        "/recording/canonical_yellow_mask/compressed",
        True,
    ),
    ImageStream(
        "/perception/canonical_valid_mask",
        "/recording/canonical_valid_mask/compressed",
        True,
    ),
    ImageStream(
        "/rule_drive/canonical_debug_image",
        "/recording/rule_canonical_debug/compressed",
        False,
        True,
    ),
    ImageStream(
        "/lane_seg/debug_image",
        "/recording/lane_seg_debug/compressed",
        False,
        True,
    ),
    ImageStream(
        "/my_rule/object_detection/debug_image",
        "/recording/object_detection_debug/compressed",
        False,
        True,
    ),
)


def _image_array(message: Image) -> np.ndarray:
    encoding = message.encoding.lower()
    channels_by_encoding = {
        "mono8": 1,
        "8uc1": 1,
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {message.encoding}")
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0 or step < width * channels:
        raise ValueError(
            f"invalid image dimensions: {width}x{height}, step={step}"
        )
    raw = np.frombuffer(message.data, dtype=np.uint8)
    required = height * step
    if raw.size < required:
        raise ValueError(
            f"image data is short: got {raw.size}, expected {required}"
        )
    rows = raw[:required].reshape(height, step)
    pixels = rows[:, : width * channels]
    if channels == 1:
        return pixels.reshape(height, width)
    image = pixels.reshape(height, width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA)
    return image


class DiagnosticImageCompressor(Node):
    def __init__(self) -> None:
        super().__init__("drive_diagnostic_image_compressor")
        self.declare_parameter("max_rate_hz", 5.0)
        self.declare_parameter("jpeg_quality", 75)
        self.declare_parameter("png_compression", 3)
        self.declare_parameter("include_debug_images", True)

        self.max_rate_hz = max(
            0.1, float(self.get_parameter("max_rate_hz").value)
        )
        self.jpeg_quality = max(
            20, min(100, int(self.get_parameter("jpeg_quality").value))
        )
        self.png_compression = max(
            0, min(9, int(self.get_parameter("png_compression").value))
        )
        include_debug = bool(
            self.get_parameter("include_debug_images").value
        )
        self.last_publish: dict[str, float] = {}
        self._publishers_by_source = {}
        self._image_subscriptions = []

        for stream in STREAMS:
            if stream.debug_only and not include_debug:
                continue
            publisher = self.create_publisher(
                CompressedImage, stream.output, qos_profile_sensor_data
            )
            self._publishers_by_source[stream.source] = publisher
            self._image_subscriptions.append(
                self.create_subscription(
                    Image,
                    stream.source,
                    lambda message, spec=stream: self._on_image(spec, message),
                    qos_profile_sensor_data,
                )
            )
        self.get_logger().info(
            "diagnostic image compression ready: "
            f"{self.max_rate_hz:.1f} Hz, "
            f"streams={len(self._image_subscriptions)}"
        )

    def _on_image(self, stream: ImageStream, message: Image) -> None:
        publisher = self._publishers_by_source[stream.source]
        if publisher.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if now - self.last_publish.get(stream.source, -1.0e9) < (
            1.0 / self.max_rate_hz
        ):
            return
        try:
            image = _image_array(message)
            if stream.lossless:
                extension = ".png"
                parameters = [cv2.IMWRITE_PNG_COMPRESSION, self.png_compression]
                format_name = f"{message.encoding}; png compressed"
            else:
                extension = ".jpg"
                parameters = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                format_name = f"{message.encoding}; jpeg compressed bgr8"
            success, encoded = cv2.imencode(extension, image, parameters)
            if not success:
                raise RuntimeError("cv2.imencode returned false")
        except (ValueError, RuntimeError, cv2.error) as error:
            self.get_logger().warning(
                f"failed to compress {stream.source}: {error}",
                throttle_duration_sec=5.0,
            )
            return

        output = CompressedImage()
        output.header = message.header
        output.format = format_name
        output.data = encoded.tobytes()
        publisher.publish(output)
        self.last_publish[stream.source] = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DiagnosticImageCompressor()
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
