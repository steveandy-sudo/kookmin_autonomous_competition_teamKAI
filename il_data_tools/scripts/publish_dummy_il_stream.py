#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


REAL_MOTOR_TOPIC = "/xycar_motor"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Publish a safe synthetic IL stream for local recorder dry-runs."
    )
    parser.add_argument("--image-topic", default="/test/image")
    parser.add_argument("--motor-topic", default="/test/xycar_motor")
    parser.add_argument("--label-topic", default="/il/mission_label")
    parser.add_argument(
        "--motor-msg-type",
        default="float32_multi_array",
        choices=["float32_multi_array", "std_msgs/msg/Float32MultiArray"],
    )
    parser.add_argument("--label", default="general_drive")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--duration-sec", type=float, default=12.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument(
        "--no-label",
        action="store_true",
        help="Do not publish std_msgs/String labels.",
    )
    parser.add_argument(
        "--allow-real-motor-topic",
        action="store_true",
        help="Allow publishing to /xycar_motor. This is intentionally opt-in.",
    )
    return parser


class DummyILStreamPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("publish_dummy_il_stream")
        self.args = args
        self.started = time.monotonic()
        self.frame_index = 0
        self.done = False
        self.image_pub = self.create_publisher(Image, args.image_topic, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, args.motor_topic, 10)
        self.label_pub = None
        if not args.no_label:
            self.label_pub = self.create_publisher(String, args.label_topic, 10)
        period = 1.0 / max(args.rate_hz, 1e-6)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            "publishing dummy stream image=%s motor=%s label=%s duration=%.2fs"
            % (args.image_topic, args.motor_topic, args.label_topic, args.duration_sec)
        )

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.started
        if self.args.duration_sec > 0 and elapsed >= self.args.duration_sec:
            self.get_logger().info("dummy stream duration reached; shutting down")
            self.done = True
            return

        self.image_pub.publish(self._make_image(elapsed))

        motor = Float32MultiArray()
        motor.data = [20.0 * math.sin(elapsed), 8.0]
        self.motor_pub.publish(motor)

        if self.label_pub is not None:
            label = String()
            label.data = self.args.label
            self.label_pub.publish(label)

        self.frame_index += 1

    def _make_image(self, elapsed: float) -> Image:
        width = self.args.width
        height = self.args.height
        data = bytearray(width * height * 3)
        shift = int(45.0 * math.sin(elapsed * 0.8))
        horizon = height // 2
        lane_half_width = max(50, width // 5)
        slope = max(0.25, width / max(height, 1) * 0.42)

        for y in range(horizon, height):
            progress = y - horizon
            left_x = int(width / 2 - lane_half_width - progress * slope + shift)
            right_x = int(width / 2 + lane_half_width + progress * slope + shift)
            self._draw_vertical_mark(data, width, height, left_x, y)
            self._draw_vertical_mark(data, width, height, right_x, y)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "dummy_front_camera"
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = bytes(data)
        return msg

    @staticmethod
    def _draw_vertical_mark(data: bytearray, width: int, height: int, x: int, y: int) -> None:
        for dx in range(-2, 3):
            px = x + dx
            if px < 0 or px >= width or y < 0 or y >= height:
                continue
            offset = (y * width + px) * 3
            data[offset : offset + 3] = b"\xff\xff\xff"


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive")
    if args.motor_topic == REAL_MOTOR_TOPIC and not args.allow_real_motor_topic:
        print(
            "\nWARNING: refusing to publish dummy motor commands to /xycar_motor.\n"
            "Use /test/xycar_motor for local dry-runs. If you really intend to publish\n"
            "to /xycar_motor, pass --allow-real-motor-topic explicitly.\n",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = DummyILStreamPublisher(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
