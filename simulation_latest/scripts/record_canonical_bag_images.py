#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


CSV_FIELDS = [
    "timestamp_ns",
    "image_timestamp_ns",
    "scan_timestamp_ns",
    "scan_time_offset_ms",
    "front_image_path",
    "scan_npz_path",
    "motor_angle",
    "motor_speed",
    "mission_label",
    "dataset_profile",
    "session_id",
]


def stamp_ns(message: Image) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def atomic_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, delete=False
    ) as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False, sort_keys=True)
        handle.write("\n")
        temporary = handle.name
    os.replace(temporary, path)


class CanonicalBagImageRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("canonical_bag_image_recorder")
        self.args = args
        self.output_dir = args.output_dir.expanduser().resolve()
        self.image_dir = self.output_dir / "images" / "front"
        if self.output_dir.exists() and any(self.output_dir.iterdir()):
            raise RuntimeError(f"output directory is not empty: {self.output_dir}")
        self.image_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / "debug").mkdir(exist_ok=True)
        self.csv_path = self.output_dir / "samples.csv"
        self.csv_handle = self.csv_path.open(
            "w", newline="", encoding="utf-8"
        )
        self.csv_writer = csv.DictWriter(
            self.csv_handle, fieldnames=CSV_FIELDS
        )
        self.csv_writer.writeheader()

        self.bridge = CvBridge()
        self.seen_stamps: set[int] = set()
        self.image_count = 0
        self.first_stamp_ns: int | None = None
        self.last_stamp_ns: int | None = None
        self.last_message_wall_time: float | None = None
        self.start_wall_time = time.monotonic()
        self.finished = False
        self.exit_reason = "running"

        qos = QoSProfile(
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Image, args.topic, self.on_image, qos
        )
        self.timer = self.create_timer(0.25, self.check_completion)
        self.get_logger().info(
            f"recording {args.topic} -> {self.output_dir}"
        )

    def on_image(self, message: Image) -> None:
        timestamp_ns = stamp_ns(message)
        if timestamp_ns <= 0:
            self.get_logger().error("image has a zero timestamp; refusing it")
            return
        if timestamp_ns in self.seen_stamps:
            return

        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if image.shape[:2] != (self.args.height, self.args.width):
            raise RuntimeError(
                "canonical image dimensions differ from the requested "
                f"contract: got {image.shape[1]}x{image.shape[0]}, expected "
                f"{self.args.width}x{self.args.height}"
            )
        image_path = self.image_dir / f"{timestamp_ns}.png"
        if not cv2.imwrite(
            str(image_path),
            image,
            [int(cv2.IMWRITE_PNG_COMPRESSION), 3],
        ):
            raise RuntimeError(f"failed to write {image_path}")

        self.csv_writer.writerow(
            {
                "timestamp_ns": timestamp_ns,
                "image_timestamp_ns": timestamp_ns,
                "scan_timestamp_ns": "",
                "scan_time_offset_ms": "",
                "front_image_path": str(
                    image_path.relative_to(self.output_dir)
                ),
                "scan_npz_path": "",
                "motor_angle": "",
                "motor_speed": "",
                "mission_label": "unlabeled_real_bag",
                "dataset_profile": "real_bag_unlabeled",
                "session_id": self.output_dir.name,
            }
        )
        self.seen_stamps.add(timestamp_ns)
        self.image_count += 1
        self.first_stamp_ns = (
            timestamp_ns
            if self.first_stamp_ns is None
            else min(self.first_stamp_ns, timestamp_ns)
        )
        self.last_stamp_ns = (
            timestamp_ns
            if self.last_stamp_ns is None
            else max(self.last_stamp_ns, timestamp_ns)
        )
        self.last_message_wall_time = time.monotonic()
        if self.image_count % 100 == 0:
            self.csv_handle.flush()
            self.get_logger().info(
                f"saved {self.image_count}/{self.args.expected_frames or '?'}"
            )
        if (
            self.args.expected_frames > 0
            and self.image_count >= self.args.expected_frames
        ):
            self.finish("expected_frame_count_reached")

    def check_completion(self) -> None:
        if self.finished or self.last_message_wall_time is None:
            return
        if (
            time.monotonic() - self.last_message_wall_time
            >= self.args.idle_timeout_sec
        ):
            self.finish("image_topic_idle")

    def finish(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.exit_reason = reason
        self.csv_handle.flush()
        self.write_metadata()
        self.get_logger().info(
            f"finished: reason={reason}, images={self.image_count}"
        )

    def write_metadata(self) -> None:
        expected = int(self.args.expected_frames)
        complete = expected <= 0 or self.image_count == expected
        metadata = {
            "canonical_contract": {
                "encoding": "bgr8",
                "forward_range_m": 1.5,
                "height": self.args.height,
                "lateral_range_m": 1.4,
                "observation_only": True,
                "width": self.args.width,
            },
            "collected_frames": self.image_count,
            "complete": complete,
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "dataset_profile": "real_bag_unlabeled",
            "exit_reason": self.exit_reason,
            "expected_source_frames": expected,
            "first_timestamp_ns": self.first_stamp_ns,
            "image_topic": self.args.topic,
            "labels": {
                "motor_angle": False,
                "motor_speed": False,
                "reason": "source rosbag contains no /xycar_motor topic",
            },
            "last_timestamp_ns": self.last_stamp_ns,
            "session_id": self.output_dir.name,
            "source_bag": str(self.args.source_bag.expanduser().resolve()),
            "training_ready": False,
            "use": [
                "real-camera canonical inspection",
                "domain comparison",
                "manual or pseudo labeling input",
            ],
            "wall_time_sec": round(time.monotonic() - self.start_wall_time, 3),
        }
        atomic_json(self.output_dir / "metadata.json", metadata)

    def close(self) -> None:
        if not self.finished:
            self.finish("interrupted")
        if not self.csv_handle.closed:
            self.csv_handle.close()


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description="Save canonical ROS images from a replayed rosbag."
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--source-bag", type=Path, required=True)
    parser.add_argument(
        "--topic", default="/perception/canonical_road_image"
    )
    parser.add_argument("--expected-frames", type=int, default=0)
    parser.add_argument("--idle-timeout-sec", type=float, default=5.0)
    parser.add_argument("--width", type=int, default=256)
    parser.add_argument("--height", type=int, default=144)
    return parser.parse_known_args()


def main() -> int:
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = CanonicalBagImageRecorder(args)

    def request_stop(_signum, _frame):
        node.finish("signal")

    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.25)
    except KeyboardInterrupt:
        node.finish("keyboard_interrupt")
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.image_count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
