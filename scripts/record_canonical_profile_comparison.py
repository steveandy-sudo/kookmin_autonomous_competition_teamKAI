#!/usr/bin/env python3
"""Save source, competition, and temporary canonical outputs per frame."""

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
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


TOPICS = {
    "source": "/profile_compare/competition/rectified_camera_image",
    "competition": "/profile_compare/competition/canonical_road_image",
    "competition_tracking": (
        "/profile_compare/competition/canonical_tracking_debug"
    ),
    "temporary": "/profile_compare/temporary/canonical_road_image",
    "temporary_tracking": (
        "/profile_compare/temporary/canonical_tracking_debug"
    ),
}

CSV_FIELDS = [
    "frame_index",
    "timestamp_ns",
    "time_sec",
    "comparison_path",
    "source_path",
    "competition_path",
    "temporary_path",
    "competition_tracking_path",
    "temporary_tracking_path",
    "different_pixels",
    "agreement_ratio",
    "competition_white_pixels",
    "competition_yellow_pixels",
    "temporary_white_pixels",
    "temporary_yellow_pixels",
]


def stamp_ns(message: Image) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def atomic_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, delete=False
    ) as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False, sort_keys=True)
        handle.write("\n")
        temporary = handle.name
    os.replace(temporary, path)


def letterbox(image: np.ndarray, width: int, height: int) -> np.ndarray:
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(
        image,
        (
            max(1, int(round(image.shape[1] * scale))),
            max(1, int(round(image.shape[0] * scale))),
        ),
        interpolation=cv2.INTER_AREA,
    )
    canvas = np.full((height, width, 3), 22, dtype=np.uint8)
    top = (height - resized.shape[0]) // 2
    left = (width - resized.shape[1]) // 2
    canvas[top : top + resized.shape[0], left : left + resized.shape[1]] = (
        resized
    )
    return canvas


def canonical_pixel_counts(image: np.ndarray) -> tuple[int, int]:
    white = np.all(image == np.array([255, 255, 255]), axis=2)
    yellow = np.all(image == np.array([0, 220, 255]), axis=2)
    return int(np.count_nonzero(white)), int(np.count_nonzero(yellow))


class ProfileComparisonRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("canonical_profile_comparison_recorder")
        self.args = args
        self.output_dir = args.output_dir.expanduser().resolve()
        if self.output_dir.exists() and any(self.output_dir.iterdir()):
            raise RuntimeError(
                f"output directory is not empty: {self.output_dir}"
            )
        self.directories = {
            "comparison": self.output_dir / "comparison_frames",
            "source": self.output_dir / "source",
            "competition": self.output_dir / "competition_canonical",
            "competition_tracking": (
                self.output_dir / "competition_tracking"
            ),
            "temporary": self.output_dir / "temporary_canonical",
            "temporary_tracking": self.output_dir / "temporary_tracking",
        }
        for directory in self.directories.values():
            directory.mkdir(parents=True, exist_ok=True)

        self.csv_handle = (self.output_dir / "frames.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.writer = csv.DictWriter(
            self.csv_handle, fieldnames=CSV_FIELDS
        )
        self.writer.writeheader()
        self.bridge = CvBridge()
        self.pending: dict[int, dict[str, Image]] = {}
        self.rows: list[dict[str, str | int | float]] = []
        self.saved_stamps: set[int] = set()
        self.first_stamp_ns: int | None = None
        self.last_message_wall: float | None = None
        self.start_wall = time.monotonic()
        self.finished = False
        self.exit_reason = "running"

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_subscriptions = [
            self.create_subscription(
                Image,
                topic,
                lambda message, key=key: self.on_image(key, message),
                qos,
            )
            for key, topic in TOPICS.items()
        ]
        self.timer = self.create_timer(0.25, self.check_completion)
        self.get_logger().info(
            f"saving canonical profile comparison -> {self.output_dir}"
        )

    def on_image(self, key: str, message: Image) -> None:
        timestamp_ns = stamp_ns(message)
        if timestamp_ns <= 0 or timestamp_ns in self.saved_stamps:
            return
        self.last_message_wall = time.monotonic()
        bundle = self.pending.setdefault(timestamp_ns, {})
        bundle[key] = message
        if all(required in bundle for required in TOPICS):
            self.save_bundle(timestamp_ns, bundle)
            self.pending.pop(timestamp_ns, None)
        if len(self.pending) > 300:
            for stale in sorted(self.pending)[:-150]:
                self.pending.pop(stale, None)

    def save_bundle(self, timestamp_ns: int, bundle: dict[str, Image]) -> None:
        images = {
            key: self.bridge.imgmsg_to_cv2(message, "bgr8")
            for key, message in bundle.items()
        }
        for key in ("competition", "temporary"):
            if images[key].shape[:2] != (144, 256):
                raise RuntimeError(
                    f"{key} canonical is not 256x144: {images[key].shape}"
                )

        index = len(self.rows)
        stem = f"{timestamp_ns}"
        paths = {
            "source": self.directories["source"] / f"{stem}.jpg",
            "competition": self.directories["competition"] / f"{stem}.png",
            "competition_tracking": (
                self.directories["competition_tracking"] / f"{stem}.png"
            ),
            "temporary": self.directories["temporary"] / f"{stem}.png",
            "temporary_tracking": (
                self.directories["temporary_tracking"] / f"{stem}.png"
            ),
            "comparison": (
                self.directories["comparison"] / f"{index:06d}.jpg"
            ),
        }
        png_options = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
        jpg_options = [int(cv2.IMWRITE_JPEG_QUALITY), 91]
        cv2.imwrite(str(paths["source"]), images["source"], jpg_options)
        for key in (
            "competition",
            "competition_tracking",
            "temporary",
            "temporary_tracking",
        ):
            cv2.imwrite(str(paths[key]), images[key], png_options)

        panel = self.make_panel(index, timestamp_ns, images)
        cv2.imwrite(str(paths["comparison"]), panel, jpg_options)

        different = np.any(
            images["competition"] != images["temporary"], axis=2
        )
        competition_white, competition_yellow = canonical_pixel_counts(
            images["competition"]
        )
        temporary_white, temporary_yellow = canonical_pixel_counts(
            images["temporary"]
        )
        if self.first_stamp_ns is None:
            self.first_stamp_ns = timestamp_ns
        relative = {
            key: str(path.relative_to(self.output_dir))
            for key, path in paths.items()
        }
        row = {
            "frame_index": index,
            "timestamp_ns": timestamp_ns,
            "time_sec": round(
                (timestamp_ns - self.first_stamp_ns) * 1.0e-9, 6
            ),
            "comparison_path": relative["comparison"],
            "source_path": relative["source"],
            "competition_path": relative["competition"],
            "temporary_path": relative["temporary"],
            "competition_tracking_path": relative[
                "competition_tracking"
            ],
            "temporary_tracking_path": relative["temporary_tracking"],
            "different_pixels": int(np.count_nonzero(different)),
            "agreement_ratio": round(1.0 - float(np.mean(different)), 6),
            "competition_white_pixels": competition_white,
            "competition_yellow_pixels": competition_yellow,
            "temporary_white_pixels": temporary_white,
            "temporary_yellow_pixels": temporary_yellow,
        }
        self.writer.writerow(row)
        self.rows.append(row)
        self.saved_stamps.add(timestamp_ns)
        if len(self.rows) % 100 == 0:
            self.csv_handle.flush()
            self.get_logger().info(f"saved {len(self.rows)} comparisons")
        if self.args.expected_frames and len(self.rows) >= self.args.expected_frames:
            self.finish("expected_frame_count_reached")

    def make_panel(
        self,
        index: int,
        timestamp_ns: int,
        images: dict[str, np.ndarray],
    ) -> np.ndarray:
        width = 512
        top = 288
        bottom = 288
        header = 48
        canvas = np.full(
            (header + top + bottom, width * 3, 3), 22, dtype=np.uint8
        )
        canvas[header:, :width] = letterbox(
            images["source"], width, top + bottom
        )
        canvas[header : header + top, width : width * 2] = cv2.resize(
            images["competition"],
            (width, top),
            interpolation=cv2.INTER_NEAREST,
        )
        canvas[header + top :, width : width * 2] = cv2.resize(
            images["competition_tracking"],
            (width, bottom),
            interpolation=cv2.INTER_NEAREST,
        )
        canvas[header : header + top, width * 2 :] = cv2.resize(
            images["temporary"],
            (width, top),
            interpolation=cv2.INTER_NEAREST,
        )
        canvas[header + top :, width * 2 :] = cv2.resize(
            images["temporary_tracking"],
            (width, bottom),
            interpolation=cv2.INTER_NEAREST,
        )
        labels = (
            "REAL RECTIFIED CAMERA",
            "COMPETITION (TEMP PROFILE)",
            "TEMPORARY PROFILE",
        )
        for column, label in enumerate(labels):
            cv2.putText(
                canvas,
                label,
                (column * width + 12, 29),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 225, 255),
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            canvas,
            f"idx={index:04d} stamp={timestamp_ns}",
            (12, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )
        return canvas

    def check_completion(self) -> None:
        if self.finished or self.last_message_wall is None:
            return
        if time.monotonic() - self.last_message_wall >= self.args.idle_timeout_sec:
            self.finish("image_topics_idle")

    def finish(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.exit_reason = reason
        self.csv_handle.flush()
        self.write_contact_sheet()
        atomic_json(
            self.output_dir / "metadata.json",
            {
                "created_at_utc": datetime.now(timezone.utc).isoformat(),
                "exit_reason": reason,
                "frames": len(self.rows),
                "source_bag": str(self.args.source_bag.expanduser().resolve()),
                "topics": TOPICS,
                "profiles": {
                    "competition": {
                        "expected_half_lane_width_m": 0.49,
                        "white_min_component_median_v": 140.0,
                        "yellow_fit_gate_m": 0.08,
                    },
                    "temporary": {
                        "expected_half_lane_width_m": 0.49,
                        "white_min_component_median_v": 140.0,
                        "yellow_fit_gate_m": 0.08,
                    },
                },
                "wall_time_sec": round(time.monotonic() - self.start_wall, 3),
            },
        )
        self.get_logger().info(
            f"finished: reason={reason}, comparisons={len(self.rows)}"
        )

    def write_contact_sheet(self) -> None:
        if not self.rows:
            return
        indices = np.linspace(0, len(self.rows) - 1, min(12, len(self.rows)))
        panels = []
        for value in indices:
            row = self.rows[int(round(value))]
            image = cv2.imread(
                str(self.output_dir / str(row["comparison_path"]))
            )
            panels.append(
                cv2.resize(image, (768, 312), interpolation=cv2.INTER_AREA)
            )
        while len(panels) % 2:
            panels.append(np.full_like(panels[0], 22))
        sheet = np.vstack(
            [np.hstack(panels[index : index + 2]) for index in range(0, len(panels), 2)]
        )
        cv2.imwrite(
            str(self.output_dir / "contact_sheet.jpg"),
            sheet,
            [int(cv2.IMWRITE_JPEG_QUALITY), 92],
        )

    def close(self) -> None:
        if not self.finished:
            self.finish("interrupted")
        if not self.csv_handle.closed:
            self.csv_handle.close()


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--source-bag", type=Path, required=True)
    parser.add_argument("--expected-frames", type=int, default=0)
    parser.add_argument("--idle-timeout-sec", type=float, default=10.0)
    return parser.parse_known_args()


def main() -> int:
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = ProfileComparisonRecorder(args)

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
    return 0 if node.rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
