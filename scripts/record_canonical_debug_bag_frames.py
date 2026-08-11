#!/usr/bin/env python3
"""Save synchronized canonical-perception topics as a debug dataset."""

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


SAMPLE_FIELDS = [
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

DEBUG_FIELDS = [
    "frame_index",
    "timestamp_ns",
    "time_sec",
    "canonical_path",
    "source_path",
    "white_mask_path",
    "yellow_mask_path",
    "bev_debug_path",
    "tracking_debug_path",
    "white_pixels",
    "yellow_pixels",
    "white_components",
    "yellow_components",
]

TOPIC_DEFAULTS = {
    "canonical": "/perception/canonical_road_image",
    "source": "/perception/rectified_camera_image",
    "white": "/perception/canonical_white_mask",
    "yellow": "/perception/canonical_yellow_mask",
    "bev_debug": "/perception/debug_image",
    "tracking_debug": "/perception/canonical_tracking_debug",
}


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


def component_count(mask: np.ndarray, min_area_px: int = 10) -> int:
    count, _, stats, _ = cv2.connectedComponentsWithStats(
        (mask > 0).astype(np.uint8), connectivity=8
    )
    return sum(
        int(stats[index, cv2.CC_STAT_AREA]) >= min_area_px
        for index in range(1, count)
    )


class CanonicalDebugRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("canonical_debug_bag_frame_recorder")
        self.args = args
        self.output_dir = args.output_dir.expanduser().resolve()
        if self.output_dir.exists() and any(self.output_dir.iterdir()):
            raise RuntimeError(f"output directory is not empty: {self.output_dir}")

        self.directories = {
            "canonical": self.output_dir / "images" / "front",
            "source": self.output_dir / "images" / "source",
            "white": self.output_dir / "masks" / "white",
            "yellow": self.output_dir / "masks" / "yellow",
            "bev_debug": self.output_dir / "debug" / "bev",
            "tracking_debug": self.output_dir / "debug" / "tracking",
        }
        for directory in self.directories.values():
            directory.mkdir(parents=True, exist_ok=True)

        self.samples_path = self.output_dir / "samples.csv"
        self.debug_csv_path = self.output_dir / "frame_debug.csv"
        self.samples_handle = self.samples_path.open(
            "w", newline="", encoding="utf-8"
        )
        self.debug_handle = self.debug_csv_path.open(
            "w", newline="", encoding="utf-8"
        )
        self.samples_writer = csv.DictWriter(
            self.samples_handle, fieldnames=SAMPLE_FIELDS
        )
        self.debug_writer = csv.DictWriter(
            self.debug_handle, fieldnames=DEBUG_FIELDS
        )
        self.samples_writer.writeheader()
        self.debug_writer.writeheader()

        self.bridge = CvBridge()
        self.pending: dict[int, dict[str, Image]] = {}
        self.saved_stamps: set[int] = set()
        self.saved_rows: list[dict[str, str | int | float]] = []
        self.image_count = 0
        self.first_stamp_ns: int | None = None
        self.last_stamp_ns: int | None = None
        self.last_message_wall_time: float | None = None
        self.start_wall_time = time.monotonic()
        self.finished = False
        self.exit_reason = "running"

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._topic_subscriptions = []
        for key, topic in args.topics.items():
            self._topic_subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda message, topic_key=key: self.on_image(
                        topic_key, message
                    ),
                    qos,
                )
            )
        self.timer = self.create_timer(0.25, self.check_completion)
        self.get_logger().info(
            f"recording synchronized canonical debug frames -> {self.output_dir}"
        )

    def on_image(self, key: str, message: Image) -> None:
        timestamp_ns = stamp_ns(message)
        if timestamp_ns <= 0 or timestamp_ns in self.saved_stamps:
            return
        self.last_message_wall_time = time.monotonic()
        bundle = self.pending.setdefault(timestamp_ns, {})
        bundle[key] = message
        if all(required in bundle for required in TOPIC_DEFAULTS):
            self.save_bundle(timestamp_ns, bundle)
            self.pending.pop(timestamp_ns, None)
        self.drop_stale_pending()

    def drop_stale_pending(self) -> None:
        if len(self.pending) <= 200:
            return
        for timestamp_ns in sorted(self.pending)[:-100]:
            self.pending.pop(timestamp_ns, None)

    def save_bundle(self, timestamp_ns: int, bundle: dict[str, Image]) -> None:
        images = {
            "canonical": self.bridge.imgmsg_to_cv2(
                bundle["canonical"], desired_encoding="bgr8"
            ),
            "source": self.bridge.imgmsg_to_cv2(
                bundle["source"], desired_encoding="bgr8"
            ),
            "white": self.bridge.imgmsg_to_cv2(
                bundle["white"], desired_encoding="mono8"
            ),
            "yellow": self.bridge.imgmsg_to_cv2(
                bundle["yellow"], desired_encoding="mono8"
            ),
            "bev_debug": self.bridge.imgmsg_to_cv2(
                bundle["bev_debug"], desired_encoding="bgr8"
            ),
            "tracking_debug": self.bridge.imgmsg_to_cv2(
                bundle["tracking_debug"], desired_encoding="bgr8"
            ),
        }
        if images["canonical"].shape[:2] != (144, 256):
            raise RuntimeError(
                "canonical image contract changed: expected 256x144, got "
                f"{images['canonical'].shape[1]}x{images['canonical'].shape[0]}"
            )

        paths = {
            "canonical": self.directories["canonical"] / f"{timestamp_ns}.png",
            "source": self.directories["source"] / f"{timestamp_ns}.jpg",
            "white": self.directories["white"] / f"{timestamp_ns}.png",
            "yellow": self.directories["yellow"] / f"{timestamp_ns}.png",
            "bev_debug": self.directories["bev_debug"] / f"{timestamp_ns}.jpg",
            "tracking_debug": self.directories["tracking_debug"]
            / f"{timestamp_ns}.png",
        }
        png_options = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
        jpg_options = [int(cv2.IMWRITE_JPEG_QUALITY), 92]
        for key in ("canonical", "white", "yellow", "tracking_debug"):
            if not cv2.imwrite(str(paths[key]), images[key], png_options):
                raise RuntimeError(f"failed to write {paths[key]}")
        for key in ("source", "bev_debug"):
            if not cv2.imwrite(str(paths[key]), images[key], jpg_options):
                raise RuntimeError(f"failed to write {paths[key]}")

        if self.first_stamp_ns is None:
            self.first_stamp_ns = timestamp_ns
        time_sec = (timestamp_ns - self.first_stamp_ns) * 1e-9
        relative = {
            key: str(path.relative_to(self.output_dir))
            for key, path in paths.items()
        }
        self.samples_writer.writerow(
            {
                "timestamp_ns": timestamp_ns,
                "image_timestamp_ns": timestamp_ns,
                "scan_timestamp_ns": "",
                "scan_time_offset_ms": "",
                "front_image_path": relative["canonical"],
                "scan_npz_path": "",
                "motor_angle": "",
                "motor_speed": "",
                "mission_label": "unlabeled_temp_track_debug",
                "dataset_profile": "real_bag_canonical_debug",
                "session_id": self.output_dir.name,
            }
        )
        debug_row = {
            "frame_index": self.image_count,
            "timestamp_ns": timestamp_ns,
            "time_sec": round(time_sec, 6),
            "canonical_path": relative["canonical"],
            "source_path": relative["source"],
            "white_mask_path": relative["white"],
            "yellow_mask_path": relative["yellow"],
            "bev_debug_path": relative["bev_debug"],
            "tracking_debug_path": relative["tracking_debug"],
            "white_pixels": int(np.count_nonzero(images["white"])),
            "yellow_pixels": int(np.count_nonzero(images["yellow"])),
            "white_components": component_count(images["white"]),
            "yellow_components": component_count(images["yellow"]),
        }
        self.debug_writer.writerow(debug_row)
        self.saved_rows.append(debug_row)
        self.saved_stamps.add(timestamp_ns)
        self.image_count += 1
        self.last_stamp_ns = timestamp_ns

        if self.image_count % 100 == 0:
            self.samples_handle.flush()
            self.debug_handle.flush()
            self.get_logger().info(f"saved {self.image_count} frame bundles")
        if self.args.expected_frames and self.image_count >= self.args.expected_frames:
            self.finish("expected_frame_count_reached")

    def check_completion(self) -> None:
        if self.finished or self.last_message_wall_time is None:
            return
        if time.monotonic() - self.last_message_wall_time >= self.args.idle_timeout_sec:
            self.finish("image_topics_idle")

    def finish(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        self.exit_reason = reason
        self.samples_handle.flush()
        self.debug_handle.flush()
        self.write_contact_sheet()
        self.write_metadata()
        self.get_logger().info(
            f"finished: reason={reason}, synchronized_frames={self.image_count}"
        )

    def write_contact_sheet(self) -> None:
        if not self.saved_rows:
            return
        sample_count = min(16, len(self.saved_rows))
        indices = np.linspace(0, len(self.saved_rows) - 1, sample_count)
        panels: list[np.ndarray] = []
        for value in indices:
            row = self.saved_rows[int(round(value))]
            source = cv2.imread(str(self.output_dir / str(row["source_path"])))
            canonical = cv2.imread(
                str(self.output_dir / str(row["canonical_path"]))
            )
            white = cv2.imread(
                str(self.output_dir / str(row["white_mask_path"])),
                cv2.IMREAD_GRAYSCALE,
            )
            yellow = cv2.imread(
                str(self.output_dir / str(row["yellow_mask_path"])),
                cv2.IMREAD_GRAYSCALE,
            )
            canvas = np.full((300, 640, 3), 28, dtype=np.uint8)
            canvas[40:296, :320] = cv2.resize(
                source, (320, 256), interpolation=cv2.INTER_AREA
            )
            canvas[40:220, 320:] = cv2.resize(
                canonical, (320, 180), interpolation=cv2.INTER_NEAREST
            )
            mask_panel = np.zeros((72, 320, 3), dtype=np.uint8)
            white_small = cv2.resize(
                white, (320, 72), interpolation=cv2.INTER_NEAREST
            )
            yellow_small = cv2.resize(
                yellow, (320, 72), interpolation=cv2.INTER_NEAREST
            )
            mask_panel[white_small > 0] = (255, 255, 255)
            mask_panel[yellow_small > 0] = (0, 255, 255)
            canvas[224:296, 320:] = mask_panel
            cv2.putText(
                canvas,
                f"idx={row['frame_index']} t={float(row['time_sec']):.2f}s",
                (8, 27),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
            panels.append(canvas)

        rows = []
        blank = np.full_like(panels[0], 28)
        for offset in range(0, len(panels), 2):
            pair = panels[offset: offset + 2]
            if len(pair) == 1:
                pair.append(blank)
            rows.append(np.hstack(pair))
        cv2.imwrite(
            str(self.output_dir / "debug" / "contact_sheet.jpg"),
            np.vstack(rows),
            [int(cv2.IMWRITE_JPEG_QUALITY), 92],
        )

    def write_metadata(self) -> None:
        atomic_json(
            self.output_dir / "metadata.json",
            {
                "canonical_contract": {
                    "encoding": "bgr8",
                    "forward_range_m": 1.5,
                    "height": 144,
                    "lateral_range_m": 1.4,
                    "observation_only": True,
                    "width": 256,
                },
                "collected_frames": self.image_count,
                "complete": self.image_count > 0,
                "created_at_utc": datetime.now(timezone.utc).isoformat(),
                "dataset_profile": "real_bag_canonical_debug",
                "exit_reason": self.exit_reason,
                "first_timestamp_ns": self.first_stamp_ns,
                "last_timestamp_ns": self.last_stamp_ns,
                "labels": {
                    "motor_angle": False,
                    "motor_speed": False,
                    "reason": "perception-debug export; control labels intentionally omitted",
                },
                "saved_topics": self.args.topics,
                "session_id": self.output_dir.name,
                "source_bag": str(self.args.source_bag.expanduser().resolve()),
                "training_ready": False,
                "use": [
                    "frame-by-frame perception debugging",
                    "canonical mask inspection",
                    "temporary-track versus competition-track comparison",
                ],
                "wall_time_sec": round(time.monotonic() - self.start_wall_time, 3),
            },
        )

    def close(self) -> None:
        if not self.finished:
            self.finish("interrupted")
        for handle in (self.samples_handle, self.debug_handle):
            if not handle.closed:
                handle.close()


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--source-bag", type=Path, required=True)
    parser.add_argument("--expected-frames", type=int, default=0)
    parser.add_argument("--idle-timeout-sec", type=float, default=5.0)
    for key, default in TOPIC_DEFAULTS.items():
        parser.add_argument(f"--{key.replace('_', '-')}-topic", default=default)
    args, ros_args = parser.parse_known_args()
    args.topics = {
        key: getattr(args, f"{key}_topic") for key in TOPIC_DEFAULTS
    }
    return args, ros_args


def main() -> int:
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = CanonicalDebugRecorder(args)

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
