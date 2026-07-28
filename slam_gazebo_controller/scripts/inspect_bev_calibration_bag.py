#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


SUPPORTED_IMAGE_TYPES = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
}
PREFERRED_IMAGE_TOPICS = (
    "/wide_camera_mjpeg/image_raw/compressed",
    "/wide_camera/rect/image_raw/compressed",
    "/wide_camera/rect/image_raw",
    "/image_raw/compressed",
    "/image_raw",
)


def open_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def choose_image_topic(topic_types: dict[str, str], requested: str) -> str:
    if requested:
        if topic_types.get(requested) not in SUPPORTED_IMAGE_TYPES:
            raise RuntimeError(
                f"unsupported or missing image topic: {requested}"
            )
        return requested
    for topic in PREFERRED_IMAGE_TOPICS:
        if topic_types.get(topic) in SUPPORTED_IMAGE_TYPES:
            return topic
    for topic, message_type in topic_types.items():
        if message_type in SUPPORTED_IMAGE_TYPES:
            return topic
    raise RuntimeError("the bag contains no supported image topic")


def decode_image(message, message_type: str, bridge: CvBridge) -> np.ndarray:
    if message_type == "sensor_msgs/msg/CompressedImage":
        image = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        if image is None:
            raise RuntimeError("OpenCV failed to decode a compressed frame")
        return image
    return bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")


def add_label(image: np.ndarray, label: str) -> np.ndarray:
    output = image.copy()
    cv2.rectangle(output, (0, 0), (output.shape[1], 42), (0, 0, 0), -1)
    cv2.putText(
        output,
        label,
        (12, 29),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return output


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate a real-camera BEV calibration rosbag."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--image-topic", default="")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for the preview and JSON report (defaults to BAG).",
    )
    args = parser.parse_args()

    bag_path = args.bag.expanduser().resolve()
    if not (bag_path / "metadata.yaml").is_file():
        raise RuntimeError(f"metadata.yaml is missing from {bag_path}")
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else bag_path
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    reader = open_reader(bag_path)
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    image_topic = choose_image_topic(topic_types, args.image_topic)
    image_type = topic_types[image_topic]
    image_message_type = get_message(image_type)

    frame_count = 0
    first_timestamp = None
    last_timestamp = None
    while reader.has_next():
        topic, _, timestamp = reader.read_next()
        if topic != image_topic:
            continue
        frame_count += 1
        if first_timestamp is None:
            first_timestamp = timestamp
        last_timestamp = timestamp
    if frame_count == 0 or first_timestamp is None or last_timestamp is None:
        raise RuntimeError(f"no messages found on {image_topic}")

    target_indices = {0, frame_count // 2, frame_count - 1}
    samples: dict[int, np.ndarray] = {}
    reader = open_reader(bag_path)
    bridge = CvBridge()
    index = 0
    while reader.has_next() and len(samples) < len(target_indices):
        topic, data, _ = reader.read_next()
        if topic != image_topic:
            continue
        if index in target_indices:
            message = deserialize_message(data, image_message_type)
            samples[index] = decode_image(message, image_type, bridge)
        index += 1

    duration_sec = max(0.0, (last_timestamp - first_timestamp) * 1e-9)
    average_hz = (
        (frame_count - 1) / duration_sec if duration_sec > 0.0 else 0.0
    )
    first_image = samples[min(samples)]
    height, width = first_image.shape[:2]
    panels = []
    for sample_index in sorted(samples):
        panel = cv2.resize(
            samples[sample_index],
            (640, 512),
            interpolation=cv2.INTER_AREA,
        )
        panels.append(
            add_label(panel, f"frame {sample_index}/{frame_count - 1}")
        )
    preview_path = output_dir / "bev_calibration_preview.png"
    cv2.imwrite(str(preview_path), np.hstack(panels))

    report = {
        "bag": str(bag_path),
        "image_topic": image_topic,
        "image_type": image_type,
        "frame_count": frame_count,
        "duration_sec": round(duration_sec, 3),
        "average_hz": round(average_hz, 3),
        "image_width": width,
        "image_height": height,
        "preview": str(preview_path),
        "warnings": [],
    }
    if frame_count < 100:
        report["warnings"].append("fewer than 100 camera frames")
    if average_hz < 20.0:
        report["warnings"].append("camera rate below 20 Hz")
    if (width, height) != (1280, 1024):
        report["warnings"].append(
            "resolution differs from calibrated 1280x1024"
        )

    report_path = output_dir / "bev_calibration_report.json"
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, ensure_ascii=True))
    return 0 if not report["warnings"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
