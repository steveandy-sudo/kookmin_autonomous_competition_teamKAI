#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    import cv2
    from cv_bridge import CvBridge
    import numpy as np
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:  # pragma: no cover - depends on sourced ROS2 Humble
    cv2 = None
    CvBridge = None
    np = None
    rosbag2_py = None
    deserialize_message = None
    get_message = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None

from il_data_tools.record_utils import (
    atomic_write_json,
    image_suffix,
    make_session_dir,
    motor_from_msg,
    numeric_stats,
    open_samples_csv,
    relative_to_session,
    stamp_to_ns,
    string_from_msg,
    write_session_readme,
)


def nearest(items: Sequence[Tuple[int, Any]], stamp_ns: int, tolerance_ns: int):
    if not items:
        return None
    best = min(items, key=lambda item: abs(item[0] - stamp_ns))
    if abs(best[0] - stamp_ns) <= tolerance_ns:
        return best
    return None


def make_reader(bag_dir: Path):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    return reader, topic_types


def read_index_topics(bag_dir: Path, args) -> Tuple[List[Tuple[int, Any]], List[Tuple[int, Any]], List[Tuple[int, Any]], Dict[str, str]]:
    reader, topic_types = make_reader(bag_dir)
    motor_msgs: List[Tuple[int, Any]] = []
    label_msgs: List[Tuple[int, Any]] = []
    scan_msgs: List[Tuple[int, Any]] = []
    needed = {args.motor_topic, args.mission_label_topic}
    if args.save_scan_npz:
        needed.add(args.scan_topic)

    while reader.has_next():
        topic, data, bag_time = reader.read_next()
        if topic not in needed:
            continue
        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)
        stamp_ns = stamp_to_ns(msg, fallback_ns=bag_time)
        if topic == args.motor_topic:
            motor_msgs.append((stamp_ns, msg))
        elif topic == args.mission_label_topic:
            label_msgs.append((stamp_ns, msg))
        elif topic == args.scan_topic:
            scan_msgs.append((stamp_ns, msg))
    return motor_msgs, label_msgs, scan_msgs, topic_types


def save_image(bridge, session_dir: Path, stamp_ns: int, msg, image_format: str, jpeg_quality: int) -> Path:
    out_path = session_dir / "images" / "front" / f"{stamp_ns}.{image_format}"
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    if image_format == "jpg":
        params = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]
    else:
        params = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
    ok = cv2.imwrite(str(out_path), cv_image, params)
    if not ok:
        raise RuntimeError(f"failed to write image: {out_path}")
    return out_path


def save_scan(session_dir: Path, stamp_ns: int, msg) -> Optional[Path]:
    if np is None:
        return None
    out_path = session_dir / "scan" / f"{stamp_ns}.npz"
    np.savez_compressed(
        out_path,
        ranges=np.array(msg.ranges, dtype=np.float32),
        intensities=np.array(msg.intensities, dtype=np.float32),
        angle_min=float(msg.angle_min),
        angle_max=float(msg.angle_max),
        angle_increment=float(msg.angle_increment),
        range_min=float(msg.range_min),
        range_max=float(msg.range_max),
    )
    return out_path


def estimated_fps(stamps: Sequence[int]) -> float:
    if len(stamps) < 2:
        return 0.0
    duration = (max(stamps) - min(stamps)) / 1e9
    return (len(stamps) - 1) / duration if duration > 0 else 0.0


def json_safe_args(args) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in vars(args).items():
        if isinstance(value, Path):
            result[key] = str(value)
        else:
            result[key] = value
    return result


def convert(args) -> int:
    if IMPORT_ERROR is not None:
        print(f"Could not import ROS2 bag/image dependencies: {IMPORT_ERROR}")
        print()
        print("Fallback:")
        print("  1. source /opt/ros/humble/setup.bash")
        print("  2. source ~/xycar_ws/install/setup.bash")
        print("  3. ros2 launch il_data_tools record_dataset.launch.py session_name:=bag_replay")
        print(f"  4. ros2 bag play {args.bag_dir}")
        return 2

    bag_dir = args.bag_dir.expanduser().resolve()
    image_format = image_suffix(args.image_format)
    session_dir = make_session_dir(str(args.output_dir), args.session_name)
    if args.write_session_readme:
        write_session_readme(
            session_dir,
            args.session_name,
            {
                "camera_front_topic": args.camera_front_topic,
                "scan_topic": args.scan_topic,
                "motor_topic": args.motor_topic,
                "mission_label_topic": args.mission_label_topic,
            },
        )
    csv_handle, writer = open_samples_csv(session_dir / "samples.csv")
    bridge = CvBridge()
    tolerance_ns = int(args.approximate_sync_tolerance_sec * 1e9)
    min_delta_ns = int(1e9 / args.save_rate_hz) if args.save_rate_hz > 0 else 0

    motor_msgs, label_msgs, scan_msgs, topic_types = read_index_topics(bag_dir, args)
    reader, _ = make_reader(bag_dir)

    image_stamps: List[int] = []
    saved_stamps: List[int] = []
    motor_stamps = [stamp for stamp, _ in motor_msgs]
    angles: List[float] = []
    speeds: List[float] = []
    labels = Counter()
    missing_motor = 0
    samples = 0
    images = 0
    last_saved_ns: Optional[int] = None

    while reader.has_next():
        topic, data, bag_time = reader.read_next()
        if topic != args.camera_front_topic:
            continue
        msg_type = get_message(topic_types[topic])
        image_msg = deserialize_message(data, msg_type)
        stamp_ns = stamp_to_ns(image_msg, fallback_ns=bag_time)
        image_stamps.append(stamp_ns)
        if last_saved_ns is not None and min_delta_ns > 0 and stamp_ns - last_saved_ns < min_delta_ns:
            continue

        motor_item = nearest(motor_msgs, stamp_ns, tolerance_ns)
        if motor_item is None:
            missing_motor += 1
            if args.require_motor_command:
                continue
            motor_angle, motor_speed = "", ""
        else:
            motor_angle, motor_speed = motor_from_msg(motor_item[1])
            if abs(float(motor_speed)) < args.min_abs_speed_to_save and not args.save_when_stopped:
                continue
            angles.append(float(motor_angle))
            speeds.append(float(motor_speed))

        label_item = nearest(label_msgs, stamp_ns, tolerance_ns)
        mission_label = string_from_msg(label_item[1]) if label_item else "idle"
        labels[mission_label] += 1

        image_path = save_image(
            bridge,
            session_dir,
            stamp_ns,
            image_msg,
            image_format,
            args.jpeg_quality,
        )
        images += 1
        scan_path = ""
        if args.save_scan_npz:
            scan_item = nearest(scan_msgs, stamp_ns, tolerance_ns)
            if scan_item is not None:
                scan_path_obj = save_scan(session_dir, stamp_ns, scan_item[1])
                scan_path = relative_to_session(session_dir, scan_path_obj)

        row = {field: "" for field in writer.fieldnames or []}
        row.update(
            {
                "timestamp_ns": stamp_ns,
                "image_front": relative_to_session(session_dir, image_path),
                "motor_angle": motor_angle,
                "motor_speed": motor_speed,
                "mission_label": mission_label,
                "lap_index": -1,
                "source_mode": args.source_mode,
                "scan_path": scan_path,
                "notes": "converted_from_rosbag2",
            }
        )
        writer.writerow(row)
        samples += 1
        saved_stamps.append(stamp_ns)
        last_saved_ns = stamp_ns

    csv_handle.close()
    report = {
        "bag_dir": str(bag_dir),
        "session_dir": str(session_dir),
        "number_of_images": images,
        "number_of_samples": samples,
        "missing_motor_ratio": (missing_motor / max(1, len(image_stamps))),
        "label_distribution": dict(labels),
        "speed_distribution": numeric_stats(speeds),
        "angle_distribution": numeric_stats(angles),
        "estimated_camera_fps": estimated_fps(image_stamps),
        "estimated_saved_sample_fps": estimated_fps(saved_stamps),
        "estimated_motor_command_fps": estimated_fps(motor_stamps),
        "topic_types": topic_types,
    }
    if args.write_metadata:
        atomic_write_json(session_dir / "conversion_report.json", report)
        atomic_write_json(
            session_dir / "metadata.json",
            {
                "package": "il_data_tools",
                "source": "rosbag2",
                "session_name": args.session_name,
                "session_dir": str(session_dir),
                "parameters": json_safe_args(args),
                "sample_count": samples,
                "image_count": images,
                "conversion_report": report,
                "safety": {
                    "publishes_xycar_motor": False,
                    "note": "bag_to_dataset only reads bags and writes files.",
                },
            },
        )
    print(f"Converted {samples} samples to {session_dir}")
    print(f"Report: {session_dir / 'conversion_report.json'}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Convert a ROS2 bag into an il_data_tools dataset.")
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("~/xycar_ws/datasets/il").expanduser())
    parser.add_argument("--session-name", default="bag_conversion")
    parser.add_argument("--camera-front-topic", default="/usb_cam/image_raw/front")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--motor-topic", default="/xycar_motor")
    parser.add_argument("--mission-label-topic", default="/il/mission_label")
    parser.add_argument("--save-rate-hz", type=float, default=10.0)
    parser.add_argument("--min-abs-speed-to-save", type=float, default=0.0)
    parser.add_argument("--save-when-stopped", action="store_true", default=True)
    parser.add_argument("--drop-stopped", dest="save_when_stopped", action="store_false")
    parser.add_argument("--require-motor-command", action="store_true", default=True)
    parser.add_argument("--allow-missing-motor", dest="require_motor_command", action="store_false")
    parser.add_argument("--approximate-sync-tolerance-sec", type=float, default=0.20)
    parser.add_argument("--image-format", choices=["jpg", "png"], default="jpg")
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument("--save-scan-npz", dest="save_scan_npz", action="store_true", default=True)
    parser.add_argument("--no-save-scan-npz", dest="save_scan_npz", action="store_false")
    parser.add_argument("--write-metadata", dest="write_metadata", action="store_true", default=True)
    parser.add_argument("--no-write-metadata", dest="write_metadata", action="store_false")
    parser.add_argument("--write-session-readme", action="store_true")
    parser.add_argument("--source-mode", default="bag")
    return parser


def main() -> int:
    return convert(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
