#!/usr/bin/env python3
"""Find the rosbag S offset: two missing left_4 frames after confirmation."""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage

from lane_seg_control.camera_input import CameraRectifier

from .trigger_detector import Left4TriggerDetector, TriggerConfig


DEFAULT_SOURCE_TOPIC = "/wide_camera_mjpeg/image_raw/compressed"


def positive_float(value: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return number


def probability(value: str) -> float:
    number = float(value)
    if not 0.0 <= number <= 1.0:
        raise argparse.ArgumentTypeError("value must be in [0, 1]")
    return number


def build_parser() -> argparse.ArgumentParser:
    my_rule_share = Path(get_package_share_directory("my_rule"))
    perception_share = Path(get_package_share_directory("xycar_perception"))
    parser = argparse.ArgumentParser(
        description=(
            "Run the production object model at detector cadence and find "
            "the second missing left_4 frame (S)."
        )
    )
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--source-topic", default=DEFAULT_SOURCE_TOPIC)
    parser.add_argument(
        "--object-model",
        type=Path,
        default=(my_rule_share / "models" / "kookmin_objects_best_20260804.pt"),
    )
    parser.add_argument(
        "--camera-yaml",
        type=Path,
        default=(
            perception_share
            / "config"
            / "wide_camera_fisheye_1280x1024_20260708.yaml"
        ),
    )
    parser.add_argument("--detector-hz", type=positive_float, default=3.0)
    parser.add_argument("--minimum-confidence", type=probability, default=0.50)
    parser.add_argument("--required-visible-frames", type=int, default=2)
    parser.add_argument("--required-absent-frames", type=int, default=2)
    parser.add_argument("--input-size", type=int, default=640)
    parser.add_argument("--iou-threshold", type=probability, default=0.50)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--cpu-threads", type=int, default=2)
    parser.add_argument("--rect-balance", type=float, default=0.3)
    return parser


def message_stamp_ns(message: CompressedImage, storage_stamp_ns: int) -> int:
    stamp_ns = (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )
    return stamp_ns if stamp_ns > 0 else int(storage_stamp_ns)


def best_left_confidence(result, model) -> float:
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return 0.0
    names = getattr(result, "names", getattr(model, "names", {}))
    confidences = boxes.conf.detach().cpu().numpy()
    class_ids = boxes.cls.detach().cpu().numpy().astype(int)
    best = 0.0
    for class_id, confidence in zip(class_ids, confidences):
        if isinstance(names, dict):
            class_name = str(names.get(int(class_id), class_id))
        else:
            class_name = str(names[int(class_id)])
        if class_name == "left_4":
            best = max(best, float(confidence))
    return best


def analyze(args: argparse.Namespace) -> dict:
    bag_path = args.bag_path.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    object_model = args.object_model.expanduser().resolve()
    camera_yaml = args.camera_yaml.expanduser().resolve()
    if not bag_path.is_dir():
        raise FileNotFoundError(f"rosbag directory not found: {bag_path}")
    if not object_model.is_file():
        raise FileNotFoundError(f"object model not found: {object_model}")
    if not camera_yaml.is_file():
        raise FileNotFoundError(f"camera calibration not found: {camera_yaml}")
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    import torch
    from ultralytics import YOLO

    torch.set_num_threads(max(1, int(args.cpu_threads)))
    try:
        torch.set_num_interop_threads(1)
    except RuntimeError:
        pass
    cv2.setNumThreads(1)
    model = YOLO(str(object_model))
    rectifier = CameraRectifier(str(camera_yaml), float(args.rect_balance))
    detector = Left4TriggerDetector(
        TriggerConfig(
            minimum_confidence=float(args.minimum_confidence),
            required_visible_frames=int(args.required_visible_frames),
            required_absent_frames=int(args.required_absent_frames),
        )
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    expected_type = "sensor_msgs/msg/CompressedImage"
    if topics.get(args.source_topic) != expected_type:
        raise RuntimeError(
            f"{args.source_topic} must have type {expected_type}; "
            f"bag has {topics.get(args.source_topic)!r}"
        )

    period_ns = int(round(1_000_000_000.0 / float(args.detector_hz)))
    first_stamp_ns: int | None = None
    next_sample_ns: int | None = None
    last_seen_offset_sec: float | None = None
    first_missing_offset_sec: float | None = None
    s_offset_sec: float | None = None
    s_timestamp_ns: int | None = None
    sampled_frames = 0
    source_messages = 0
    rows: list[dict] = []
    started = time.perf_counter()

    while reader.has_next():
        topic, serialized, storage_stamp_ns = reader.read_next()
        if topic != args.source_topic:
            continue
        source_messages += 1
        message = deserialize_message(serialized, CompressedImage)
        stamp_ns = message_stamp_ns(message, int(storage_stamp_ns))
        if first_stamp_ns is None:
            first_stamp_ns = stamp_ns
            next_sample_ns = stamp_ns
        if next_sample_ns is None or stamp_ns < next_sample_ns:
            continue
        while next_sample_ns <= stamp_ns:
            next_sample_ns += period_ns

        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            continue
        frame = rectifier.rectify(frame)
        results = model.predict(
            source=frame,
            imgsz=int(args.input_size),
            conf=min(0.50, float(args.minimum_confidence)),
            iou=float(args.iou_threshold),
            max_det=50,
            device=str(args.device),
            half=False,
            verbose=False,
        )
        confidence = best_left_confidence(results[0], model) if results else 0.0
        before_absent = detector.absent_streak
        state = detector.update(confidence)
        offset_sec = (stamp_ns - first_stamp_ns) / 1_000_000_000.0
        if state.present:
            last_seen_offset_sec = offset_sec
            first_missing_offset_sec = None
        elif state.confirmed and state.absent_streak == 1:
            first_missing_offset_sec = offset_sec
        newly_triggered = bool(
            state.triggered
            and s_offset_sec is None
            and before_absent < int(args.required_absent_frames)
        )
        if newly_triggered:
            s_offset_sec = offset_sec
            s_timestamp_ns = stamp_ns

        rows.append(
            {
                "sample_index": sampled_frames,
                "timestamp_ns": stamp_ns,
                "offset_sec": offset_sec,
                "left_4_confidence": confidence,
                "present": int(state.present),
                "visible_streak": state.visible_streak,
                "confirmed": int(state.confirmed),
                "absent_streak": state.absent_streak,
                "s_trigger": int(newly_triggered),
            }
        )
        sampled_frames += 1
        if sampled_frames % 30 == 0:
            print(
                f"scanned {offset_sec:.1f}s ({sampled_frames} detector frames)",
                flush=True,
            )
        if newly_triggered:
            break

    timeline_path = output_dir / "left_4_timeline.csv"
    with timeline_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else [])
        if rows:
            writer.writeheader()
            writer.writerows(rows)

    elapsed_sec = time.perf_counter() - started
    summary = {
        "bag_path": str(bag_path),
        "source_topic": str(args.source_topic),
        "object_model": str(object_model),
        "camera_yaml": str(camera_yaml),
        "detector_hz": float(args.detector_hz),
        "minimum_confidence": float(args.minimum_confidence),
        "required_visible_frames": int(args.required_visible_frames),
        "required_absent_frames": int(args.required_absent_frames),
        "source_messages_scanned": int(source_messages),
        "detector_frames_scanned": int(sampled_frames),
        "last_seen_offset_sec": last_seen_offset_sec,
        "first_missing_offset_sec": first_missing_offset_sec,
        "s_offset_sec": s_offset_sec,
        "s_timestamp_ns": s_timestamp_ns,
        "suggested_review_start_offset_sec": (
            max(0.0, s_offset_sec - 3.0) if s_offset_sec is not None else None
        ),
        "elapsed_processing_sec": elapsed_sec,
        "s_definition": (
            "second missing 3 Hz detector frame after left_4 was present in "
            "two consecutive detector frames"
        ),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> None:
    args = build_parser().parse_args()
    summary = analyze(args)
    print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    if summary["s_offset_sec"] is None:
        raise SystemExit("left_4 S point was not found")


if __name__ == "__main__":
    main()
