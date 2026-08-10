#!/usr/bin/env python3
"""Classify recorded rule paths as straight or curved without replaying ROS."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np
from my_msgs.msg import Centerline
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

from my_control.canonical_stanley_pursuit_driver import (
    path_heading_change_per_m,
    path_segment_heading_changes_per_m,
)


PATH_TOPIC = "/rule_drive/connected_yellow_path"
DIAGNOSTICS_TOPIC = "/rule_drive/diagnostics"
DEBUG_IMAGE_TOPIC = "/recording/rule_canonical_debug/compressed"
SOURCE_IMAGE_TOPIC = "/wide_camera_mjpeg/image_raw/compressed"
PREVIOUS_NEAR_X_M = 0.16
PREVIOUS_FAR_X_M = 0.30
PREVIOUS_SEGMENT_COUNT = 1


@dataclass(frozen=True)
class CurveSample:
    timestamp_ns: int
    elapsed_sec: float
    classification: str
    curvature_rad_per_m: float
    previous_classification: str
    previous_curvature_rad_per_m: float
    threshold_rad_per_m: float
    segment_curvatures_rad_per_m: tuple[float, ...]
    point_count: int
    source: str
    steering_command: float | None = None
    speed_command: float | None = None


def resolve_bag_path(path: Path) -> Path:
    candidate = path.expanduser().resolve()
    if candidate.is_file() and candidate.suffix == ".db3":
        return candidate
    if (candidate / "metadata.yaml").is_file():
        databases = sorted(candidate.glob("*.db3"))
        if len(databases) == 1 and Path(f"{databases[0]}.zstd").is_file():
            return databases[0]
        return candidate
    nested = candidate / "bag"
    if (nested / "metadata.yaml").is_file():
        databases = sorted(nested.glob("*.db3"))
        if len(databases) == 1 and Path(f"{databases[0]}.zstd").is_file():
            return databases[0]
        return nested
    raise FileNotFoundError(
        f"metadata.yaml not found in {candidate} or {nested}"
    )


def open_reader(bag_path: Path, topics: list[str]):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    available = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    selected = [topic for topic in topics if topic in available]
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    return reader, available


def nearest_index(stamps: np.ndarray, timestamp_ns: int) -> int | None:
    if stamps.size == 0:
        return None
    insertion = int(np.searchsorted(stamps, int(timestamp_ns)))
    candidates = [
        index
        for index in (insertion - 1, insertion)
        if 0 <= index < stamps.size
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda index: abs(int(stamps[index]) - timestamp_ns))


def read_topic_stamps(bag_path: Path, topic_name: str) -> np.ndarray:
    reader, available = open_reader(bag_path, [topic_name])
    if topic_name not in available:
        return np.empty(0, dtype=np.int64)
    stamps: list[int] = []
    while reader.has_next():
        topic, _serialized, timestamp_ns = reader.read_next()
        if topic == topic_name:
            stamps.append(int(timestamp_ns))
    return np.asarray(stamps, dtype=np.int64)


def fit_to_panel(image: np.ndarray, width: int, height: int) -> np.ndarray:
    if image is None or image.size == 0:
        return np.zeros((height, width, 3), dtype=np.uint8)
    source_height, source_width = image.shape[:2]
    scale = min(width / source_width, height / source_height)
    resized_width = max(1, int(round(source_width * scale)))
    resized_height = max(1, int(round(source_height * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_NEAREST
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=interpolation,
    )
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    left = (width - resized_width) // 2
    top = (height - resized_height) // 2
    panel[top : top + resized_height, left : left + resized_width] = resized
    return panel


def read_diagnostics(
    bag_path: Path,
) -> tuple[np.ndarray, np.ndarray]:
    reader, available = open_reader(bag_path, [DIAGNOSTICS_TOPIC])
    if DIAGNOSTICS_TOPIC not in available:
        return np.empty(0, dtype=np.int64), np.empty((0, 2), dtype=np.float64)
    stamps: list[int] = []
    commands: list[tuple[float, float]] = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic != DIAGNOSTICS_TOPIC:
            continue
        message = deserialize_message(serialized, Float32MultiArray)
        if len(message.data) < 11:
            continue
        stamps.append(int(timestamp_ns))
        commands.append((float(message.data[9]), float(message.data[10])))
    return (
        np.asarray(stamps, dtype=np.int64),
        np.asarray(commands, dtype=np.float64).reshape(-1, 2),
    )


def classify_paths(
    bag_path: Path,
    *,
    near_x_m: float,
    far_x_m: float,
    segment_count: int,
    threshold: float,
) -> list[CurveSample]:
    diagnostic_stamps, commands = read_diagnostics(bag_path)
    reader, available = open_reader(bag_path, [PATH_TOPIC])
    if PATH_TOPIC not in available:
        raise RuntimeError(f"required topic is missing: {PATH_TOPIC}")

    raw_samples: list[
        tuple[
            int,
            float,
            tuple[float, ...],
            float,
            int,
            str,
            float | None,
            float | None,
        ]
    ] = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic != PATH_TOPIC:
            continue
        message = deserialize_message(serialized, Centerline)
        path = np.asarray(
            [(float(point.x), float(point.y)) for point in message.points],
            dtype=np.float64,
        ).reshape(-1, 2)
        segment_curvatures = path_segment_heading_changes_per_m(
            path,
            near_x_m=near_x_m,
            far_x_m=far_x_m,
            segment_count=segment_count,
        )
        curvature = float(np.max(segment_curvatures))
        previous_curvature = path_heading_change_per_m(
            path,
            near_x_m=PREVIOUS_NEAR_X_M,
            far_x_m=PREVIOUS_FAR_X_M,
            segment_count=PREVIOUS_SEGMENT_COUNT,
        )
        steering = None
        speed = None
        index = nearest_index(diagnostic_stamps, int(timestamp_ns))
        if index is not None and abs(int(diagnostic_stamps[index]) - int(timestamp_ns)) <= 250_000_000:
            steering = float(commands[index, 0])
            speed = float(commands[index, 1])
        raw_samples.append(
            (
                int(timestamp_ns),
                float(curvature),
                tuple(float(value) for value in segment_curvatures),
                float(previous_curvature),
                int(path.shape[0]),
                str(message.source),
                steering,
                speed,
            )
        )

    if not raw_samples:
        raise RuntimeError(f"no messages were read from {PATH_TOPIC}")
    first_timestamp = raw_samples[0][0]
    return [
        CurveSample(
            timestamp_ns=timestamp_ns,
            elapsed_sec=(timestamp_ns - first_timestamp) / 1.0e9,
            classification=(
                "STRAIGHT" if curvature <= threshold else "CURVE"
            ),
            curvature_rad_per_m=curvature,
            previous_classification=(
                "STRAIGHT" if previous_curvature <= threshold else "CURVE"
            ),
            previous_curvature_rad_per_m=previous_curvature,
            threshold_rad_per_m=threshold,
            segment_curvatures_rad_per_m=segment_curvatures,
            point_count=point_count,
            source=source,
            steering_command=steering,
            speed_command=speed,
        )
        for timestamp_ns, curvature, segment_curvatures, previous_curvature, point_count, source, steering, speed
        in raw_samples
    ]


def write_csv(path: Path, samples: list[CurveSample]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(asdict(samples[0])))
        writer.writeheader()
        writer.writerows(asdict(sample) for sample in samples)


def write_summary(path: Path, samples: list[CurveSample], args) -> dict:
    counts = {
        label: sum(sample.classification == label for sample in samples)
        for label in ("STRAIGHT", "CURVE")
    }
    previous_counts = {
        label: sum(
            sample.previous_classification == label for sample in samples
        )
        for label in ("STRAIGHT", "CURVE")
    }
    transitions = []
    previous = samples[0].classification
    for sample in samples[1:]:
        if sample.classification == previous:
            continue
        transitions.append(
            {
                "elapsed_sec": round(sample.elapsed_sec, 3),
                "from": previous,
                "to": sample.classification,
                "curvature_rad_per_m": sample.curvature_rad_per_m,
            }
        )
        previous = sample.classification
    finite_curvatures = np.asarray(
        [
            sample.curvature_rad_per_m
            for sample in samples
            if math.isfinite(sample.curvature_rad_per_m)
        ],
        dtype=np.float64,
    )
    summary = {
        "bag": str(args.bag),
        "path_topic": PATH_TOPIC,
        "sample_count": len(samples),
        "duration_sec": samples[-1].elapsed_sec,
        "settings": {
            "near_x_m": args.near,
            "far_x_m": args.far,
            "segment_count": args.segments,
            "threshold_rad_per_m": args.threshold,
        },
        "counts": counts,
        "previous_counts": previous_counts,
        "ratios": {
            key: value / len(samples) for key, value in counts.items()
        },
        "finite_curvature": {
            "median": float(np.median(finite_curvatures)),
            "p90": float(np.percentile(finite_curvatures, 90.0)),
            "maximum": float(np.max(finite_curvatures)),
        },
        "transition_count": len(transitions),
        "classification_disagreement_count": sum(
            sample.classification != sample.previous_classification
            for sample in samples
        ),
        "transitions": transitions,
    }
    path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary


def write_timeline(path: Path, samples: list[CurveSample]) -> None:
    width = 1800
    height = 640
    margin_left = 90
    margin_right = 30
    margin_top = 70
    margin_bottom = 90
    canvas = np.full((height, width, 3), 24, dtype=np.uint8)
    times = np.asarray([sample.elapsed_sec for sample in samples])
    curvatures = np.asarray(
        [sample.curvature_rad_per_m for sample in samples], dtype=np.float64
    )
    finite = curvatures[np.isfinite(curvatures)]
    display_max = max(
        samples[0].threshold_rad_per_m * 2.0,
        float(np.percentile(finite, 98.0)) if finite.size else 1.0,
    )
    duration = max(1.0e-6, float(times[-1]))
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    def pixel_x(elapsed: float) -> int:
        return margin_left + int(round((elapsed / duration) * plot_width))

    def pixel_y(value: float) -> int:
        clipped = min(max(0.0, value), display_max)
        return margin_top + plot_height - int(round(clipped / display_max * plot_height))

    cv2.rectangle(
        canvas,
        (margin_left, margin_top),
        (width - margin_right, height - margin_bottom),
        (70, 70, 70),
        1,
    )
    threshold_y = pixel_y(samples[0].threshold_rad_per_m)
    cv2.line(
        canvas,
        (margin_left, threshold_y),
        (width - margin_right, threshold_y),
        (0, 180, 255),
        2,
    )
    previous_point = None
    previous_label = None
    for sample in samples:
        value = sample.curvature_rad_per_m
        if not math.isfinite(value):
            value = display_max
        point = (pixel_x(sample.elapsed_sec), pixel_y(value))
        if previous_point is not None:
            color = (
                (80, 220, 80)
                if previous_label == "STRAIGHT"
                else (80, 80, 255)
            )
            cv2.line(canvas, previous_point, point, color, 1, cv2.LINE_AA)
        previous_point = point
        previous_label = sample.classification

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(
        canvas,
        "Recorded path curve classification",
        (margin_left, 38),
        font,
        0.9,
        (235, 235, 235),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        f"threshold={samples[0].threshold_rad_per_m:.3f} rad/m  "
        "green=STRAIGHT  red=CURVE",
        (margin_left + 560, 38),
        font,
        0.62,
        (210, 210, 210),
        1,
        cv2.LINE_AA,
    )
    for fraction in np.linspace(0.0, 1.0, 9):
        elapsed = duration * float(fraction)
        x = pixel_x(elapsed)
        cv2.line(
            canvas,
            (x, height - margin_bottom),
            (x, height - margin_bottom + 8),
            (160, 160, 160),
            1,
        )
        cv2.putText(
            canvas,
            f"{elapsed:.0f}s",
            (x - 20, height - margin_bottom + 30),
            font,
            0.45,
            (200, 200, 200),
            1,
            cv2.LINE_AA,
        )
    cv2.putText(
        canvas,
        "elapsed time",
        (width // 2 - 55, height - 20),
        font,
        0.52,
        (210, 210, 210),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        f"{display_max:.2f}",
        (10, margin_top + 5),
        font,
        0.45,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "0.00",
        (18, height - margin_bottom + 5),
        font,
        0.45,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )
    if not cv2.imwrite(str(path), canvas):
        raise RuntimeError(f"failed to write timeline image: {path}")


def write_annotated_video(
    bag_path: Path,
    output_path: Path,
    samples: list[CurveSample],
) -> int:
    debug_stamps = read_topic_stamps(bag_path, DEBUG_IMAGE_TOPIC)
    if debug_stamps.size == 0:
        return 0
    if debug_stamps.size > 1:
        intervals = np.diff(debug_stamps) / 1.0e9
        intervals = intervals[(intervals > 0.01) & (intervals < 1.0)]
        fps = 1.0 / float(np.median(intervals)) if intervals.size else 5.0
    else:
        fps = 5.0
    fps = min(30.0, max(1.0, fps))

    reader, available = open_reader(
        bag_path,
        [SOURCE_IMAGE_TOPIC, DEBUG_IMAGE_TOPIC],
    )
    if DEBUG_IMAGE_TOPIC not in available:
        return 0
    sample_stamps = np.asarray(
        [sample.timestamp_ns for sample in samples], dtype=np.int64
    )
    panel_width = 640
    panel_height = 512
    header_height = 120
    output_width = panel_width * 2
    output_height = header_height + panel_height
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (output_width, output_height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"failed to open video writer: {output_path}")
    frame_count = 0
    latest_source_data = None
    latest_source_stamp = None
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic == SOURCE_IMAGE_TOPIC:
            message = deserialize_message(serialized, CompressedImage)
            latest_source_data = bytes(message.data)
            latest_source_stamp = int(timestamp_ns)
            continue
        if topic != DEBUG_IMAGE_TOPIC:
            continue
        index = nearest_index(sample_stamps, int(timestamp_ns))
        if index is None or abs(int(sample_stamps[index]) - int(timestamp_ns)) > 400_000_000:
            continue
        message = deserialize_message(serialized, CompressedImage)
        debug_image = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        if debug_image is None:
            continue
        sample = samples[index]
        source_available = (
            latest_source_data is not None
            and latest_source_stamp is not None
            and abs(int(timestamp_ns) - latest_source_stamp) <= 150_000_000
        )
        source_image = None
        if source_available:
            source_image = cv2.imdecode(
                np.frombuffer(latest_source_data, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
        source_panel = fit_to_panel(
            source_image,
            panel_width,
            panel_height,
        )
        debug_panel = fit_to_panel(debug_image, panel_width, panel_height)
        image = np.zeros((output_height, output_width, 3), dtype=np.uint8)
        image[header_height:, :panel_width] = source_panel
        image[header_height:, panel_width:] = debug_panel
        color = (40, 220, 40) if sample.classification == "STRAIGHT" else (40, 40, 255)
        previous_color = (
            (40, 220, 40)
            if sample.previous_classification == "STRAIGHT"
            else (40, 40, 255)
        )
        cv2.rectangle(image, (0, 0), (output_width, header_height), (12, 12, 12), -1)
        cv2.putText(
            image,
            f"NEW: {sample.classification} k={sample.curvature_rad_per_m:.3f} "
            "(0.15-0.60m/3)",
            (18, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.64,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"PREVIOUS: {sample.previous_classification} "
            f"k={sample.previous_curvature_rad_per_m:.3f} "
            "(0.16-0.30m/1)",
            (650, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            previous_color,
            2,
            cv2.LINE_AA,
        )
        segment_text = "  ".join(
            f"S{index + 1}={value:.3f}"
            for index, value in enumerate(
                sample.segment_curvatures_rad_per_m
            )
        )
        cv2.putText(
            image,
            f"NEW segments: {segment_text} | limit={sample.threshold_rad_per_m:.3f}",
            (18, 66),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.61,
            (215, 215, 215),
            2,
            cv2.LINE_AA,
        )
        steering = "n/a" if sample.steering_command is None else f"{sample.steering_command:+.1f}"
        speed = "n/a" if sample.speed_command is None else f"{sample.speed_command:.1f}"
        cv2.putText(
            image,
            f"t={sample.elapsed_sec:.2f}s | steer={steering} | speed={speed}",
            (18, 101),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (235, 235, 235),
            2,
            cv2.LINE_AA,
        )
        for text, x in (("RAW CAMERA", 12), ("CANONICAL PATH", panel_width + 12)):
            cv2.rectangle(
                image,
                (x - 5, header_height + 6),
                (x + 205, header_height + 38),
                (12, 12, 12),
                -1,
            )
            cv2.putText(
                image,
                text,
                (x, header_height + 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (245, 245, 245),
                2,
                cv2.LINE_AA,
            )
        writer.write(image)
        frame_count += 1
    writer.release()
    return frame_count


def parse_args():
    parser = argparse.ArgumentParser(
        description="Classify recorded rule paths with the current curve detector."
    )
    parser.add_argument("bag", type=Path, help="session directory or nested bag directory")
    parser.add_argument("--near", type=float, default=0.15)
    parser.add_argument("--far", type=float, default=0.60)
    parser.add_argument("--segments", type=int, default=3)
    parser.add_argument("--threshold", type=float, default=0.16)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--no-video", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bag_path = resolve_bag_path(args.bag)
    if args.output_dir is not None:
        output_dir = args.output_dir.expanduser().resolve()
    else:
        session_dir = bag_path.parent if bag_path.is_file() else bag_path
        if session_dir.name == "bag":
            session_dir = session_dir.parent
        output_dir = session_dir / "curve_classification_analysis"
    output_dir.mkdir(parents=True, exist_ok=True)
    samples = classify_paths(
        bag_path,
        near_x_m=args.near,
        far_x_m=args.far,
        segment_count=args.segments,
        threshold=args.threshold,
    )
    csv_path = output_dir / "curve_classification.csv"
    summary_path = output_dir / "curve_classification_summary.json"
    timeline_path = output_dir / "curve_classification_timeline.png"
    write_csv(csv_path, samples)
    summary = write_summary(summary_path, samples, args)
    write_timeline(timeline_path, samples)
    video_count = 0
    video_path = output_dir / "curve_classification_preview.mp4"
    if not args.no_video:
        video_count = write_annotated_video(bag_path, video_path, samples)

    print("Curve classification analysis complete")
    print(f"  bag:        {bag_path}")
    print(f"  samples:    {len(samples)}")
    print(f"  STRAIGHT:   {summary['counts']['STRAIGHT']}")
    print(f"  CURVE:      {summary['counts']['CURVE']}")
    print(f"  transitions:{summary['transition_count']}")
    print(f"  csv:        {csv_path}")
    print(f"  timeline:   {timeline_path}")
    if video_count:
        print(f"  video:      {video_path} ({video_count} frames)")
    elif not args.no_video:
        print(f"  video:      unavailable ({DEBUG_IMAGE_TOPIC} missing or empty)")


if __name__ == "__main__":
    main()
