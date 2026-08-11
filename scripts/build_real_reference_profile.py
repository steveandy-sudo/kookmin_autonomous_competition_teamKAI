#!/usr/bin/env python3
"""Build a lane-visibility reference profile from real canonical observations."""

from __future__ import annotations

import argparse
import bisect
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np


ROAD_TOPIC = "/perception/canonical_road_image"
MOTOR_TOPIC = "/xycar_motor"


@dataclass
class Observation:
    stamp_ns: int
    time_sec: float
    white_count: int
    yellow_visible: bool
    motor_angle: float | None = None
    motor_speed: float | None = None


def decode_image(message: Any) -> np.ndarray:
    encoding = str(message.encoding).lower()
    channels = 1 if encoding in {"mono8", "8uc1"} else 3
    array = np.frombuffer(message.data, dtype=np.uint8)
    rows = array.reshape(int(message.height), int(message.step))
    image = rows[:, : int(message.width) * channels]
    if channels == 1:
        return image.reshape(int(message.height), int(message.width)).copy()
    image = image.reshape(int(message.height), int(message.width), channels).copy()
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def lane_visibility(image: np.ndarray) -> tuple[int, bool]:
    white = np.all(image >= 235, axis=2).astype(np.uint8)
    blue, green, red = cv2.split(image)
    yellow = (
        (blue <= 80)
        & (green >= 150)
        & (red >= 180)
        & ((red.astype(np.int16) - blue.astype(np.int16)) >= 100)
    )
    count, _, stats, _ = cv2.connectedComponentsWithStats(white, connectivity=8)
    line_count = sum(
        int(stats[index, cv2.CC_STAT_AREA]) >= 15
        and int(stats[index, cv2.CC_STAT_HEIGHT]) >= 7
        for index in range(1, count)
    )
    yellow_rows, yellow_columns = np.nonzero(yellow)
    yellow_visible = bool(
        yellow_columns.size >= 15
        and int(yellow_rows.max() - yellow_rows.min() + 1) >= 7
    ) if yellow_rows.size else False
    return min(2, line_count), yellow_visible


def session_observations(session: Path) -> list[Observation]:
    csv_path = session / "samples.csv"
    with csv_path.open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError(f"empty canonical session: {session}")
    start_ns = int(rows[0].get("image_timestamp_ns") or rows[0]["timestamp_ns"])
    observations = []
    for row in rows:
        stamp_ns = int(row.get("image_timestamp_ns") or row["timestamp_ns"])
        image_path = session / row["front_image_path"]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        white_count, yellow_visible = lane_visibility(image)
        observations.append(
            Observation(
                stamp_ns=stamp_ns,
                time_sec=(stamp_ns - start_ns) * 1.0e-9,
                white_count=white_count,
                yellow_visible=yellow_visible,
                motor_angle=parse_optional_float(row.get("motor_angle")),
                motor_speed=parse_optional_float(row.get("motor_speed")),
            )
        )
    return observations


def image_directory_observations(directory: Path) -> list[Observation]:
    image_paths = sorted(directory.glob("*.png"))
    if not image_paths:
        raise RuntimeError(f"no PNG canonical images in: {directory}")
    stamps = [int(path.stem) for path in image_paths]
    start_ns = stamps[0]
    observations = []
    for path, stamp_ns in zip(image_paths, stamps):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        white_count, yellow_visible = lane_visibility(image)
        observations.append(
            Observation(
                stamp_ns=stamp_ns,
                time_sec=(stamp_ns - start_ns) * 1.0e-9,
                white_count=white_count,
                yellow_visible=yellow_visible,
            )
        )
    return observations


def bag_observations(bag: Path) -> list[Observation]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if ROAD_TOPIC not in topics:
        raise RuntimeError(f"{ROAD_TOPIC} missing from: {bag}")
    road_type = get_message(topics[ROAD_TOPIC])
    motor_type = get_message(topics[MOTOR_TOPIC]) if MOTOR_TOPIC in topics else None
    images: list[tuple[int, np.ndarray]] = []
    motors: list[tuple[int, float, float]] = []
    while reader.has_next():
        topic, data, bag_stamp_ns = reader.read_next()
        if topic == ROAD_TOPIC:
            images.append((bag_stamp_ns, decode_image(deserialize_message(data, road_type))))
        elif topic == MOTOR_TOPIC and motor_type is not None:
            message = deserialize_message(data, motor_type)
            if len(message.data) >= 2:
                motors.append((bag_stamp_ns, float(message.data[0]), float(message.data[1])))
    if not images:
        raise RuntimeError(f"no canonical road images in: {bag}")

    motor_stamps = [item[0] for item in motors]
    start_ns = images[0][0]
    observations = []
    for stamp_ns, image in images:
        white_count, yellow_visible = lane_visibility(image)
        angle = speed = None
        if motors:
            index = bisect.bisect_left(motor_stamps, stamp_ns)
            candidates = [value for value in (index - 1, index) if 0 <= value < len(motors)]
            nearest = min(candidates, key=lambda value: abs(motors[value][0] - stamp_ns))
            if abs(motors[nearest][0] - stamp_ns) <= 150_000_000:
                _, angle, speed = motors[nearest]
        observations.append(
            Observation(
                stamp_ns=stamp_ns,
                time_sec=(stamp_ns - start_ns) * 1.0e-9,
                white_count=white_count,
                yellow_visible=yellow_visible,
                motor_angle=angle,
                motor_speed=speed,
            )
        )
    return observations


def parse_optional_float(value: str | None) -> float | None:
    try:
        return float(value) if value not in {None, ""} else None
    except ValueError:
        return None


def consecutive_ranges(
    observations: list[Observation],
    predicate,
    minimum_duration_sec: float,
    reason: str,
) -> list[dict[str, Any]]:
    ranges = []
    start = None
    for index, observation in enumerate(observations):
        if predicate(observation):
            if start is None:
                start = index
        elif start is not None:
            ranges += close_range(observations, start, index - 1, minimum_duration_sec, reason)
            start = None
    if start is not None:
        ranges += close_range(
            observations, start, len(observations) - 1, minimum_duration_sec, reason
        )
    return ranges


def close_range(
    observations: list[Observation],
    start_index: int,
    end_index: int,
    minimum_duration_sec: float,
    reason: str,
) -> list[dict[str, Any]]:
    start = observations[start_index].time_sec
    end = observations[end_index].time_sec
    if end - start < minimum_duration_sec:
        return []
    return [{"start_sec": start, "end_sec": end, "reason": reason}]


def merge_ranges(
    ranges: Iterable[dict[str, Any]], duration_sec: float, padding_sec: float
) -> list[dict[str, Any]]:
    padded = sorted(
        (
            max(0.0, float(item["start_sec"]) - padding_sec),
            min(duration_sec, float(item["end_sec"]) + padding_sec),
            str(item["reason"]),
        )
        for item in ranges
    )
    merged: list[dict[str, Any]] = []
    for start, end, reason in padded:
        if merged and start <= float(merged[-1]["end_sec"]):
            merged[-1]["end_sec"] = max(float(merged[-1]["end_sec"]), end)
            reasons = set(str(merged[-1]["reason"]).split("+"))
            reasons.add(reason)
            merged[-1]["reason"] = "+".join(sorted(reasons))
        else:
            merged.append({"start_sec": start, "end_sec": end, "reason": reason})
    return merged


def inside_ranges(time_sec: float, ranges: list[dict[str, Any]]) -> bool:
    return any(
        float(item["start_sec"]) <= time_sec <= float(item["end_sec"])
        for item in ranges
    )


def summarize(observations: list[Observation]) -> dict[str, Any]:
    total = len(observations)
    white_counts = {state: sum(item.white_count == state for item in observations) for state in (0, 1, 2)}
    yellow = sum(item.yellow_visible for item in observations)
    angles = [item.motor_angle for item in observations if item.motor_angle is not None]
    abs_angles = np.abs(angles) if angles else np.array([], dtype=np.float64)
    return {
        "frames": total,
        "white_state_probabilities": {
            "both": white_counts[2] / max(1, total),
            "one": white_counts[1] / max(1, total),
            "none": white_counts[0] / max(1, total),
        },
        "yellow_visible_probability": yellow / max(1, total),
        "steering": {
            "samples": len(angles),
            "left": sum(value < -2.0 for value in angles),
            "right": sum(value > 2.0 for value in angles),
            "straight": sum(abs(value) <= 2.0 for value in angles),
            "abs_p90": float(np.percentile(abs_angles, 90)) if angles else None,
            "abs_p99": float(np.percentile(abs_angles, 99)) if angles else None,
            "abs_ge_35_ratio": float(np.mean(abs_angles >= 35.0)) if angles else None,
        },
    }


def parse_explicit_ranges(values: list[str], source_name: str) -> list[dict[str, Any]]:
    output = []
    for value in values:
        parts = value.rsplit(":", 2)
        if len(parts) != 3:
            raise SystemExit(f"invalid --exclude-range: {value}")
        selector, start, end = parts
        if selector not in {"*", source_name} and selector not in source_name:
            continue
        output.append(
            {"start_sec": float(start), "end_sec": float(end), "reason": "manual"}
        )
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, action="append", default=[])
    parser.add_argument("--image-dir", type=Path, action="append", default=[])
    parser.add_argument("--bag", type=Path, action="append", default=[])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--exclude-range",
        action="append",
        default=[],
        metavar="SOURCE:START:END",
    )
    parser.add_argument("--blank-min-sec", type=float, default=0.60)
    parser.add_argument("--stopped-min-sec", type=float, default=0.50)
    parser.add_argument("--invalid-padding-sec", type=float, default=0.50)
    args = parser.parse_args()
    sources = [
        *(('session', path.expanduser().resolve()) for path in args.session),
        *(('image_dir', path.expanduser().resolve()) for path in args.image_dir),
        *(('bag', path.expanduser().resolve()) for path in args.bag),
    ]
    if not sources:
        raise SystemExit("provide at least one --session, --image-dir, or --bag")

    reports = []
    all_valid: list[Observation] = []
    for source_type, source in sources:
        if source_type == "session":
            observations = session_observations(source)
        elif source_type == "image_dir":
            observations = image_directory_observations(source)
        else:
            observations = bag_observations(source)
        duration = observations[-1].time_sec
        invalid = parse_explicit_ranges(args.exclude_range, source.name)
        invalid += consecutive_ranges(
            observations,
            lambda item: item.white_count == 0 and not item.yellow_visible,
            args.blank_min_sec,
            "sustained_blank_lane",
        )
        invalid += consecutive_ranges(
            observations,
            lambda item: item.motor_speed is not None and abs(item.motor_speed) <= 0.05,
            args.stopped_min_sec,
            "stopped_or_turnaround",
        )
        invalid = merge_ranges(invalid, duration, args.invalid_padding_sec)
        valid = [item for item in observations if not inside_ranges(item.time_sec, invalid)]
        all_valid.extend(valid)
        reports.append(
            {
                "source": str(source),
                "source_type": source_type,
                "duration_sec": duration,
                "all": summarize(observations),
                "valid": summarize(valid),
                "invalid_intervals": invalid,
                "invalid_frames": len(observations) - len(valid),
            }
        )

    aggregate = summarize(all_valid)
    white = aggregate["white_state_probabilities"]
    document = {
        "schema_version": 1,
        "sources": reports,
        "aggregate_valid": aggregate,
        "recommended_sim_visibility": {
            "white_both_probability": white["both"],
            "white_one_probability": white["one"],
            "white_none_probability": white["none"],
            "yellow_visible_probability": aggregate["yellow_visible_probability"],
            "prevent_blank": True,
            "maximum_synthetic_blank_frames": 0,
        },
        "exclusion_policy": {
            "sustained_blank_lane_min_sec": args.blank_min_sec,
            "stopped_or_turnaround_min_sec": args.stopped_min_sec,
            "padding_sec": args.invalid_padding_sec,
            "note": "Lane-gap and turnaround intervals are excluded, not imitated.",
        },
    }
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(document, indent=2))
    print(f"profile: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
