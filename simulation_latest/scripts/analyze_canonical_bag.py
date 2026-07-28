#!/usr/bin/env python3
"""Measure lane-mask quality and create visual samples from a ROS 2 bag."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


RAW_TOPIC = "/wide_camera_mjpeg/image_raw/compressed"
RECTIFIED_TOPIC = "/perception/rectified_camera_image"
ROAD_TOPIC = "/perception/canonical_road_image"
WHITE_TOPIC = "/perception/canonical_white_mask"
YELLOW_TOPIC = "/perception/canonical_yellow_mask"


def open_reader(bag: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def decode_image(message: Any, message_type: str) -> np.ndarray:
    if message_type == "sensor_msgs/msg/CompressedImage":
        decoded = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if decoded is None:
            raise RuntimeError("OpenCV could not decode a compressed image")
        return decoded

    encoding = str(message.encoding).lower()
    channels = 1 if encoding in {"mono8", "8uc1"} else 3
    array = np.frombuffer(message.data, dtype=np.uint8)
    rows = array.reshape(int(message.height), int(message.step))
    image = rows[:, : int(message.width) * channels]
    if channels == 1:
        return image.reshape(int(message.height), int(message.width)).copy()
    image = image.reshape(int(message.height), int(message.width), channels).copy()
    if encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def component_count(mask: np.ndarray, min_area: int = 10) -> int:
    count, _, stats, _ = cv2.connectedComponentsWithStats(
        (mask > 0).astype(np.uint8), connectivity=8
    )
    return sum(
        int(stats[index, cv2.CC_STAT_AREA]) >= min_area
        for index in range(1, count)
    )


def line_components(mask: np.ndarray) -> list[dict[str, Any]]:
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        (mask > 0).astype(np.uint8), connectivity=8
    )
    components: list[dict[str, Any]] = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        ys, xs = np.nonzero(labels == label)
        if area < 15 or ys.size == 0:
            continue
        rows = np.unique(ys)
        span = int(rows[-1] - rows[0] + 1)
        if span < 7:
            continue
        row_centers = {
            int(row): float(np.median(xs[ys == row])) for row in rows
        }
        components.append(
            {
                "area": area,
                "span_rows": span,
                "centroid_x": float(centroids[label, 0]),
                "row_centers": row_centers,
            }
        )
    return components


def best_white_pair(mask: np.ndarray) -> dict[str, Any] | None:
    components = line_components(mask)
    metres_per_pixel = 1.4 / float(mask.shape[1])
    best: dict[str, Any] | None = None
    for first_index, first in enumerate(components):
        for second in components[first_index + 1:]:
            rows = sorted(
                set(first["row_centers"]).intersection(second["row_centers"])
            )
            if len(rows) < 7:
                continue
            differences = np.array(
                [
                    abs(first["row_centers"][row] - second["row_centers"][row])
                    * metres_per_pixel
                    for row in rows
                ],
                dtype=np.float64,
            )
            median = float(np.median(differences))
            if not 0.45 <= median <= 1.15:
                continue
            score = abs(median - 0.824) - min(len(rows), 100) * 0.001
            if best is None or score < best["score"]:
                best = {
                    "score": score,
                    "first": first,
                    "second": second,
                    "rows": rows,
                    "spacings_m": differences,
                }
    return best


def mask_metrics(mask: np.ndarray, is_white: bool) -> dict[str, Any]:
    binary = mask > 0
    ys, xs = np.nonzero(binary)
    result: dict[str, Any] = {
        "pixels": int(xs.size),
        "fraction": float(np.mean(binary)),
        "components": component_count(mask),
        "occupied_rows": int(np.unique(ys).size) if ys.size else 0,
        "span_rows": int(ys.max() - ys.min() + 1) if ys.size else 0,
        "present": bool(xs.size >= 15 and (ys.max() - ys.min() + 1) >= 7)
        if ys.size
        else False,
        "centroid_x": float(np.mean(xs)) if xs.size else None,
    }
    if is_white:
        components = line_components(mask)
        pair = best_white_pair(mask)
        result["line_components"] = len(components)
        result["pair_present"] = pair is not None
        result["line_count"] = 2 if pair is not None else min(1, len(components))
    return result


def lane_geometry(white: np.ndarray, yellow: np.ndarray) -> dict[str, Any]:
    metres_per_pixel = 1.4 / float(white.shape[1])
    pair = best_white_pair(white)
    if pair is None:
        return {
            "paired_white_rows": 0,
            "white_spacing_m_median": None,
            "white_spacing_m_p10": None,
            "white_spacing_m_p90": None,
            "yellow_center_offset_m_median": None,
        }

    spacings = pair["spacings_m"]
    center_offsets: list[float] = []
    first = pair["first"]
    second = pair["second"]
    for row in pair["rows"]:
        yellow_x = np.flatnonzero(yellow[row] > 0)
        if yellow_x.size:
            lane_center = 0.5 * (
                first["row_centers"][row] + second["row_centers"][row]
            )
            center_offsets.append(
                (float(np.median(yellow_x)) - lane_center) * metres_per_pixel
            )

    return {
        "paired_white_rows": int(spacings.size),
        "white_spacing_m_median": float(np.median(spacings))
        if spacings.size
        else None,
        "white_spacing_m_p10": float(np.percentile(spacings, 10))
        if spacings.size
        else None,
        "white_spacing_m_p90": float(np.percentile(spacings, 90))
        if spacings.size
        else None,
        "yellow_center_offset_m_median": float(np.median(center_offsets))
        if center_offsets
        else None,
    }


@dataclass
class Frame:
    stamp_ns: int
    white: dict[str, Any] | None = None
    yellow: dict[str, Any] | None = None
    geometry: dict[str, Any] | None = None
    white_mask: np.ndarray | None = None
    yellow_mask: np.ndarray | None = None


def longest_false_run(frames: list[Frame], field: str) -> dict[str, float | int]:
    best_start = best_end = current_start = -1
    for index, frame in enumerate(frames):
        value: Any = frame
        for part in field.split("."):
            value = getattr(value, part) if hasattr(value, part) else value[part]
        if not bool(value):
            if current_start < 0:
                current_start = index
            if best_start < 0 or index - current_start > best_end - best_start:
                best_start, best_end = current_start, index
        else:
            current_start = -1
    if best_start < 0:
        return {"frames": 0, "duration_sec": 0.0}
    return {
        "frames": best_end - best_start + 1,
        "duration_sec": round(
            (frames[best_end].stamp_ns - frames[best_start].stamp_ns) * 1e-9,
            3,
        ),
        "start_sec": round(
            (frames[best_start].stamp_ns - frames[0].stamp_ns) * 1e-9, 3
        ),
        "end_sec": round(
            (frames[best_end].stamp_ns - frames[0].stamp_ns) * 1e-9, 3
        ),
    }


def percent(count: int, total: int) -> float:
    return round(100.0 * count / total, 2) if total else 0.0


def summarize(frames: list[Frame]) -> dict[str, Any]:
    total = len(frames)
    both_white = sum(bool(frame.white["pair_present"]) for frame in frames)
    one_white = sum(int(frame.white["line_count"]) == 1 for frame in frames)
    no_white = total - both_white - one_white
    yellow = sum(bool(frame.yellow["present"]) for frame in frames)
    all_three = sum(
        bool(
            frame.white["pair_present"] and frame.yellow["present"]
        )
        for frame in frames
    )
    spacings = [
        frame.geometry["white_spacing_m_median"]
        for frame in frames
        if frame.geometry["white_spacing_m_median"] is not None
    ]
    offsets = [
        frame.geometry["yellow_center_offset_m_median"]
        for frame in frames
        if frame.geometry["yellow_center_offset_m_median"] is not None
    ]
    yellow_components = [int(frame.yellow["components"]) for frame in frames]
    component_histogram = {
        str(value): yellow_components.count(value)
        for value in sorted(set(yellow_components))
    }

    return {
        "frames": total,
        "duration_sec": round((frames[-1].stamp_ns - frames[0].stamp_ns) * 1e-9, 3),
        "average_hz": round(
            (total - 1) / ((frames[-1].stamp_ns - frames[0].stamp_ns) * 1e-9), 3
        ),
        "detection": {
            "both_white": {"frames": both_white, "percent": percent(both_white, total)},
            "one_white": {"frames": one_white, "percent": percent(one_white, total)},
            "no_white": {"frames": no_white, "percent": percent(no_white, total)},
            "yellow": {"frames": yellow, "percent": percent(yellow, total)},
            "all_three": {"frames": all_three, "percent": percent(all_three, total)},
        },
        "yellow_component_histogram": component_histogram,
        "geometry": {
            "frames_with_white_spacing": len(spacings),
            "white_spacing_m_median": round(float(np.median(spacings)), 4)
            if spacings
            else None,
            "white_spacing_m_p10": round(float(np.percentile(spacings, 10)), 4)
            if spacings
            else None,
            "white_spacing_m_p90": round(float(np.percentile(spacings, 90)), 4)
            if spacings
            else None,
            "yellow_center_offset_m_median": round(float(np.median(offsets)), 4)
            if offsets
            else None,
            "yellow_center_offset_abs_m_p90": round(
                float(np.percentile(np.abs(offsets), 90)), 4
            )
            if offsets
            else None,
        },
        "longest_missing_runs": {
            "white_pair": longest_false_run(frames, "white.pair_present"),
            "any_white": longest_false_run(frames, "white.present"),
            "yellow": longest_false_run(frames, "yellow.present"),
        },
    }


def failure_score(frame: Frame) -> float:
    score = 0.0
    score += 3.0 if not frame.yellow["present"] else 0.0
    score += 4.0 if not frame.white["pair_present"] else 0.0
    score += 2.0 if not frame.white["present"] else 0.0
    spacing = frame.geometry["white_spacing_m_median"]
    if spacing is not None:
        score += min(3.0, abs(spacing - 0.824) / 0.08)
    score += min(2.0, max(0, int(frame.white["line_components"]) - 2) * 0.5)
    return score


def choose_indices(frames: list[Frame], sample_count: int) -> tuple[list[int], list[int]]:
    even = np.linspace(0, len(frames) - 1, min(sample_count, len(frames)))
    even_indices = sorted(set(int(round(value)) for value in even))
    ranked = sorted(
        range(len(frames)), key=lambda index: failure_score(frames[index]), reverse=True
    )
    minimum_gap = max(1, len(frames) // max(1, sample_count * 3))
    failures: list[int] = []
    for index in ranked:
        if all(abs(index - other) >= minimum_gap for other in failures):
            failures.append(index)
        if len(failures) >= sample_count:
            break
    return even_indices, sorted(failures)


def panel(
    raw: np.ndarray | None,
    road: np.ndarray | None,
    white: np.ndarray,
    yellow: np.ndarray,
    label: str,
) -> np.ndarray:
    canvas = np.full((320, 640, 3), 28, dtype=np.uint8)
    if raw is not None:
        resized = cv2.resize(raw, (320, 256), interpolation=cv2.INTER_AREA)
        canvas[40:296, :320] = resized
    if road is not None:
        resized = cv2.resize(road, (320, 180), interpolation=cv2.INTER_NEAREST)
        canvas[40:220, 320:] = resized
    masks = np.zeros((72, 320, 3), dtype=np.uint8)
    white_small = cv2.resize(white, (320, 72), interpolation=cv2.INTER_NEAREST)
    yellow_small = cv2.resize(yellow, (320, 72), interpolation=cv2.INTER_NEAREST)
    masks[white_small > 0] = (255, 255, 255)
    masks[yellow_small > 0] = (0, 255, 255)
    canvas[224:296, 320:] = masks
    cv2.putText(
        canvas, label, (8, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
        (0, 255, 255), 1, cv2.LINE_AA
    )
    return canvas


def collect_images(
    bag: Path, topic_types: dict[str, str], target_stamps: set[int]
) -> dict[int, dict[str, np.ndarray]]:
    images: dict[int, dict[str, np.ndarray]] = {stamp: {} for stamp in target_stamps}
    reader = open_reader(bag)
    wanted = {
        RAW_TOPIC, RECTIFIED_TOPIC, ROAD_TOPIC, WHITE_TOPIC, YELLOW_TOPIC
    }
    message_types = {
        topic: get_message(topic_types[topic]) for topic in wanted if topic in topic_types
    }
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        key = stamp_ns(message)
        if key in target_stamps:
            images[key][topic] = decode_image(message, topic_types[topic])
    return images


def write_sheet(
    path: Path, frames: list[Frame], indices: list[int], images: dict[int, dict[str, np.ndarray]]
) -> None:
    panels: list[np.ndarray] = []
    start = frames[0].stamp_ns
    for index in indices:
        frame = frames[index]
        found = images[frame.stamp_ns]
        label = (
            f"t={(frame.stamp_ns - start) * 1e-9:5.1f}s idx={index} "
            f"W={frame.white['line_count']} "
            f"Y={int(frame.yellow['present'])} score={failure_score(frame):.1f}"
        )
        panels.append(
            panel(
                found.get(RAW_TOPIC, found.get(RECTIFIED_TOPIC)),
                found.get(ROAD_TOPIC),
                frame.white_mask, frame.yellow_mask, label
            )
        )
    if not panels:
        return
    columns = 2
    rows = []
    blank = np.full_like(panels[0], 28)
    for offset in range(0, len(panels), columns):
        row = panels[offset: offset + columns]
        while len(row) < columns:
            row.append(blank)
        rows.append(np.hstack(row))
    cv2.imwrite(str(path), np.vstack(rows))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sample-count", type=int, default=12)
    args = parser.parse_args()
    bag = args.bag.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    reader = open_reader(bag)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    for topic in (WHITE_TOPIC, YELLOW_TOPIC):
        if topic not in topic_types:
            raise RuntimeError(f"required topic missing: {topic}")
    message_types = {
        WHITE_TOPIC: get_message(topic_types[WHITE_TOPIC]),
        YELLOW_TOPIC: get_message(topic_types[YELLOW_TOPIC]),
    }
    frames_by_stamp: dict[int, Frame] = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        key = stamp_ns(message)
        frame = frames_by_stamp.setdefault(key, Frame(key))
        mask = decode_image(message, topic_types[topic])
        if topic == WHITE_TOPIC:
            frame.white = mask_metrics(mask, True)
            frame.white_mask = mask
        else:
            frame.yellow = mask_metrics(mask, False)
            frame.yellow_mask = mask

    frames = [
        frame for _, frame in sorted(frames_by_stamp.items())
        if frame.white is not None and frame.yellow is not None
    ]
    if not frames:
        raise RuntimeError("no synchronized white/yellow mask pairs found")
    for frame in frames:
        frame.geometry = lane_geometry(frame.white_mask, frame.yellow_mask)

    report = summarize(frames)
    report.update({
        "bag": str(bag),
        "canonical_contract": {"width_px": 256, "height_px": 144, "width_m": 1.4},
    })
    report_path = output_dir / "canonical_lane_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    csv_path = output_dir / "canonical_lane_frames.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "index", "time_sec", "white_line_count", "white_pair", "yellow",
            "white_components", "yellow_components", "white_spacing_m",
            "yellow_center_offset_m", "failure_score",
        ])
        for index, frame in enumerate(frames):
            writer.writerow([
                index,
                f"{(frame.stamp_ns - frames[0].stamp_ns) * 1e-9:.6f}",
                frame.white["line_count"],
                int(frame.white["pair_present"]),
                int(frame.yellow["present"]),
                frame.white["components"],
                frame.yellow["components"],
                frame.geometry["white_spacing_m_median"],
                frame.geometry["yellow_center_offset_m_median"],
                f"{failure_score(frame):.3f}",
            ])

    even_indices, failure_indices = choose_indices(frames, args.sample_count)
    target_stamps = {
        frames[index].stamp_ns for index in even_indices + failure_indices
    }
    images = collect_images(bag, topic_types, target_stamps)
    write_sheet(output_dir / "canonical_even_samples.jpg", frames, even_indices, images)
    write_sheet(output_dir / "canonical_failure_samples.jpg", frames, failure_indices, images)
    print(json.dumps(report, indent=2))
    print(f"report: {report_path}")
    print(f"frames: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
