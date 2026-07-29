#!/usr/bin/env python3
"""Summarize waypoint tracking error and large steering reversals in a bag."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import statistics

from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageOptions,
)
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


def percentile(values: list[float], ratio: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(float(value) for value in values)
    index = min(len(ordered) - 1, int(round((len(ordered) - 1) * ratio)))
    return ordered[index]


def analyze_bag(uri: str | Path) -> dict:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(Path(uri).expanduser().resolve())),
        ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    message_types = {
        name: get_message(message_type)
        for name, message_type in topic_types.items()
    }

    debug_rows: list[tuple[float, list[float]]] = []
    modes: list[tuple[float, str]] = []
    path_point_count = 0
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in {
            "/map_nav/debug",
            "/map_nav/control_mode",
            "/map_nav/global_path",
        }:
            continue
        message = deserialize_message(serialized, message_types[topic])
        timestamp_sec = float(timestamp_ns) * 1.0e-9
        if topic == "/map_nav/debug":
            debug_rows.append(
                (timestamp_sec, [float(value) for value in message.data])
            )
        elif topic == "/map_nav/control_mode":
            modes.append((timestamp_sec, str(message.data)))
        elif topic == "/map_nav/global_path":
            path_point_count = max(path_point_count, len(message.poses))

    tracking = [
        (timestamp, row)
        for timestamp, row in debug_rows
        if len(row) >= 12 and row[1] > 0.0
    ]
    cross_track = [abs(row[6]) for _, row in tracking]
    heading = [abs(row[7]) for _, row in tracking]
    steering = [abs(row[0]) for _, row in tracking]
    speed_commands = [row[1] for _, row in tracking]
    measured_speed = [row[8] for _, row in tracking]
    path_curvature = [abs(row[5]) for _, row in tracking]

    stable_straight = []
    window_start = 0
    window_end = 0
    for index, (timestamp, _) in enumerate(tracking):
        while tracking[window_start][0] < timestamp - 0.5:
            window_start += 1
        window_end = max(window_end, index)
        while (
            window_end + 1 < len(tracking)
            and tracking[window_end + 1][0] <= timestamp + 0.5
        ):
            window_end += 1
        stable_straight.append(
            max(
                abs(tracking[item][1][5])
                for item in range(window_start, window_end + 1)
            )
            <= 0.16
        )

    large_reversals = 0
    reversal_events = []
    previous_sign = 0
    previous_time = 0.0
    previous_command = 0.0
    tracking_start = tracking[0][0] if tracking else 0.0
    for index, (timestamp, row) in enumerate(tracking):
        straight = stable_straight[index]
        sign = 1 if row[0] >= 12.0 else -1 if row[0] <= -12.0 else 0
        if not straight:
            previous_sign = 0
            continue
        if (
            sign
            and previous_sign
            and sign != previous_sign
            and timestamp - previous_time <= 2.0
        ):
            large_reversals += 1
            reversal_events.append(
                {
                    "elapsed_sec": round(timestamp - tracking_start, 3),
                    "path_index": int(round(row[2])),
                    "from_command": round(previous_command, 3),
                    "to_command": round(row[0], 3),
                    "cross_track_error_m": round(row[6], 4),
                    "heading_error_rad": round(row[7], 4),
                }
            )
        if sign:
            previous_sign = sign
            previous_time = timestamp
            previous_command = row[0]

    laps = 0
    if path_point_count > 0:
        previous_index = None
        for _, row in tracking:
            current_index = int(round(row[2]))
            if (
                previous_index is not None
                and previous_index > path_point_count * 0.80
                and current_index < path_point_count * 0.20
            ):
                laps += 1
            previous_index = current_index

    mode_counts = Counter(mode for _, mode in modes)
    transitions = []
    previous_mode = None
    for timestamp, mode in modes:
        if mode != previous_mode:
            transitions.append(
                {"time_sec": round(timestamp, 3), "mode": mode}
            )
            previous_mode = mode

    tracking_timeline = []
    if tracking:
        start_time = tracking[0][0]
        next_sample_time = start_time
        for timestamp, row in tracking:
            if timestamp + 1.0e-6 < next_sample_time:
                continue
            tracking_timeline.append(
                {
                    "elapsed_sec": round(timestamp - start_time, 2),
                    "path_index": int(round(row[2])),
                    "steering_command": round(row[0], 2),
                    "speed_command": round(row[1], 2),
                    "path_curvature_per_m": round(row[5], 3),
                    "cross_track_error_m": round(row[6], 3),
                    "heading_error_rad": round(row[7], 3),
                }
            )
            next_sample_time = timestamp + 1.0

    return {
        "path_points": path_point_count,
        "tracking_samples": len(tracking),
        "completed_laps": laps,
        "mode_sample_counts": dict(sorted(mode_counts.items())),
        "mode_transitions": transitions,
        "tracking_timeline_1hz": tracking_timeline,
        "cross_track_error_m": {
            "mean_abs": (
                statistics.fmean(cross_track) if cross_track else 0.0
            ),
            "p95_abs": percentile(cross_track, 0.95),
            "max_abs": max(cross_track, default=0.0),
        },
        "heading_error_rad": {
            "p95_abs": percentile(heading, 0.95),
            "max_abs": max(heading, default=0.0),
        },
        "steering_command": {
            "p95_abs": percentile(steering, 0.95),
            "max_abs": max(steering, default=0.0),
            "large_straight_reversals": large_reversals,
            "large_straight_reversal_events": reversal_events,
        },
        "speed": {
            "mean_command": (
                statistics.fmean(speed_commands) if speed_commands else 0.0
            ),
            "mean_measured_mps": (
                statistics.fmean(measured_speed) if measured_speed else 0.0
            ),
        },
        "path_curvature_per_m": {
            "p95_abs": percentile(path_curvature, 0.95),
            "max_abs": max(path_curvature, default=0.0),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output-json", default="")
    options = parser.parse_args()
    result = analyze_bag(options.bag)
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if options.output_json:
        output = Path(options.output_json).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
