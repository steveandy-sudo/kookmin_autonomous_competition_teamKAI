#!/usr/bin/env python3
from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
import json
from pathlib import Path
import struct

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


RAW_TOPIC = "/wide_camera_mjpeg/image_raw/compressed"
CANONICAL_TOPIC = "/recording/canonical_road_image/compressed"
IMU_TOPIC = "/imu"
RULE_TOPIC = "/rule_drive/diagnostics"
MODEL_TOPIC = "/rl/policy_debug"
VESC_TOPIC = "/vehicle/vesc_state"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare real-vehicle rule and learned-policy rosbag runs."
    )
    parser.add_argument("--rule-bag", type=Path, required=True)
    parser.add_argument("--model-bag", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--montage-frames", type=int, default=12)
    return parser.parse_args()


def open_reader(path: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def decode_image(data: bytes | bytearray | memoryview) -> np.ndarray | None:
    encoded = np.frombuffer(bytes(data), dtype=np.uint8)
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def message_stamp_seconds(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1.0e9


def parse_vesc_state(raw: bytes, bag_stamp: float) -> dict[str, float] | None:
    """Read the fixed numeric tail of the locally recorded XycarVescState CDR."""
    if len(raw) < 24 or raw[:4] != b"\x00\x01\x00\x00":
        return None
    try:
        header_sec, header_nanosec, frame_length = struct.unpack_from("<III", raw, 4)
        values_offset = 16 + int(frame_length)
        values_offset += (-values_offset) % 4
        values = struct.unpack_from("<15f", raw, values_offset)
    except (struct.error, ValueError):
        return None
    return {
        "stamp": bag_stamp,
        "header_stamp": float(header_sec) + float(header_nanosec) / 1.0e9,
        "battery_voltage": float(values[0]),
        "motor_erpm": float(values[4]),
        "speed_mps": float(values[5]),
    }


def canonical_metrics(data: bytes) -> dict[str, float]:
    image = decode_image(data)
    if image is None:
        return {
            "white_pixels": 0.0,
            "yellow_pixels": 0.0,
            "yellow_shift_px": float("nan"),
            "yellow_curve": float("nan"),
        }
    blue, green, red = cv2.split(image)
    white = (blue >= 180) & (green >= 180) & (red >= 180)
    yellow = (blue <= 140) & (green >= 140) & (red >= 140)
    rows: list[float] = []
    centers: list[float] = []
    for row in range(yellow.shape[0]):
        columns = np.flatnonzero(yellow[row])
        if columns.size >= 2:
            rows.append(float(row))
            centers.append(float(np.median(columns)))
    shift = float("nan")
    curve = float("nan")
    if len(rows) >= 6:
        row_values = np.asarray(rows, dtype=np.float64)
        center_values = np.asarray(centers, dtype=np.float64)
        order = np.argsort(row_values)
        row_values = row_values[order]
        center_values = center_values[order]
        section = max(2, len(center_values) // 3)
        far_x = float(np.median(center_values[:section]))
        near_x = float(np.median(center_values[-section:]))
        shift = far_x - near_x
        normalized_y = (
            row_values - float(np.mean(row_values))
        ) / max(1.0, float(np.ptp(row_values)))
        curve = float(np.polyfit(normalized_y, center_values, 2)[0])
    return {
        "white_pixels": float(np.count_nonzero(white)),
        "yellow_pixels": float(np.count_nonzero(yellow)),
        "yellow_shift_px": shift,
        "yellow_curve": curve,
    }


def read_run(path: Path, kind: str) -> dict:
    reader = open_reader(path)
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    selected = {RAW_TOPIC, CANONICAL_TOPIC, IMU_TOPIC, VESC_TOPIC}
    selected.add(RULE_TOPIC if kind == "rule" else MODEL_TOPIC)
    raw_images: list[tuple[float, bytes]] = []
    canonical_images: list[tuple[float, bytes]] = []
    raw_source_ages_ms: list[float] = []
    imu: list[dict[str, float]] = []
    vesc: list[dict[str, float]] = []
    commands: list[dict[str, float]] = []
    first_stamp = None
    last_stamp = None
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        stamp = stamp_ns / 1.0e9
        first_stamp = stamp if first_stamp is None else min(first_stamp, stamp)
        last_stamp = stamp if last_stamp is None else max(last_stamp, stamp)
        if topic not in selected:
            continue
        if topic == VESC_TOPIC:
            state = parse_vesc_state(raw, stamp)
            if state is not None:
                vesc.append(state)
            continue
        message_type = get_message(topic_types[topic])
        message = deserialize_message(raw, message_type)
        if topic == RAW_TOPIC:
            raw_images.append((stamp, bytes(message.data)))
            source_stamp = message_stamp_seconds(message)
            if source_stamp > 0.0:
                raw_source_ages_ms.append((stamp - source_stamp) * 1000.0)
        elif topic == CANONICAL_TOPIC:
            canonical_images.append(
                (stamp, message_stamp_seconds(message), bytes(message.data))
            )
        elif topic == IMU_TOPIC:
            imu.append(
                {
                    "stamp": stamp,
                    "yaw_rate": float(message.angular_velocity.z),
                    "accel_x": float(message.linear_acceleration.x),
                    "accel_y": float(message.linear_acceleration.y),
                }
            )
        elif topic == RULE_TOPIC:
            values = list(message.data)
            if len(values) < 12:
                continue
            commands.append(
                {
                    "stamp": stamp,
                    "lane_visible": float(values[0]),
                    "path_valid": float(values[1]),
                    "observations": float(values[2]),
                    "forward_span_m": float(values[3]),
                    "max_gap_m": float(values[4]),
                    "fit_residual_m": float(values[5]),
                    "pure_pursuit_rad": float(values[6]),
                    "stanley_rad": float(values[7]),
                    "fused_rad": float(values[8]),
                    "angle_command": float(values[9]),
                    "speed_command": float(values[10]),
                    "path_source": float(values[11]),
                }
            )
        elif topic == MODEL_TOPIC:
            values = list(message.data)
            if len(values) < 8:
                continue
            commands.append(
                {
                    "stamp": stamp,
                    "base_norm": float(values[0]),
                    "residual_norm": float(values[1]),
                    "final_norm": float(values[2]),
                    "angle_command": float(values[3]),
                    "speed_command": float(values[4]),
                    "inference_ms": float(values[6]),
                    "sensor_age_ms": float(values[7]),
                }
            )
    canonical = []
    for stamp, source_stamp, encoded in canonical_images:
        canonical.append(
            {
                "stamp": stamp,
                "header_stamp": source_stamp,
                "source_age_ms": (
                    (stamp - source_stamp) * 1000.0
                    if source_stamp > 0.0
                    else float("nan")
                ),
                "encoded": encoded,
                **canonical_metrics(encoded),
            }
        )
    return {
        "kind": kind,
        "path": str(path),
        "first_stamp": first_stamp,
        "last_stamp": last_stamp,
        "raw_images": raw_images,
        "raw_source_ages_ms": raw_source_ages_ms,
        "canonical": canonical,
        "imu": imu,
        "vesc": vesc,
        "commands": commands,
    }


def rate_hz(stamps: np.ndarray) -> float:
    if stamps.size < 2:
        return float("nan")
    span = float(stamps[-1] - stamps[0])
    return float((stamps.size - 1) / span) if span > 0.0 else float("nan")


def sign_change_count(values: np.ndarray, deadband: float = 2.0) -> int:
    signs = np.sign(values[np.abs(values) >= deadband])
    if signs.size < 2:
        return 0
    return int(np.count_nonzero(signs[1:] != signs[:-1]))


def estimated_yaw_delay(run: dict) -> dict[str, float]:
    commands = run["commands"]
    imu = run["imu"]
    if len(commands) < 8 or len(imu) < 20:
        return {"yaw_delay_sec": float("nan"), "yaw_correlation": float("nan")}
    command_t = np.asarray([row["stamp"] for row in commands])
    angle = np.asarray([row["angle_command"] for row in commands])
    imu_t = np.asarray([row["stamp"] for row in imu])
    yaw = np.asarray([row["yaw_rate"] for row in imu])
    best_delay = float("nan")
    best_correlation = 0.0
    for delay in np.arange(0.0, 0.81, 0.02):
        target_t = command_t + delay
        overlap = (target_t >= imu_t[0]) & (target_t <= imu_t[-1])
        if int(np.count_nonzero(overlap)) < 8:
            continue
        shifted = np.interp(target_t[overlap], imu_t, yaw)
        overlap_angle = angle[overlap]
        if (
            float(np.std(shifted)) < 1.0e-6
            or float(np.std(overlap_angle)) < 1.0e-6
        ):
            continue
        correlation = float(np.corrcoef(overlap_angle, shifted)[0, 1])
        if abs(correlation) > abs(best_correlation):
            best_correlation = correlation
            best_delay = float(delay)
    return {
        "yaw_delay_sec": best_delay,
        "yaw_correlation": best_correlation,
    }


def numeric_summary(values: np.ndarray) -> dict[str, float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return {
            "mean": float("nan"),
            "std": float("nan"),
            "min": float("nan"),
            "max": float("nan"),
        }
    return {
        "mean": float(np.mean(finite)),
        "std": float(np.std(finite)),
        "min": float(np.min(finite)),
        "max": float(np.max(finite)),
    }


def command_source_ages_ms(run: dict) -> np.ndarray:
    canonical = run["canonical"]
    if not canonical:
        return np.asarray([], dtype=np.float64)
    arrival_stamps = np.asarray([row["stamp"] for row in canonical])
    source_stamps = np.asarray([row["header_stamp"] for row in canonical])
    ages = []
    for command in run["commands"]:
        index = int(np.searchsorted(arrival_stamps, command["stamp"], side="right") - 1)
        if index >= 0 and source_stamps[index] > 0.0:
            ages.append((command["stamp"] - source_stamps[index]) * 1000.0)
    return np.asarray(ages, dtype=np.float64)


def summarize_run(run: dict) -> dict:
    commands = run["commands"]
    canonical = run["canonical"]
    imu = run["imu"]
    vesc = run["vesc"]
    command_stamps = np.asarray([row["stamp"] for row in commands])
    command_gaps_ms = np.diff(command_stamps) * 1000.0
    angles = np.asarray([row["angle_command"] for row in commands])
    speeds = np.asarray([row["speed_command"] for row in commands])
    canonical_stamps = np.asarray([row["stamp"] for row in canonical])
    yellow_pixels = np.asarray([row["yellow_pixels"] for row in canonical])
    white_pixels = np.asarray([row["white_pixels"] for row in canonical])
    yaw = np.asarray([row["yaw_rate"] for row in imu])
    command_source_age = command_source_ages_ms(run)
    canonical_source_age = np.asarray(
        [row["source_age_ms"] for row in canonical]
    )
    raw_source_age = np.asarray(run["raw_source_ages_ms"])
    active_vesc = [
        row
        for row in vesc
        if commands and commands[0]["stamp"] <= row["stamp"] <= commands[-1]["stamp"]
    ]
    actual_speed = np.asarray([row["speed_mps"] for row in active_vesc])
    active_canonical = [
        row
        for row in canonical
        if commands and commands[0]["stamp"] <= row["stamp"] <= commands[-1]["stamp"]
    ]
    active_yellow = np.asarray(
        [row["yellow_pixels"] for row in active_canonical]
    )
    active_white = np.asarray([row["white_pixels"] for row in active_canonical])
    yaw_delay = estimated_yaw_delay(run)
    summary = {
        "kind": run["kind"],
        "path": run["path"],
        "bag_duration_sec": float(run["last_stamp"] - run["first_stamp"]),
        "command_count": len(commands),
        "command_active_span_sec": (
            float(command_stamps[-1] - command_stamps[0])
            if command_stamps.size >= 2
            else 0.0
        ),
        "command_rate_hz": rate_hz(command_stamps),
        "command_gap_ms": numeric_summary(command_gaps_ms),
        "command_gap_over_200ms_fraction": (
            float(np.mean(command_gaps_ms > 200.0))
            if command_gaps_ms.size
            else 0.0
        ),
        "canonical_count": len(canonical),
        "canonical_rate_hz": rate_hz(canonical_stamps),
        "raw_image_count": len(run["raw_images"]),
        "raw_source_age_ms": numeric_summary(raw_source_age),
        "canonical_source_age_ms": numeric_summary(canonical_source_age),
        "command_source_age_ms": numeric_summary(command_source_age),
        "angle_command": numeric_summary(angles),
        "speed_command": numeric_summary(speeds),
        "actual_speed_mps": numeric_summary(actual_speed),
        "actual_speed_sample_count": len(active_vesc),
        "steering_sign_changes": sign_change_count(angles),
        "steering_saturation_fraction": (
            float(np.mean(np.abs(angles) >= 35.0)) if angles.size else 0.0
        ),
        "yellow_pixels": numeric_summary(yellow_pixels),
        "white_pixels": numeric_summary(white_pixels),
        "yellow_missing_fraction": (
            float(np.mean(yellow_pixels < 3.0)) if yellow_pixels.size else 1.0
        ),
        "white_missing_fraction": (
            float(np.mean(white_pixels < 3.0)) if white_pixels.size else 1.0
        ),
        "active_yellow_missing_fraction": (
            float(np.mean(active_yellow < 3.0)) if active_yellow.size else 1.0
        ),
        "active_white_missing_fraction": (
            float(np.mean(active_white < 3.0)) if active_white.size else 1.0
        ),
        "yaw_rate": numeric_summary(yaw),
        **yaw_delay,
    }
    if command_source_age.size and np.isfinite(yaw_delay["yaw_delay_sec"]):
        summary["estimated_camera_to_yaw_sec"] = float(
            np.median(command_source_age) / 1000.0
            + yaw_delay["yaw_delay_sec"]
        )
    if run["kind"] == "rule" and commands:
        sources = np.asarray([row["path_source"] for row in commands])
        summary["path_source_counts"] = {
            "yellow": int(np.count_nonzero(sources == 1.0)),
            "white": int(np.count_nonzero(sources == 2.0)),
            "fused": int(np.count_nonzero(sources == 3.0)),
            "none": int(np.count_nonzero(sources == 0.0)),
        }
        summary["forward_span_m"] = numeric_summary(
            np.asarray([row["forward_span_m"] for row in commands])
        )
        summary["max_gap_m"] = numeric_summary(
            np.asarray([row["max_gap_m"] for row in commands])
        )
    if run["kind"] == "model" and commands:
        summary["inference_ms"] = numeric_summary(
            np.asarray([row["inference_ms"] for row in commands])
        )
        summary["sensor_age_ms"] = numeric_summary(
            np.asarray([row["sensor_age_ms"] for row in commands])
        )
        inferred_source_stamps = command_stamps - (
            np.asarray([row["sensor_age_ms"] for row in commands]) / 1000.0
        )
        summary["model_input_frame_gap_ms"] = numeric_summary(
            np.diff(inferred_source_stamps) * 1000.0
        )
    return summary


def nearest_item(items: list, stamp: float):
    if not items:
        return None
    stamps = [item[0] if isinstance(item, tuple) else item["stamp"] for item in items]
    index = bisect_left(stamps, stamp)
    candidates = [max(0, index - 1), min(len(items) - 1, index)]
    return min(
        (items[item_index] for item_index in candidates),
        key=lambda item: abs(
            (item[0] if isinstance(item, tuple) else item["stamp"]) - stamp
        ),
    )


def fit_panel(image: np.ndarray, width: int, height: int) -> np.ndarray:
    canvas = np.full((height, width, 3), 24, dtype=np.uint8)
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(
        image,
        (
            max(1, int(round(image.shape[1] * scale))),
            max(1, int(round(image.shape[0] * scale))),
        ),
        interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_NEAREST,
    )
    x = (width - resized.shape[1]) // 2
    y = (height - resized.shape[0]) // 2
    canvas[y: y + resized.shape[0], x: x + resized.shape[1]] = resized
    return canvas


def make_montage(run: dict, output: Path, count: int) -> None:
    commands = run["commands"]
    if not commands:
        return
    selected = np.linspace(
        0, len(commands) - 1, min(max(1, count), len(commands)), dtype=int
    )
    cells = []
    start = commands[0]["stamp"]
    for command_index in selected:
        command = commands[int(command_index)]
        raw_item = nearest_item(run["raw_images"], command["stamp"])
        canonical_item = nearest_item(run["canonical"], command["stamp"])
        if raw_item is None or canonical_item is None:
            continue
        raw_image = decode_image(raw_item[1])
        canonical_image = decode_image(canonical_item["encoded"])
        if raw_image is None or canonical_image is None:
            continue
        raw_panel = fit_panel(raw_image, 390, 250)
        canonical_panel = fit_panel(canonical_image, 250, 250)
        cell = np.hstack([raw_panel, canonical_panel])
        title = (
            f"t={command['stamp'] - start:5.2f}s "
            f"steer={command['angle_command']:6.1f} "
            f"speed={command['speed_command']:4.1f}"
        )
        if run["kind"] == "rule":
            title += (
                f" src={int(command['path_source'])} "
                f"span={command['forward_span_m']:.2f}m"
            )
        else:
            title += (
                f" infer={command['inference_ms']:.0f}ms "
                f"age={command['sensor_age_ms']:.0f}ms"
            )
        cv2.rectangle(cell, (0, 0), (cell.shape[1], 28), (15, 15, 15), -1)
        cv2.putText(
            cell,
            title,
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (0, 230, 255),
            1,
            cv2.LINE_AA,
        )
        cells.append(cell)
    if not cells:
        return
    columns = 2
    rows = int(np.ceil(len(cells) / columns))
    blank = np.full_like(cells[0], 20)
    cells.extend([blank] * (rows * columns - len(cells)))
    montage = np.vstack(
        [
            np.hstack(cells[row * columns: (row + 1) * columns])
            for row in range(rows)
        ]
    )
    cv2.imwrite(str(output), montage)


def write_command_csv(run: dict, path: Path) -> None:
    if not run["commands"]:
        return
    fields = list(run["commands"][0])
    start = run["commands"][0]["stamp"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["relative_sec", *fields])
        writer.writeheader()
        for row in run["commands"]:
            writer.writerow(
                {"relative_sec": row["stamp"] - start, **row}
            )


def make_plot(run: dict, output: Path) -> None:
    if not run["commands"]:
        return
    commands = run["commands"]
    start = commands[0]["stamp"]
    command_t = np.asarray([row["stamp"] - start for row in commands])
    angle = np.asarray([row["angle_command"] for row in commands])
    speed = np.asarray([row["speed_command"] for row in commands])
    vesc_t = np.asarray(
        [row["stamp"] - start for row in run["vesc"]]
    )
    actual_speed = np.asarray([row["speed_mps"] for row in run["vesc"]])
    imu_t = np.asarray([row["stamp"] - start for row in run["imu"]])
    yaw = np.asarray([row["yaw_rate"] for row in run["imu"]])
    canonical_t = np.asarray(
        [row["stamp"] - start for row in run["canonical"]]
    )
    yellow = np.asarray([row["yellow_pixels"] for row in run["canonical"]])
    white = np.asarray([row["white_pixels"] for row in run["canonical"]])
    shift = np.asarray([row["yellow_shift_px"] for row in run["canonical"]])

    figure, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    axes[0].plot(command_t, angle, label="steering command", color="tab:blue")
    axes[0].axhline(35.0, color="gray", linestyle=":", linewidth=0.8)
    axes[0].axhline(-35.0, color="gray", linestyle=":", linewidth=0.8)
    axes[0].set_ylabel("steer cmd")
    axes[0].grid(True, alpha=0.25)
    speed_axis = axes[0].twinx()
    speed_axis.plot(command_t, speed, label="speed", color="tab:orange")
    speed_axis.plot(
        vesc_t,
        actual_speed,
        label="actual m/s",
        color="tab:red",
        alpha=0.75,
    )
    speed_axis.set_ylabel("speed cmd")

    axes[1].plot(imu_t, yaw, color="tab:green")
    axes[1].set_ylabel("IMU yaw rad/s")
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(canonical_t, yellow, label="yellow pixels", color="gold")
    axes[2].plot(canonical_t, white, label="white pixels", color="lightgray")
    axes[2].set_ylabel("canonical pixels")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.25)
    shift_axis = axes[2].twinx()
    shift_axis.plot(canonical_t, shift, color="tab:red", alpha=0.7)
    shift_axis.set_ylabel("yellow far-near px")

    if run["kind"] == "rule":
        pp = np.asarray([row["pure_pursuit_rad"] for row in commands])
        stanley = np.asarray([row["stanley_rad"] for row in commands])
        fused = np.asarray([row["fused_rad"] for row in commands])
        axes[3].plot(command_t, pp, label="pure pursuit")
        axes[3].plot(command_t, stanley, label="Stanley")
        axes[3].plot(command_t, fused, label="fused")
        axes[3].set_ylabel("steering rad")
    else:
        inference = np.asarray([row["inference_ms"] for row in commands])
        age = np.asarray([row["sensor_age_ms"] for row in commands])
        axes[3].plot(command_t, inference, label="inference ms")
        axes[3].plot(command_t, age, label="sensor age ms")
        axes[3].set_ylabel("latency ms")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.25)
    axes[3].set_xlabel("seconds from first control command")
    figure.suptitle(f"{run['kind']} real-vehicle run")
    figure.tight_layout()
    figure.savefig(output, dpi=150)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    runs = [
        read_run(args.rule_bag.expanduser().resolve(), "rule"),
        read_run(args.model_bag.expanduser().resolve(), "model"),
    ]
    summaries = {}
    for run in runs:
        kind = run["kind"]
        summaries[kind] = summarize_run(run)
        make_montage(
            run,
            output_dir / f"{kind}_camera_canonical_montage.jpg",
            args.montage_frames,
        )
        make_plot(run, output_dir / f"{kind}_timeseries.png")
        write_command_csv(run, output_dir / f"{kind}_commands.csv")
    with (output_dir / "summary.json").open("w", encoding="utf-8") as handle:
        json.dump(summaries, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps(summaries, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
