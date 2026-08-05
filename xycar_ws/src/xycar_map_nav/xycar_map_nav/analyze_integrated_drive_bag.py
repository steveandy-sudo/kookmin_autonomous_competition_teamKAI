"""Summarize sensor health and decision authority in an integrated-drive bag."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import statistics

import yaml


STATE_TOPICS = (
    "/map_nav/lane_guard/mode",
    "/map_nav/control_mode",
    "/map_nav/mission_reason",
    "/map_nav/localization_guard/status",
    "/map_nav/route_localization/status",
    "/hybrid/mode",
    "/hybrid_gate/mode",
)
COMMAND_TOPICS = (
    "/xycar_motor",
    "/map_nav/global_lane_guard_shadow",
    "/map_nav/xycar_motor_shadow",
    "/xycar_motor_shadow",
    "/my_rule/cone_cmd",
    "/hybrid/rule_candidate",
    "/rl/policy_motor_shadow",
    "/hybrid_gate/xycar_motor_shadow",
)
SENSOR_TOPICS = (
    "/wide_camera_mjpeg/image_raw/compressed",
    "/scan",
    "/slam/scan_filtered",
    "/imu",
    "/vehicle/vesc_state",
    "/vehicle/system_telemetry",
    "/odom",
    "/slam/odom",
)
CONTROL_TOPICS = (
    "/my_rule/object_detections",
    "/my_rule/cone_cmd",
    "/hybrid/rule_candidate",
    "/hybrid/avoidance_lateral_offset",
    "/hybrid/avoidance_debug",
    "/rl/policy_motor_shadow",
    "/rl/policy_debug",
    "/rl/policy_status",
    "/hybrid_gate/drive_armed",
    "/hybrid_gate/mode",
    "/hybrid_gate/status",
    "/hybrid_gate/diagnostics",
    "/hybrid_gate/xycar_motor_shadow",
    "/xycar_motor",
)


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    return parser.parse_args()


def _resolve_bag(path: Path) -> tuple[Path, Path]:
    run_dir = path.expanduser().resolve()
    if (run_dir / "metadata.yaml").is_file():
        return run_dir, run_dir.parent
    bag_dir = run_dir / "bag"
    if (bag_dir / "metadata.yaml").is_file():
        return bag_dir, run_dir
    raise FileNotFoundError(f"metadata.yaml not found below {run_dir}")


def _metadata(bag_dir: Path) -> dict:
    content = yaml.safe_load(
        (bag_dir / "metadata.yaml").read_text(encoding="utf-8")
    )
    return content.get("rosbag2_bagfile_information", {})


def _topic_timing(stamps: list[float]) -> dict:
    if not stamps:
        return {"count": 0, "mean_hz": None, "maximum_gap_sec": None}
    if len(stamps) == 1:
        return {"count": 1, "mean_hz": None, "maximum_gap_sec": None}
    gaps = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    duration = stamps[-1] - stamps[0]
    return {
        "count": len(stamps),
        "mean_hz": ((len(stamps) - 1) / duration if duration > 0.0 else None),
        "maximum_gap_sec": max(gaps) if gaps else None,
    }


def _number_summary(values: list[float]) -> dict:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {"count": 0, "minimum": None, "mean": None, "maximum": None}
    return {
        "count": len(finite),
        "minimum": min(finite),
        "mean": statistics.fmean(finite),
        "maximum": max(finite),
    }


def _read(run: Path) -> tuple[dict, Path]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir, run_dir = _resolve_bag(run)
    metadata = _metadata(bag_dir)
    compression = str(metadata.get("compression_format", ""))
    temporary_decompressed_paths = []
    if compression:
        suffix = f".{compression}"
        for relative_path in metadata.get("relative_file_paths", []):
            compressed_path = bag_dir / str(relative_path)
            if str(compressed_path).endswith(suffix):
                decompressed_path = Path(str(compressed_path)[: -len(suffix)])
                if not decompressed_path.exists():
                    temporary_decompressed_paths.append(decompressed_path)
    reader = (
        rosbag2_py.SequentialCompressionReader()
        if compression
        else rosbag2_py.SequentialReader()
    )
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_dir),
            storage_id=str(metadata.get("storage_identifier", "sqlite3")),
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    classes = {}
    for topic, message_type in types.items():
        if (
            topic in STATE_TOPICS
            or topic in COMMAND_TOPICS
            or topic
            in {
                "/map_nav/lane_guard/diagnostics",
                "/map_nav/route_localization/ready",
                "/my_rule/object_detections",
                "/vehicle/vesc_state",
            }
        ):
            try:
                classes[topic] = get_message(message_type)
            except (AttributeError, ModuleNotFoundError, ValueError):
                pass

    stamps = defaultdict(list)
    states = defaultdict(Counter)
    transitions = defaultdict(list)
    commands = defaultdict(lambda: {"steering": [], "speed": []})
    object_classes = Counter()
    object_confidences = defaultdict(list)
    strict_ready_samples = []
    translation_residuals = []
    yaw_residuals_deg = []
    route_ready = []
    voltages = []
    measured_speeds = []
    last_state = {}

    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        stamp = float(stamp_ns) * 1.0e-9
        stamps[topic].append(stamp)
        message_class = classes.get(topic)
        if message_class is None:
            continue
        message = deserialize_message(serialized, message_class)
        if topic in STATE_TOPICS:
            value = str(message.data)
            states[topic][value] += 1
            if last_state.get(topic) != value:
                transitions[topic].append({"time_sec": stamp, "value": value})
                last_state[topic] = value
        elif topic in COMMAND_TOPICS and len(message.data) >= 2:
            commands[topic]["steering"].append(float(message.data[0]))
            commands[topic]["speed"].append(float(message.data[1]))
        elif topic == "/map_nav/lane_guard/diagnostics":
            data = message.data
            if len(data) > 14:
                strict_ready_samples.append(float(data[14]))
            if len(data) > 15 and math.isfinite(float(data[15])):
                translation_residuals.append(float(data[15]))
            if len(data) > 16 and math.isfinite(float(data[16])):
                yaw_residuals_deg.append(math.degrees(float(data[16])))
        elif topic == "/map_nav/route_localization/ready":
            route_ready.append(bool(message.data))
        elif topic == "/my_rule/object_detections":
            for detection in message.detections:
                name = str(detection.class_name)
                object_classes[name] += 1
                object_confidences[name].append(float(detection.confidence))
        elif topic == "/vehicle/vesc_state":
            voltages.append(float(message.voltage_input))
            measured_speeds.append(float(message.speed_mps))

    report = {
        "bag": str(bag_dir),
        "duration_sec": float(metadata.get("duration", {}).get("nanoseconds", 0))
        * 1.0e-9,
        "topic_count": len(types),
        "topic_timing": {
            topic: _topic_timing(stamps.get(topic, []))
            for topic in sorted(
                set(SENSOR_TOPICS) | set(CONTROL_TOPICS) | set(types)
            )
            if (
                topic in SENSOR_TOPICS
                or topic in CONTROL_TOPICS
                or topic.startswith("/recording/")
            )
        },
        "state_counts": {
            topic: dict(counter) for topic, counter in states.items()
        },
        "state_transitions": dict(transitions),
        "commands": {
            topic: {
                "steering": _number_summary(values["steering"]),
                "speed": _number_summary(values["speed"]),
            }
            for topic, values in commands.items()
        },
        "route_localization_ready_fraction": (
            sum(route_ready) / len(route_ready) if route_ready else None
        ),
        "strict_global_alignment_ready_fraction": (
            sum(value >= 0.5 for value in strict_ready_samples)
            / len(strict_ready_samples)
            if strict_ready_samples
            else None
        ),
        "strict_translation_residual_m": _number_summary(
            translation_residuals
        ),
        "strict_yaw_residual_deg": _number_summary(yaw_residuals_deg),
        "objects": {
            name: {
                "count": count,
                "confidence": _number_summary(object_confidences[name]),
            }
            for name, count in object_classes.items()
        },
        "vesc_voltage_v": _number_summary(voltages),
        "vesc_speed_mps": _number_summary(measured_speeds),
    }
    del reader
    for temporary_path in temporary_decompressed_paths:
        if temporary_path.is_file():
            temporary_path.unlink()
    return report, run_dir


def _format_number(value, digits=2) -> str:
    return "n/a" if value is None else f"{float(value):.{digits}f}"


def _markdown(report: dict) -> str:
    lines = [
        "# Integrated drive report",
        "",
        f"- Bag: `{report['bag']}`",
        f"- Duration: `{report['duration_sec']:.1f} s`",
        f"- Topics: `{report['topic_count']}`",
        "- Route-ready fraction: `"
        + _format_number(report["route_localization_ready_fraction"], 3)
        + "`",
        "- Strict-global-ready fraction: `"
        + _format_number(report["strict_global_alignment_ready_fraction"], 3)
        + "`",
        "",
        "## Logic states",
        "",
    ]
    if report["state_counts"]:
        for topic, counts in report["state_counts"].items():
            lines.append(f"- `{topic}`: `{json.dumps(counts, ensure_ascii=False)}`")
    else:
        lines.append("- No logic-state messages were recorded.")

    lines.extend(["", "## Sensor health", "", "| Topic | Messages | Mean Hz | Max gap s |", "|---|---:|---:|---:|"])
    for topic, timing in report["topic_timing"].items():
        lines.append(
            f"| `{topic}` | {timing['count']} | "
            f"{_format_number(timing['mean_hz'])} | "
            f"{_format_number(timing['maximum_gap_sec'], 3)} |"
        )

    lines.extend(["", "## Commands", "", "| Topic | Count | Steering min/mean/max | Speed min/mean/max |", "|---|---:|---:|---:|"])
    for topic, values in report["commands"].items():
        steering = values["steering"]
        speed = values["speed"]
        lines.append(
            f"| `{topic}` | {steering['count']} | "
            f"{_format_number(steering['minimum'])}/"
            f"{_format_number(steering['mean'])}/"
            f"{_format_number(steering['maximum'])} | "
            f"{_format_number(speed['minimum'])}/"
            f"{_format_number(speed['mean'])}/"
            f"{_format_number(speed['maximum'])} |"
        )

    voltage = report["vesc_voltage_v"]
    speed = report["vesc_speed_mps"]
    lines.extend(
        [
            "",
            "## Vehicle telemetry",
            "",
            "- VESC voltage min/mean/max: `"
            f"{_format_number(voltage['minimum'])}/"
            f"{_format_number(voltage['mean'])}/"
            f"{_format_number(voltage['maximum'])} V`",
            "- Measured speed min/mean/max: `"
            f"{_format_number(speed['minimum'])}/"
            f"{_format_number(speed['mean'])}/"
            f"{_format_number(speed['maximum'])} m/s`",
            "",
            "## Objects",
            "",
        ]
    )
    if report["objects"]:
        for name, values in sorted(report["objects"].items()):
            confidence = values["confidence"]
            lines.append(
                f"- `{name}`: {values['count']} detections, confidence "
                f"mean/max {_format_number(confidence['mean'], 3)}/"
                f"{_format_number(confidence['maximum'], 3)}"
            )
    else:
        lines.append("- No object detections were recorded.")
    return "\n".join(lines) + "\n"


def main() -> None:
    args = _arguments()
    report, run_dir = _read(args.run)
    json_path = run_dir / "integrated_drive_report.json"
    markdown_path = run_dir / "integrated_drive_report.md"
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    markdown_path.write_text(_markdown(report), encoding="utf-8")
    print(markdown_path.read_text(encoding="utf-8"))
    print(f"JSON: {json_path}")
    print(f"Markdown: {markdown_path}")


if __name__ == "__main__":
    main()
