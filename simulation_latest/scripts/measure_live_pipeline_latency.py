#!/usr/bin/env python3
"""Measure live canonical-perception and policy timing without motor output."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


def summarize(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "p50": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def summarize_rate(times: list[float]) -> dict[str, float | int]:
    if len(times) < 2:
        return {"count": len(times), "hz": 0.0}
    intervals = np.diff(np.asarray(times, dtype=np.float64))
    positive = intervals[intervals > 0.0]
    if positive.size == 0:
        return {"count": len(times), "hz": 0.0}
    return {
        "count": len(times),
        "hz": float(1.0 / np.mean(positive)),
        "interval_p95_ms": float(np.percentile(positive, 95) * 1000.0),
        "interval_max_ms": float(np.max(positive) * 1000.0),
    }


class PipelineLatencyProbe(Node):
    def __init__(self) -> None:
        super().__init__("pipeline_latency_probe")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.lane_rows: list[list[float]] = []
        self.policy_rows: list[list[float]] = []
        self.canonical_times: list[float] = []
        self.command_times: list[float] = []
        self.create_subscription(
            Float32MultiArray,
            "/lane_seg/diagnostics",
            self.on_lane_diagnostics,
            qos,
        )
        self.create_subscription(
            Float32MultiArray,
            "/rl/policy_debug",
            self.on_policy_diagnostics,
            qos,
        )
        self.create_subscription(
            Image,
            "/perception/canonical_road_image",
            self.on_canonical,
            qos,
        )
        self.create_subscription(
            Float32MultiArray,
            "/rl/policy_motor_shadow",
            self.on_command,
            qos,
        )

    def on_lane_diagnostics(self, message: Float32MultiArray) -> None:
        if len(message.data) >= 14:
            self.lane_rows.append(list(message.data))

    def on_policy_diagnostics(self, message: Float32MultiArray) -> None:
        if len(message.data) >= 13:
            self.policy_rows.append(list(message.data))

    def on_canonical(self, _: Image) -> None:
        self.canonical_times.append(time.monotonic())

    def on_command(self, _: Float32MultiArray) -> None:
        self.command_times.append(time.monotonic())

    def report(self, duration_sec: float) -> dict:
        report = {
            "duration_sec": float(duration_sec),
            "canonical_rate": summarize_rate(self.canonical_times),
            "policy_command_rate": summarize_rate(self.command_times),
            "lane": {},
            "policy": {},
        }
        lane_fields = {
            "model_ms": 0,
            "callback_total_ms": 6,
            "decode_rectify_ms": 9,
            "canonical_ms": 10,
            "source_age_ms": 11,
            "replaced_input_count": 12,
            "stale_input_count": 13,
        }
        if self.lane_rows:
            rows = np.asarray(self.lane_rows, dtype=np.float64)
            for name, index in lane_fields.items():
                report["lane"][name] = summarize(rows[:, index].tolist())
        policy_fields = {
            "inference_ms": 6,
            "image_age_at_command_ms": 7,
            "rate_limited_frame_count": 11,
            "stale_frame_count": 12,
        }
        if self.policy_rows:
            rows = np.asarray(self.policy_rows, dtype=np.float64)
            for name, index in policy_fields.items():
                report["policy"][name] = summarize(rows[:, index].tolist())
        return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    duration = max(1.0, float(args.duration))
    rclpy.init()
    node = PipelineLatencyProbe()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    elapsed = time.monotonic() - started
    report = node.report(elapsed)
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered + "\n", encoding="utf-8")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
