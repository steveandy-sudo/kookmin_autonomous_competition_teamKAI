#!/usr/bin/env python3
"""Monitor one isolated waypoint-navigation profile and write JSON metrics."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import time

import rclpy
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32MultiArray


def percentile(values: list[float], ratio: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, round((len(ordered) - 1) * ratio))
    return float(ordered[index])


class ProfileMonitor(Node):
    def __init__(self) -> None:
        super().__init__(f"waypoint_profile_monitor_{os.getpid()}")
        self.path_point_count = 0
        self.rows: list[tuple[float, list[float]]] = []
        self.sim_time_sec = 0.0
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(
            PathMessage,
            "/map_nav/global_path",
            self._on_path,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/map_nav/debug",
            self._on_debug,
            50,
        )

    def _on_clock(self, message: Clock) -> None:
        self.sim_time_sec = (
            float(message.clock.sec)
            + float(message.clock.nanosec) * 1.0e-9
        )

    def _on_path(self, message: PathMessage) -> None:
        self.path_point_count = max(
            self.path_point_count,
            len(message.poses),
        )

    def _on_debug(self, message: Float32MultiArray) -> None:
        if len(message.data) < 9:
            return
        stamp = self.sim_time_sec or time.monotonic()
        self.rows.append(
            (stamp, [float(value) for value in message.data])
        )


def calculate_metrics(
    rows: list[tuple[float, list[float]]],
    *,
    path_point_count: int,
    reason: str,
    completed_laps: int,
) -> dict:
    tracking = [(stamp, row) for stamp, row in rows if row[1] > 0.0]
    cte = [abs(row[6]) for _, row in tracking]
    heading = [abs(row[7]) for _, row in tracking]
    steering = [abs(row[0]) for _, row in tracking]
    speed_commands = [row[1] for _, row in tracking]
    measured_speed = [row[8] for _, row in tracking]

    large_reversals = 0
    previous_sign = 0
    previous_time = -math.inf
    previous_command = 0.0
    reversal_events = []
    for stamp, row in tracking:
        if abs(row[5]) > 0.16:
            previous_sign = 0
            continue
        sign = 1 if row[0] >= 12.0 else -1 if row[0] <= -12.0 else 0
        if (
            sign
            and previous_sign
            and sign != previous_sign
            and stamp - previous_time <= 2.0
        ):
            large_reversals += 1
            reversal_events.append(
                {
                    "path_index": int(round(row[2])),
                    "from_command": round(previous_command, 3),
                    "to_command": round(row[0], 3),
                    "cross_track_error_m": round(row[6], 4),
                }
            )
        if sign:
            previous_sign = sign
            previous_time = stamp
            previous_command = row[0]

    duration = (
        tracking[-1][0] - tracking[0][0] if len(tracking) >= 2 else 0.0
    )
    path_indices = [int(round(row[2])) for _, row in tracking]
    context_start = max(0, len(tracking) - 60)
    final_context = [
        {
            "elapsed_sec": round(stamp - tracking[0][0], 3),
            "path_index": int(round(row[2])),
            "steering_command": round(row[0], 3),
            "speed_command": round(row[1], 3),
            "path_curvature_per_m": round(row[5], 4),
            "cross_track_error_m": round(row[6], 4),
            "heading_error_rad": round(row[7], 4),
            "measured_speed_mps": round(row[8], 4),
        }
        for stamp, row in tracking[context_start::4]
    ]
    return {
        "reason": reason,
        "success": completed_laps > 0,
        "completed_laps": completed_laps,
        "path_points": path_point_count,
        "first_path_index": path_indices[0] if path_indices else None,
        "last_path_index": path_indices[-1] if path_indices else None,
        "maximum_path_index": max(path_indices, default=None),
        "tracking_samples": len(tracking),
        "lap_time_sec": duration if completed_laps else None,
        "tracking_duration_sec": duration,
        "speed_command": {
            "mean": statistics.fmean(speed_commands)
            if speed_commands
            else 0.0,
            "p95": percentile(speed_commands, 0.95),
            "maximum": max(speed_commands, default=0.0),
        },
        "measured_speed_mps": {
            "mean": statistics.fmean(measured_speed)
            if measured_speed
            else 0.0,
            "p95": percentile(measured_speed, 0.95),
            "maximum": max(measured_speed, default=0.0),
        },
        "cross_track_error_m": {
            "mean_abs": statistics.fmean(cte) if cte else 0.0,
            "p95_abs": percentile(cte, 0.95),
            "maximum_abs": max(cte, default=0.0),
        },
        "heading_error_rad": {
            "p95_abs": percentile(heading, 0.95),
            "maximum_abs": max(heading, default=0.0),
        },
        "steering_command": {
            "p95_abs": percentile(steering, 0.95),
            "maximum_abs": max(steering, default=0.0),
            "large_straight_reversals": large_reversals,
            "large_straight_reversal_events": reversal_events,
        },
        "final_context_5hz": final_context,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--timeout-sec", type=float, default=70.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=20.0)
    parser.add_argument("--cte-failure-m", type=float, default=0.40)
    parser.add_argument("--cte-failure-hold-sec", type=float, default=0.35)
    parser.add_argument("--expected-path-points", type=int, default=268)
    options = parser.parse_args()

    rclpy.init()
    node = ProfileMonitor()
    wall_start = time.monotonic()
    tracking_start = None
    previous_index = None
    completed_laps = 0
    cte_failure_start = None
    reason = "timeout"

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            tracking = [
                item for item in node.rows if item[1][1] > 0.0
            ]
            if not tracking:
                if now - wall_start >= options.startup_timeout_sec:
                    reason = "no_tracking_debug"
                    break
                continue

            if tracking_start is None:
                tracking_start = tracking[0][0]
            latest_stamp, latest = tracking[-1]
            current_index = int(round(latest[2]))
            point_count = (
                node.path_point_count or options.expected_path_points
            )
            if (
                previous_index is not None
                and previous_index > point_count * 0.80
                and current_index < point_count * 0.20
            ):
                completed_laps += 1
                reason = "lap_complete"
                break
            previous_index = current_index

            if abs(latest[6]) > options.cte_failure_m:
                if cte_failure_start is None:
                    cte_failure_start = latest_stamp
                elif (
                    latest_stamp - cte_failure_start
                    >= options.cte_failure_hold_sec
                ):
                    reason = "cross_track_failure"
                    break
            else:
                cte_failure_start = None

            if latest_stamp - tracking_start >= options.timeout_sec:
                reason = "timeout"
                break
    finally:
        metrics = calculate_metrics(
            node.rows,
            path_point_count=(
                node.path_point_count or options.expected_path_points
            ),
            reason=reason,
            completed_laps=completed_laps,
        )
        output = Path(options.output_json).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(metrics, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(metrics, sort_keys=True), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
