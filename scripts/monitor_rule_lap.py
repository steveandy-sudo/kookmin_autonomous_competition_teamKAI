#!/usr/bin/env python3
"""Measure a rule-driver lap against the CAD-derived track reference."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from tf2_msgs.msg import TFMessage

from xycar_rl.track_geometry import TrackReference


class RuleLapMonitor(Node):
    def __init__(
        self,
        *,
        world_path: Path,
        reverse_direction: bool,
        target_right_offset_m: float,
        offtrack_m: float,
        frame_dir: Path | None,
        frame_step_m: float,
    ) -> None:
        super().__init__("rule_lap_monitor")
        self.track = TrackReference.from_sdf(
            world_path,
            target_right_offset_m=target_right_offset_m,
            reverse_direction=reverse_direction,
        )
        self.offtrack_m = float(offtrack_m)
        self.frame_dir = frame_dir
        self.frame_step_m = max(0.1, float(frame_step_m))
        self.next_frame_progress_m = 0.0
        self.image_bridge = CvBridge()
        self.latest_canonical: Image | None = None
        self.latest_rule_debug: Image | None = None
        if self.frame_dir is not None:
            self.frame_dir.mkdir(parents=True, exist_ok=True)
        self.last_projection = None
        self.accumulated_progress_m = 0.0
        self.negative_progress_m = 0.0
        self.max_abs_cte_m = 0.0
        self.pose_count = 0
        self.command_count = 0
        self.canonical_count = 0
        self.last_command = [0.0, 0.0]
        self.bridge_debug = [0.0] * 6
        self.rule_diagnostics = [0.0] * 12
        self.hybrid_debug = [0.0] * 15
        self.hybrid_mode = "unavailable"
        self.hybrid_mode_counts: Counter[str] = Counter()
        self.steering_commands: list[float] = []
        self.speed_commands: list[float] = []
        self.hybrid_decisions: list[dict[str, float | int]] = []
        self.started_at = time.monotonic()
        self.motion_started_at: float | None = None
        self.last_print_at = 0.0
        self.reason = "timeout"
        self.done = False

        self.create_subscription(
            TFMessage,
            "/world/kookmin_xycar_track/dynamic_pose/info",
            self.on_pose,
            20,
        )
        self.create_subscription(
            Float32MultiArray,
            "/xycar_motor",
            self.on_command,
            20,
        )
        self.create_subscription(
            Float32MultiArray,
            "/xycar_motor_bridge/debug",
            self.on_bridge_debug,
            20,
        )
        self.create_subscription(
            Float32MultiArray,
            "/rule_drive/diagnostics",
            self.on_rule_diagnostics,
            20,
        )
        self.create_subscription(
            String,
            "/hybrid/mode",
            self.on_hybrid_mode,
            20,
        )
        self.create_subscription(
            Float32MultiArray,
            "/hybrid/debug",
            self.on_hybrid_debug,
            20,
        )
        self.create_subscription(
            Image,
            "/perception/canonical_road_image",
            self.on_canonical,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/rule_drive/canonical_debug_image",
            self.on_rule_debug,
            qos_profile_sensor_data,
        )

    def on_command(self, message: Float32MultiArray) -> None:
        if len(message.data) >= 2:
            self.last_command = list(message.data[:2])
            self.steering_commands.append(float(message.data[0]))
            self.speed_commands.append(float(message.data[1]))
            self.command_count += 1

    def on_bridge_debug(self, message: Float32MultiArray) -> None:
        self.bridge_debug = list(message.data)

    def on_rule_diagnostics(self, message: Float32MultiArray) -> None:
        self.rule_diagnostics = list(message.data)

    def on_hybrid_mode(self, message: String) -> None:
        self.hybrid_mode = message.data.split(" | ", maxsplit=1)[0]
        self.hybrid_mode_counts[self.hybrid_mode] += 1

    def on_hybrid_debug(self, message: Float32MultiArray) -> None:
        self.hybrid_debug = list(message.data)
        if len(message.data) >= 15:
            self.hybrid_decisions.append(
                {
                    "mode": int(round(float(message.data[0]))),
                    "angle": float(message.data[1]),
                    "speed": float(message.data[2]),
                    "model_angle": float(message.data[3]),
                    "rule_angle": float(message.data[5]),
                    "route_curve": int(round(float(message.data[14]))),
                }
            )

    def on_canonical(self, message: Image) -> None:
        self.canonical_count += 1
        self.latest_canonical = message

    def on_rule_debug(self, message: Image) -> None:
        self.latest_rule_debug = message

    def save_progress_frame(self, progress_m: float) -> None:
        if (
            self.frame_dir is None
            or self.latest_canonical is None
            or self.latest_rule_debug is None
        ):
            return
        canonical = self.image_bridge.imgmsg_to_cv2(
            self.latest_canonical,
            desired_encoding="bgr8",
        )
        rule_debug = self.image_bridge.imgmsg_to_cv2(
            self.latest_rule_debug,
            desired_encoding="bgr8",
        )
        panel = np.hstack((canonical, rule_debug))
        filename = (
            f"progress_{progress_m:05.2f}m_"
            f"cmd_{self.last_command[0]:+05.1f}.png"
        )
        cv2.imwrite(str(self.frame_dir / filename), panel)

    def on_pose(self, message: TFMessage) -> None:
        if not message.transforms:
            return
        transform = message.transforms[0].transform
        rotation = transform.rotation
        yaw = math.atan2(
            2.0
            * (
                rotation.w * rotation.z
                + rotation.x * rotation.y
            ),
            1.0
            - 2.0
            * (
                rotation.y * rotation.y
                + rotation.z * rotation.z
            ),
        )
        hint = (
            self.last_projection.segment_index
            if self.last_projection is not None
            else None
        )
        projection = self.track.project(
            transform.translation.x,
            transform.translation.y,
            yaw,
            hint_segment_index=hint,
        )
        if self.last_projection is not None:
            delta = self.track.progress_delta(
                self.last_projection.progress_m,
                projection.progress_m,
            )
            if delta > 0.0:
                self.accumulated_progress_m += delta
                if (
                    self.motion_started_at is None
                    and self.accumulated_progress_m >= 0.05
                ):
                    self.motion_started_at = time.monotonic()
            else:
                self.negative_progress_m -= delta
        self.last_projection = projection
        self.pose_count += 1
        self.max_abs_cte_m = max(
            self.max_abs_cte_m,
            abs(projection.cross_track_error_m),
        )
        if self.accumulated_progress_m >= self.next_frame_progress_m:
            self.save_progress_frame(self.accumulated_progress_m)
            self.next_frame_progress_m += self.frame_step_m

        now = time.monotonic()
        if now - self.last_print_at >= 0.13:
            self.last_print_at = now
            pursuit = (
                self.rule_diagnostics[6]
                if len(self.rule_diagnostics) > 6
                else 0.0
            )
            stanley = (
                self.rule_diagnostics[7]
                if len(self.rule_diagnostics) > 7
                else 0.0
            )
            source = (
                int(self.rule_diagnostics[11])
                if len(self.rule_diagnostics) > 11
                else 0
            )
            applied = (
                self.bridge_debug[0]
                if self.bridge_debug
                else 0.0
            )
            yaw_rate = (
                self.bridge_debug[4]
                if len(self.bridge_debug) > 4
                else 0.0
            )
            model_angle = (
                self.hybrid_debug[3]
                if len(self.hybrid_debug) > 3
                else 0.0
            )
            rule_angle = (
                self.hybrid_debug[5]
                if len(self.hybrid_debug) > 5
                else 0.0
            )
            curvature = (
                self.hybrid_debug[10]
                if len(self.hybrid_debug) > 10
                else 0.0
            )
            path_turn = (
                self.hybrid_debug[13]
                if len(self.hybrid_debug) > 13
                else 0.0
            )
            route_curve = (
                int(self.hybrid_debug[14])
                if len(self.hybrid_debug) > 14
                else -1
            )
            print(
                f"{now - self.started_at:5.2f} "
                f"prog={self.accumulated_progress_m:5.2f} "
                f"abs={projection.progress_m:5.2f} "
                f"cte={projection.cross_track_error_m:+.3f} "
                f"head={projection.heading_error_rad:+.2f} "
                f"cmd={self.last_command[0]:+5.1f} "
                f"app={applied:+5.1f} "
                f"yr={yaw_rate:+.2f} "
                f"pp/st={pursuit:+.2f}/{stanley:+.2f} "
                f"src={source} mode={self.hybrid_mode} "
                f"model/rule={model_angle:+.1f}/{rule_angle:+.1f} "
                f"curve={curvature:.3f} turn={path_turn:+.3f} "
                f"route={route_curve}",
                flush=True,
            )

        if (
            abs(projection.cross_track_error_m) > self.offtrack_m
            and self.accumulated_progress_m > 1.0
        ):
            self.reason = "offtrack"
            self.done = True
        elif self.accumulated_progress_m >= self.track.length_m * 0.98:
            self.reason = "lap_complete"
            self.done = True

    def result(self, reverse_direction: bool) -> dict[str, object]:
        canonical_curve_commands = sum(
            min(abs(value - allowed) for allowed in (-42.0, 0.0, 42.0))
            <= 1.0e-3
            for value in self.steering_commands
        )
        curve_decisions = [
            item for item in self.hybrid_decisions if item["mode"] == 1
        ]
        straight_decisions = [
            item for item in self.hybrid_decisions if item["mode"] == 0
        ]
        curve_angles = [float(item["angle"]) for item in curve_decisions]
        curve_deltas = [
            abs(float(current["angle"]) - float(previous["angle"]))
            for previous, current in zip(
                self.hybrid_decisions,
                self.hybrid_decisions[1:],
            )
            if previous["mode"] == 1 and current["mode"] == 1
        ]
        curve_sign_changes = sum(
            float(previous["angle"]) * float(current["angle"]) < 0.0
            for previous, current in zip(
                self.hybrid_decisions,
                self.hybrid_decisions[1:],
            )
            if previous["mode"] == 1 and current["mode"] == 1
        )
        active_curve_decisions = [
            item for item in curve_decisions if float(item["speed"]) > 0.0
        ]
        straight_deltas = [
            abs(float(current["angle"]) - float(previous["angle"]))
            for previous, current in zip(
                self.hybrid_decisions,
                self.hybrid_decisions[1:],
            )
            if previous["mode"] == 0 and current["mode"] == 0
        ]
        return {
            "reason": self.reason,
            "reverse_direction": reverse_direction,
            "track_length_m": self.track.length_m,
            "accumulated_progress_m": self.accumulated_progress_m,
            "negative_progress_m": self.negative_progress_m,
            "progress_fraction": (
                self.accumulated_progress_m / self.track.length_m
            ),
            "max_abs_cte_m": self.max_abs_cte_m,
            "final_absolute_progress_m": (
                None
                if self.last_projection is None
                else self.last_projection.progress_m
            ),
            "pose_count": self.pose_count,
            "command_count": self.command_count,
            "steering_command_values": sorted(
                {round(value, 3) for value in self.steering_commands}
            ),
            "canonical_three_level_command_count": canonical_curve_commands,
            "speed_command_min": (
                min(self.speed_commands) if self.speed_commands else None
            ),
            "speed_command_mean": (
                float(np.mean(self.speed_commands))
                if self.speed_commands
                else None
            ),
            "speed_command_max": (
                max(self.speed_commands) if self.speed_commands else None
            ),
            "hybrid_mode_counts": dict(self.hybrid_mode_counts),
            "hybrid_decision_count": len(self.hybrid_decisions),
            "curve_decision_count": len(curve_decisions),
            "curve_command_values": sorted(
                {round(float(item["angle"]), 3) for item in curve_decisions}
            ),
            "curve_command_min": min(curve_angles) if curve_angles else None,
            "curve_command_mean": (
                float(np.mean(curve_angles)) if curve_angles else None
            ),
            "curve_command_max": max(curve_angles) if curve_angles else None,
            "curve_command_abs_mean": (
                float(np.mean(np.abs(curve_angles)))
                if curve_angles
                else None
            ),
            "curve_max_command_delta": (
                max(curve_deltas) if curve_deltas else 0.0
            ),
            "curve_command_sign_changes": curve_sign_changes,
            "curve_three_level_command_count": sum(
                min(abs(value - allowed) for allowed in (-42.0, 0.0, 42.0))
                <= 1.0e-3
                for value in curve_angles
            ),
            "curve_startup_stop_count": (
                len(curve_decisions) - len(active_curve_decisions)
            ),
            "curve_speed_min": (
                min(float(item["speed"]) for item in active_curve_decisions)
                if active_curve_decisions
                else None
            ),
            "curve_speed_mean": (
                float(
                    np.mean([item["speed"] for item in active_curve_decisions])
                )
                if active_curve_decisions
                else None
            ),
            "curve_speed_max": (
                max(float(item["speed"]) for item in active_curve_decisions)
                if active_curve_decisions
                else None
            ),
            "straight_decision_count": len(straight_decisions),
            "straight_max_command_delta": (
                max(straight_deltas) if straight_deltas else 0.0
            ),
            "canonical_count": self.canonical_count,
            "elapsed_sec": time.monotonic() - self.started_at,
            "motion_elapsed_sec": (
                None
                if self.motion_started_at is None
                else time.monotonic() - self.motion_started_at
            ),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--world",
        type=Path,
        default=Path("worlds/kookmin_xycar_track_final.sdf"),
    )
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--target-right-offset-m", type=float, default=0.20)
    parser.add_argument("--offtrack-m", type=float, default=1.20)
    parser.add_argument("--reverse-direction", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--frame-dir", type=Path)
    parser.add_argument("--frame-step-m", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = RuleLapMonitor(
        world_path=args.world,
        reverse_direction=args.reverse_direction,
        target_right_offset_m=args.target_right_offset_m,
        offtrack_m=args.offtrack_m,
        frame_dir=args.frame_dir,
        frame_step_m=args.frame_step_m,
    )
    deadline = time.monotonic() + max(0.1, args.duration_sec)
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        result = node.result(args.reverse_direction)
        print(f"RESULT {json.dumps(result, indent=2)}", flush=True)
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(result, indent=2) + "\n",
                encoding="utf-8",
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
