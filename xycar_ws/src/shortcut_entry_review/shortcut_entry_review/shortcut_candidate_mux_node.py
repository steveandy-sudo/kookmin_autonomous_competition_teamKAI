#!/usr/bin/env python3
"""Combine semantic entry control with existing shortcut cruise control."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from xycar_msgs.msg import XycarVescState

from .spatial_entry_control import (
    blend_rule_and_w1_command,
    distance_blend_ratio,
    dynamic_blend_start_distance_m,
)


class ShortcutCandidateMuxNode(Node):
    """Publish one hybrid shortcut candidate without owning /xycar_motor."""

    def __init__(self) -> None:
        super().__init__("shortcut_entry_candidate_mux")
        self.declare_parameter(
            "processing_enabled_topic", "/hybrid/shortcut_processing_enabled"
        )
        self.declare_parameter(
            "entry_controller_topic", "/shortcut/entry/controller_candidate"
        )
        self.declare_parameter(
            "rule_candidate_topic", "/hybrid/rule_candidate"
        )
        self.declare_parameter("rule_angle_index", 0)
        self.declare_parameter("rule_speed_index", 1)
        self.declare_parameter("entry_ready_topic", "/shortcut/entry/ready")
        self.declare_parameter(
            "entry_distance_topic", "/shortcut/entry/entry_distance_m"
        )
        self.declare_parameter("vehicle_state_topic", "/vehicle/vesc_state")
        self.declare_parameter(
            "legacy_cruise_topic", "/shortcut/legacy_cruise_candidate"
        )
        self.declare_parameter(
            "path_valid_topic", "/shortcut/entry/path_valid"
        )
        self.declare_parameter(
            "cruise_handoff_topic", "/shortcut/entry/cruise_enabled"
        )
        self.declare_parameter("phase_topic", "/shortcut/entry/phase")
        self.declare_parameter("candidate_topic", "/hybrid/shortcut_candidate")
        self.declare_parameter("status_topic", "/shortcut/entry/mux_status")
        self.declare_parameter(
            "control_ready_topic", "/shortcut/entry/control_ready"
        )
        self.declare_parameter(
            "control_blend_topic", "/shortcut/entry/control_blend"
        )
        self.declare_parameter(
            "control_debug_topic", "/shortcut/entry/control_debug"
        )
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("candidate_timeout_sec", 0.35)
        self.declare_parameter("vehicle_state_timeout_sec", 0.35)
        self.declare_parameter("full_control_distance_m", 0.15)
        self.declare_parameter("minimum_start_distance_m", 0.55)
        self.declare_parameter("maximum_start_distance_m", 1.20)
        self.declare_parameter("control_latency_sec", 0.25)
        self.declare_parameter("distance_margin_m", 0.08)
        self.declare_parameter("minimum_control_blend", 0.05)
        self.declare_parameter("speed_command_to_mps", 0.08)
        self.declare_parameter("default_enabled", False)

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.processing_enabled = bool(
            self.get_parameter("default_enabled").value
        )
        self.path_valid = False
        self.cruise_handoff = False
        self.phase = 0.0
        self.w1_locked = False
        self.entry_distance_m = math.nan
        self.minimum_entry_distance_m = math.inf
        self.control_blend = 0.0
        self.control_active = False
        self.control_start_logged = False
        self.entry_command = (0.0, 0.0)
        self.entry_command_time = float("-inf")
        self.rule_command = (0.0, 0.0)
        self.rule_command_time = float("-inf")
        self.vehicle_speed_mps = 0.0
        self.vehicle_state_time = float("-inf")
        self.legacy_command = (0.0, 0.0, 0.0, 0.0)
        self.legacy_command_time = float("-inf")
        self.handoff_request_time = float("-inf")
        self.legacy_control_logged = False
        self.last_status = ""

        self.create_subscription(
            Bool,
            str(self.get_parameter("processing_enabled_topic").value),
            self.on_processing,
            state_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("path_valid_topic").value),
            self.on_path_valid,
            state_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("cruise_handoff_topic").value),
            self.on_cruise_handoff,
            state_qos,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("phase_topic").value),
            self.on_phase,
            state_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("entry_controller_topic").value),
            self.on_entry_command,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("rule_candidate_topic").value),
            self.on_rule_command,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("entry_ready_topic").value),
            self.on_entry_ready,
            state_qos,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("entry_distance_topic").value),
            self.on_entry_distance,
            state_qos,
        )
        self.create_subscription(
            XycarVescState,
            str(self.get_parameter("vehicle_state_topic").value),
            self.on_vehicle_state,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("legacy_cruise_topic").value),
            self.on_legacy_command,
            10,
        )
        self.candidate_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("candidate_topic").value),
            10,
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.control_ready_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("control_ready_topic").value),
            state_qos,
        )
        self.control_blend_publisher = self.create_publisher(
            Float32,
            str(self.get_parameter("control_blend_topic").value),
            state_qos,
        )
        self.control_debug_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("control_debug_topic").value),
            10,
        )
        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(
            "shortcut candidate mux ready: distance-based RULE->W1 steering, "
            "RULE speed passthrough; /xycar_motor publisher is intentionally "
            "absent"
        )
        self.publish_control_state()

    def on_processing(self, message: Bool) -> None:
        enabled = bool(message.data)
        if self.processing_enabled and not enabled:
            self.path_valid = False
            self.cruise_handoff = False
            self.phase = 0.0
            self.w1_locked = False
            self.entry_distance_m = math.nan
            self.minimum_entry_distance_m = math.inf
            self.control_blend = 0.0
            self.control_active = False
            self.control_start_logged = False
            self.entry_command_time = float("-inf")
            self.rule_command_time = float("-inf")
            self.legacy_command_time = float("-inf")
            self.handoff_request_time = float("-inf")
            self.legacy_control_logged = False
        self.processing_enabled = enabled
        self.publish_control_state()

    def on_path_valid(self, message: Bool) -> None:
        self.path_valid = bool(message.data)

    def on_cruise_handoff(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested and not self.cruise_handoff:
            self.handoff_request_time = time.monotonic()
            self.legacy_control_logged = False
            self.get_logger().warning(
                "SHORTCUT HANDOFF GATE: semantic entry finished; waiting for "
                "the first fresh existing ShortcutCore cruise candidate"
            )
        elif not requested:
            self.handoff_request_time = float("-inf")
            self.legacy_control_logged = False
        self.cruise_handoff = requested

    def on_phase(self, message: Float32) -> None:
        self.phase = float(message.data)

    def on_entry_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            return
        self.entry_command = (float(message.data[0]), float(message.data[1]))
        self.entry_command_time = time.monotonic()

    def on_rule_command(self, message: Float32MultiArray) -> None:
        angle_index = int(self.get_parameter("rule_angle_index").value)
        speed_index = int(self.get_parameter("rule_speed_index").value)
        if (
            angle_index < 0
            or speed_index < 0
            or len(message.data) <= max(angle_index, speed_index)
        ):
            return
        self.rule_command = (
            float(message.data[angle_index]),
            float(message.data[speed_index]),
        )
        self.rule_command_time = time.monotonic()

    def on_entry_ready(self, message: Bool) -> None:
        requested = bool(message.data)
        if self.w1_locked and not requested:
            self.entry_distance_m = math.nan
            self.minimum_entry_distance_m = math.inf
            self.control_blend = 0.0
            self.control_active = False
            self.control_start_logged = False
            self.publish_control_state()
        self.w1_locked = requested

    def on_entry_distance(self, message: Float32) -> None:
        distance = float(message.data)
        self.entry_distance_m = distance
        if self.w1_locked and math.isfinite(distance) and distance >= 0.0:
            self.minimum_entry_distance_m = min(
                self.minimum_entry_distance_m, distance
            )

    def on_vehicle_state(self, message: XycarVescState) -> None:
        speed = float(message.speed_mps)
        if not math.isfinite(speed):
            return
        self.vehicle_speed_mps = abs(speed)
        self.vehicle_state_time = time.monotonic()

    def publish_control_state(self) -> None:
        self.control_ready_publisher.publish(
            Bool(data=bool(self.control_active))
        )
        self.control_blend_publisher.publish(
            Float32(data=float(self.control_blend))
        )

    def update_spatial_blend(self, now: float) -> tuple[float, float, float]:
        timeout = float(self.get_parameter("vehicle_state_timeout_sec").value)
        if now - self.vehicle_state_time <= timeout:
            speed_mps = self.vehicle_speed_mps
        else:
            speed_mps = abs(self.rule_command[1]) * float(
                self.get_parameter("speed_command_to_mps").value
            )
        start_distance = dynamic_blend_start_distance_m(
            speed_mps=speed_mps,
            full_control_distance_m=float(
                self.get_parameter("full_control_distance_m").value
            ),
            minimum_start_distance_m=float(
                self.get_parameter("minimum_start_distance_m").value
            ),
            maximum_start_distance_m=float(
                self.get_parameter("maximum_start_distance_m").value
            ),
            control_latency_sec=float(
                self.get_parameter("control_latency_sec").value
            ),
            distance_margin_m=float(
                self.get_parameter("distance_margin_m").value
            ),
        )
        distance = self.minimum_entry_distance_m
        if self.w1_locked and math.isfinite(distance):
            spatial_blend = distance_blend_ratio(
                remaining_distance_m=distance,
                start_distance_m=start_distance,
                full_control_distance_m=float(
                    self.get_parameter("full_control_distance_m").value
                ),
            )
            # Once the physical entry is reached, perception jitter must not
            # move control back toward RULE.
            self.control_blend = max(self.control_blend, spatial_blend)
        threshold = float(
            self.get_parameter("minimum_control_blend").value
        )
        if self.w1_locked and self.control_blend >= threshold:
            self.control_active = True
        return float(distance), float(start_distance), float(speed_mps)

    def on_legacy_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 3:
            return
        self.legacy_command = (
            float(message.data[0]),
            float(message.data[1]),
            float(message.data[2]),
            float(message.data[3]) if len(message.data) >= 4 else 2.0,
        )
        self.legacy_command_time = time.monotonic()

    def publish_status(self, status: str) -> None:
        if status == self.last_status:
            return
        self.last_status = status
        self.status_publisher.publish(String(data=status))
        self.get_logger().info(status)

    def step(self) -> None:
        if not self.processing_enabled:
            return
        now = time.monotonic()
        timeout = float(self.get_parameter("candidate_timeout_sec").value)
        distance, start_distance, speed_mps = self.update_spatial_blend(now)
        rule_fresh = now - self.rule_command_time <= timeout
        entry_fresh = now - self.entry_command_time <= timeout
        blended_angle, preserved_speed = blend_rule_and_w1_command(
            rule_angle=self.rule_command[0],
            rule_speed=self.rule_command[1],
            w1_angle=self.entry_command[0],
            blend_ratio=self.control_blend,
        )
        if self.control_active and not self.control_start_logged:
            self.get_logger().warning(
                "[MISSION] SHORTCUT ENTRY STEERING START: distance-based "
                f"gate; fork={distance:.3f}m start={start_distance:.3f}m "
                f"speed_mps={speed_mps:.3f}"
            )
            self.control_start_logged = True
        if self.control_active and rule_fresh and entry_fresh:
            self.get_logger().info(
                "[MISSION] W1 STEERING: "
                f"rule={self.rule_command[0]:+.2f}deg "
                f"w1={self.entry_command[0]:+.2f}deg "
                f"blend={self.control_blend:.3f} "
                f"output={blended_angle:+.2f}deg "
                f"RULE_speed={preserved_speed:.2f}"
            )
        self.control_debug_publisher.publish(
            Float32MultiArray(
                data=[
                    float(blended_angle),
                    float(preserved_speed),
                    float(self.control_blend),
                    float(distance),
                    float(start_distance),
                    float(self.entry_command[0]),
                    float(self.rule_command[0]),
                    float(speed_mps),
                ]
            )
        )
        if self.cruise_handoff:
            if now - self.legacy_command_time <= timeout and rule_fresh:
                command = (
                    self.legacy_command[0],
                    self.rule_command[1],
                    self.legacy_command[2],
                    self.legacy_command[3],
                )
                if not self.legacy_control_logged:
                    delay_ms = max(
                        0.0, (now - self.handoff_request_time) * 1000.0
                    )
                    self.get_logger().warning(
                        "SHORTCUT CONTROL SWITCHED: semantic W1/Y1 entry -> "
                        "existing ShortcutCore cruise/exit; "
                        f"handoff_delay_ms={delay_ms:.1f} phase={self.phase:.0f}"
                    )
                    self.legacy_control_logged = True
                self.publish_status(
                    "existing ShortcutCore cruise/exit steering; RULE speed "
                    "preserved"
                )
            else:
                command = (0.0, 0.0, 0.0, 4.0)
                self.publish_status("legacy cruise candidate stale; safe stop")
        elif self.path_valid and entry_fresh and rule_fresh:
            command = (
                blended_angle,
                preserved_speed,
                0.0,
                10.0 + float(self.phase),
            )
            if self.control_active:
                self.publish_status(
                    "spatial RULE->W1 steering blend active; RULE speed "
                    "preserved"
                )
            else:
                self.publish_status(
                    "W1 locked at distance; RULE steering and speed "
                    "retained"
                )
        else:
            command = (0.0, 0.0, 0.0, 10.0 + float(self.phase))
            self.publish_status("entry path/controller unavailable; safe stop")
        self.candidate_publisher.publish(
            Float32MultiArray(data=[float(value) for value in command])
        )
        self.publish_control_state()


def main() -> None:
    rclpy.init()
    node = ShortcutCandidateMuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
