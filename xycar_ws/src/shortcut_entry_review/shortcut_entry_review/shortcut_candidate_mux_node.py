#!/usr/bin/env python3
"""Combine semantic entry control with the existing shortcut cruise candidate."""

from __future__ import annotations

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
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("candidate_timeout_sec", 0.35)
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
        self.entry_command = (0.0, 0.0)
        self.entry_command_time = float("-inf")
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
        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(
            "shortcut candidate mux ready; /xycar_motor publisher is intentionally absent"
        )

    def on_processing(self, message: Bool) -> None:
        enabled = bool(message.data)
        if self.processing_enabled and not enabled:
            self.path_valid = False
            self.cruise_handoff = False
            self.phase = 0.0
            self.entry_command_time = float("-inf")
            self.legacy_command_time = float("-inf")
            self.handoff_request_time = float("-inf")
            self.legacy_control_logged = False
        self.processing_enabled = enabled

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
        if self.cruise_handoff:
            if now - self.legacy_command_time <= timeout:
                command = self.legacy_command
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
                self.publish_status("existing ShortcutCore cruise/exit candidate")
            else:
                command = (0.0, 0.0, 0.0, 4.0)
                self.publish_status("legacy cruise candidate stale; safe stop")
        elif self.path_valid and now - self.entry_command_time <= timeout:
            command = (
                self.entry_command[0],
                self.entry_command[1],
                0.0,
                10.0 + float(self.phase),
            )
            self.publish_status("semantic W1/Y1 entry candidate")
        else:
            command = (0.0, 0.0, 0.0, 10.0 + float(self.phase))
            self.publish_status("entry path/controller unavailable; safe stop")
        self.candidate_publisher.publish(
            Float32MultiArray(data=[float(value) for value in command])
        )


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
