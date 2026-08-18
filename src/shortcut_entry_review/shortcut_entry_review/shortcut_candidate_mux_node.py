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


def rate_limit_steering(
    *,
    previous_angle: float,
    target_angle: float,
    maximum_rate: float,
    dt_sec: float,
) -> float:
    """Limit a steering transition while preserving its requested direction."""
    previous = float(previous_angle)
    target = float(target_angle)
    rate = max(0.0, float(maximum_rate))
    elapsed = max(0.0, float(dt_sec))
    if rate <= 0.0 or elapsed <= 0.0:
        return target if rate <= 0.0 else previous
    maximum_delta = rate * elapsed
    return previous + min(
        maximum_delta,
        max(-maximum_delta, target - previous),
    )


def semantic_entry_candidate(
    *,
    rule_command: tuple[float, float],
    entry_command: tuple[float, float],
    steering_blend: float,
    phase: float,
) -> tuple[float, float, float, float]:
    blend = min(1.0, max(0.0, float(steering_blend)))
    output_angle = (
        (1.0 - blend) * float(rule_command[0])
        + blend * float(entry_command[0])
    )
    return (
        output_angle,
        float(rule_command[1]),
        0.0,
        10.0 + float(phase),
    )


def shortcut_core_candidate(
    *,
    rule_command: tuple[float, float],
    legacy_command: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return (
        float(legacy_command[0]),
        float(rule_command[1]),
        float(legacy_command[2]),
        float(legacy_command[3]),
    )


def rule_handoff_candidate(
    *, rule_command: tuple[float, float]
) -> tuple[float, float, float, float]:
    """Signal completion while preserving the current Xbin RULE command."""
    return (
        float(rule_command[0]),
        float(rule_command[1]),
        1.0,
        4.0,
    )


def rule_search_candidate(
    *, rule_command: tuple[float, float], phase: float
) -> tuple[float, float, float, float]:
    """Keep normal RULE control while W1 has not produced a path yet."""
    return (
        float(rule_command[0]),
        float(rule_command[1]),
        0.0,
        10.0 + float(phase),
    )


def enforce_directional_hold(
    *, candidate_angle: float, hold_command: float
) -> float:
    """Keep at least the configured steering magnitude in one direction."""
    candidate = float(candidate_angle)
    hold = float(hold_command)
    if hold < 0.0:
        return min(candidate, hold)
    if hold > 0.0:
        return max(candidate, hold)
    return candidate


def entry_speed_cap(*, rule_speed: float, maximum_entry_speed: float) -> float:
    """Slow for shortcut perception without accelerating a degraded RULE path."""
    speed = float(rule_speed)
    if speed <= 0.0:
        return speed
    return min(speed, max(0.0, float(maximum_entry_speed)))


def held_w1_candidate(
    *, rule_command: tuple[float, float], held_angle: float, phase: float
) -> tuple[float, float, float, float]:
    """Hold the latest observed W1 steering through a short mask gap."""
    return (
        float(held_angle),
        float(rule_command[1]),
        0.0,
        10.0 + float(phase),
    )


def semantic_entry_control_available(
    *,
    path_valid: bool,
    entry_ready: bool,
    phase: float,
    command_age_sec: float,
    timeout_sec: float,
) -> bool:
    """Allow W1 steering only after the selected entry trigger is ready."""
    return bool(
        path_valid
        and entry_ready
        and float(phase) >= 1.0
        and float(command_age_sec) <= max(0.0, float(timeout_sec))
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
            "legacy_cruise_topic", "/shortcut/legacy_cruise_candidate"
        )
        self.declare_parameter("rule_command_topic", "/hybrid/rule_candidate")
        self.declare_parameter(
            "steering_blend_topic", "/shortcut/entry/steering_blend"
        )
        self.declare_parameter(
            "path_valid_topic", "/shortcut/entry/path_valid"
        )
        self.declare_parameter("entry_ready_topic", "/shortcut/entry/ready")
        self.declare_parameter(
            "cruise_handoff_topic", "/shortcut/entry/cruise_enabled"
        )
        self.declare_parameter("phase_topic", "/shortcut/entry/phase")
        self.declare_parameter("candidate_topic", "/hybrid/shortcut_candidate")
        self.declare_parameter("status_topic", "/shortcut/entry/mux_status")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("candidate_timeout_sec", 0.35)
        self.declare_parameter("rule_command_timeout_sec", 0.35)
        self.declare_parameter("w1_steering_hold_sec", 1.3)
        self.declare_parameter("entry_direction_hold_command", -30.0)
        self.declare_parameter("entry_direction_hold_enabled", False)
        self.declare_parameter("entry_steering_rate_limit_cmd_per_sec", 90.0)
        self.declare_parameter("entry_speed_command", 9.0)
        self.declare_parameter("handoff_to_rule", True)
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
        self.entry_ready = False
        self.cruise_handoff = False
        self.phase = 0.0
        self.entry_command = (0.0, 0.0)
        self.entry_command_time = float("-inf")
        self.legacy_command = (0.0, 0.0, 0.0, 0.0)
        self.legacy_command_time = float("-inf")
        self.rule_command = (0.0, 0.0)
        self.rule_command_time = float("-inf")
        self.steering_blend = 0.0
        self.handoff_request_time = float("-inf")
        self.legacy_control_logged = False
        self.last_status = ""
        self.last_steering_log_time = float("-inf")
        self.last_valid_w1_angle = 0.0
        self.last_valid_w1_time = float("-inf")
        self.last_output_angle: float | None = None
        self.last_output_time = time.monotonic()

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
            str(self.get_parameter("entry_ready_topic").value),
            self.on_entry_ready,
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
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("rule_command_topic").value),
            self.on_rule_command,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("steering_blend_topic").value),
            self.on_steering_blend,
            state_qos,
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
            self.entry_ready = False
            self.cruise_handoff = False
            self.phase = 0.0
            self.entry_command_time = float("-inf")
            self.legacy_command_time = float("-inf")
            self.handoff_request_time = float("-inf")
            self.legacy_control_logged = False
            self.steering_blend = 0.0
            self.last_valid_w1_angle = 0.0
            self.last_valid_w1_time = float("-inf")
            self.last_output_angle = None
            self.last_output_time = time.monotonic()
        self.processing_enabled = enabled

    def on_path_valid(self, message: Bool) -> None:
        self.path_valid = bool(message.data)

    def on_entry_ready(self, message: Bool) -> None:
        self.entry_ready = bool(message.data)

    def on_cruise_handoff(self, message: Bool) -> None:
        requested = bool(message.data)
        if requested and not self.processing_enabled:
            return
        if requested and not self.cruise_handoff:
            self.handoff_request_time = time.monotonic()
            self.legacy_control_logged = False
            if bool(self.get_parameter("handoff_to_rule").value):
                self.get_logger().warning(
                    "\033[95m[MISSION] SHORTCUT ENTRY COMPLETE -> "
                    "REQUEST YELLOW XBIN RULE\033[0m"
                )
            else:
                self.get_logger().warning(
                    "SHORTCUT HANDOFF GATE: semantic entry finished; waiting "
                    "for the first fresh existing ShortcutCore cruise "
                    "candidate"
                )
        elif not requested:
            self.handoff_request_time = float("-inf")
            self.legacy_control_logged = False
        self.cruise_handoff = requested

    def on_phase(self, message: Float32) -> None:
        new_phase = float(message.data)
        if self.phase < 1.0 <= new_phase:
            # Discard the controller's pre-W1 zero command.  The next command
            # is then guaranteed to have been computed from the W1 path.
            self.entry_command_time = float("-inf")
        self.phase = new_phase

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

    def on_rule_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            return
        self.rule_command = (float(message.data[0]), float(message.data[1]))
        self.rule_command_time = time.monotonic()

    def on_steering_blend(self, message: Float32) -> None:
        self.steering_blend = min(1.0, max(0.0, float(message.data)))

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
        output_dt_sec = min(0.25, max(0.0, now - self.last_output_time))
        timeout = float(self.get_parameter("candidate_timeout_sec").value)
        rule_fresh = (
            now - self.rule_command_time
            <= float(self.get_parameter("rule_command_timeout_sec").value)
        )
        if not rule_fresh:
            self.candidate_publisher.publish(
                Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])
            )
            self.publish_status("RULE candidate stale; safe stop")
            self.last_output_angle = 0.0
            self.last_output_time = now
            return
        entry_speed = entry_speed_cap(
            rule_speed=self.rule_command[1],
            maximum_entry_speed=float(
                self.get_parameter("entry_speed_command").value
            ),
        )
        if self.cruise_handoff:
            if bool(self.get_parameter("handoff_to_rule").value):
                command = rule_handoff_candidate(
                    rule_command=self.rule_command
                )
                self.publish_status(
                    "semantic shortcut entry complete; yellow Xbin RULE "
                    "handoff requested"
                )
            elif now - self.legacy_command_time <= timeout:
                command = shortcut_core_candidate(
                    rule_command=self.rule_command,
                    legacy_command=self.legacy_command,
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
                self.publish_status("existing ShortcutCore cruise/exit candidate")
            else:
                command = (0.0, 0.0, 0.0, 4.0)
                self.publish_status("legacy cruise candidate stale; safe stop")
        elif semantic_entry_control_available(
            path_valid=self.path_valid,
            entry_ready=self.entry_ready,
            phase=self.phase,
            command_age_sec=now - self.entry_command_time,
            timeout_sec=timeout,
        ):
            blend = float(self.steering_blend)
            command = semantic_entry_candidate(
                rule_command=self.rule_command,
                entry_command=self.entry_command,
                steering_blend=blend,
                phase=self.phase,
            )
            hold_enabled = bool(
                self.get_parameter("entry_direction_hold_enabled").value
            )
            output_angle = float(command[0])
            if hold_enabled:
                output_angle = enforce_directional_hold(
                    candidate_angle=output_angle,
                    hold_command=float(
                        self.get_parameter(
                            "entry_direction_hold_command"
                        ).value
                    ),
                )
            # Follow the W1 controller directly, but limit the command change
            # per control tick so the branch transition cannot create a step.
            if self.last_output_angle is not None:
                output_angle = rate_limit_steering(
                    previous_angle=self.last_output_angle,
                    target_angle=output_angle,
                    maximum_rate=float(
                        self.get_parameter(
                            "entry_steering_rate_limit_cmd_per_sec"
                        ).value
                    ),
                    dt_sec=output_dt_sec,
                )
            command = (output_angle, command[1], command[2], command[3])
            command = (command[0], entry_speed, command[2], command[3])
            self.last_valid_w1_angle = float(output_angle)
            self.last_valid_w1_time = now
            if now - self.last_steering_log_time >= 0.5:
                self.get_logger().info(
                    "[MISSION] W1 STEERING: "
                    f"RULE={self.rule_command[0]:+.2f} "
                    f"W1={self.entry_command[0]:+.2f} "
                    f"blend={blend:.2f} held_output={output_angle:+.2f} "
                    f"RULE speed={self.rule_command[1]:.2f} "
                    f"entry speed={entry_speed:.2f}"
                )
                self.last_steering_log_time = now
            self.publish_status("semantic W1/Y1 entry candidate")
        else:
            hold_sec = max(
                0.0, float(self.get_parameter("w1_steering_hold_sec").value)
            )
            if now - self.last_valid_w1_time <= hold_sec:
                command = held_w1_candidate(
                    rule_command=self.rule_command,
                    held_angle=self.last_valid_w1_angle,
                    phase=self.phase,
                )
                command = (command[0], entry_speed, command[2], command[3])
                self.publish_status(
                    "[MISSION] W1 TEMPORARY LOSS: holding last W1 steering"
                )
            else:
                command = rule_search_candidate(
                    rule_command=self.rule_command,
                    phase=self.phase,
                )
                command = (command[0], entry_speed, command[2], command[3])
                self.publish_status(
                    "[MISSION] W1 SEARCH: retaining yellow Xbin RULE"
                )
        self.candidate_publisher.publish(
            Float32MultiArray(data=[float(value) for value in command])
        )
        self.last_output_angle = float(command[0])
        self.last_output_time = now


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
