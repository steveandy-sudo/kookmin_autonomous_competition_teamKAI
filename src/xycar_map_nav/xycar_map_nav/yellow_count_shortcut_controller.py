#!/usr/bin/env python3
"""Yellow-dash-count shortcut candidate with a time-bounded forced turn."""

from __future__ import annotations

import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, String

from xycar_map_nav.yellow_count_core import (
    W1BevYellowProjector,
    YellowBandPassCounter,
    yellow_mask_occupies_count_band,
)


YELLOW = "\033[93m"
RESET = "\033[0m"


def capped_entry_speed(rule_speed: float, maximum_entry_speed: float) -> float:
    speed = float(rule_speed)
    if speed <= 0.0:
        return speed
    return min(speed, max(0.0, float(maximum_entry_speed)))


class YellowCountShortcutController(Node):
    """Publish only `/hybrid/shortcut_candidate`, never `/xycar_motor`."""

    def __init__(self) -> None:
        super().__init__("yellow_count_shortcut_controller")
        self.declare_parameter(
            "processing_enabled_topic", "/hybrid/shortcut_processing_enabled"
        )
        self.declare_parameter(
            "yellow_mask_topic", "/yellow_count/lraspp/yellow_mask"
        )
        self.declare_parameter("rule_command_topic", "/hybrid/rule_candidate")
        self.declare_parameter("candidate_topic", "/hybrid/shortcut_candidate")
        self.declare_parameter("ready_topic", "/shortcut/entry/ready")
        self.declare_parameter(
            "status_topic", "/shortcut/yellow_count/status"
        )
        # Keep the time-based RULE handoff within roughly one 20 ms tick of
        # the operator-selected duration (for example 0.7 s).
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("rule_command_timeout_sec", 0.35)
        self.declare_parameter("entry_speed_command", 9.0)
        self.declare_parameter("forced_steering_command", -42.0)
        self.declare_parameter("forced_steering_sec", 0.7)
        self.declare_parameter("maximum_abs_steering_command", 42.0)
        self.declare_parameter("yellow_pass_target", 1)
        self.declare_parameter("yellow_pass_component_minimum_area_px", 80)
        self.declare_parameter("yellow_pass_line_ratio", 0.68)
        self.declare_parameter("yellow_pass_band_half_height_px", 10)
        self.declare_parameter("yellow_visible_frames", 2)
        self.declare_parameter("yellow_absent_frames", 1)

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.bridge = CvBridge()
        self.projector = W1BevYellowProjector()
        self.counter = YellowBandPassCounter(
            target=int(self.get_parameter("yellow_pass_target").value),
            visible_frames=int(
                self.get_parameter("yellow_visible_frames").value
            ),
            absent_frames=int(
                self.get_parameter("yellow_absent_frames").value
            ),
        )
        self.processing_enabled = False
        self.rule_command = (0.0, 0.0)
        self.rule_command_time = float("-inf")
        self.steering_started_time = float("-inf")
        self.steering_active = False
        self.completed = False
        self.completion_logged = False
        self.last_status = ""

        self.create_subscription(
            Bool,
            str(self.get_parameter("processing_enabled_topic").value),
            self.on_processing,
            state_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("yellow_mask_topic").value),
            self.on_yellow_mask,
            image_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("rule_command_topic").value),
            self.on_rule_command,
            10,
        )
        self.candidate_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("candidate_topic").value),
            10,
        )
        self.ready_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("ready_topic").value),
            state_qos,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.ready_publisher.publish(Bool(data=False))
        rate = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(
            "yellow_count shortcut ready; candidate-only node, no direct "
            "/xycar_motor publisher"
        )

    def reset_mission(self) -> None:
        self.counter.reset()
        self.steering_started_time = float("-inf")
        self.steering_active = False
        self.completed = False
        self.completion_logged = False
        self.ready_publisher.publish(Bool(data=False))

    def on_processing(self, message: Bool) -> None:
        enabled = bool(message.data)
        if enabled == self.processing_enabled:
            return
        self.processing_enabled = enabled
        self.reset_mission()
        if enabled:
            self.get_logger().warning(
                f"{YELLOW}[MISSION] YELLOW_COUNT PERCEPTION ACTIVE: "
                "waiting for 2 count-band passes\033[0m"
            )
        else:
            self.publish_status("yellow_count inactive; RULE authority")

    def on_rule_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            return
        self.rule_command = (float(message.data[0]), float(message.data[1]))
        self.rule_command_time = time.monotonic()

    def on_yellow_mask(self, message: Image) -> None:
        if not self.processing_enabled or self.steering_active or self.completed:
            return
        try:
            raw_yellow = self.bridge.imgmsg_to_cv2(message, "mono8")
        except Exception as exc:
            self.get_logger().warning(f"yellow mask conversion failed: {exc}")
            return
        bev_yellow = self.projector.project(raw_yellow)
        occupied = yellow_mask_occupies_count_band(
            bev_yellow,
            line_ratio=float(
                self.get_parameter("yellow_pass_line_ratio").value
            ),
            half_height_px=int(
                self.get_parameter(
                    "yellow_pass_band_half_height_px"
                ).value
            ),
            component_minimum_area_px=int(
                self.get_parameter(
                    "yellow_pass_component_minimum_area_px"
                ).value
            ),
        )
        entered, passed, triggered_now = self.counter.update(occupied)
        if entered:
            self.get_logger().info(
                f"[MISSION] yellow dash {self.counter.passed + 1} "
                "entered count band"
            )
        if passed:
            self.get_logger().warning(
                f"{YELLOW}[MISSION] YELLOW LINE DISAPPEARED "
                f"{self.counter.passed}/{self.counter.target}{RESET}"
            )
        if not triggered_now:
            return
        now = time.monotonic()
        if not self.rule_is_fresh(now):
            self.publish_status(
                "yellow_count trigger pending; waiting for fresh RULE candidate"
            )
            return
        self.start_forced_turn(now)

    def start_forced_turn(self, now: float) -> None:
        """Start the timed turn only while the upstream RULE path is healthy."""
        if self.steering_active or self.completed:
            return
        self.steering_active = True
        self.steering_started_time = float(now)
        # Cache the forced candidate before exposing readiness to the hybrid
        # selector.  Both topics remain candidate-only and never own hardware.
        self.publish_forced_candidate()
        self.ready_publisher.publish(Bool(data=True))
        self.get_logger().warning(
            f"[MISSION] YELLOW_COUNT {self.counter.passed}/"
            f"{self.counter.target} -> FORCED LEFT "
            f"angle={self.forced_angle():+.1f} for "
            f"{self.forced_duration():.3f}s"
        )

    def forced_angle(self) -> float:
        maximum = max(
            0.0,
            float(
                self.get_parameter("maximum_abs_steering_command").value
            ),
        )
        requested = float(
            self.get_parameter("forced_steering_command").value
        )
        return float(np.clip(requested, -maximum, 0.0))

    def forced_duration(self) -> float:
        return max(
            0.0, float(self.get_parameter("forced_steering_sec").value)
        )

    def rule_is_fresh(self, now: float) -> bool:
        return bool(
            now - self.rule_command_time
            <= max(
                0.0,
                float(
                    self.get_parameter("rule_command_timeout_sec").value
                ),
            )
        )

    def entry_speed(self) -> float:
        return capped_entry_speed(
            self.rule_command[1],
            float(self.get_parameter("entry_speed_command").value),
        )

    def publish_forced_candidate(self) -> None:
        self.candidate_publisher.publish(
            Float32MultiArray(
                data=[self.forced_angle(), self.entry_speed(), 0.0, 15.0]
            )
        )

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
        if not self.rule_is_fresh(now):
            self.candidate_publisher.publish(
                Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0])
            )
            self.publish_status("yellow_count RULE candidate stale; safe stop")
            return

        # If the second dash was counted during a momentary RULE dropout,
        # begin the 0.7 s (or CLI-selected) interval only after RULE recovers.
        # This prevents the steering timer from expiring without a real turn.
        if (
            self.counter.triggered
            and not self.steering_active
            and not self.completed
        ):
            self.start_forced_turn(now)

        if self.completed:
            self.candidate_publisher.publish(
                Float32MultiArray(
                    data=[
                        float(self.rule_command[0]),
                        float(self.rule_command[1]),
                        1.0,
                        4.0,
                    ]
                )
            )
            return

        if self.steering_active:
            elapsed = max(0.0, now - self.steering_started_time)
            if elapsed >= self.forced_duration():
                self.steering_active = False
                self.completed = True
                self.ready_publisher.publish(Bool(data=False))
                self.candidate_publisher.publish(
                    Float32MultiArray(
                        data=[
                            float(self.rule_command[0]),
                            float(self.rule_command[1]),
                            1.0,
                            4.0,
                        ]
                    )
                )
                if not self.completion_logged:
                    self.get_logger().warning(
                        "[MISSION] YELLOW_COUNT FORCED TURN "
                        f"{elapsed:.3f}s COMPLETE -> REQUEST RULE"
                    )
                    self.completion_logged = True
                return
            self.publish_forced_candidate()
            # Repeat readiness after the candidate. ROS does not guarantee
            # callback order across two different topics, so a one-shot ready
            # pulse could otherwise arrive before the selector cached -42.
            self.ready_publisher.publish(Bool(data=True))
            self.publish_status(
                "yellow_count forced-left candidate active"
            )
            return

        self.candidate_publisher.publish(
            Float32MultiArray(
                data=[
                    float(self.rule_command[0]),
                    self.entry_speed(),
                    0.0,
                    0.0,
                ]
            )
        )
        self.publish_status(
            f"yellow_count waiting {self.counter.passed}/{self.counter.target}; "
            "RULE steering retained"
        )


def main() -> None:
    rclpy.init()
    node = YellowCountShortcutController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.ready_publisher.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
