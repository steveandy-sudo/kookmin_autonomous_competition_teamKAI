"""Final motor gate for the 4-lamp signal and one shortcut turn."""

from __future__ import annotations

import time

import rclpy
from my_rule_msgs.msg import ObjectDetectionArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32MultiArray, String

from my_drive.traffic_shortcut import SignalGateConfig
from my_drive.traffic_shortcut import SignalGateController


class TrafficShortcutGate(Node):
    def __init__(self) -> None:
        super().__init__("traffic_shortcut_gate")
        defaults = {
            "drive_enabled": False,
            "candidate_topic": "/hybrid_gate/xycar_motor_shadow",
            "detections_topic": "/my_rule/object_detections",
            "startup_green_topic": "/my_rule/start_signal_green",
            "motor_topic": "/xycar_motor",
            "status_topic": "/my_drive/mission_status",
            "publish_rate_hz": 20.0,
            "candidate_timeout_sec": 0.40,
            "signal_required_frames": 2,
            "signal_min_confidence": 0.50,
            "start_clear_sec": 1.0,
            "shortcut_green_sync_sec": 0.75,
            "shortcut_duration_sec": 1.8,
            "shortcut_left_command": -18.0,
            "shortcut_lane_command_weight": 0.35,
            "shortcut_speed_command": 8.0,
            "maximum_abs_angle_command": 42.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.controller = SignalGateController(
            SignalGateConfig(
                candidate_timeout_sec=self._float("candidate_timeout_sec"),
                signal_required_frames=int(
                    self.get_parameter("signal_required_frames").value
                ),
                start_clear_sec=self._float("start_clear_sec"),
                shortcut_green_sync_sec=self._float(
                    "shortcut_green_sync_sec"
                ),
                shortcut_duration_sec=self._float("shortcut_duration_sec"),
                shortcut_left_command=self._float("shortcut_left_command"),
                shortcut_lane_command_weight=self._float(
                    "shortcut_lane_command_weight"
                ),
                shortcut_speed_command=self._float("shortcut_speed_command"),
                maximum_abs_angle_command=self._float(
                    "maximum_abs_angle_command"
                ),
            )
        )
        self.candidate = (0.0, 0.0)
        self.candidate_time = float("-inf")
        self.last_reason = ""
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("motor_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("candidate_topic").value),
            self._on_candidate,
            10,
        )
        self.create_subscription(
            ObjectDetectionArray,
            str(self.get_parameter("detections_topic").value),
            self._on_detections,
            10,
        )
        startup_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("startup_green_topic").value),
            self._on_start_green,
            startup_qos,
        )
        rate = max(1.0, self._float("publish_rate_hz"))
        self.timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            "integrated signal gate ready: initial green required, "
            "red re-encounter stop enabled, left_4 shortcut enabled once"
        )

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    @staticmethod
    def _name(value: str) -> str:
        return str(value).strip().lower().replace("-", "_").replace(" ", "_")

    def _on_candidate(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            return
        self.candidate = (float(message.data[0]), float(message.data[1]))
        self.candidate_time = time.monotonic()

    def _on_start_green(self, message: Bool) -> None:
        if message.data:
            self.controller.observe_start_green(time.monotonic())

    def _on_detections(self, message: ObjectDetectionArray) -> None:
        threshold = self._float("signal_min_confidence")
        names = {
            self._name(item.class_name)
            for item in message.detections
            if float(item.confidence) >= threshold
        }
        self.controller.observe_detections(
            now_sec=time.monotonic(),
            red=bool(names & {"red", "red_4"}),
            green=bool(names & {"green", "green_4"}),
            yellow=bool(names & {"yellow", "yellow_4"}),
            left=bool(names & {"left", "left_4", "left_turn"}),
        )

    def _publish_stop(self) -> None:
        self.motor_pub.publish(Float32MultiArray(data=[0.0, 0.0]))

    def _on_timer(self) -> None:
        now = time.monotonic()
        if not bool(self.get_parameter("drive_enabled").value):
            angle, speed, reason = 0.0, 0.0, "DRIVE_DISABLED"
        else:
            angle, speed, reason = self.controller.command(
                now_sec=now,
                candidate_age_sec=now - self.candidate_time,
                candidate_angle=self.candidate[0],
                candidate_speed=self.candidate[1],
            )
        self.motor_pub.publish(Float32MultiArray(data=[angle, speed]))
        if reason != self.last_reason:
            text = (
                f"state={self.controller.state.value} reason={reason} "
                f"cmd=[{angle:.1f},{speed:.1f}] "
                f"shortcut_taken={int(self.controller.shortcut_taken)}"
            )
            self.status_pub.publish(String(data=text))
            self.get_logger().info(text)
            self.last_reason = reason

    def stop(self) -> None:
        self.timer.cancel()
        self._publish_stop()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TrafficShortcutGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
