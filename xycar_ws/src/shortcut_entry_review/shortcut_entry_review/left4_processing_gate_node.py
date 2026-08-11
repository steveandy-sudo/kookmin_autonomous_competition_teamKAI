#!/usr/bin/env python3
"""Motor-free rosbag gate that opens LR-ASPP exactly at the annotated S event."""

from __future__ import annotations

import rclpy
from my_rule_msgs.msg import ObjectDetectionArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32MultiArray, String

from .trigger_detector import Left4TriggerDetector, TriggerConfig


def normalize_class_name(value: str) -> str:
    return str(value).strip().lower().replace("-", "_").replace(" ", "_")


class Left4ProcessingGateNode(Node):
    """Open the review perception gate on the second missing detector frame."""

    def __init__(self) -> None:
        super().__init__("shortcut_left4_processing_gate")
        self.declare_parameter(
            "detections_topic", "/shortcut/review/object_detections"
        )
        self.declare_parameter(
            "processing_enabled_topic", "/hybrid/shortcut_processing_enabled"
        )
        self.declare_parameter("class_name", "left_4")
        self.declare_parameter("minimum_confidence", 0.50)
        self.declare_parameter("required_visible_frames", 2)
        self.declare_parameter("required_absent_frames", 2)
        self.declare_parameter("status_topic", "/shortcut/review/left4_gate_status")
        self.declare_parameter(
            "diagnostics_topic", "/shortcut/review/left4_gate_diagnostics"
        )

        self.target = normalize_class_name(
            str(self.get_parameter("class_name").value)
        )
        self.detector = Left4TriggerDetector(
            TriggerConfig(
                minimum_confidence=float(
                    self.get_parameter("minimum_confidence").value
                ),
                required_visible_frames=int(
                    self.get_parameter("required_visible_frames").value
                ),
                required_absent_frames=int(
                    self.get_parameter("required_absent_frames").value
                ),
            )
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        detection_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.processing_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("processing_enabled_topic").value),
            state_qos,
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.diagnostics_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.create_subscription(
            ObjectDetectionArray,
            str(self.get_parameter("detections_topic").value),
            self.on_detections,
            detection_qos,
        )
        self.processing_publisher.publish(Bool(data=False))
        self.get_logger().info(
            "left_4 review gate armed: 2 visible then 2 missing detector frames"
        )

    def on_detections(self, message: ObjectDetectionArray) -> None:
        confidence = max(
            (
                float(item.confidence)
                for item in message.detections
                if normalize_class_name(item.class_name) == self.target
            ),
            default=0.0,
        )
        was_triggered = self.detector.triggered
        state = self.detector.update(confidence)
        if state.triggered and not was_triggered:
            self.processing_publisher.publish(Bool(data=True))
            self.get_logger().warning(
                "S TRIGGER: second left_4-missing detector frame; LR-ASPP search enabled"
            )
        self.status_publisher.publish(
            String(
                data=(
                    f"left_4={int(state.present)} confidence={state.confidence:.3f} "
                    f"visible={state.visible_streak}/"
                    f"{self.detector.config.required_visible_frames} "
                    f"absent={state.absent_streak}/"
                    f"{self.detector.config.required_absent_frames} "
                    f"S={int(state.triggered)}"
                )
            )
        )
        self.diagnostics_publisher.publish(
            Float32MultiArray(
                data=[
                    float(state.confidence),
                    float(state.visible_streak),
                    1.0 if state.confirmed else 0.0,
                    float(state.absent_streak),
                    1.0 if state.triggered else 0.0,
                ]
            )
        )


def main() -> None:
    rclpy.init()
    node = Left4ProcessingGateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.processing_publisher.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
