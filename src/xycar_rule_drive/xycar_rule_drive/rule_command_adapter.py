"""Final adaptive-speed and one-frame camera avoidance command adapter."""

from __future__ import annotations

import time

from cv_bridge import CvBridge
from my_rule_msgs.msg import ObjectDetectionArray
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String

from .rule_command_adapter_core import adaptive_speed_for_steering
from .rule_command_adapter_core import fit_yellow_reference
from .rule_command_adapter_core import ObstacleOffsetLatch
from .rule_command_adapter_core import obstacle_side_from_reference


class RuleCommandAdapter(Node):
    """Apply speed planning and camera-only lateral obstacle correction."""

    def __init__(self) -> None:
        super().__init__("rule_command_adapter")
        defaults = {
            "drive_enabled": False,
            "obstacle_avoidance_enabled": True,
            "base_command_topic": "/rule_drive/base_motor_shadow",
            "motor_topic": "/xycar_motor",
            "shadow_motor_topic": "/xycar_motor_shadow",
            "detections_topic": "/my_rule/object_detections",
            "yellow_mask_topic": "/lane_seg/yellow_centerline_mask",
            "lateral_offset_topic": "/rule_drive/obstacle_lateral_offset",
            "status_topic": "/rule_drive/command_adapter_status",
            "status_text_topic": "/rule_drive/command_adapter_state",
            "vehicle_class_names": ["car", "obstacle_vehicle"],
            "vehicle_min_confidence": 0.45,
            "straight_speed_command": 8.0,
            "turn_speed_command": 8.0,
            "slowdown_start_angle_command": 20.0,
            "full_slowdown_angle_command": 42.0,
            "speed_curve_exponent": 1.0,
            "obstacle_shift_m": 0.20,
            "obstacle_release_delay_sec": 2.0,
            "yellow_reference_timeout_sec": 0.50,
            "yellow_reference_residual_px": 6.0,
            "offset_publish_rate_hz": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.drive_enabled = bool(self.get_parameter("drive_enabled").value)
        self.vehicle_names = {
            self.normalize_class_name(value)
            for value in self.get_parameter("vehicle_class_names").value
        }
        self.bridge = CvBridge()
        self.yellow_coefficients = None
        self.yellow_width = 0
        self.yellow_height = 0
        self.yellow_time = float("-inf")
        self.latch = ObstacleOffsetLatch(
            shift_m=float(self.get_parameter("obstacle_shift_m").value),
            release_delay_sec=float(
                self.get_parameter("obstacle_release_delay_sec").value
            ),
        )
        self.last_basis = "none"
        self.last_output = (0.0, 0.0)

        self.shadow_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("shadow_motor_topic").value),
            10,
        )
        self.motor_pub = None
        if self.drive_enabled:
            self.motor_pub = self.create_publisher(
                Float32MultiArray,
                str(self.get_parameter("motor_topic").value),
                10,
            )
        self.offset_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("lateral_offset_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.status_text_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_text_topic").value),
            10,
        )
        self.command_sub = self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("base_command_topic").value),
            self.on_base_command,
            10,
        )
        self.detection_sub = self.create_subscription(
            ObjectDetectionArray,
            str(self.get_parameter("detections_topic").value),
            self.on_detections,
            qos_profile_sensor_data,
        )
        self.yellow_sub = self.create_subscription(
            Image,
            str(self.get_parameter("yellow_mask_topic").value),
            self.on_yellow_mask,
            qos_profile_sensor_data,
        )
        rate_hz = max(
            1.0, float(self.get_parameter("offset_publish_rate_hz").value)
        )
        self.timer = self.create_timer(1.0 / rate_hz, self.on_timer)
        mode = "AUTO" if self.drive_enabled else "SHADOW"
        self.get_logger().info(
            "rule command adapter ready: "
            f"{mode}, speed={self.turn_speed:.1f}..{self.straight_speed:.1f}, "
            f"full_slowdown={self.full_slowdown_angle:.1f}deg, "
            f"avoidance=one-frame/{self.latch.shift_m:.2f}m/"
            f"{self.latch.release_delay_sec:.1f}s"
        )

    @property
    def straight_speed(self) -> float:
        return float(self.get_parameter("straight_speed_command").value)

    @property
    def turn_speed(self) -> float:
        return float(self.get_parameter("turn_speed_command").value)

    @property
    def full_slowdown_angle(self) -> float:
        return float(
            self.get_parameter("full_slowdown_angle_command").value
        )

    @property
    def slowdown_start_angle(self) -> float:
        return float(
            self.get_parameter("slowdown_start_angle_command").value
        )

    @staticmethod
    def normalize_class_name(value: str) -> str:
        return str(value).strip().lower().replace("-", "_").replace(" ", "_")

    def on_yellow_mask(self, message: Image) -> None:
        try:
            mask = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().warning(f"yellow mask conversion failed: {exc}")
            return
        coefficients = fit_yellow_reference(
            np.asarray(mask),
            residual_px=float(
                self.get_parameter("yellow_reference_residual_px").value
            ),
        )
        if coefficients is None:
            return
        self.yellow_coefficients = coefficients
        self.yellow_height, self.yellow_width = mask.shape[:2]
        self.yellow_time = time.monotonic()

    def select_vehicle(self, message: ObjectDetectionArray):
        threshold = float(
            self.get_parameter("vehicle_min_confidence").value
        )
        candidates = [
            item
            for item in message.detections
            if self.normalize_class_name(item.class_name) in self.vehicle_names
            and float(item.confidence) >= threshold
        ]
        if not candidates:
            return None
        return max(
            candidates,
            key=lambda item: (
                max(0, int(item.xmax) - int(item.xmin))
                * max(0, int(item.ymax) - int(item.ymin)),
                float(item.confidence),
            ),
        )

    def on_detections(self, message: ObjectDetectionArray) -> None:
        if not bool(
            self.get_parameter("obstacle_avoidance_enabled").value
        ):
            return
        vehicle = self.select_vehicle(message)
        if vehicle is None:
            return
        now = time.monotonic()
        yellow_fresh = (
            now - self.yellow_time
            <= float(
                self.get_parameter("yellow_reference_timeout_sec").value
            )
        )
        coefficients = self.yellow_coefficients if yellow_fresh else None
        side, basis = obstacle_side_from_reference(
            object_center_x=0.5 * (float(vehicle.xmin) + float(vehicle.xmax)),
            object_bottom_y=float(vehicle.ymax),
            image_width=int(message.image_width),
            image_height=int(message.image_height),
            yellow_coefficients=coefficients,
            yellow_width=self.yellow_width,
            yellow_height=self.yellow_height,
        )
        started = self.latch.observe(side, now)
        if started:
            self.last_basis = basis
            direction = "RIGHT" if side.value == "left" else "LEFT"
            self.get_logger().warning(
                f"OBSTACLE {side.value.upper()} ({basis}) -> "
                f"shift {direction} {self.latch.shift_m:.2f}m"
            )
            self.publish_state("AVOIDING")
        self.publish_offset()

    def on_timer(self) -> None:
        if self.latch.update(time.monotonic()):
            self.get_logger().info(
                "obstacle absent for "
                f"{self.latch.release_delay_sec:.1f}s -> base path restored"
            )
            self.last_basis = "none"
            self.publish_state("BASE_PATH")
        self.publish_offset()

    def publish_offset(self) -> None:
        self.offset_pub.publish(Float32(data=float(self.latch.offset_m)))

    def publish_state(self, state: str) -> None:
        self.status_text_pub.publish(String(data=state))

    def on_base_command(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            return
        angle = float(message.data[0])
        base_speed = float(message.data[1])
        speed = 0.0
        if base_speed > 0.0:
            speed = adaptive_speed_for_steering(
                angle,
                straight_speed_command=self.straight_speed,
                turn_speed_command=self.turn_speed,
                slowdown_start_angle_command=self.slowdown_start_angle,
                full_slowdown_angle_command=self.full_slowdown_angle,
                exponent=float(
                    self.get_parameter("speed_curve_exponent").value
                ),
            )
        output = Float32MultiArray(data=[angle, float(speed)])
        self.last_output = (angle, float(speed))
        self.shadow_pub.publish(output)
        if self.motor_pub is not None:
            self.motor_pub.publish(output)
        side_code = (
            -1.0
            if self.latch.side is not None and self.latch.side.value == "left"
            else 1.0
            if self.latch.side is not None
            else 0.0
        )
        self.status_pub.publish(
            Float32MultiArray(
                data=[
                    angle,
                    base_speed,
                    float(speed),
                    float(self.latch.offset_m),
                    side_code,
                ]
            )
        )

    def stop_vehicle(self) -> None:
        stop = Float32MultiArray(data=[0.0, 0.0])
        try:
            self.shadow_pub.publish(stop)
            if self.motor_pub is not None:
                self.motor_pub.publish(stop)
        except RuntimeError:
            pass


def main() -> None:
    rclpy.init()
    node = RuleCommandAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop_vehicle()
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
