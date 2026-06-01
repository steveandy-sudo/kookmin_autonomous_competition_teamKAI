#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
from std_msgs.msg import Bool, Float32

from cone_il.cone_ai_driver_node import ConeAIDriver


class SwitchableConeAIDriver(ConeAIDriver):
    def __init__(self):
        super().__init__()

        self.declare_parameter('enable_topic', '/cone_ai/enable')
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('speed_limit_topic', '/cone_ai/speed_limit')
        self.declare_parameter('turn_speed_limit_enabled', True)
        self.declare_parameter('turn_speed', 11.0)
        self.declare_parameter('turn_speed_start_steer_deg', 6.0)
        self.declare_parameter('turn_speed_full_steer_deg', 35.0)
        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.speed_limit = -1.0
        self._last_logged_speed_limit = None

        enable_topic = str(self.get_parameter('enable_topic').value)
        speed_limit_topic = str(self.get_parameter('speed_limit_topic').value)
        self.create_subscription(Bool, enable_topic, self.enable_callback, 10)
        self.create_subscription(Float32, speed_limit_topic, self.speed_limit_callback, 10)
        self.get_logger().info(
            f'AI direct enable topic={enable_topic}, speed_limit_topic={speed_limit_topic}, '
            f'start_enabled={self.enabled}'
        )

    def enable_callback(self, msg: Bool):
        enabled = bool(msg.data)
        if self.enabled != enabled:
            self._log_status_once('enabled' if enabled else 'disabled')
        self.enabled = enabled

    def speed_limit_callback(self, msg: Float32):
        self.speed_limit = float(msg.data)
        rounded = round(self.speed_limit, 2)
        if self._last_logged_speed_limit != rounded:
            self._last_logged_speed_limit = rounded
            text = 'off' if self.speed_limit < 0.0 else f'{self.speed_limit:.1f}'
            self.get_logger().info(f'speed_limit={text}')

    def drive(self, angle: float, speed: float):
        speed = self._apply_turn_speed_limit(angle, speed)
        if self.speed_limit >= 0.0 and speed > 0.0:
            speed = min(float(speed), self.speed_limit)
        super().drive(angle, speed)

    def _apply_turn_speed_limit(self, angle: float, speed: float) -> float:
        if not bool(self.get_parameter('turn_speed_limit_enabled').value):
            return speed
        if speed <= 0.0:
            return speed

        start_steer = max(float(self.get_parameter('turn_speed_start_steer_deg').value), 0.0)
        full_steer = max(float(self.get_parameter('turn_speed_full_steer_deg').value), start_steer + 1.0)
        turn_speed = max(float(self.get_parameter('turn_speed').value), 0.0)
        steer_abs = abs(float(angle))
        if steer_abs < start_steer:
            return speed

        ratio = min(max((steer_abs - start_steer) / (full_steer - start_steer), 0.0), 1.0)
        target_speed = float(speed) + (turn_speed - float(speed)) * ratio
        return min(float(speed), target_speed)

    def control_once(self):
        if not self.enabled:
            self._log_status_once('disabled')
            return
        super().control_once()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SwitchableConeAIDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.drive(0.0, 0.0)
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
