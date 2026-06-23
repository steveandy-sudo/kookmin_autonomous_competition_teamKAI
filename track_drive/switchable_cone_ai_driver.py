#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
import time
from std_msgs.msg import Bool, Float32

from cone_il.cone_ai_driver_node import ConeAIDriver


class SwitchableConeAIDriver(ConeAIDriver):
    def __init__(self):
        # SwitchableConeAIDriver 객체를 초기화하고 필요한 파라미터와 내부 상태를 준비한다.
        super().__init__()

        self.declare_parameter('enable_topic', '/cone_ai/enable')
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('speed_limit_topic', '/cone_ai/speed_limit')
        self.declare_parameter('turn_speed_override_topic', '/cone_ai/turn_speed')
        self.declare_parameter('speed_limit_override_threshold', 30.0)
        self.declare_parameter('speed_rise_limit_enabled', False)
        self.declare_parameter('speed_rise_per_sec', 10.0)
        self.declare_parameter('turn_speed_limit_enabled', True)
        self.declare_parameter('turn_speed', 10.0)
        self.declare_parameter('turn_speed_start_steer_deg', 6.0)
        self.declare_parameter('turn_speed_full_steer_deg', 35.0)
        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.speed_limit = -1.0
        self.turn_speed_override = -1.0
        self._last_logged_speed_limit = None
        self._last_logged_turn_speed_override = None
        self.last_output_speed = 0.0
        self.last_speed_ramp_sec = None

        enable_topic = str(self.get_parameter('enable_topic').value)
        speed_limit_topic = str(self.get_parameter('speed_limit_topic').value)
        turn_speed_override_topic = str(self.get_parameter('turn_speed_override_topic').value)
        self.create_subscription(Bool, enable_topic, self.enable_callback, 10)
        self.create_subscription(Float32, speed_limit_topic, self.speed_limit_callback, 10)
        self.create_subscription(Float32, turn_speed_override_topic, self.turn_speed_override_callback, 10)
        self.get_logger().info(
            f'AI direct enable topic={enable_topic}, speed_limit_topic={speed_limit_topic}, '
            f'turn_speed_override_topic={turn_speed_override_topic}, start_enabled={self.enabled}'
        )

    def enable_callback(self, msg: Bool):
        # ROS 토픽 콜백으로 들어온 메시지를 내부 상태에 반영한다.
        enabled = bool(msg.data)
        if self.enabled != enabled:
            self._log_status_once('enabled' if enabled else 'disabled')
        self.enabled = enabled

    def speed_limit_callback(self, msg: Float32):
        # ROS 토픽 콜백으로 들어온 메시지를 내부 상태에 반영한다.
        self.speed_limit = float(msg.data)
        rounded = round(self.speed_limit, 2)
        if self._last_logged_speed_limit != rounded:
            self._last_logged_speed_limit = rounded
            text = 'off' if self.speed_limit < 0.0 else f'{self.speed_limit:.1f}'
            self.get_logger().info(f'speed_limit={text}')

    def turn_speed_override_callback(self, msg: Float32):
        # ROS 토픽 콜백으로 들어온 메시지를 내부 상태에 반영한다.
        self.turn_speed_override = float(msg.data)
        rounded = round(self.turn_speed_override, 2)
        if self._last_logged_turn_speed_override != rounded:
            self._last_logged_turn_speed_override = rounded
            text = 'default' if self.turn_speed_override < 0.0 else f'{self.turn_speed_override:.1f}'
            self.get_logger().info(f'turn_speed_override={text}')

    def drive(self, angle: float, speed: float):
        # 전환 가능한 콘 AI 주행의 주행 로직을 수행한다.
        speed = self._apply_turn_speed_limit(angle, speed)
        override_threshold = max(float(self.get_parameter('speed_limit_override_threshold').value), 0.0)
        speed_limit_override_active = False
        if self.speed_limit >= 0.0 and speed > 0.0:
            if override_threshold > 0.0 and self.speed_limit >= override_threshold:
                speed = self.speed_limit
                speed_limit_override_active = True
            else:
                speed = min(float(speed), self.speed_limit)
        if not speed_limit_override_active:
            speed = self._apply_speed_rise_limit(speed)
        self.last_output_speed = float(speed)
        super().drive(angle, speed)

    def _apply_speed_rise_limit(self, speed: float) -> float:
        # apply 속도 rise limit 조건을 현재 명령이나 상태에 적용한다.
        if not bool(self.get_parameter('speed_rise_limit_enabled').value):
            self.last_speed_ramp_sec = time.monotonic()
            return speed

        now = time.monotonic()
        if self.last_speed_ramp_sec is None:
            self.last_speed_ramp_sec = now
            return speed

        if speed <= self.last_output_speed or speed <= 0.0:
            self.last_speed_ramp_sec = now
            return speed

        rise_per_sec = max(float(self.get_parameter('speed_rise_per_sec').value), 0.0)
        if rise_per_sec <= 0.0:
            self.last_speed_ramp_sec = now
            return speed

        dt = max(now - self.last_speed_ramp_sec, 0.0)
        self.last_speed_ramp_sec = now
        return min(float(speed), float(self.last_output_speed) + rise_per_sec * dt)

    def _log_status_once(self, text: str):
        # 로그 status once 정보를 사람이 읽기 쉬운 로그 문자열로 만든다.
        if text.startswith('cmd angle=') and ', speed=' in text:
            prefix = text.rsplit(', speed=', 1)[0]
            text = f'{prefix}, speed={self.last_output_speed:.1f}'
        super()._log_status_once(text)

    def _apply_turn_speed_limit(self, angle: float, speed: float) -> float:
        # apply 회전 속도 limit 조건을 현재 명령이나 상태에 적용한다.
        if not bool(self.get_parameter('turn_speed_limit_enabled').value):
            return speed
        if speed <= 0.0:
            return speed

        start_steer = max(float(self.get_parameter('turn_speed_start_steer_deg').value), 0.0)
        full_steer = max(float(self.get_parameter('turn_speed_full_steer_deg').value), start_steer + 1.0)
        turn_speed = self._current_turn_speed()
        steer_abs = abs(float(angle))
        if steer_abs < start_steer:
            return speed

        ratio = min(max((steer_abs - start_steer) / (full_steer - start_steer), 0.0), 1.0)
        target_speed = float(speed) + (turn_speed - float(speed)) * ratio
        return min(float(speed), target_speed)

    def _current_turn_speed(self) -> float:
        # 전환 가능한 콘 AI 주행의 current 회전 속도 로직을 수행한다.
        if self.turn_speed_override >= 0.0:
            return max(float(self.turn_speed_override), 0.0)
        return max(float(self.get_parameter('turn_speed').value), 0.0)

    def control_once(self):
        # 전환 가능한 콘 AI 주행의 제어 once 로직을 수행한다.
        if not self.enabled:
            self.last_output_speed = float(self.get_parameter('speed').value)
            self.last_speed_ramp_sec = None
            self._log_status_once('disabled')
            return
        super().control_once()


def main(args=None):
    # ROS2 노드를 초기화하고 실행 루프를 시작한다.
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
