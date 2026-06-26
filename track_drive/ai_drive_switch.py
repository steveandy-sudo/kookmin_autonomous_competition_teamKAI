#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# CNN 조향 노드를 외부 제어 토픽으로 켜고 끄는 보조 노드이다.
# 메인 주행 노드가 발행하는 enable, speed_limit, turn_speed 토픽을 받아 AI 주행 속도와 활성 상태를 조절한다.
import cv2
import rclpy
import time
from std_msgs.msg import Bool, Float32

from .cnn_steering_driver import ConeAIDriver


# AI 조향 노드를 감싸서 외부 미션 노드가 주행 활성/비활성을 제어할 수 있게 한다.
class SwitchableConeAIDriver(ConeAIDriver):
    """메인 주행 노드의 토픽 명령에 따라 CNN 조향 노드 출력을 제한하는 클래스이다."""

    def __init__(self):
        # SwitchableConeAIDriver 객체를 초기화하고 필요한 파라미터와 내부 상태를 준비한다.
        super().__init__()

        # 메인 노드는 이 토픽으로 CNN 조향 노드를 켜거나 끄며, 미션 제어와 AI 주행을 전환한다.
        self.declare_parameter('enable_topic', '/cone_ai/enable')
        self.declare_parameter('start_enabled', False)
        # 보호구역, 신호등, 보행자 등 상황별 제한 속도는 별도 토픽으로 들어와 AI 속도 위에 적용된다.
        self.declare_parameter('speed_limit_topic', '/cone_ai/speed_limit')
        self.declare_parameter('turn_speed_override_topic', '/cone_ai/turn_speed')
        self.declare_parameter('speed_limit_override_threshold', 30.0)
        self.declare_parameter('speed_rise_limit_enabled', False)
        self.declare_parameter('speed_rise_per_sec', 10.0)
        # 조향각이 커질수록 속도를 낮춰 라바콘 구간에서 회전반경이 과도하게 커지는 것을 막는다.
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

    # /cone_ai/enable 토픽으로 AI 주행 사용 여부를 갱신한다.
    def enable_callback(self, msg: Bool):
        # ROS 토픽 콜백으로 들어온 메시지를 내부 상태에 반영한다.
        enabled = bool(msg.data)
        if self.enabled != enabled:
            self._log_status_once('enabled' if enabled else 'disabled')
        self.enabled = enabled

    # 미션 상황에서 전달되는 외부 속도 제한값을 저장한다.
    def speed_limit_callback(self, msg: Float32):
        # ROS 토픽 콜백으로 들어온 메시지를 내부 상태에 반영한다.
        self.speed_limit = float(msg.data)
        rounded = round(self.speed_limit, 2)
        if self._last_logged_speed_limit != rounded:
            self._last_logged_speed_limit = rounded
            text = 'off' if self.speed_limit < 0.0 else f'{self.speed_limit:.1f}'
            self.get_logger().info(f'speed_limit={text}')

    # 급커브 구간 등에서 사용할 임시 코너 최저속도 override를 반영한다.
    def turn_speed_override_callback(self, msg: Float32):
        # ROS 토픽 콜백으로 들어온 메시지를 내부 상태에 반영한다.
        self.turn_speed_override = float(msg.data)
        rounded = round(self.turn_speed_override, 2)
        if self._last_logged_turn_speed_override != rounded:
            self._last_logged_turn_speed_override = rounded
            text = 'default' if self.turn_speed_override < 0.0 else f'{self.turn_speed_override:.1f}'
            self.get_logger().info(f'turn_speed_override={text}')

    # 최종 조향각과 속도를 XycarMotor 메시지로 변환해 차량 제어 토픽에 발행한다.
    def drive(self, angle: float, speed: float):
        # AI 모델이 낸 원래 속도에 회전 속도 제한과 외부 속도 제한을 적용한 뒤 발행한다.
        speed = self._apply_turn_speed_limit(angle, speed)
        override_threshold = max(float(self.get_parameter('speed_limit_override_threshold').value), 0.0)
        speed_limit_override_active = False
        # 음수 speed_limit은 제한 없음, 0 이상은 현재 명령 속도에 적용할 외부 제한값으로 해석한다.
        if self.speed_limit >= 0.0 and speed > 0.0:
            if override_threshold > 0.0 and self.speed_limit >= override_threshold:
                speed = self.speed_limit
                speed_limit_override_active = True
            else:
                speed = min(float(speed), self.speed_limit)
        # 고속 override가 아닌 일반 제한에서는 급격한 재가속을 완화할 수 있다.
        if not speed_limit_override_active:
            speed = self._apply_speed_rise_limit(speed)
        self.last_output_speed = float(speed)
        super().drive(angle, speed)

    # 속도가 한 번에 튀지 않도록 상승량을 제한해 출발과 재가속을 부드럽게 만든다.
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

    # 조향각이 커질수록 목표 속도를 낮춰 코너에서 차량이 바깥으로 밀리는 현상을 줄인다.
    def _apply_turn_speed_limit(self, angle: float, speed: float) -> float:
        # 조향각이 커지는 구간에서 속도를 선형으로 낮춰 라바콘 코너 주행 안정성을 높인다.
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

    # 최신 이미지 한 장을 CNN 모델에 넣어 조향을 예측하고 현재 제한 조건을 적용한다.
    def control_once(self):
        # enable 토픽이 꺼져 있으면 주행 명령을 내지 않고, 켜져 있을 때만 CNN 조향 제어를 수행한다.
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
