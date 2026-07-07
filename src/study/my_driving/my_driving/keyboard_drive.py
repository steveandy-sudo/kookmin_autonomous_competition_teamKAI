#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import select
import signal
import sys
import termios
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


KEY_START = 'START'
KEY_LEFT = 'LEFT'
KEY_RIGHT = 'RIGHT'
KEY_PAUSE = 'PAUSE'
KEY_EXIT = 'EXIT'

SEPARATOR = '-' * 73

# Measured calibration table: (real wheel angle in degrees, motor command angle).
# The keyboard target angle is treated as the desired real wheel angle, then
# converted to the motor command angle before publishing to /xycar_motor.
STEERING_ACTUAL_TO_COMMAND = (
    (0.0, 0.0),
    (4.0, 10.0),
    (10.0, 20.0),
    (16.0, 30.0),
    (26.0, 42.0),
)


class TerminalKeyboard:
    def __init__(self):
        self._tty = None
        self._old_settings = None
        self._should_close = False

    def __enter__(self):
        if sys.stdin.isatty():
            self._tty = sys.stdin
        else:
            self._tty = open('/dev/tty', 'r', buffering=1)
            self._should_close = True

        fd = self._tty.fileno()
        self._old_settings = termios.tcgetattr(fd)
        new_settings = termios.tcgetattr(fd)

        # Non-canonical input. Disable echo and terminal flow control so keys
        # such as Ctrl+S cannot freeze the terminal during a driving test.
        new_settings[0] &= ~(termios.IXON | termios.IXOFF | termios.ICRNL)
        new_settings[3] &= ~(termios.ICANON | termios.ECHO)
        new_settings[6][termios.VMIN] = 0
        new_settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, new_settings)
        return self

    def __exit__(self, exc_type, exc, tb):
        self.restore()

    def restore(self):
        if self._old_settings is not None and self._tty is not None:
            termios.tcsetattr(self._tty.fileno(), termios.TCSADRAIN, self._old_settings)
            self._old_settings = None

        if self._should_close and self._tty is not None:
            self._tty.close()
            self._tty = None
            self._should_close = False

    def read_keys(self):
        keys = []
        while self._has_char(0.0):
            char = self._tty.read(1)
            key = self._decode_char(char)
            if key is not None:
                keys.append(key)
        return keys

    def _decode_char(self, char):
        if char in ('w', 'W'):
            return KEY_START
        if char in ('a', 'A'):
            return KEY_LEFT
        if char in ('d', 'D'):
            return KEY_RIGHT
        if char == ' ':
            return KEY_PAUSE
        if char == '\x1b':
            if self._has_char(0.02):
                self._flush_escape_sequence()
                return None
            return KEY_EXIT

        # Ignore every unassigned key.
        return None

    def _flush_escape_sequence(self):
        deadline = time.monotonic() + 0.03
        while time.monotonic() < deadline and self._has_char(0.0):
            self._tty.read(1)

    def _has_char(self, timeout_sec):
        readable, _, _ = select.select([self._tty], [], [], timeout_sec)
        return bool(readable)


class KeyboardDriveNode(Node):
    def __init__(self):
        super().__init__('keyboard_driver')

        self.motor_topic = self.declare_parameter('motor_topic', '/xycar_motor').value
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 50.0).value)
        self.auto_forward_speed = abs(
            float(self.declare_parameter('auto_forward_speed', 3.0).value)
        )
        self.start_delay_sec = max(
            0.0,
            float(self.declare_parameter('start_delay_sec', 3.0).value),
        )
        self.max_turn_angle = abs(float(self.declare_parameter('max_turn_angle', 30.0).value))
        self.steering_step_angle = abs(
            float(self.declare_parameter('steering_step_angle', 1.0).value)
        )
        self.accel_speed_per_sec = abs(
            float(self.declare_parameter('accel_speed_per_sec', 4.0).value)
        )
        self.decel_speed_per_sec = abs(
            float(self.declare_parameter('decel_speed_per_sec', 16.0).value)
        )
        self.steering_rate_deg_per_sec = abs(
            float(self.declare_parameter('steering_rate_deg_per_sec', 150.0).value)
        )
        self.steering_return_rate_deg_per_sec = abs(
            float(self.declare_parameter('steering_return_rate_deg_per_sec', 200.0).value)
        )
        self.steering_relax_delay_sec = float(
            self.declare_parameter('steering_relax_delay_sec', 0.7).value
        )
        self.steering_relax_rate_deg_per_sec = abs(
            float(self.declare_parameter('steering_relax_rate_deg_per_sec', 4.0).value)
        )
        self.invert_steering = bool(self.declare_parameter('invert_steering', False).value)
        self.stop_burst_count = int(self.declare_parameter('stop_burst_count', 10).value)
        self.stop_burst_dt_sec = float(self.declare_parameter('stop_burst_dt_sec', 0.05).value)

        self.motor_publisher = self.create_publisher(Float32MultiArray, self.motor_topic, 1)

        self.target_angle = 0.0
        self.target_speed = 0.0
        self.current_angle = 0.0
        self.current_speed = 0.0
        self.stop_requested = False

        self.is_driving = False
        self.is_counting_down = False
        self.countdown_deadline = None
        self.last_countdown_sec = None

        now = time.monotonic()
        self.last_steer_time = now
        self.last_safety_time = now
        self.last_update_time = now

    def print_help(self):
        print(SEPARATOR)
        print(f'{"자이카 자동 전진 키보드 주행":^65}')
        print(SEPARATOR)
        print(f'실행 직후      : {self.start_delay_sec:.0f}초 카운트다운 후 자동 전진')
        print('A             : 좌회전')
        print('D             : 우회전')
        print('SPACE         : 일시정지')
        print(f'W             : 재출발 ({self.start_delay_sec:.0f}초 카운트다운)')
        print('ESC, Ctrl+C   : 정지 + 프로그램 종료')
        print(SEPARATOR)
        print(f'-- 토픽 : {self.motor_topic}')
        print(f'-- 주기 : {self.publish_rate_hz:.1f}Hz')
        print(f'-- 전진 속도 : {self.auto_forward_speed:.1f}')
        print(f'-- 좌우 목표 조향 최대치 : {self.max_turn_angle:.1f}')
        print(f'-- 조향 1회 입력 단위 : {self.steering_step_angle:.1f}')
        print('-- 조향 보정 : 실측 테이블 적용 (목표각 -> 모터명령)')
        print(f'-- {self.steering_relax_delay_sec:.1f}초 동안 입력 없으면, 0도로 천천히 복귀')
        print(SEPARATOR)

    def start_countdown(self, source):
        now = time.monotonic()
        self.is_driving = False
        self.is_counting_down = True
        self.countdown_deadline = now + self.start_delay_sec
        self.last_countdown_sec = None
        self.target_speed = 0.0
        self.print_command(source)
        print(flush=True)

    def handle_key(self, key):
        now = time.monotonic()

        if key == KEY_START:
            if not self.is_driving:
                self.start_countdown('W')
            else:
                self.print_command('W')
        elif key == KEY_LEFT:
            self.target_angle = self._clamp_angle(self.target_angle + self._left_step())
            self.last_steer_time = now
            self.print_command('A')
        elif key == KEY_RIGHT:
            self.target_angle = self._clamp_angle(self.target_angle + self._right_step())
            self.last_steer_time = now
            self.print_command('D')
        elif key == KEY_PAUSE:
            self.pause_motion()
            self.print_command('SPACE')
        elif key == KEY_EXIT:
            self.print_command('ESC')
            self.stop_requested = True

    def update_control_state(self):
        now = time.monotonic()
        dt = max(0.0, now - self.last_safety_time)
        self.last_safety_time = now

        if self.is_counting_down:
            self.target_speed = 0.0
            remaining = self.countdown_deadline - now if self.countdown_deadline is not None else 0.0
            if remaining <= 0.0:
                self.is_counting_down = False
                self.is_driving = True
                self.target_speed = self.auto_forward_speed
                print(flush=True)
                self.print_command('자동출발')
            else:
                self._print_countdown(remaining)
        elif self.is_driving:
            self.target_speed = self.auto_forward_speed
        else:
            self.target_speed = 0.0

        if self._should_relax_steering(now):
            self.target_angle = self._move_toward(
                self.target_angle,
                0.0,
                self.steering_relax_rate_deg_per_sec * dt,
            )

    def pause_motion(self):
        self.is_driving = False
        self.is_counting_down = False
        self.countdown_deadline = None
        self.last_countdown_sec = None
        self.target_angle = 0.0
        self.target_speed = 0.0
        self.current_angle = 0.0
        self.current_speed = 0.0
        self.last_steer_time = time.monotonic()
        self.publish_motor(0.0, 0.0)

    def emergency_stop(self):
        self.is_driving = False
        self.is_counting_down = False
        self.target_angle = 0.0
        self.target_speed = 0.0
        self.current_angle = 0.0
        self.current_speed = 0.0
        self.get_logger().warn('긴급 정지: 종료 전 [0.0, 0.0] 명령을 반복 publish합니다.')

        for _ in range(max(1, self.stop_burst_count)):
            self.publish_motor(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(max(0.0, self.stop_burst_dt_sec))

    def publish_current_command(self):
        self._update_smooth_outputs()
        self.publish_motor(self.current_angle, self.current_speed)

    def _update_smooth_outputs(self):
        now = time.monotonic()
        dt = max(0.0, now - self.last_update_time)
        self.last_update_time = now

        speed_rate = self._active_speed_rate(self.current_speed, self.target_speed)
        angle_rate = self._active_angle_rate(self.current_angle, self.target_angle)
        self.current_speed = self._move_toward(self.current_speed, self.target_speed, speed_rate * dt)
        self.current_angle = self._move_toward(self.current_angle, self.target_angle, angle_rate * dt)

    def _active_speed_rate(self, current, target):
        if target == 0.0:
            return self.decel_speed_per_sec
        if current == 0.0 or (current > 0.0) == (target > 0.0):
            if abs(target) > abs(current):
                return self.accel_speed_per_sec
        return self.decel_speed_per_sec

    def _active_angle_rate(self, current, target):
        if target == 0.0:
            return self.steering_return_rate_deg_per_sec
        if abs(target) > abs(current):
            return self.steering_rate_deg_per_sec
        return self.steering_return_rate_deg_per_sec

    def publish_motor(self, angle, speed):
        msg = Float32MultiArray()
        msg.data = [float(self._steering_command_from_actual_angle(angle)), float(speed)]
        self.motor_publisher.publish(msg)

    def print_command(self, source):
        if source == '자동시작':
            text = (
                f'입력=자동시작 상태={self._state_text()} 토픽={self.motor_topic} '
                f'목표조향각={self.target_angle:>6.1f} 목표속도={self.target_speed:>4.1f}'
            )
            print(text, flush=True)
            return

        text = (
            f'입력={self._source_text(source)} / 상태={self._state_text()} / '
            f'목표조향각={self.target_angle:>6.1f} 목표속도={self.target_speed:>4.1f}'
        )
        print(text, flush=True)

    def _print_countdown(self, remaining):
        seconds = max(1, int(math.ceil(remaining)))
        if seconds != self.last_countdown_sec:
            print(f'{seconds}초 후 자동 전진 시작...', flush=True)
            self.last_countdown_sec = seconds

    def _state_text(self):
        if self.is_counting_down:
            return '카운트다운'
        if self.is_driving:
            return '주행중'
        return '일시정지'

    def _source_text(self, source):
        if source == 'A':
            return 'A(좌회전)'
        if source == 'D':
            return 'D(우회전)'
        if source == 'SPACE':
            return 'SPACE(일시정지)'
        if source == 'W':
            return 'W(재출발)' if not self.is_driving else 'W(주행중)'
        if source == 'ESC':
            return 'ESC(종료)'
        return source

    def _should_relax_steering(self, now):
        if self.steering_relax_delay_sec <= 0.0 or self.steering_relax_rate_deg_per_sec <= 0.0:
            return False
        if self.target_angle == 0.0:
            return False
        return now - self.last_steer_time > self.steering_relax_delay_sec

    def _left_step(self):
        return self.steering_step_angle if self.invert_steering else -self.steering_step_angle

    def _right_step(self):
        return -self.steering_step_angle if self.invert_steering else self.steering_step_angle

    def _clamp_angle(self, angle):
        return max(-self.max_turn_angle, min(self.max_turn_angle, angle))

    def _steering_command_from_actual_angle(self, angle):
        sign = -1.0 if angle < 0.0 else 1.0
        actual_angle = abs(angle)

        if actual_angle <= STEERING_ACTUAL_TO_COMMAND[0][0]:
            return 0.0

        for index in range(1, len(STEERING_ACTUAL_TO_COMMAND)):
            low_actual, low_command = STEERING_ACTUAL_TO_COMMAND[index - 1]
            high_actual, high_command = STEERING_ACTUAL_TO_COMMAND[index]
            if actual_angle <= high_actual:
                ratio = (actual_angle - low_actual) / (high_actual - low_actual)
                command = low_command + ratio * (high_command - low_command)
                return sign * command

        low_actual, low_command = STEERING_ACTUAL_TO_COMMAND[-2]
        high_actual, high_command = STEERING_ACTUAL_TO_COMMAND[-1]
        ratio = (actual_angle - low_actual) / (high_actual - low_actual)
        command = low_command + ratio * (high_command - low_command)
        return sign * command

    def _move_toward(self, value, target, max_delta):
        if max_delta <= 0.0:
            return value
        delta = target - value
        if abs(delta) <= max_delta:
            return target
        return value + max_delta if delta > 0.0 else value - max_delta


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardDriveNode()
    shutdown_requested = False

    def request_shutdown(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True
        node.stop_requested = True

    previous_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, request_shutdown)

    node.print_help()
    node.start_countdown('자동시작')
    period_sec = 1.0 / max(1.0, node.publish_rate_hz)

    try:
        with TerminalKeyboard() as keyboard:
            while rclpy.ok() and not node.stop_requested and not shutdown_requested:
                for key in keyboard.read_keys():
                    node.handle_key(key)

                node.update_control_state()
                node.publish_current_command()
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(period_sec)
    except Exception as exc:
        node.get_logger().error(f'키보드 주행 노드가 오류로 중단되었습니다: {exc!r}')
    finally:
        node.emergency_stop()
        signal.signal(signal.SIGINT, previous_sigint)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
