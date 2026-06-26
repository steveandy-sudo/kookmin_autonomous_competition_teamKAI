#!/usr/bin/env python3
import os
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor


class CvKeyboardTeleop(Node):
    # 설명: OpenCV 창 기반 키보드 조작 노드의 상태와 표시 화면을 초기화한다.
    def __init__(self):
        super().__init__('xycar_cv_keyboard_teleop')

        self.declare_parameter('motor_topic', 'xycar_motor')
        self.declare_parameter('publish_rate_hz', 40.0)
        self.declare_parameter('speed_step', 5.0)
        self.declare_parameter('steer_step', 3.0)
        self.declare_parameter('max_speed', 10.0)
        self.declare_parameter('max_steer_deg', 100.0)
        self.declare_parameter('drive_mode', 'arcade')
        self.declare_parameter('forward_speed', 10.0)
        self.declare_parameter('reverse_speed', 0.0)
        self.declare_parameter('turn_steer_deg', 100.0)
        self.declare_parameter('key_hold_timeout_sec', 0.35)
        self.declare_parameter('speed_hold_timeout_sec', 0.35)
        self.declare_parameter('steer_hold_timeout_sec', 0.0)

        self.motor_topic = str(self.get_parameter('motor_topic').value)
        self.pub = self.create_publisher(XycarMotor, self.motor_topic, 10)
        self.speed = 0.0
        self.steer = 0.0
        self.running = True
        self.last_subscription_count = -1
        self.last_speed_key_time = 0.0
        self.last_steer_key_time = 0.0
        self.last_logged_cmd = None
        self.window_positioned = False
        self.window_position_warning_logged = False
        self.window_name = 'xycar WASD teleop - focus this window'

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 360)
        self.get_logger().info(f'CV keyboard teleop publishing to {self.motor_topic}')

    # 설명: OpenCV 조작 창의 화면 위치를 설정한다.
    def _position_window(self):
        try:
            cv2.resizeWindow(self.window_name, 640, 360)
            cv2.moveWindow(self.window_name, 80, 80)
        except Exception as exc:
            if not self.window_position_warning_logged:
                self.window_position_warning_logged = True
                self.get_logger().warn(f'failed to position teleop window: {exc}')

    # 설명: 입력 키를 주행 조작 명령으로 변환한다.
    def handle_key(self, key_code: int):
        if key_code < 0:
            return

        key = chr(key_code & 0xFF).lower()
        mode = str(self.get_parameter('drive_mode').value).lower()
        speed_step = float(self.get_parameter('speed_step').value)
        steer_step = float(self.get_parameter('steer_step').value)
        max_speed = float(self.get_parameter('max_speed').value)
        max_steer = float(self.get_parameter('max_steer_deg').value)
        forward_speed = float(self.get_parameter('forward_speed').value)
        reverse_speed = float(self.get_parameter('reverse_speed').value)
        turn_steer = float(self.get_parameter('turn_steer_deg').value)
        now = time.monotonic()

        if mode in ('arcade', 'hold') and key == 'w':
            self.speed = min(max(forward_speed, -max_speed), max_speed)
            self.last_speed_key_time = now
        elif mode in ('arcade', 'hold') and key == 's':
            self.speed = min(max(reverse_speed, -max_speed), max_speed)
            self.last_speed_key_time = now
        elif mode in ('arcade', 'hold') and key == 'a':
            self.steer = -min(abs(turn_steer), max_steer)
            self.last_steer_key_time = now
        elif mode in ('arcade', 'hold') and key == 'd':
            self.steer = min(abs(turn_steer), max_steer)
            self.last_steer_key_time = now
        elif key == 'w':
            self.speed = min(self.speed + speed_step, max_speed)
        elif key == 's':
            self.speed = max(self.speed - speed_step, -max_speed)
        elif key == 'a':
            self.steer = max(self.steer - steer_step, -max_steer)
        elif key == 'd':
            self.steer = min(self.steer + steer_step, max_steer)
        elif key == 'x':
            self.speed = 0.0
        elif key == 'c':
            self.steer = 0.0
            self.last_steer_key_time = 0.0
        elif key == ' ':
            self.speed = 0.0
            self.steer = 0.0
            self.last_speed_key_time = 0.0
            self.last_steer_key_time = 0.0
        elif key == 'q' or key_code == 27:
            self.running = False
            return
        else:
            return

        self.log_cmd_if_changed()

    # 설명: 키 입력이 끊긴 경우 조작 유지 시간을 기준으로 명령을 완화한다.
    def apply_hold_timeout(self):
        mode = str(self.get_parameter('drive_mode').value).lower()
        if mode not in ('arcade', 'hold'):
            return

        fallback_timeout = max(float(self.get_parameter('key_hold_timeout_sec').value), 0.05)
        speed_timeout = max(float(self.get_parameter('speed_hold_timeout_sec').value), 0.05)
        steer_timeout = float(self.get_parameter('steer_hold_timeout_sec').value)
        if speed_timeout <= 0.0:
            speed_timeout = fallback_timeout
        if steer_timeout <= 0.0:
            steer_timeout = fallback_timeout
        now = time.monotonic()
        changed = False
        if mode == 'hold' and self.last_speed_key_time > 0.0 and now - self.last_speed_key_time > speed_timeout:
            if abs(self.speed) > 1e-3:
                self.speed = 0.0
                changed = True
            self.last_speed_key_time = 0.0
        if steer_timeout > 0.0 and self.last_steer_key_time > 0.0 and now - self.last_steer_key_time > steer_timeout:
            if abs(self.steer) > 1e-3:
                self.steer = 0.0
                changed = True
            self.last_steer_key_time = 0.0
        if changed:
            self.log_cmd_if_changed()

    # 설명: 주행 명령이 바뀌었을 때만 로그를 남긴다.
    def log_cmd_if_changed(self):
        cmd = (round(float(self.steer), 1), round(float(self.speed), 1))
        if cmd == self.last_logged_cmd:
            return
        self.last_logged_cmd = cmd
        self.get_logger().info(f'cmd angle={self.steer:.1f}, speed={self.speed:.1f}')

    # 설명: 현재 키보드 조작 상태를 모터 명령으로 발행한다.
    def publish_cmd(self):
        if not rclpy.ok():
            return

        msg = XycarMotor()
        msg.angle = float(self.steer)
        msg.speed = float(self.speed)
        try:
            self.pub.publish(msg)
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().warn(f'motor publish failed: {exc}')
            return

        try:
            count = self.pub.get_subscription_count()
        except Exception:
            return
        if count != self.last_subscription_count:
            self.last_subscription_count = count
            self.get_logger().info(f'{self.motor_topic} subscribers={count}')

    # 설명: 키보드 조작 상태를 보여주는 OpenCV 안내 화면을 그린다.
    def render(self):
        img = np.full((360, 640, 3), 245, dtype=np.uint8)
        lines = [
            'Focus this window, then use WASD',
            '',
            f'angle: {self.steer:6.1f}',
            f'speed: {self.speed:6.1f}',
            '',
            'W: speed 10, S/X: speed 0',
            'A/D: steer +/-100',
            'C: steering center',
            'X: speed 0',
            'SPACE: stop',
            'Q or ESC: quit',
        ]

        y = 45
        for i, text in enumerate(lines):
            scale = 0.85 if i != 0 else 0.9
            thickness = 2 if i in (0, 2, 3) else 1
            color = (25, 25, 25) if i != 0 else (0, 70, 170)
            cv2.putText(
                img, text, (34, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thickness, cv2.LINE_AA,
            )
            y += 34

        cv2.imshow(self.window_name, img)
        if not self.window_positioned:
            self._position_window()
            self.window_positioned = True

    # 설명: 텔레오퍼레이션 종료 시 차량 정지 명령과 자원 정리를 수행한다.
    def stop(self):
        self.speed = 0.0
        self.steer = 0.0
        if not rclpy.ok():
            return
        for _ in range(5):
            self.publish_cmd()
            time.sleep(0.02)


# 설명: ROS 노드나 스크립트 실행을 시작하는 진입점이다.
def main(args=None):
    rclpy.init(args=args)
    node = CvKeyboardTeleop()

    try:
        rate = max(float(node.get_parameter('publish_rate_hz').value), 1.0)
        period = 1.0 / rate
        while rclpy.ok() and node.running:
            start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            node.render()
            key = cv2.waitKey(1)
            if key != -1:
                node.handle_key(key)
            node.apply_hold_timeout()
            node.publish_cmd()

            elapsed = time.monotonic() - start
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
