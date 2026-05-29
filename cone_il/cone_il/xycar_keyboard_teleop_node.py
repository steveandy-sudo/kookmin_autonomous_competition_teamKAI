#!/usr/bin/env python3
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from xycar_msgs.msg import XycarMotor

HELP = """
Keyboard teleop for XycarMotor
-----------------------------
  w : speed 10
  s : speed 0
  a : steer left 100
  d : steer right 100
  c : steering center
  x : speed 0
  space : stop speed and steer
  q : quit

No OpenCV window is used.
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('xycar_keyboard_teleop')
        self.declare_parameter('motor_topic', 'xycar_motor')
        self.declare_parameter('label_topic', '/cone_il/manual_cmd')
        self.declare_parameter('publish_rate_hz', 40.0)
        self.declare_parameter('speed_step', 5.0)
        self.declare_parameter('steer_step', 3.0)
        self.declare_parameter('max_speed', 10.0)
        self.declare_parameter('max_steer_deg', 100.0)
        self.declare_parameter('forward_speed', 10.0)
        self.declare_parameter('turn_steer_deg', 100.0)
        self.declare_parameter('steer_hold_timeout_sec', 0.0)

        motor_topic = str(self.get_parameter('motor_topic').value)
        label_topic = str(self.get_parameter('label_topic').value)
        self.motor_pub = self.create_publisher(XycarMotor, motor_topic, 10)
        self.label_pub = self.create_publisher(XycarMotor, label_topic, 10)
        self.speed = 0.0
        self.steer = 0.0
        self.running = True
        self.msg = XycarMotor()
        self.last_steer_key_time = 0.0
        self.last_logged_cmd = None

        rate = max(float(self.get_parameter('publish_rate_hz').value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)
        self.thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'Keyboard teleop publishing drive={motor_topic}, label={label_topic}'
        )
        print(HELP)

    def publish_cmd(self):
        if not rclpy.ok():
            return

        self.apply_hold_timeout()
        msg = XycarMotor()
        msg.angle = float(self.steer)
        msg.speed = float(self.speed)
        try:
            self.motor_pub.publish(msg)
            self.label_pub.publish(msg)
        except Exception as exc:
            if rclpy.ok():
                self.get_logger().warn(f'publish failed: {exc}')

    def keyboard_loop(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self.handle_key(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def handle_key(self, key: str):
        key = key.lower()
        speed_step = float(self.get_parameter('speed_step').value)
        steer_step = float(self.get_parameter('steer_step').value)
        max_speed = float(self.get_parameter('max_speed').value)
        max_steer = float(self.get_parameter('max_steer_deg').value)
        forward_speed = float(self.get_parameter('forward_speed').value)
        turn_steer = float(self.get_parameter('turn_steer_deg').value)
        now = time.monotonic()

        if key == 'w':
            self.speed = min(max(forward_speed, -max_speed), max_speed)
        elif key == 's':
            self.speed = 0.0
        elif key == 'a':
            self.steer = -min(abs(turn_steer), max_steer)
            self.last_steer_key_time = now
        elif key == 'd':
            self.steer = min(abs(turn_steer), max_steer)
            self.last_steer_key_time = now
        elif key == 'c':
            self.steer = 0.0
            self.last_steer_key_time = 0.0
        elif key == 'x':
            self.speed = 0.0
        elif key == ' ':
            self.speed = 0.0
            self.steer = 0.0
            self.last_steer_key_time = 0.0
        elif key == 'q':
            self.running = False
            rclpy.shutdown()
            return

        self.log_cmd_if_changed()

    def apply_hold_timeout(self):
        steer_timeout = float(self.get_parameter('steer_hold_timeout_sec').value)
        now = time.monotonic()
        changed = False

        if steer_timeout > 0.0 and self.last_steer_key_time > 0.0 and now - self.last_steer_key_time > steer_timeout:
            if abs(self.steer) > 1e-3:
                self.steer = 0.0
                changed = True
            self.last_steer_key_time = 0.0

        if changed:
            self.log_cmd_if_changed()

    def log_cmd_if_changed(self):
        cmd = (round(float(self.steer), 1), round(float(self.speed), 1))
        if cmd == self.last_logged_cmd:
            return
        self.last_logged_cmd = cmd
        print(f'angle={self.steer:6.1f}, speed={self.speed:5.1f}', flush=True)

    def stop(self):
        self.speed = 0.0
        self.steer = 0.0
        if not rclpy.ok():
            return
        for _ in range(5):
            self.publish_cmd()
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
