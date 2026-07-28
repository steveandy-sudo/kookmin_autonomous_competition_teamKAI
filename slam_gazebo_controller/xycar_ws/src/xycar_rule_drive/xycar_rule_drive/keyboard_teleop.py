from __future__ import annotations

import select
import sys
import termios
import tty

from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32MultiArray


HELP_TEXT = """
Xycar keyboard teleop

  w / s       speed up / slow down
  a / d       steer left / steer right
  e           center steering
  x           speed zero
  space       full stop
  q           quit

Commands are published as std_msgs/Float32MultiArray [angle, speed] on /xycar_motor.
Do not run this together with lane_rule_driver because both publish /xycar_motor.
"""


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("xycar_keyboard_teleop")
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("angle_step", 5.0)
        self.declare_parameter("speed_step", 2.0)
        self.declare_parameter("angle_min", -42.0)
        self.declare_parameter("angle_max", 42.0)
        self.declare_parameter("speed_min", -20.0)
        self.declare_parameter("speed_max", 20.0)

        self.angle_step = float(self.get_parameter("angle_step").value)
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.speed_min = float(self.get_parameter("speed_min").value)
        self.speed_max = float(self.get_parameter("speed_max").value)

        self.angle = 0.0
        self.speed = 0.0
        self.quit_requested = False
        self.stdin_is_tty = sys.stdin.isatty()
        self.original_terminal_settings = None
        if self.stdin_is_tty:
            self.original_terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            self.get_logger().warning("stdin is not a TTY; keyboard input will not work")

        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("motor_topic").value),
            10,
        )
        publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / publish_rate_hz, self.on_timer)
        self.get_logger().info(HELP_TEXT)

    def restore_terminal(self) -> None:
        if self.original_terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_terminal_settings)
            self.original_terminal_settings = None

    def read_key(self) -> str | None:
        if not self.stdin_is_tty:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        return sys.stdin.read(1)

    def on_timer(self) -> None:
        while True:
            key = self.read_key()
            if key is None:
                break
            self.handle_key(key)
        self.publish_motor()
        if self.quit_requested:
            self.get_logger().info("keyboard teleop quit requested")
            raise KeyboardInterrupt

    def handle_key(self, key: str) -> None:
        if key == "w":
            self.speed += self.speed_step
        elif key == "s":
            self.speed -= self.speed_step
        elif key == "a":
            self.angle -= self.angle_step
        elif key == "d":
            self.angle += self.angle_step
        elif key == "e":
            self.angle = 0.0
        elif key == "x":
            self.speed = 0.0
        elif key == " ":
            self.angle = 0.0
            self.speed = 0.0
        elif key in {"q", "\x03"}:
            self.angle = 0.0
            self.speed = 0.0
            self.quit_requested = True

        self.angle = clamp(self.angle, self.angle_min, self.angle_max)
        self.speed = clamp(self.speed, self.speed_min, self.speed_max)
        self.get_logger().info(
            f"keyboard command angle={self.angle:.1f}, speed={self.speed:.1f}",
            throttle_duration_sec=0.05,
        )

    def publish_motor(self) -> None:
        msg = Float32MultiArray()
        msg.data = [float(self.angle), float(self.speed)]
        self.motor_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.angle = 0.0
        node.speed = 0.0
        node.publish_motor()
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
