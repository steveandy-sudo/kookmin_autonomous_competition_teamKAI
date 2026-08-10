from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty

from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32MultiArray

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
try:
    import pygame
except ImportError:
    pygame = None


HELP_TEXT = """
Xycar keyboard teleop

  w (hold)    drive at fixed speed 17; release to stop
  a (hold)    ramp steering by 10 to -30; release to center
  d (hold)    ramp steering by 10 to +30; release to center
  x           stop
  space       full stop
  q           quit

Commands are published as std_msgs/Float32MultiArray [angle, speed] on /xycar_motor.
Do not run this together with lane_rule_driver because both publish /xycar_motor.
"""


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def hold_steering_command(
    left_pressed: bool,
    right_pressed: bool,
    full_lock_angle: float,
) -> float:
    if left_pressed == right_pressed:
        return 0.0
    if left_pressed:
        return -abs(full_lock_angle)
    return abs(full_lock_angle)


def move_towards(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + max_delta * (1.0 if delta > 0.0 else -1.0)


def ramped_steering_command(
    current: float,
    left_pressed: bool,
    right_pressed: bool,
    full_lock_angle: float,
    angle_step: float,
) -> float:
    target = hold_steering_command(
        left_pressed,
        right_pressed,
        full_lock_angle,
    )
    return move_towards(current, target, abs(angle_step))


def hold_speed_command(drive_pressed: bool, fixed_speed: float) -> float:
    return fixed_speed if drive_pressed else 0.0


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("xycar_keyboard_teleop")
        self.declare_parameter("motor_topic", "/xycar_motor")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("input_mode", "pygame_hold")
        self.declare_parameter("fixed_speed", 17.0)
        self.declare_parameter("full_lock_angle", 30.0)
        self.declare_parameter("angle_step", 10.0)
        self.declare_parameter("steering_step_interval_sec", 0.10)
        self.declare_parameter("speed_step", 2.0)
        self.declare_parameter("angle_min", -42.0)
        self.declare_parameter("angle_max", 42.0)
        self.declare_parameter("speed_min", -20.0)
        self.declare_parameter("speed_max", 20.0)

        self.angle_step = abs(float(self.get_parameter("angle_step").value))
        self.steering_step_interval_sec = max(
            0.0,
            float(self.get_parameter("steering_step_interval_sec").value),
        )
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.speed_min = float(self.get_parameter("speed_min").value)
        self.speed_max = float(self.get_parameter("speed_max").value)
        self.input_mode = str(self.get_parameter("input_mode").value)
        self.fixed_speed = clamp(
            float(self.get_parameter("fixed_speed").value),
            self.speed_min,
            self.speed_max,
        )
        self.full_lock_angle = min(
            abs(float(self.get_parameter("full_lock_angle").value)),
            abs(self.angle_min),
            abs(self.angle_max),
        )

        self.angle = 0.0
        self.steering_target = 0.0
        self.next_steering_step_time = time.monotonic()
        self.speed = 0.0
        self.drive_enabled = False
        self.quit_requested = False
        self.stdin_is_tty = (
            self.input_mode == "terminal_step" and sys.stdin.isatty()
        )
        self.original_terminal_settings = None
        if self.stdin_is_tty:
            self.original_terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        elif self.input_mode == "terminal_step":
            self.get_logger().warning("stdin is not a TTY; keyboard input will not work")
        self.pygame_screen = None
        self.pygame_font = None
        if self.input_mode == "pygame_hold":
            if pygame is None:
                raise RuntimeError(
                    "pygame_hold mode requires python3-pygame"
                )
            pygame.init()
            self.pygame_screen = pygame.display.set_mode((420, 150))
            pygame.display.set_caption("Xycar Manual Control")
            self.pygame_font = pygame.font.Font(None, 30)

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
        if self.input_mode == "pygame_hold":
            self.update_pygame_controls()
        else:
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
            self.drive_enabled = True
            self.speed = self.fixed_speed
        elif key == "s":
            self.drive_enabled = False
            self.speed = 0.0
        elif key == "a":
            self.angle -= self.angle_step
        elif key == "d":
            self.angle += self.angle_step
        elif key == "e":
            self.angle = 0.0
        elif key == "x":
            self.drive_enabled = False
            self.speed = 0.0
        elif key == " ":
            self.drive_enabled = False
            self.angle = 0.0
            self.speed = 0.0
        elif key in {"q", "\x03"}:
            self.drive_enabled = False
            self.angle = 0.0
            self.speed = 0.0
            self.quit_requested = True

        self.angle = clamp(self.angle, self.angle_min, self.angle_max)
        self.speed = clamp(self.speed, self.speed_min, self.speed_max)
        self.get_logger().info(
            f"keyboard command angle={self.angle:.1f}, speed={self.speed:.1f}",
            throttle_duration_sec=0.05,
        )

    def update_pygame_controls(self) -> None:
        assert pygame is not None
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.drive_enabled = False
                self.quit_requested = True
            elif event.type == pygame.KEYDOWN:
                if event.key in (
                    pygame.K_x,
                    pygame.K_SPACE,
                    pygame.K_DOWN,
                ):
                    self.drive_enabled = False
                elif event.key in (pygame.K_q, pygame.K_ESCAPE):
                    self.drive_enabled = False
                    self.quit_requested = True
            elif event.type == pygame.WINDOWFOCUSLOST:
                self.drive_enabled = False

        keys = pygame.key.get_pressed()
        left_pressed = bool(keys[pygame.K_a] or keys[pygame.K_LEFT])
        right_pressed = bool(keys[pygame.K_d] or keys[pygame.K_RIGHT])
        drive_pressed = bool(keys[pygame.K_w] or keys[pygame.K_UP])
        stop_pressed = bool(
            keys[pygame.K_x] or keys[pygame.K_SPACE] or keys[pygame.K_DOWN]
        )
        steering_target = hold_steering_command(
            left_pressed,
            right_pressed,
            self.full_lock_angle,
        )
        now = time.monotonic()
        if steering_target != self.steering_target:
            self.steering_target = steering_target
            self.next_steering_step_time = now
        if now >= self.next_steering_step_time:
            self.angle = move_towards(
                self.angle,
                self.steering_target,
                self.angle_step,
            )
            self.next_steering_step_time = (
                now + self.steering_step_interval_sec
            )
        self.drive_enabled = drive_pressed and not stop_pressed
        self.speed = hold_speed_command(self.drive_enabled, self.fixed_speed)
        self.draw_pygame_status()

    def draw_pygame_status(self) -> None:
        if self.pygame_screen is None or self.pygame_font is None:
            return
        self.pygame_screen.fill((24, 26, 28))
        color = (90, 220, 130) if self.drive_enabled else (230, 110, 90)
        status = "DRIVE" if self.drive_enabled else "STOP"
        lines = (
            f"{status}  speed={self.speed:.0f}",
            f"steering={self.angle:+.0f}",
            "Hold W drive | hold A/D steer | Q quit",
        )
        for index, line in enumerate(lines):
            surface = self.pygame_font.render(
                line,
                True,
                color if index == 0 else (235, 235, 235),
            )
            self.pygame_screen.blit(surface, (18, 18 + index * 40))
        pygame.display.flip()

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
        if pygame is not None and node.input_mode == "pygame_hold":
            pygame.quit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
