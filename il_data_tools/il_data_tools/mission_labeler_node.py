#!/usr/bin/env python3

import select
import sys
import termios
import tty
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


LABELS: Dict[str, str] = {
    "0": "idle",
    "1": "lane_drive",
    "2": "cone_drive",
    "3": "hill_drive",
    "4": "pedestrian_avoid",
    "5": "vehicle_follow",
    "6": "vehicle_overtake",
    "7": "traffic_light_start",
    "8": "route_select",
    "9": "shortcut",
    "p": "parking",
    "r": "recovery",
    "x": "bad_data",
}


HELP_TEXT = """
Mission label keys:
  0 idle | 1 lane_drive | 2 cone_drive | 3 hill_drive
  4 pedestrian_avoid | 5 vehicle_follow | 6 vehicle_overtake
  7 traffic_light_start | 8 route_select | 9 shortcut
  p parking | r recovery | x bad_data | q quit
"""


class MissionLabelerNode(Node):
    def __init__(self) -> None:
        super().__init__("il_mission_labeler")
        self.declare_parameter("mission_label_topic", "/il/mission_label")
        self.declare_parameter("initial_label", "idle")
        topic = str(self.get_parameter("mission_label_topic").value)
        self.current_label = str(self.get_parameter("initial_label").value)
        self.publisher = self.create_publisher(String, topic, 10)
        self._old_terminal_settings = None
        self._terminal_ready = False
        self._configure_terminal()
        self.create_timer(0.2, self._tick)
        self.get_logger().info(HELP_TEXT)
        self.get_logger().info(f"Current label: {self.current_label}")

    def _configure_terminal(self) -> None:
        if not sys.stdin.isatty():
            self.get_logger().warn("stdin is not a TTY; publishing initial label only")
            return
        self._old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self._terminal_ready = True

    def restore_terminal(self) -> None:
        if self._terminal_ready and self._old_terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_terminal_settings)

    def _tick(self) -> None:
        self._read_key_once()
        msg = String()
        msg.data = self.current_label
        self.publisher.publish(msg)

    def _read_key_once(self) -> None:
        if not self._terminal_ready:
            return
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        if not ready:
            return
        key = sys.stdin.read(1)
        if key == "q":
            self.get_logger().info("Quit requested.")
            rclpy.shutdown()
            return
        if key in LABELS:
            self.current_label = LABELS[key]
            self.get_logger().info(f"Mission label changed: {self.current_label}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionLabelerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
