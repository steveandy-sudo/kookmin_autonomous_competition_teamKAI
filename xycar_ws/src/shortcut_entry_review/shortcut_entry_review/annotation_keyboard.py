#!/usr/bin/env python3
"""Small raw-key terminal for the RViz BEV annotation node."""

from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP = """
Shortcut BEV annotation keys
  w : current line finish -> start next WHITE line (W1, W2)
  y : current line finish -> start next YELLOW line (Y1, Y2)
  Publish Point clicks in RViz add points to the active line
  z : undo last point    Enter/f : finish current line
  r : reset clicks       n : release frame and wait for next frame
  s : save W1/W2/Y1/Y2  q : close this keyboard window
"""


class AnnotationKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("shortcut_annotation_keyboard")
        self.publisher = self.create_publisher(
            String, "/shortcut_annotation/key", 10
        )
        self.create_subscription(
            String, "/shortcut_annotation/status", self.on_status, 10
        )

    def on_status(self, message: String) -> None:
        print(f"\n[STATUS] {message.data}", flush=True)


def main() -> None:
    if not sys.stdin.isatty():
        raise RuntimeError("annotation_keyboard must run in an interactive terminal")
    rclpy.init()
    node = AnnotationKeyboard()
    original = termios.tcgetattr(sys.stdin)
    print(HELP, flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.03)
            if not readable:
                continue
            key = sys.stdin.read(1).lower()
            if key == "q":
                break
            command = "enter" if key in ("\r", "\n") else key
            if command not in ("w", "y", "z", "f", "enter", "r", "n", "s"):
                continue
            node.publisher.publish(String(data=command))
            print(f"\n[KEY] {command}", flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
