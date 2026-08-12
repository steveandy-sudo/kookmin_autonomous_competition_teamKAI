#!/usr/bin/env python3
"""Large live graph for recorded versus shadow steering commands."""

from collections import deque
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SteeringCompareViewer(Node):
    def __init__(self) -> None:
        super().__init__("steering_compare_viewer")
        self.declare_parameter(
            "base_topic", "/comparison/base_rule_candidate"
        )
        self.declare_parameter(
            "current_topic", "/comparison/row_rule_candidate"
        )
        self.declare_parameter("third_topic", "")
        self.declare_parameter("history_sec", 20.0)
        self.declare_parameter("window_width", 1200)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("base_label", "BAG BASE")
        self.declare_parameter("current_label", "ROW MODEL")
        self.declare_parameter("third_label", "")
        self.declare_parameter(
            "window_name", "Steering: BAG BASE vs ROW MODEL"
        )

        self.history_sec = max(
            2.0, float(self.get_parameter("history_sec").value)
        )
        self.width = max(
            640, int(self.get_parameter("window_width").value)
        )
        self.height = max(
            420, int(self.get_parameter("window_height").value)
        )
        self.base_label = str(self.get_parameter("base_label").value)
        self.current_label = str(
            self.get_parameter("current_label").value
        )
        self.third_topic = str(self.get_parameter("third_topic").value)
        self.third_label = str(self.get_parameter("third_label").value)
        self.started = time.monotonic()
        self.base_value = None
        self.current_value = None
        self.third_value = None
        self.base_history = deque()
        self.current_history = deque()
        self.third_history = deque()
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("base_topic").value),
            self.on_base,
            10,
        )
        if self.third_topic:
            self.create_subscription(
                Float32MultiArray,
                self.third_topic,
                self.on_third,
                10,
            )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("current_topic").value),
            self.on_current,
            10,
        )
        self.create_timer(0.05, self.draw)

        self.window_name = str(self.get_parameter("window_name").value)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.width, self.height)
        self.get_logger().info(
            f"steering viewer ready: first={self.base_label}, "
            f"second={self.current_label}, third={self.third_label or 'off'}"
        )

    def elapsed(self):
        return time.monotonic() - self.started

    def on_base(self, message):
        if message.data:
            self.base_value = float(message.data[0])
            self.base_history.append((self.elapsed(), self.base_value))

    def on_current(self, message):
        if message.data:
            self.current_value = float(message.data[0])
            self.current_history.append((self.elapsed(), self.current_value))

    def on_third(self, message):
        if message.data:
            self.third_value = float(message.data[0])
            self.third_history.append((self.elapsed(), self.third_value))

    def trim(self, now):
        oldest = now - self.history_sec
        for history in (
            self.base_history,
            self.current_history,
            self.third_history,
        ):
            while history and history[0][0] < oldest:
                history.popleft()

    @staticmethod
    def value_text(value):
        return "WAIT" if value is None else f"{value:+6.2f}"

    def plot_points(self, history, now, left, top, right, bottom):
        points = []
        for stamp, value in history:
            x = right - (now - stamp) / self.history_sec * (right - left)
            value = float(np.clip(value, -42.0, 42.0))
            y = bottom - (value + 42.0) / 84.0 * (bottom - top)
            points.append((int(round(x)), int(round(y))))
        return np.asarray(points, dtype=np.int32)

    def draw(self):
        now = self.elapsed()
        self.trim(now)
        canvas = np.full((self.height, self.width, 3), 28, dtype=np.uint8)
        base_color = (255, 190, 40)
        current_color = (0, 170, 255)
        third_color = (80, 235, 80)
        text_color = (235, 235, 235)

        cv2.putText(
            canvas,
            "STEERING COMMAND COMPARISON  (-42 ... +42)",
            (36, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.95,
            text_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"{self.base_label} {self.value_text(self.base_value)}",
            (28, 102),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.15,
            base_color,
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"{self.current_label} {self.value_text(self.current_value)}",
            (390, 102),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.15,
            current_color,
            3,
            cv2.LINE_AA,
        )
        if self.third_topic:
            cv2.putText(
                canvas,
                f"{self.third_label} {self.value_text(self.third_value)}",
                (790, 102),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.05,
                third_color,
                3,
                cv2.LINE_AA,
            )
        else:
            difference = (
                None
                if self.base_value is None or self.current_value is None
                else self.current_value - self.base_value
            )
            cv2.putText(
                canvas,
                f"DELTA {self.value_text(difference)}",
                (870, 102),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                text_color,
                2,
                cv2.LINE_AA,
            )

        left, right = 86, self.width - 34
        top, bottom = 142, self.height - 62
        for command in (-42, -30, -20, -10, 0, 10, 20, 30, 42):
            y = int(
                round(bottom - (command + 42.0) / 84.0 * (bottom - top))
            )
            color = (105, 105, 105) if command == 0 else (58, 58, 58)
            cv2.line(canvas, (left, y), (right, y), color, 1)
            cv2.putText(
                canvas,
                f"{command:+d}",
                (24, y + 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (170, 170, 170),
                1,
                cv2.LINE_AA,
            )
        cv2.rectangle(canvas, (left, top), (right, bottom), (95, 95, 95), 1)

        base_points = self.plot_points(
            self.base_history, now, left, top, right, bottom
        )
        current_points = self.plot_points(
            self.current_history, now, left, top, right, bottom
        )
        third_points = self.plot_points(
            self.third_history, now, left, top, right, bottom
        )
        if len(base_points) >= 2:
            cv2.polylines(
                canvas, [base_points], False, base_color, 2, cv2.LINE_AA
            )
        if len(current_points) >= 2:
            cv2.polylines(
                canvas, [current_points], False, current_color, 2, cv2.LINE_AA
            )
        if len(third_points) >= 2:
            cv2.polylines(
                canvas, [third_points], False, third_color, 2, cv2.LINE_AA
            )
        cv2.putText(
            canvas,
            f"last {self.history_sec:.0f}s     Q or ESC: close viewer",
            (left, self.height - 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (170, 170, 170),
            1,
            cv2.LINE_AA,
        )

        cv2.imshow(self.window_name, canvas)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyWindow(self.window_name)
        return super().destroy_node()


def main():
    rclpy.init()
    node = SteeringCompareViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
