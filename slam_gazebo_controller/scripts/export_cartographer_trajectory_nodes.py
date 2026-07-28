#!/usr/bin/env python3
"""Export every optimized Cartographer trajectory marker point to CSV."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trajectory-id", type=int, default=0)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    return parser.parse_args()


class TrajectoryNodeExporter(Node):
    def __init__(self, output: Path, trajectory_id: int) -> None:
        super().__init__("cartographer_trajectory_node_exporter")
        self.output = output
        self.namespace = f"Trajectory {trajectory_id}"
        self.completed = False
        self.create_subscription(
            MarkerArray,
            "/trajectory_node_list",
            self._on_markers,
            10,
        )

    def _on_markers(self, message: MarkerArray) -> None:
        candidates = [
            marker
            for marker in message.markers
            if marker.ns == self.namespace and len(marker.points) >= 2
        ]
        if self.completed or not candidates:
            return
        marker = max(candidates, key=lambda item: len(item.points))
        points = [
            (float(point.x), float(point.y)) for point in marker.points
        ]
        self.output.parent.mkdir(parents=True, exist_ok=True)
        with self.output.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["index", "x", "y", "yaw"])
            for index, point in enumerate(points):
                before = points[max(0, index - 1)]
                after = points[min(len(points) - 1, index + 1)]
                yaw = math.atan2(
                    after[1] - before[1],
                    after[0] - before[0],
                )
                writer.writerow([index, point[0], point[1], yaw])
        self.get_logger().info(
            f"exported {len(points)} nodes to {self.output}"
        )
        self.completed = True


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = TrajectoryNodeExporter(
        args.output.expanduser().resolve(),
        args.trajectory_id,
    )
    deadline = time.monotonic() + max(1.0, args.timeout_sec)
    try:
        while (
            rclpy.ok()
            and not node.completed
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.25)
        if not node.completed:
            raise RuntimeError(
                "timed out waiting for /trajectory_node_list"
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
