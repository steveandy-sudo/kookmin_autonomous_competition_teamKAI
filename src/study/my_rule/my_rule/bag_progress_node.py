#!/usr/bin/env python3
"""Show human-readable rosbag playback progress from the player's /clock."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time
from typing import Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.utilities import remove_ros_args
from rosgraph_msgs.msg import Clock
import yaml


@dataclass(frozen=True)
class BagTiming:
    metadata_path: Path
    start_ns: int
    duration_ns: int


def find_metadata(path_text: str) -> Path:
    """Resolve a bag directory, database, or outer extraction directory."""
    path = Path(path_text).expanduser().resolve()
    if path.is_file():
        candidate = path if path.name == "metadata.yaml" else path.parent / "metadata.yaml"
        if candidate.is_file():
            return candidate
    elif path.is_dir():
        candidate = path / "metadata.yaml"
        if candidate.is_file():
            return candidate
        matches = sorted(path.rglob("metadata.yaml"))
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            raise ValueError(
                f"multiple bags found below {path}; pass one bag directory"
            )
    raise FileNotFoundError(f"metadata.yaml not found for bag path: {path}")


def load_bag_timing(path_text: str) -> BagTiming:
    metadata_path = find_metadata(path_text)
    with metadata_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    information = document["rosbag2_bagfile_information"]
    start_ns = int(information["starting_time"]["nanoseconds_since_epoch"])
    duration_ns = int(information["duration"]["nanoseconds"])
    if duration_ns <= 0:
        raise ValueError(f"bag duration must be positive: {duration_ns}")
    return BagTiming(metadata_path, start_ns, duration_ns)


def format_duration(seconds: float) -> str:
    seconds = max(0.0, float(seconds))
    hours = int(seconds // 3600.0)
    minutes = int(seconds // 60.0) % 60
    remainder = seconds % 60.0
    if hours:
        return f"{hours:d}:{minutes:02d}:{remainder:04.1f}"
    return f"{minutes:02d}:{remainder:04.1f}"


def progress_line(
    state: str,
    elapsed_sec: float,
    total_sec: float,
    rate: float,
    loop_count: int,
    width: int = 24,
) -> str:
    fraction = max(0.0, min(1.0, elapsed_sec / max(total_sec, 1e-9)))
    filled = min(width, int(round(fraction * width)))
    bar = "#" * filled + "-" * (width - filled)
    loop_text = f" | loop {loop_count + 1}" if loop_count else ""
    return (
        f"[{state:<7}] {format_duration(elapsed_sec)} / "
        f"{format_duration(total_sec)} | {fraction * 100:5.1f}% "
        f"[{bar}] | {rate:4.2f}x{loop_text}"
    )


class BagProgressNode(Node):
    def __init__(self, timing: BagTiming, refresh_hz: float) -> None:
        super().__init__("my_rule_bag_progress")
        self.timing = timing
        self.total_sec = timing.duration_ns / 1.0e9
        self.elapsed_sec: float | None = None
        self.previous_elapsed_sec: float | None = None
        self.previous_render_wall = time.monotonic()
        self.last_advance_wall: float | None = None
        self.loop_count = 0
        self.last_print_length = 0
        self.terminal = sys.stdout.isatty()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        self.create_timer(1.0 / max(0.5, refresh_hz), self.render)

        print(f"bag: {timing.metadata_path.parent}")
        print(f"duration: {format_duration(self.total_sec)}")
        print("waiting for /clock (play the bag with --clock 20 or --clock 40)")

    def on_clock(self, message: Clock) -> None:
        absolute_ns = (
            int(message.clock.sec) * 1_000_000_000
            + int(message.clock.nanosec)
        )
        elapsed = (absolute_ns - self.timing.start_ns) / 1.0e9
        elapsed = max(0.0, min(self.total_sec, elapsed))
        now = time.monotonic()
        if self.elapsed_sec is None or elapsed > self.elapsed_sec + 1e-4:
            self.last_advance_wall = now
        elif elapsed + 0.25 < self.elapsed_sec:
            self.loop_count += 1
            self.last_advance_wall = now
            self.previous_elapsed_sec = None
        self.elapsed_sec = elapsed

    def render(self) -> None:
        now = time.monotonic()
        if self.elapsed_sec is None:
            line = "[WAITING] no /clock received"
        else:
            wall_delta = max(1e-6, now - self.previous_render_wall)
            if self.previous_elapsed_sec is None:
                rate = 0.0
            else:
                rate = max(
                    0.0,
                    (self.elapsed_sec - self.previous_elapsed_sec) / wall_delta,
                )
            since_advance = (
                float("inf")
                if self.last_advance_wall is None
                else now - self.last_advance_wall
            )
            if self.elapsed_sec >= self.total_sec - 0.05:
                state = "DONE"
            elif since_advance <= 0.6:
                state = "PLAYING"
            else:
                state = "PAUSED"
                rate = 0.0
            line = progress_line(
                state,
                self.elapsed_sec,
                self.total_sec,
                rate,
                self.loop_count,
            )
            self.previous_elapsed_sec = self.elapsed_sec
        self.previous_render_wall = now

        if self.terminal:
            padding = " " * max(0, self.last_print_length - len(line))
            print(f"\r{line}{padding}", end="", flush=True)
            self.last_print_length = len(line)
        else:
            print(line, flush=True)

    def finish_line(self) -> None:
        if self.terminal and self.last_print_length:
            print()


def parse_arguments(arguments: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display elapsed/total rosbag time from /clock."
    )
    parser.add_argument(
        "bag_path",
        help="bag directory, metadata.yaml, db3, or directory containing one bag",
    )
    parser.add_argument("--refresh-hz", type=float, default=4.0)
    return parser.parse_args(list(arguments))


def main(args=None) -> None:
    raw_args = sys.argv if args is None else list(args)
    cli_args = remove_ros_args(args=raw_args)[1:]
    try:
        options = parse_arguments(cli_args)
        timing = load_bag_timing(options.bag_path)
    except (KeyError, OSError, TypeError, ValueError) as exc:
        raise SystemExit(f"bag progress error: {exc}") from exc

    rclpy.init(args=raw_args)
    node = BagProgressNode(timing, options.refresh_hz)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish_line()
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
