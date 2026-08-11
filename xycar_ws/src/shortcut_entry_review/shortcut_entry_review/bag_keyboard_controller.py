#!/usr/bin/env python3
"""Interactive rosbag keyboard with deterministic 0.2-second rewind."""

from __future__ import annotations

import argparse
from bisect import bisect_left, bisect_right
from pathlib import Path
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
import rosbag2_py
from rosbag2_interfaces.srv import IsPaused, Pause, PlayNext, Resume, Seek
from rosgraph_msgs.msg import Clock


SOURCE_TOPIC = "/wide_camera_mjpeg/image_raw/compressed"
REWIND_NS = 200_000_000


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--source-topic", default=SOURCE_TOPIC)
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument("--autoplay", action="store_true")
    return parser


def read_frame_timestamps(bag_path: Path, source_topic: str) -> list[int]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    timestamps = []
    while reader.has_next():
        topic, _, timestamp_ns = reader.read_next()
        if topic == source_topic:
            timestamps.append(int(timestamp_ns))
    if not timestamps:
        raise RuntimeError(f"bag has no frames on {source_topic}")
    return timestamps


class BagKeyboardController(Node):
    def __init__(
        self,
        *,
        timestamps_ns: list[int],
        start_offset_sec: float,
    ) -> None:
        super().__init__("shortcut_bag_keyboard_controller")
        self.timestamps_ns = timestamps_ns
        self.first_timestamp_ns = int(timestamps_ns[0])
        requested_ns = self.first_timestamp_ns + int(
            round(float(start_offset_sec) * 1_000_000_000.0)
        )
        self.display_index = min(
            len(timestamps_ns) - 1,
            max(0, bisect_left(timestamps_ns, requested_ns)),
        )
        self.clock_ns = int(timestamps_ns[self.display_index])
        self.paused = True
        self.pause_client = self.create_client(Pause, "/rosbag2_player/pause")
        self.resume_client = self.create_client(
            Resume, "/rosbag2_player/resume"
        )
        self.seek_client = self.create_client(Seek, "/rosbag2_player/seek")
        self.play_next_client = self.create_client(
            PlayNext, "/rosbag2_player/play_next"
        )
        self.is_paused_client = self.create_client(
            IsPaused, "/rosbag2_player/is_paused"
        )
        self.create_subscription(Clock, "/clock", self.on_clock, 10)

    def wait_for_player(self, timeout_sec: float = 15.0) -> None:
        deadline = time.monotonic() + float(timeout_sec)
        for client in (
            self.pause_client,
            self.resume_client,
            self.seek_client,
            self.play_next_client,
            self.is_paused_client,
        ):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not client.wait_for_service(
                timeout_sec=remaining
            ):
                raise RuntimeError("timed out waiting for rosbag player services")

    def call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"rosbag service call failed: {client.srv_name}")
        return future.result()

    def on_clock(self, message: Clock) -> None:
        self.clock_ns = (
            int(message.clock.sec) * 1_000_000_000
            + int(message.clock.nanosec)
        )
        if not self.paused:
            self.display_index = min(
                len(self.timestamps_ns) - 1,
                max(0, bisect_right(self.timestamps_ns, self.clock_ns) - 1),
            )

    def show_frame(self, index: int) -> None:
        target = min(len(self.timestamps_ns) - 1, max(0, int(index)))
        self.call(self.pause_client, Pause.Request())
        self.paused = True
        timestamp_ns = int(self.timestamps_ns[target])
        request = Seek.Request()
        request.time.sec = timestamp_ns // 1_000_000_000
        request.time.nanosec = timestamp_ns % 1_000_000_000
        response = self.call(self.seek_client, request)
        if not response.success:
            raise RuntimeError("rosbag seek rejected the requested frame")
        response = self.call(self.play_next_client, PlayNext.Request())
        if not response.success:
            raise RuntimeError("rosbag play_next failed while paused")
        self.display_index = target
        self.clock_ns = timestamp_ns
        rclpy.spin_once(self, timeout_sec=0.05)

    def rewind(self) -> None:
        reference_ns = (
            self.timestamps_ns[self.display_index]
            if self.paused
            else self.clock_ns
        )
        target_ns = max(
            self.first_timestamp_ns,
            int(reference_ns) - REWIND_NS,
        )
        target_index = max(
            0, bisect_right(self.timestamps_ns, target_ns) - 1
        )
        self.show_frame(target_index)

    def toggle(self) -> None:
        if self.paused:
            self.call(self.resume_client, Resume.Request())
            self.paused = False
        else:
            self.call(self.pause_client, Pause.Request())
            self.paused = True
            self.display_index = min(
                len(self.timestamps_ns) - 1,
                max(0, bisect_right(self.timestamps_ns, self.clock_ns) - 1),
            )

    def status(self) -> str:
        offset_sec = (
            self.timestamps_ns[self.display_index] - self.first_timestamp_ns
        ) / 1_000_000_000.0
        state = "PAUSED" if self.paused else "PLAYING"
        return (
            f"{state} | frame {self.display_index + 1}/{len(self.timestamps_ns)} "
            f"| BAG OFFSET {offset_sec:8.3f} s "
            "| a back 0.2s | SPACE play/pause | q quit"
        )


def read_key() -> str:
    first = sys.stdin.read(1)
    if first != "\x1b":
        return first
    sequence = first
    for _ in range(2):
        readable, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not readable:
            break
        sequence += sys.stdin.read(1)
    return sequence


def main() -> None:
    args = build_parser().parse_args()
    bag_path = args.bag_path.expanduser().resolve()
    timestamps = read_frame_timestamps(bag_path, str(args.source_topic))
    if not sys.stdin.isatty():
        raise RuntimeError("bag keyboard controller requires an interactive terminal")
    rclpy.init()
    node = BagKeyboardController(
        timestamps_ns=timestamps,
        start_offset_sec=float(args.start_offset),
    )
    original = termios.tcgetattr(sys.stdin)
    try:
        node.wait_for_player()
        node.show_frame(node.display_index)
        if bool(args.autoplay):
            node.toggle()
        tty.setcbreak(sys.stdin.fileno())
        print("\nCustom shortcut rosbag controls are ready.\n", flush=True)
        print("\r" + node.status(), end="", flush=True)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            readable, _, _ = select.select([sys.stdin], [], [], 0.03)
            if readable:
                key = read_key()
                try:
                    if key.lower() == "a":
                        node.rewind()
                    elif key == " ":
                        node.toggle()
                    elif key.lower() == "q":
                        break
                except RuntimeError as exc:
                    print(f"\nERROR: {exc}", flush=True)
            print("\r" + node.status(), end="", flush=True)
    finally:
        print(flush=True)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
