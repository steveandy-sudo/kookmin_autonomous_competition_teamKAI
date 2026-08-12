"""Enable the Gazebo GUI chase camera after its transport service appears."""

from __future__ import annotations

import argparse
import subprocess
import time
from typing import Sequence


def follow_command(model_name: str) -> list[str]:
    return [
        "gz",
        "service",
        "-s",
        "/gui/follow",
        "--reqtype",
        "gz.msgs.StringMsg",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "1000",
        "--req",
        f'data: "{model_name}"',
    ]


def offset_command(x: float, y: float, z: float) -> list[str]:
    return [
        "gz",
        "service",
        "-s",
        "/gui/follow/offset",
        "--reqtype",
        "gz.msgs.Vector3d",
        "--reptype",
        "gz.msgs.Boolean",
        "--timeout",
        "1000",
        "--req",
        f"x: {float(x):.6f} y: {float(y):.6f} z: {float(z):.6f}",
    ]


def service_succeeded(command: Sequence[str]) -> bool:
    result = subprocess.run(
        list(command),
        check=False,
        capture_output=True,
        text=True,
        timeout=2.0,
    )
    return result.returncode == 0 and "data: true" in result.stdout.lower()


def enable_follow(
    *,
    model_name: str,
    offset_x: float,
    offset_y: float,
    offset_z: float,
    startup_timeout_sec: float,
) -> bool:
    deadline = time.monotonic() + max(0.1, float(startup_timeout_sec))
    while time.monotonic() < deadline:
        try:
            follow_ready = service_succeeded(follow_command(model_name))
            offset_ready = follow_ready and service_succeeded(
                offset_command(offset_x, offset_y, offset_z)
            )
        except (OSError, subprocess.SubprocessError):
            follow_ready = False
            offset_ready = False
        if follow_ready and offset_ready:
            return True
        time.sleep(0.25)
    return False


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="xycar_ackermann")
    parser.add_argument("--offset-x", type=float, default=-2.5)
    parser.add_argument("--offset-y", type=float, default=0.0)
    parser.add_argument("--offset-z", type=float, default=1.6)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    if enable_follow(
        model_name=str(args.model),
        offset_x=float(args.offset_x),
        offset_y=float(args.offset_y),
        offset_z=float(args.offset_z),
        startup_timeout_sec=float(args.startup_timeout),
    ):
        print(
            "Gazebo camera following "
            f"{args.model} at offset "
            f"({args.offset_x:.2f}, {args.offset_y:.2f}, {args.offset_z:.2f})"
        )
        return 0
    print(f"Gazebo camera follow service unavailable for {args.model}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
