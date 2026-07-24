"""Pure control math for global-path and dynamic-vehicle rule driving."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .grid_planner import Point


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(float(minimum), min(float(maximum), float(value)))


def steering_command_for_curvature(
    curvature_per_m: float,
    commands: Sequence[float],
    curvatures: Sequence[float],
) -> float:
    if len(commands) != len(curvatures) or len(commands) < 2:
        raise ValueError("steering command and curvature maps must match")
    pairs = sorted(
        (float(curvature), float(command))
        for command, curvature in zip(commands, curvatures)
    )
    return float(
        np.interp(
            float(curvature_per_m),
            [pair[0] for pair in pairs],
            [pair[1] for pair in pairs],
        )
    )


def nearest_path_index(
    points: Sequence[Point],
    x: float,
    y: float,
    *,
    previous_index: int | None,
    closed: bool,
    search_back: int = 20,
    search_ahead: int = 180,
) -> int:
    if not points:
        raise ValueError("path is empty")
    if previous_index is None:
        candidates = range(len(points))
    elif closed:
        candidates = (
            (previous_index + offset) % len(points)
            for offset in range(-search_back, search_ahead + 1)
        )
    else:
        start = max(0, previous_index - search_back)
        stop = min(len(points), previous_index + search_ahead + 1)
        candidates = range(start, stop)
    return min(
        candidates,
        key=lambda index: (
            points[index][0] - float(x)
        ) ** 2
        + (points[index][1] - float(y)) ** 2,
    )


def lookahead_index(
    points: Sequence[Point],
    start_index: int,
    distance_m: float,
    *,
    closed: bool,
) -> int:
    if not points:
        raise ValueError("path is empty")
    target = max(0.01, float(distance_m))
    travelled = 0.0
    index = int(start_index)
    limit = len(points) if closed else len(points) - index - 1
    for _ in range(max(0, limit)):
        following = (index + 1) % len(points)
        travelled += math.hypot(
            points[following][0] - points[index][0],
            points[following][1] - points[index][1],
        )
        index = following
        if travelled >= target:
            break
    return index


def offset_path_point(
    points: Sequence[Point],
    index: int,
    offset_left_m: float,
    *,
    closed: bool,
) -> Point:
    if len(points) < 2 or abs(offset_left_m) < 1.0e-9:
        return points[index]
    before = index - 1
    after = index + 1
    if closed:
        before %= len(points)
        after %= len(points)
    else:
        before = max(0, before)
        after = min(len(points) - 1, after)
    heading = math.atan2(
        points[after][1] - points[before][1],
        points[after][0] - points[before][0],
    )
    return (
        points[index][0] - math.sin(heading) * float(offset_left_m),
        points[index][1] + math.cos(heading) * float(offset_left_m),
    )


@dataclass(frozen=True)
class PathCommand:
    curvature_per_m: float
    target_index: int
    target_x_vehicle_m: float
    target_y_vehicle_m: float
    target_distance_m: float


def pure_pursuit_command(
    points: Sequence[Point],
    nearest_index: int,
    *,
    vehicle_x: float,
    vehicle_y: float,
    vehicle_yaw: float,
    lookahead_m: float,
    lateral_offset_m: float,
    closed: bool,
) -> PathCommand:
    target_index = lookahead_index(
        points, nearest_index, lookahead_m, closed=closed
    )
    target = offset_path_point(
        points, target_index, lateral_offset_m, closed=closed
    )
    dx = target[0] - float(vehicle_x)
    dy = target[1] - float(vehicle_y)
    cosine = math.cos(float(vehicle_yaw))
    sine = math.sin(float(vehicle_yaw))
    forward = cosine * dx + sine * dy
    left = -sine * dx + cosine * dy
    distance_squared = max(1.0e-6, forward * forward + left * left)
    return PathCommand(
        curvature_per_m=2.0 * left / distance_squared,
        target_index=target_index,
        target_x_vehicle_m=forward,
        target_y_vehicle_m=left,
        target_distance_m=math.sqrt(distance_squared),
    )


@dataclass(frozen=True)
class DynamicAvoidanceConfig:
    right_offset_m: float = 0.28
    left_offset_m: float = 0.28
    offset_rate_mps: float = 0.35
    speed_limit_command: float = 4.0
    front_trigger: int = 1
    behind_passed_trigger: int = 1
    behind_return_trigger: int = 2
    clear_reset_sec: float = 0.5
    return_deadband_m: float = 0.02


@dataclass(frozen=True)
class DynamicAvoidanceState:
    mode: str
    lateral_offset_m: float
    speed_limit_command: float | None


class DynamicVehicleRule:
    """Meter-based version of the repository's vehicle-avoidance FSM."""

    def __init__(self, config: DynamicAvoidanceConfig) -> None:
        self.config = config
        self.mode = "NORMAL"
        self.current_offset_m = 0.0
        self.clear_started_sec: float | None = None
        self.last_update_sec: float | None = None

    def reset(self) -> None:
        self.mode = "NORMAL"
        self.clear_started_sec = None

    def update(
        self,
        *,
        now_sec: float,
        front_count: int,
        behind_count: int,
        armed: bool,
    ) -> DynamicAvoidanceState:
        now = float(now_sec)
        dt = (
            0.0
            if self.last_update_sec is None
            else clamp(now - self.last_update_sec, 0.0, 0.2)
        )
        self.last_update_sec = now
        if not armed:
            self.mode = "NORMAL"
            self.clear_started_sec = None
        elif self.mode == "NORMAL":
            if int(front_count) >= self.config.front_trigger:
                self.mode = "AVOID_RIGHT"
        elif self.mode == "AVOID_RIGHT":
            if int(behind_count) >= self.config.behind_passed_trigger:
                self.mode = "PASS_LEFT"
                self.clear_started_sec = None
            elif int(front_count) == 0 and int(behind_count) == 0:
                if self.clear_started_sec is None:
                    self.clear_started_sec = now
                elif now - self.clear_started_sec >= self.config.clear_reset_sec:
                    self.mode = "RETURN_CENTER"
            else:
                self.clear_started_sec = None
        elif self.mode == "PASS_LEFT":
            if int(behind_count) >= self.config.behind_return_trigger:
                self.mode = "RETURN_CENTER"
        elif self.mode == "RETURN_CENTER":
            if abs(self.current_offset_m) <= self.config.return_deadband_m:
                self.mode = "NORMAL"

        if self.mode == "AVOID_RIGHT":
            target = -self.config.right_offset_m
        elif self.mode == "PASS_LEFT":
            target = self.config.left_offset_m
        else:
            target = 0.0
        maximum_change = max(0.0, self.config.offset_rate_mps * dt)
        self.current_offset_m += clamp(
            target - self.current_offset_m,
            -maximum_change,
            maximum_change,
        )
        speed_limit = (
            self.config.speed_limit_command
            if self.mode in {"AVOID_RIGHT", "PASS_LEFT", "RETURN_CENTER"}
            else None
        )
        return DynamicAvoidanceState(
            mode=self.mode,
            lateral_offset_m=self.current_offset_m,
            speed_limit_command=speed_limit,
        )
