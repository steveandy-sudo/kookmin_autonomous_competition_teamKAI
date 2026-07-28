"""Pure control math for global-path and dynamic-vehicle rule driving."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .grid_planner import Point


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(float(minimum), min(float(maximum), float(value)))


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(float(angle_rad)), math.cos(float(angle_rad)))


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


@dataclass(frozen=True)
class StanleyPathCommand:
    curvature_per_m: float
    target_index: int
    cross_track_error_m: float
    heading_error_rad: float
    path_curvature_per_m: float
    steering_angle_rad: float


def _path_index_at_distance(
    points: Sequence[Point],
    start_index: int,
    distance_m: float,
    *,
    direction: int,
    closed: bool,
) -> int:
    if not points:
        raise ValueError("path is empty")
    direction = 1 if int(direction) >= 0 else -1
    target = max(0.0, float(distance_m))
    travelled = 0.0
    index = int(start_index)
    maximum_steps = len(points) if closed else len(points) - 1
    for _ in range(maximum_steps):
        following = index + direction
        if closed:
            following %= len(points)
        elif following < 0 or following >= len(points):
            break
        travelled += math.hypot(
            points[following][0] - points[index][0],
            points[following][1] - points[index][1],
        )
        index = following
        if travelled >= target:
            break
    return index


def _path_geometry(
    points: Sequence[Point],
    index: int,
    *,
    heading_window_m: float,
    curvature_window_m: float,
    closed: bool,
) -> tuple[float, float]:
    heading_half_window = max(0.05, float(heading_window_m) * 0.5)
    heading_before = _path_index_at_distance(
        points,
        index,
        heading_half_window,
        direction=-1,
        closed=closed,
    )
    heading_after = _path_index_at_distance(
        points,
        index,
        heading_half_window,
        direction=1,
        closed=closed,
    )
    heading = math.atan2(
        points[heading_after][1] - points[heading_before][1],
        points[heading_after][0] - points[heading_before][0],
    )

    curvature_half_window = max(0.08, float(curvature_window_m) * 0.5)
    before = _path_index_at_distance(
        points,
        index,
        curvature_half_window,
        direction=-1,
        closed=closed,
    )
    after = _path_index_at_distance(
        points,
        index,
        curvature_half_window,
        direction=1,
        closed=closed,
    )
    first = points[before]
    middle = points[index]
    last = points[after]
    first_to_middle = math.hypot(
        middle[0] - first[0], middle[1] - first[1]
    )
    middle_to_last = math.hypot(
        last[0] - middle[0], last[1] - middle[1]
    )
    first_to_last = math.hypot(last[0] - first[0], last[1] - first[1])
    denominator = first_to_middle * middle_to_last * first_to_last
    if denominator < 1.0e-8:
        curvature = 0.0
    else:
        cross = (
            (middle[0] - first[0]) * (last[1] - first[1])
            - (middle[1] - first[1]) * (last[0] - first[0])
        )
        curvature = 2.0 * cross / denominator
    return heading, curvature


def path_curvature_profile(
    points: Sequence[Point],
    *,
    closed: bool,
    curvature_window_m: float,
    smoothing_points: int = 0,
) -> tuple[float, ...]:
    """Calculate a robust curvature profile along a global path."""
    if len(points) < 3:
        return tuple(0.0 for _ in points)
    raw = [
        _path_geometry(
            points,
            index,
            heading_window_m=curvature_window_m,
            curvature_window_m=curvature_window_m,
            closed=closed,
        )[1]
        for index in range(len(points))
    ]
    radius = max(0, int(smoothing_points))
    if radius == 0:
        return tuple(raw)

    smoothed = []
    for index in range(len(raw)):
        values = []
        for offset in range(-radius, radius + 1):
            candidate = index + offset
            if closed:
                candidate %= len(raw)
            elif candidate < 0 or candidate >= len(raw):
                continue
            values.append(raw[candidate])
        # RMS preserves high-curvature braking demand around inflections,
        # where signed moving averages could incorrectly cancel to zero.
        magnitude = math.sqrt(
            sum(value * value for value in values) / len(values)
        )
        sign_source = raw[index]
        if abs(sign_source) < 1.0e-9:
            sign_source = sum(values)
        smoothed.append(math.copysign(magnitude, sign_source or 1.0))
    return tuple(smoothed)


def forward_backward_velocity_profile(
    points: Sequence[Point],
    curvatures_per_m: Sequence[float],
    *,
    closed: bool,
    maximum_speed_mps: float,
    maximum_lateral_accel_mps2: float,
    maximum_accel_mps2: float,
    maximum_decel_mps2: float,
) -> tuple[float, ...]:
    """Build a curvature-limited velocity profile with accel/brake passes."""
    if len(points) != len(curvatures_per_m):
        raise ValueError("path and curvature profile lengths must match")
    if not points:
        return ()

    maximum_speed = max(0.0, float(maximum_speed_mps))
    lateral_accel = max(1.0e-3, float(maximum_lateral_accel_mps2))
    acceleration = max(1.0e-3, float(maximum_accel_mps2))
    deceleration = max(1.0e-3, float(maximum_decel_mps2))
    velocities = [
        min(
            maximum_speed,
            math.sqrt(lateral_accel / max(1.0e-6, abs(curvature))),
        )
        for curvature in curvatures_per_m
    ]
    count = len(points)
    if count == 1:
        return tuple(velocities)

    segment_count = count if closed else count - 1
    segment_lengths = [
        max(
            1.0e-4,
            math.hypot(
                points[(index + 1) % count][0] - points[index][0],
                points[(index + 1) % count][1] - points[index][1],
            ),
        )
        for index in range(segment_count)
    ]

    # Repeating both passes lets a braking or acceleration constraint
    # propagate across the start/end seam of a closed course.
    maximum_passes = max(2, count if closed else 2)
    for _ in range(maximum_passes):
        changed = False
        for index in range(segment_count):
            following = (index + 1) % count
            reachable = math.sqrt(
                velocities[index] ** 2
                + 2.0 * acceleration * segment_lengths[index]
            )
            if velocities[following] > reachable:
                velocities[following] = reachable
                changed = True
        for index in reversed(range(segment_count)):
            following = (index + 1) % count
            brakeable = math.sqrt(
                velocities[following] ** 2
                + 2.0 * deceleration * segment_lengths[index]
            )
            if velocities[index] > brakeable:
                velocities[index] = brakeable
                changed = True
        if not changed:
            break
    return tuple(velocities)


def minimum_profile_value_ahead(
    points: Sequence[Point],
    values: Sequence[float],
    start_index: int,
    distance_m: float,
    *,
    closed: bool,
) -> float:
    """Return the minimum profile value through a forward path interval."""
    if len(points) != len(values) or not points:
        raise ValueError("path and profile must have the same nonzero length")
    index = int(start_index) % len(points)
    minimum = float(values[index])
    target_distance = max(0.0, float(distance_m))
    travelled = 0.0
    maximum_steps = len(points) if closed else len(points) - index - 1
    for _ in range(maximum_steps):
        if travelled >= target_distance:
            break
        following = (index + 1) % len(points)
        travelled += math.hypot(
            points[following][0] - points[index][0],
            points[following][1] - points[index][1],
        )
        index = following
        minimum = min(minimum, float(values[index]))
    return minimum


def stanley_path_command(
    points: Sequence[Point],
    nearest_index: int,
    *,
    vehicle_x: float,
    vehicle_y: float,
    vehicle_yaw: float,
    speed_mps: float,
    lateral_offset_m: float,
    closed: bool,
    wheelbase_m: float,
    front_axle_offset_m: float,
    steering_delay_sec: float,
    stanley_gain: float,
    stanley_softening_mps: float,
    heading_gain: float,
    curvature_feedforward_gain: float,
    heading_window_m: float,
    heading_preview_m: float,
    curvature_window_m: float,
    curvature_preview_m: float,
    maximum_steering_angle_rad: float,
) -> StanleyPathCommand:
    if len(points) < 2:
        raise ValueError("path needs at least two points")

    speed = max(0.0, abs(float(speed_mps)))
    control_offset = max(
        0.0,
        float(front_axle_offset_m)
        + speed * max(0.0, float(steering_delay_sec)),
    )
    control_x = float(vehicle_x) + control_offset * math.cos(vehicle_yaw)
    control_y = float(vehicle_y) + control_offset * math.sin(vehicle_yaw)
    control_index = nearest_path_index(
        points,
        control_x,
        control_y,
        previous_index=nearest_index,
        closed=closed,
        search_back=12,
        search_ahead=60,
    )
    local_heading, _ = _path_geometry(
        points,
        control_index,
        heading_window_m=heading_window_m,
        curvature_window_m=curvature_window_m,
        closed=closed,
    )
    heading_index = lookahead_index(
        points,
        control_index,
        max(0.0, float(heading_preview_m)),
        closed=closed,
    )
    path_heading, _ = _path_geometry(
        points,
        heading_index,
        heading_window_m=heading_window_m,
        curvature_window_m=curvature_window_m,
        closed=closed,
    )
    reference = offset_path_point(
        points,
        control_index,
        lateral_offset_m,
        closed=closed,
    )
    left_normal_x = -math.sin(local_heading)
    left_normal_y = math.cos(local_heading)
    cross_track_error = (
        (control_x - reference[0]) * left_normal_x
        + (control_y - reference[1]) * left_normal_y
    )
    heading_error = normalize_angle(path_heading - float(vehicle_yaw))

    preview_distance = max(
        0.0,
        float(curvature_preview_m) + speed * max(0.0, steering_delay_sec),
    )
    preview_index = lookahead_index(
        points,
        control_index,
        preview_distance,
        closed=closed,
    )
    _, path_curvature = _path_geometry(
        points,
        preview_index,
        heading_window_m=heading_window_m,
        curvature_window_m=curvature_window_m,
        closed=closed,
    )

    wheelbase = max(0.05, float(wheelbase_m))
    feedforward = math.atan(
        wheelbase * float(curvature_feedforward_gain) * path_curvature
    )
    cross_track = -math.atan2(
        max(0.0, float(stanley_gain)) * cross_track_error,
        speed + max(0.01, float(stanley_softening_mps)),
    )
    steering_angle = (
        feedforward + float(heading_gain) * heading_error + cross_track
    )
    maximum_angle = max(0.05, abs(float(maximum_steering_angle_rad)))
    steering_angle = clamp(
        steering_angle, -maximum_angle, maximum_angle
    )
    return StanleyPathCommand(
        curvature_per_m=math.tan(steering_angle) / wheelbase,
        target_index=preview_index,
        cross_track_error_m=cross_track_error,
        heading_error_rad=heading_error,
        path_curvature_per_m=path_curvature,
        steering_angle_rad=steering_angle,
    )


def filtered_steering_command(
    target_command: float,
    previous_command: float,
    *,
    dt_sec: float,
    rate_limit_command_per_sec: float,
    time_constant_sec: float,
) -> float:
    dt = clamp(dt_sec, 0.0, 0.25)
    if dt <= 0.0:
        return float(previous_command)
    time_constant = max(0.0, float(time_constant_sec))
    alpha = 1.0 if time_constant <= 1.0e-6 else dt / (time_constant + dt)
    filtered_target = float(previous_command) + alpha * (
        float(target_command) - float(previous_command)
    )
    maximum_change = max(0.0, float(rate_limit_command_per_sec)) * dt
    return float(previous_command) + clamp(
        filtered_target - float(previous_command),
        -maximum_change,
        maximum_change,
    )


def alignment_limited_speed_command(
    target_command: float,
    minimum_command: float,
    *,
    cross_track_error_m: float,
    heading_error_rad: float,
    cross_track_soft_m: float,
    cross_track_hard_m: float,
    heading_soft_rad: float,
    heading_hard_rad: float,
) -> float:
    target = float(target_command)
    minimum = min(target, float(minimum_command))

    def _error_ratio(error: float, soft: float, hard: float) -> float:
        soft_limit = max(0.0, float(soft))
        hard_limit = max(soft_limit + 1.0e-6, float(hard))
        return clamp(
            (abs(float(error)) - soft_limit)
            / (hard_limit - soft_limit),
            0.0,
            1.0,
        )

    misalignment = max(
        _error_ratio(
            cross_track_error_m,
            cross_track_soft_m,
            cross_track_hard_m,
        ),
        _error_ratio(
            heading_error_rad,
            heading_soft_rad,
            heading_hard_rad,
        ),
    )
    return target + (minimum - target) * misalignment


def rate_limited_speed_command(
    target_command: float,
    previous_command: float,
    *,
    dt_sec: float,
    acceleration_rate_command_per_sec: float,
    deceleration_rate_command_per_sec: float,
) -> float:
    dt = clamp(dt_sec, 0.0, 0.25)
    if dt <= 0.0:
        return float(previous_command)
    delta = float(target_command) - float(previous_command)
    rate = (
        max(0.0, float(acceleration_rate_command_per_sec))
        if delta >= 0.0
        else max(0.0, float(deceleration_rate_command_per_sec))
    )
    return float(previous_command) + clamp(delta, -rate * dt, rate * dt)


def minimum_effective_speed_command(
    limited_command: float,
    target_command: float,
    minimum_command: float,
) -> float:
    limited = float(limited_command)
    minimum = max(0.0, float(minimum_command))
    if float(target_command) >= minimum and 0.0 < limited < minimum:
        return minimum
    return limited


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
                elif (
                    now - self.clear_started_sec
                    >= self.config.clear_reset_sec
                ):
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
