"""Command-derived Ackermann odometry math for encoder-less testing."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class OdomState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def curvature_for_steering_command(
    command: float,
    commands: Sequence[float],
    curvatures: Sequence[float],
) -> float:
    if len(commands) != len(curvatures) or len(commands) < 2:
        raise ValueError("steering command and curvature maps must match")
    pairs = sorted(
        (float(control), float(curvature))
        for control, curvature in zip(commands, curvatures)
    )
    return float(
        np.interp(
            float(command),
            [pair[0] for pair in pairs],
            [pair[1] for pair in pairs],
        )
    )


def integrate_ackermann(
    state: OdomState,
    *,
    speed_mps: float,
    curvature_per_m: float,
    dt_sec: float,
) -> OdomState:
    dt = max(0.0, float(dt_sec))
    speed = float(speed_mps)
    yaw_rate = speed * float(curvature_per_m)
    return integrate_planar_velocity(
        state,
        speed_mps=speed,
        yaw_rate_rad_s=yaw_rate,
        dt_sec=dt,
    )


def integrate_planar_velocity(
    state: OdomState,
    *,
    speed_mps: float,
    yaw_rate_rad_s: float,
    dt_sec: float,
) -> OdomState:
    """Integrate forward speed and yaw rate with a midpoint SE(2) step."""
    dt = max(0.0, float(dt_sec))
    speed = float(speed_mps)
    yaw_rate = float(yaw_rate_rad_s)
    middle_yaw = state.yaw + yaw_rate * dt * 0.5
    return OdomState(
        x=state.x + speed * math.cos(middle_yaw) * dt,
        y=state.y + speed * math.sin(middle_yaw) * dt,
        yaw=normalize_angle(state.yaw + yaw_rate * dt),
    )


def first_order_response(
    current: float,
    target: float,
    *,
    dt_sec: float,
    time_constant_sec: float,
) -> float:
    """Apply a stable first-order actuator response for any timer period."""
    dt = max(0.0, float(dt_sec))
    tau = max(1e-4, float(time_constant_sec))
    response = 1.0 - math.exp(-dt / tau)
    return float(current) + (float(target) - float(current)) * response


def integrate_with_heading(
    state: OdomState,
    *,
    speed_mps: float,
    heading_rad: float,
    dt_sec: float,
) -> OdomState:
    """Integrate translation while taking heading from an external sensor."""
    dt = max(0.0, float(dt_sec))
    heading = normalize_angle(heading_rad)
    heading_delta = normalize_angle(heading - state.yaw)
    middle_yaw = normalize_angle(state.yaw + heading_delta * 0.5)
    distance = float(speed_mps) * dt
    return OdomState(
        x=state.x + distance * math.cos(middle_yaw),
        y=state.y + distance * math.sin(middle_yaw),
        yaw=heading,
    )
