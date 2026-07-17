from __future__ import annotations

from dataclasses import dataclass
import math

from xycar_rl.track_geometry import TrackProjection


@dataclass(frozen=True)
class RewardWeights:
    progress: float = 8.0
    cross_track: float = 1.5
    heading: float = 0.8
    steering_rate: float = 0.05
    steering_magnitude: float = 0.01
    safe_speed: float = 0.30
    unsafe_speed: float = 0.80
    time_efficiency: float = 0.01
    large_oscillation: float = 0.20
    safe_cross_track_m: float = 0.12
    safe_heading_rad: float = math.radians(12.0)
    large_steering_threshold: float = 0.18
    straight_curvature_threshold: float = 0.35
    reverse_progress: float = 4.0
    collision: float = 50.0
    off_track: float = 30.0
    stuck: float = 10.0
    lap_complete: float = 20.0


@dataclass(frozen=True)
class RewardBreakdown:
    total: float
    progress: float
    cross_track: float
    heading: float
    steering_rate: float
    steering_magnitude: float
    safe_speed: float
    unsafe_speed: float
    time_efficiency: float
    large_oscillation: float
    terminal: float


def large_steering_oscillation(
    steering_history: tuple[float, ...] | list[float],
    *,
    threshold: float = 0.18,
    minimum_reversals: int = 2,
) -> float:
    """Return amplitude only for repeated large left-right-left motion."""
    strong = [
        float(value)
        for value in steering_history
        if abs(float(value)) >= max(0.0, float(threshold))
    ]
    if len(strong) < 3:
        return 0.0
    signs: list[int] = []
    for value in strong:
        sign = 1 if value > 0.0 else -1
        if not signs or sign != signs[-1]:
            signs.append(sign)
    reversals = len(signs) - 1
    if reversals < max(1, int(minimum_reversals)):
        return 0.0
    positive_peak = max((value for value in strong if value > 0.0), default=0.0)
    negative_peak = max((-value for value in strong if value < 0.0), default=0.0)
    bilateral_amplitude = 2.0 * min(positive_peak, negative_peak)
    return bilateral_amplitude * (reversals - minimum_reversals + 1)


def calculate_reward(
    *,
    projection: TrackProjection,
    progress_delta_m: float,
    steering_norm: float,
    previous_steering_norm: float,
    linear_speed_mps: float = 0.0,
    track_curvature: float = 0.0,
    steering_history: tuple[float, ...] | list[float] = (),
    collision: bool = False,
    off_track: bool = False,
    stuck: bool = False,
    lap_complete: bool = False,
    weights: RewardWeights = RewardWeights(),
) -> RewardBreakdown:
    forward_progress = max(0.0, float(progress_delta_m))
    reverse_progress = max(0.0, -float(progress_delta_m))
    progress_term = (
        weights.progress * forward_progress
        - weights.reverse_progress * reverse_progress
    )
    cross_track_term = -weights.cross_track * abs(
        projection.cross_track_error_m
    )
    heading_term = -weights.heading * abs(projection.heading_error_rad) / math.pi
    steering_rate_term = -weights.steering_rate * abs(
        float(steering_norm) - float(previous_steering_norm)
    )
    steering_magnitude_term = -weights.steering_magnitude * abs(
        float(steering_norm)
    )
    cross_track_ratio = abs(projection.cross_track_error_m) / max(
        1.0e-6, weights.safe_cross_track_m
    )
    heading_ratio = abs(projection.heading_error_rad) / max(
        1.0e-6, weights.safe_heading_rad
    )
    safe_factor = math.exp(
        -0.5 * (cross_track_ratio * cross_track_ratio + heading_ratio * heading_ratio)
    )
    forward_speed = max(0.0, float(linear_speed_mps))
    safe_speed_term = weights.safe_speed * forward_speed * safe_factor
    unsafe_speed_term = -weights.unsafe_speed * forward_speed * (1.0 - safe_factor)
    time_efficiency_term = -weights.time_efficiency
    oscillation_measure = 0.0
    if abs(float(track_curvature)) <= weights.straight_curvature_threshold:
        oscillation_measure = large_steering_oscillation(
            steering_history,
            threshold=weights.large_steering_threshold,
        )
    large_oscillation_term = -weights.large_oscillation * oscillation_measure
    terminal_term = 0.0
    terminal_term -= weights.collision if collision else 0.0
    terminal_term -= weights.off_track if off_track else 0.0
    terminal_term -= weights.stuck if stuck else 0.0
    terminal_term += weights.lap_complete if lap_complete else 0.0
    total = (
        progress_term
        + cross_track_term
        + heading_term
        + steering_rate_term
        + steering_magnitude_term
        + safe_speed_term
        + unsafe_speed_term
        + time_efficiency_term
        + large_oscillation_term
        + terminal_term
    )
    return RewardBreakdown(
        total=float(total),
        progress=float(progress_term),
        cross_track=float(cross_track_term),
        heading=float(heading_term),
        steering_rate=float(steering_rate_term),
        steering_magnitude=float(steering_magnitude_term),
        safe_speed=float(safe_speed_term),
        unsafe_speed=float(unsafe_speed_term),
        time_efficiency=float(time_efficiency_term),
        large_oscillation=float(large_oscillation_term),
        terminal=float(terminal_term),
    )
