"""Pure Pursuit steering for the YOLO Centerline fallback source."""

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True)
class LaneFallbackParameters:
    """Vehicle geometry and preview distances for Lane Fallback."""

    wheelbase_m: float = 0.33
    near_lookahead_m: float = 0.70
    far_preview_distance_m: float = 0.75
    max_lookahead_m: float = 1.45
    far_preview_weight: float = 0.65
    steering_gain: float = 1.0
    max_steering_angle_deg: float = 26.0
    minimum_path_distance_m: float = 0.70
    minimum_point_count: int = 3

    def __post_init__(self) -> None:
        numeric_values = (
            self.wheelbase_m,
            self.near_lookahead_m,
            self.far_preview_distance_m,
            self.max_lookahead_m,
            self.far_preview_weight,
            self.steering_gain,
            self.max_steering_angle_deg,
            self.minimum_path_distance_m,
        )
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError("Lane Fallback parameters must be finite")
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        if self.near_lookahead_m <= 0.0:
            raise ValueError("near_lookahead_m must be positive")
        if self.far_preview_distance_m < 0.0:
            raise ValueError("far_preview_distance_m must be non-negative")
        if self.max_lookahead_m < self.near_lookahead_m:
            raise ValueError(
                "max_lookahead_m must not be shorter than near_lookahead_m"
            )
        if not 0.0 <= self.far_preview_weight <= 1.0:
            raise ValueError("far_preview_weight must be between 0 and 1")
        if self.steering_gain < 0.0:
            raise ValueError("steering_gain must be non-negative")
        if self.max_steering_angle_deg <= 0.0:
            raise ValueError("max_steering_angle_deg must be positive")
        if self.minimum_path_distance_m < self.near_lookahead_m:
            raise ValueError(
                "minimum_path_distance_m must reach near_lookahead_m"
            )
        if self.minimum_point_count < 2:
            raise ValueError("minimum_point_count must be at least two")


@dataclass(frozen=True)
class LaneFallbackSteering:
    """Physical steering candidate produced from one Centerline."""

    steering_angle_deg: float
    valid: bool


INVALID_STEERING = LaneFallbackSteering(
    steering_angle_deg=0.0,
    valid=False,
)


def _physical_steering_at_distance(
    path: Sequence[tuple[float, float]],
    *,
    lookahead_m: float,
    wheelbase_m: float,
) -> float:
    target_x, target_y = path[-1]
    for point_x, point_y in path:
        if math.hypot(point_x, point_y) > lookahead_m:
            target_x, target_y = point_x, point_y
            break

    target_distance = max(
        math.hypot(target_x, target_y),
        1.0e-6,
    )
    target_heading = math.atan2(target_y, target_x)
    wheel_angle_rad = math.atan2(
        2.0 * wheelbase_m * math.sin(target_heading),
        target_distance,
    )

    # Centerline y is left-positive. The physical Xycar steering convention
    # used by cone_node is negative for a left turn.
    return -math.degrees(wheel_angle_rad)


def compute_lane_fallback_steering(
    points: Sequence[Any],
    parameters: LaneFallbackParameters,
) -> LaneFallbackSteering:
    """Calculate a bounded physical steering angle from Centerline points."""

    path: list[tuple[float, float]] = []
    for point in points:
        try:
            point_x = float(point.x)
            point_y = float(point.y)
            point_z = float(point.z)
        except (AttributeError, TypeError, ValueError):
            return INVALID_STEERING
        if not all(
            math.isfinite(value)
            for value in (point_x, point_y, point_z)
        ):
            return INVALID_STEERING
        if point_x > 0.0:
            path.append((point_x, point_y))

    if len(path) < parameters.minimum_point_count:
        return INVALID_STEERING
    path.sort(key=lambda point: point[0])

    available_distance = max(math.hypot(x, y) for x, y in path)
    if available_distance < parameters.minimum_path_distance_m:
        return INVALID_STEERING

    near_angle = _physical_steering_at_distance(
        path,
        lookahead_m=parameters.near_lookahead_m,
        wheelbase_m=parameters.wheelbase_m,
    )
    far_lookahead = min(
        parameters.max_lookahead_m,
        parameters.near_lookahead_m
        + parameters.far_preview_distance_m,
    )
    far_angle = _physical_steering_at_distance(
        path,
        lookahead_m=far_lookahead,
        wheelbase_m=parameters.wheelbase_m,
    )

    angle = parameters.steering_gain * (
        (1.0 - parameters.far_preview_weight) * near_angle
        + parameters.far_preview_weight * far_angle
    )
    limit = parameters.max_steering_angle_deg
    bounded_angle = max(-limit, min(limit, angle))
    return LaneFallbackSteering(
        steering_angle_deg=float(bounded_angle),
        valid=True,
    )
