"""Pure steering-speed and camera obstacle-offset rules."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np


class ObstacleSide(str, Enum):
    """Obstacle position relative to the yellow divider."""

    LEFT = "left"
    RIGHT = "right"


def adaptive_speed_for_steering(
    angle_command: float,
    *,
    straight_speed_command: float,
    turn_speed_command: float,
    slowdown_start_angle_command: float,
    full_slowdown_angle_command: float,
    exponent: float = 1.0,
) -> float:
    """Hold the speed cap on small steering, then slow down toward full lock."""
    maximum = max(0.0, float(straight_speed_command))
    minimum = min(maximum, max(0.0, float(turn_speed_command)))
    full_angle = max(1.0e-6, abs(float(full_slowdown_angle_command)))
    start_angle = min(
        full_angle,
        max(0.0, abs(float(slowdown_start_angle_command))),
    )
    steering = abs(float(angle_command))
    if steering <= start_angle:
        return maximum
    if start_angle >= full_angle:
        return minimum
    fraction = min(1.0, (steering - start_angle) / (full_angle - start_angle))
    fraction = fraction ** max(0.05, float(exponent))
    return maximum + fraction * (minimum - maximum)


def avoidance_offset_for_obstacle(
    side: ObstacleSide,
    shift_m: float,
) -> float:
    """Return a signed path offset that moves away from the obstacle."""
    shift = max(0.0, float(shift_m))
    return -shift if side == ObstacleSide.LEFT else shift


def fit_yellow_reference(mask: np.ndarray, residual_px: float = 6.0):
    """Fit an extended x(y) yellow divider from sparse mask fragments."""
    binary = np.asarray(mask) > 0
    rows, columns = np.nonzero(binary)
    if rows.size == 0:
        return None
    unique_rows = np.unique(rows)
    row_centers = np.asarray(
        [np.median(columns[rows == row]) for row in unique_rows],
        dtype=np.float64,
    )
    row_values = unique_rows.astype(np.float64)
    if row_values.size == 1:
        return 0.0, float(row_centers[0])
    keep = np.ones(row_values.shape, dtype=bool)
    coefficients = np.polyfit(row_values, row_centers, 1)
    for _ in range(2):
        predicted = np.polyval(coefficients, row_values)
        residuals = np.abs(row_centers - predicted)
        next_keep = residuals <= max(1.0, float(residual_px))
        if np.count_nonzero(next_keep) < 2:
            break
        keep = next_keep
        coefficients = np.polyfit(row_values[keep], row_centers[keep], 1)
    return float(coefficients[0]), float(coefficients[1])


def obstacle_side_from_reference(
    *,
    object_center_x: float,
    object_bottom_y: float,
    image_width: int,
    image_height: int,
    yellow_coefficients,
    yellow_width: int,
    yellow_height: int,
) -> tuple[ObstacleSide, str]:
    """Classify the obstacle side using yellow first, image centre second."""
    width = max(1, int(image_width))
    height = max(1, int(image_height))
    object_ratio_x = float(object_center_x) / float(width)
    object_ratio_y = float(object_bottom_y) / float(height)
    if (
        yellow_coefficients is not None
        and int(yellow_width) > 0
        and int(yellow_height) > 0
    ):
        yellow_y = object_ratio_y * float(yellow_height)
        slope, intercept = yellow_coefficients
        divider_ratio_x = (
            float(slope) * yellow_y + float(intercept)
        ) / float(yellow_width)
        basis = "yellow"
    else:
        divider_ratio_x = 0.5
        basis = "image_center"
    side = (
        ObstacleSide.LEFT
        if object_ratio_x <= divider_ratio_x
        else ObstacleSide.RIGHT
    )
    return side, basis


@dataclass
class ObstacleOffsetLatch:
    """Latch the first observed side until detections are absent long enough."""

    shift_m: float = 0.20
    release_delay_sec: float = 2.0
    side: ObstacleSide | None = None
    last_seen_sec: float = -math.inf

    @property
    def active(self) -> bool:
        return self.side is not None

    @property
    def offset_m(self) -> float:
        if self.side is None:
            return 0.0
        return avoidance_offset_for_obstacle(self.side, self.shift_m)

    def observe(self, side: ObstacleSide, now_sec: float) -> bool:
        """Latch immediately; return true only when avoidance first starts."""
        started = self.side is None
        if started:
            self.side = side
        self.last_seen_sec = float(now_sec)
        return started

    def update(self, now_sec: float) -> bool:
        """Release after the configured no-detection interval."""
        if self.side is None:
            return False
        if float(now_sec) - self.last_seen_sec < self.release_delay_sec:
            return False
        self.side = None
        return True
