"""Pure helpers for measuring IMU yaw sign, scale, and continuity."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import statistics


def normalize_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


@dataclass
class YawSample:
    stamp_sec: float
    wrapped_rad: float
    unwrapped_rad: float
    relative_rad: float
    gyro_z_rad_s: float


@dataclass
class YawTracker:
    averaging_window_sec: float = 2.0
    last_wrapped_rad: float | None = None
    unwrapped_rad: float = 0.0
    zero_unwrapped_rad: float = 0.0
    first_stamp_sec: float | None = None
    last_stamp_sec: float | None = None
    maximum_gap_sec: float = 0.0
    sample_count: int = 0
    recent_samples: deque[YawSample] = field(default_factory=deque)

    def update(
        self,
        stamp_sec: float,
        wrapped_rad: float,
        gyro_z_rad_s: float,
    ) -> YawSample:
        stamp_sec = float(stamp_sec)
        wrapped_rad = normalize_angle(wrapped_rad)
        if self.last_wrapped_rad is None:
            self.unwrapped_rad = wrapped_rad
            self.zero_unwrapped_rad = wrapped_rad
            self.first_stamp_sec = stamp_sec
        else:
            self.unwrapped_rad += normalize_angle(
                wrapped_rad - self.last_wrapped_rad
            )
            if self.last_stamp_sec is not None:
                self.maximum_gap_sec = max(
                    self.maximum_gap_sec,
                    stamp_sec - self.last_stamp_sec,
                )

        self.last_wrapped_rad = wrapped_rad
        self.last_stamp_sec = stamp_sec
        self.sample_count += 1
        sample = YawSample(
            stamp_sec=stamp_sec,
            wrapped_rad=wrapped_rad,
            unwrapped_rad=self.unwrapped_rad,
            relative_rad=self.unwrapped_rad - self.zero_unwrapped_rad,
            gyro_z_rad_s=float(gyro_z_rad_s),
        )
        self.recent_samples.append(sample)
        cutoff = stamp_sec - max(0.0, self.averaging_window_sec)
        while (
            len(self.recent_samples) > 1
            and self.recent_samples[0].stamp_sec < cutoff
        ):
            self.recent_samples.popleft()
        return sample

    def set_zero(self) -> None:
        self.zero_unwrapped_rad = self.unwrapped_rad
        self.recent_samples.clear()

    def marker_measurement(self) -> tuple[float, float, int]:
        values = [
            math.degrees(sample.relative_rad)
            for sample in self.recent_samples
        ]
        if not values:
            return math.nan, math.nan, 0
        median = float(statistics.median(values))
        deviation = (
            float(statistics.pstdev(values))
            if len(values) > 1
            else 0.0
        )
        return median, deviation, len(values)

    @property
    def sample_rate_hz(self) -> float:
        if (
            self.sample_count < 2
            or self.first_stamp_sec is None
            or self.last_stamp_sec is None
        ):
            return 0.0
        duration = self.last_stamp_sec - self.first_stamp_sec
        if duration <= 0.0:
            return 0.0
        return (self.sample_count - 1) / duration


def marker_error_limit_deg(expected_deg: float) -> float:
    return 3.0 if abs(float(expected_deg)) <= 90.0 else 5.0


def summarize_markers(markers: list[dict]) -> dict:
    measured_markers = [
        marker
        for marker in markers
        if math.isfinite(float(marker["measured_deg"]))
    ]
    nonzero = [
        marker
        for marker in measured_markers
        if abs(float(marker["expected_deg"])) > 1.0e-6
    ]
    numerator = sum(
        float(marker["expected_deg"]) * float(marker["measured_deg"])
        for marker in nonzero
    )
    denominator = sum(
        float(marker["measured_deg"]) ** 2 for marker in nonzero
    )
    scale = numerator / denominator if denominator > 1.0e-9 else None
    errors = [
        abs(
            float(marker["measured_deg"])
            - float(marker["expected_deg"])
        )
        for marker in measured_markers
    ]
    passed = bool(measured_markers) and all(
        abs(
            float(marker["measured_deg"])
            - float(marker["expected_deg"])
        )
        <= marker_error_limit_deg(float(marker["expected_deg"]))
        for marker in measured_markers
    )
    return {
        "marker_count": len(measured_markers),
        "maximum_absolute_error_deg": max(errors) if errors else None,
        "suggested_yaw_scale": scale,
        "suggested_yaw_sign": (
            1.0 if scale is None or scale >= 0.0 else -1.0
        ),
        "all_markers_within_limits": passed,
    }
