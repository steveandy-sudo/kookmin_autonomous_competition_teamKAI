from __future__ import annotations

import math


_MEASURED_STEERING_COMMANDS = (
    -42.0, -40.0, -35.0, -30.0, -20.0, -10.0, 0.0,
    10.0, 20.0, 30.0, 35.0, 40.0, 42.0,
)
_MEASURED_CURVATURES_PER_M = (
    1.502435, 1.383494, 1.174860, 0.922781, 0.552809, 0.194230, 0.0,
    -0.556883, -0.959829, -1.369323, -1.601706, -1.853397, -1.939236,
)


def command_for_measured_curvature(curvature_per_m: float) -> float:
    """Invert the Gazebo bridge's measured steering response."""
    desired = float(curvature_per_m)
    if not math.isfinite(desired):
        return 0.0
    if desired >= _MEASURED_CURVATURES_PER_M[0]:
        return _MEASURED_STEERING_COMMANDS[0]
    if desired <= _MEASURED_CURVATURES_PER_M[-1]:
        return _MEASURED_STEERING_COMMANDS[-1]
    for index in range(len(_MEASURED_STEERING_COMMANDS) - 1):
        upper_curvature = _MEASURED_CURVATURES_PER_M[index]
        lower_curvature = _MEASURED_CURVATURES_PER_M[index + 1]
        if upper_curvature >= desired >= lower_curvature:
            fraction = (desired - upper_curvature) / (
                lower_curvature - upper_curvature
            )
            lower_command = _MEASURED_STEERING_COMMANDS[index]
            upper_command = _MEASURED_STEERING_COMMANDS[index + 1]
            return lower_command + fraction * (
                upper_command - lower_command
            )
    return 0.0


def cad_curve_rule_command(
    path_curvature_per_m: float,
    *,
    cross_track_error_m: float,
    heading_error_rad: float,
    cross_track_gain: float = 1.5,
    heading_gain: float = 1.5,
    maximum_command: float = 42.0,
) -> float:
    """Continuous path-curvature rule with Stanley-style pose feedback."""
    desired_curvature = (
        float(path_curvature_per_m)
        - float(heading_gain) * float(heading_error_rad)
        - float(cross_track_gain) * float(cross_track_error_m)
    )
    command = command_for_measured_curvature(desired_curvature)
    limit = abs(float(maximum_command))
    return max(-limit, min(limit, command))


def cad_pure_pursuit_curve_command(
    *,
    vehicle_x: float,
    vehicle_y: float,
    vehicle_yaw_rad: float,
    target_x: float,
    target_y: float,
    cross_track_error_m: float = 0.0,
    cross_track_gain: float = 0.0,
    maximum_command: float = 42.0,
) -> float:
    """Map a CAD lookahead point to a continuous measured steering command."""
    dx = float(target_x) - float(vehicle_x)
    dy = float(target_y) - float(vehicle_y)
    local_y = (
        -math.sin(float(vehicle_yaw_rad)) * dx
        + math.cos(float(vehicle_yaw_rad)) * dy
    )
    distance_sq = max(0.01, dx * dx + dy * dy)
    desired_curvature = (
        2.0 * local_y / distance_sq
        - float(cross_track_gain) * float(cross_track_error_m)
    )
    command = command_for_measured_curvature(desired_curvature)
    limit = abs(float(maximum_command))
    return max(-limit, min(limit, command))


class ContinuousCurveController:
    """Low-latency rate limiter for continuous high-speed curve steering."""

    def __init__(
        self,
        *,
        alpha: float = 0.65,
        rate_limit_command: float = 8.0,
        maximum_command: float = 42.0,
    ) -> None:
        self.alpha = min(1.0, max(0.0, float(alpha)))
        self.rate_limit = max(0.0, float(rate_limit_command))
        self.maximum_command = abs(float(maximum_command))
        if self.maximum_command <= 0.0:
            raise ValueError("maximum_command must be positive")
        self.value = 0.0

    def reset(self, value: float = 0.0) -> None:
        target = float(value)
        if not math.isfinite(target):
            target = 0.0
        self.value = max(
            -self.maximum_command,
            min(self.maximum_command, target),
        )

    def update(self, raw_angle_command: float) -> float:
        target = float(raw_angle_command)
        if not math.isfinite(target):
            target = 0.0
        target = max(
            -self.maximum_command,
            min(self.maximum_command, target),
        )
        filtered = self.alpha * target + (1.0 - self.alpha) * self.value
        delta = max(
            -self.rate_limit,
            min(self.rate_limit, filtered - self.value),
        )
        self.value = max(
            -self.maximum_command,
            min(self.maximum_command, self.value + delta),
        )
        if abs(self.value) < 1.0e-9:
            self.value = 0.0
        return self.value


def feedback_curve_candidate(
    camera_rule_command: float,
    *,
    route_command_sign: int,
    cross_track_error_m: float,
    heading_error_rad: float,
    cross_track_gain: float = 1.5,
    error_deadband: float = 0.08,
    minimum_command: float = 12.0,
) -> float:
    """Choose a continuous curve-rule sign from CAD-relative pose error."""
    combined_error = float(heading_error_rad) + float(cross_track_gain) * float(
        cross_track_error_m
    )
    if combined_error > abs(float(error_deadband)):
        command_sign = 1
    elif combined_error < -abs(float(error_deadband)):
        command_sign = -1
    else:
        command_sign = (
            1
            if route_command_sign > 0
            else -1
            if route_command_sign < 0
            else 0
        )
    if command_sign == 0:
        return float(camera_rule_command)
    magnitude = max(abs(float(camera_rule_command)), abs(float(minimum_command)))
    return command_sign * magnitude


class ThreeLevelCurveController:
    """Track a continuous curve command using only ``-lock, 0, +lock``.

    The accumulated-error quantizer preserves the continuous rule controller's
    average steering authority without holding full lock on every control
    frame. A sign reversal clears the residual so an S-curve can turn in to
    the opposite side immediately.
    """

    def __init__(
        self,
        *,
        full_lock_command: float = 42.0,
        zero_band_command: float = 2.0,
        duty_scale: float = 1.0,
        minimum_zero_frames: int = 1,
        maximum_zero_frames: int = 2,
        forced_pulse_min_command: float = 10.0,
        stable_pulse_min_command: float = 5.0,
        stable_pulse_frames: int = 2,
    ) -> None:
        self.full_lock = abs(float(full_lock_command))
        if self.full_lock <= 0.0:
            raise ValueError("full_lock_command must be positive")
        self.zero_band = min(
            self.full_lock,
            max(0.0, float(zero_band_command)),
        )
        self.duty_scale = min(1.0, max(0.0, float(duty_scale)))
        self.minimum_zero_frames = max(0, int(minimum_zero_frames))
        self.maximum_zero_frames = max(
            self.minimum_zero_frames,
            int(maximum_zero_frames),
        )
        self.forced_pulse_min_command = min(
            self.full_lock,
            max(self.zero_band, float(forced_pulse_min_command)),
        )
        self.stable_pulse_min_command = min(
            self.full_lock,
            max(self.zero_band, float(stable_pulse_min_command)),
        )
        self.stable_pulse_frames = max(1, int(stable_pulse_frames))
        self._residual = 0.0
        self._target_sign = 0
        self._consecutive_target_frames = 0
        self._stable_assist_used = False
        self._zero_frames_since_pulse = self.maximum_zero_frames

    def reset(self) -> None:
        self._residual = 0.0
        self._target_sign = 0
        self._consecutive_target_frames = 0
        self._stable_assist_used = False
        self._zero_frames_since_pulse = self.maximum_zero_frames

    def update(self, raw_angle_command: float) -> float:
        raw = float(raw_angle_command)
        if not math.isfinite(raw) or abs(raw) <= self.zero_band:
            self.reset()
            return 0.0

        target = max(-self.full_lock, min(self.full_lock, raw))
        target *= self.duty_scale
        target_sign = 1 if target > 0.0 else -1
        if self._target_sign not in (0, target_sign):
            self._residual = 0.0
            self._zero_frames_since_pulse = self.maximum_zero_frames
            self._consecutive_target_frames = 1
            self._stable_assist_used = False
        elif self._target_sign == target_sign:
            self._consecutive_target_frames += 1
        else:
            self._consecutive_target_frames = 1
        self._target_sign = target_sign

        self._residual += target / self.full_lock
        pulse_allowed = (
            self._zero_frames_since_pulse >= self.minimum_zero_frames
        )
        strong_pulse_due = (
            self._zero_frames_since_pulse >= self.maximum_zero_frames
            and abs(raw) > self.forced_pulse_min_command
        )
        stable_pulse_due = (
            self._zero_frames_since_pulse >= self.maximum_zero_frames
            and not self._stable_assist_used
            and self._consecutive_target_frames >= self.stable_pulse_frames
            and abs(raw) >= self.stable_pulse_min_command
        )
        pulse_due = strong_pulse_due or stable_pulse_due
        if (
            pulse_allowed
            and target > 0.0
            and (self._residual >= 0.5 or pulse_due)
        ):
            self._residual -= 1.0
            self._zero_frames_since_pulse = 0
            self._stable_assist_used = True
            return self.full_lock
        if (
            pulse_allowed
            and target < 0.0
            and (self._residual <= -0.5 or pulse_due)
        ):
            self._residual += 1.0
            self._zero_frames_since_pulse = 0
            self._stable_assist_used = True
            return -self.full_lock
        self._zero_frames_since_pulse += 1
        return 0.0


def quantize_three_level_curve_command(
    raw_angle_command: float,
    *,
    full_lock_command: float = 42.0,
    zero_band_command: float = 2.0,
) -> float:
    """Map a curve-controller output to exactly ``-full_lock, 0, +full_lock``."""
    full_lock = abs(float(full_lock_command))
    if full_lock <= 0.0:
        raise ValueError("full_lock_command must be positive")
    zero_band = min(full_lock, max(0.0, float(zero_band_command)))
    raw = float(raw_angle_command)
    if not math.isfinite(raw) or abs(raw) <= zero_band:
        return 0.0
    return math.copysign(full_lock, raw)
