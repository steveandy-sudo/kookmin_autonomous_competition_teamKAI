"""State and command selection for the interactive motor output gate."""

from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(float(value), float(lower)), float(upper))


def steering_speed_limit(
    angle_command: float,
    *,
    speed_cap_command: float,
    turn_speed_command: float,
    slowdown_start_angle_command: float,
    full_slowdown_angle_command: float,
) -> float:
    """Hold the speed cap through small steering, then slow toward full lock."""
    speed_cap = max(0.0, float(speed_cap_command))
    turn_speed = min(speed_cap, max(0.0, float(turn_speed_command)))
    start_angle = max(0.0, abs(float(slowdown_start_angle_command)))
    full_angle = max(start_angle, abs(float(full_slowdown_angle_command)))
    steering = abs(float(angle_command))
    if steering <= start_angle:
        return speed_cap
    if steering >= full_angle or full_angle <= start_angle:
        return turn_speed
    ratio = (steering - start_angle) / (full_angle - start_angle)
    return speed_cap + ratio * (turn_speed - speed_cap)


@dataclass(frozen=True)
class SpaceDriveOutput:
    angle_command: float
    speed_command: float
    reason: str


class SpaceDriveGateController:
    """Latch RUN/STOP with Space and apply a fixed test speed when valid."""

    def __init__(
        self,
        *,
        speed_command: float,
        maximum_speed_command: float = 30.0,
        maximum_abs_angle_command: float = 42.0,
        steering_only: bool = False,
        adaptive_steering_speed_enabled: bool = True,
        turn_speed_command: float = 12.0,
        slowdown_start_angle_command: float = 18.0,
        full_slowdown_angle_command: float = 42.0,
    ) -> None:
        self.maximum_speed_command = max(0.0, float(maximum_speed_command))
        self.maximum_abs_angle_command = max(
            0.0, float(maximum_abs_angle_command)
        )
        self.speed_command = clamp(
            speed_command, 0.0, self.maximum_speed_command
        )
        self.steering_only = bool(steering_only)
        self.adaptive_steering_speed_enabled = bool(
            adaptive_steering_speed_enabled
        )
        self.turn_speed_command = max(0.0, float(turn_speed_command))
        self.slowdown_start_angle_command = max(
            0.0, float(slowdown_start_angle_command)
        )
        self.full_slowdown_angle_command = max(
            self.slowdown_start_angle_command,
            float(full_slowdown_angle_command),
        )
        self.armed = False

    def toggle(self) -> bool:
        self.armed = not self.armed
        return self.armed

    def stop(self) -> None:
        self.armed = False

    def command(
        self,
        *,
        candidate_fresh: bool,
        candidate_angle_command: float,
        candidate_speed_command: float,
    ) -> SpaceDriveOutput:
        if not self.armed:
            return SpaceDriveOutput(0.0, 0.0, "SPACE_STOP")
        if not candidate_fresh:
            return SpaceDriveOutput(0.0, 0.0, "CANDIDATE_STALE")
        if self.steering_only:
            return SpaceDriveOutput(
                clamp(
                    candidate_angle_command,
                    -self.maximum_abs_angle_command,
                    self.maximum_abs_angle_command,
                ),
                0.0,
                "SPACE_RUN",
            )
        if float(candidate_speed_command) <= 0.0:
            return SpaceDriveOutput(0.0, 0.0, "SELECTOR_STOP")
        angle = clamp(
            candidate_angle_command,
            -self.maximum_abs_angle_command,
            self.maximum_abs_angle_command,
        )
        speed = min(self.speed_command, float(candidate_speed_command))
        if self.adaptive_steering_speed_enabled:
            speed = min(
                speed,
                steering_speed_limit(
                    angle,
                    speed_cap_command=speed,
                    turn_speed_command=self.turn_speed_command,
                    slowdown_start_angle_command=(
                        self.slowdown_start_angle_command
                    ),
                    full_slowdown_angle_command=(
                        self.full_slowdown_angle_command
                    ),
                ),
            )
        return SpaceDriveOutput(angle, speed, "SPACE_RUN")
