"""State and command selection for the interactive motor output gate."""

from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(float(value), float(lower)), float(upper))


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
    ) -> None:
        self.maximum_speed_command = max(0.0, float(maximum_speed_command))
        self.maximum_abs_angle_command = max(
            0.0, float(maximum_abs_angle_command)
        )
        self.speed_command = clamp(
            speed_command, 0.0, self.maximum_speed_command
        )
        self.steering_only = bool(steering_only)
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
        return SpaceDriveOutput(
            clamp(
                candidate_angle_command,
                -self.maximum_abs_angle_command,
                self.maximum_abs_angle_command,
            ),
            min(self.speed_command, float(candidate_speed_command)),
            "SPACE_RUN",
        )
