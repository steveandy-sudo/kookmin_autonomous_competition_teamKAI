"""Select Lane Fallback steering for the future single Final Driver.

The module is ROS-independent and never publishes a motor command.  Steering
stays in physical degrees; unit conversion belongs to the future Final Driver.
"""

from dataclasses import dataclass
import math

from track_drive.integration.lane_fallback_adapter import (
    lane_fallback_command_values_are_valid,
    lane_fallback_source_is_fresh,
)
from track_drive.mission.mission_types import MissionDecision
from track_drive.mission.states import ControlMode


@dataclass(frozen=True)
class LaneFallbackInputConfig:
    """Lane command limits and the required shared fallback speed."""

    fallback_speed: float
    command_timeout_sec: float = 0.2
    max_steering_angle_deg: float = 26.0

    def __post_init__(self) -> None:
        values = (
            self.fallback_speed,
            self.command_timeout_sec,
            self.max_steering_angle_deg,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("lane fallback input configuration must be finite")
        if self.command_timeout_sec < 0.0:
            raise ValueError("command_timeout_sec must be non-negative")
        if self.max_steering_angle_deg <= 0.0:
            raise ValueError("max_steering_angle_deg must be positive")


@dataclass(frozen=True)
class LaneFallbackDriveCandidate:
    """Physical steering and fallback speed selected for later conversion."""

    steering_angle_deg: float
    requested_speed: float
    steering_held: bool = False


def _decision_selects_lane_fallback(decision: MissionDecision) -> bool:
    return (
        decision.control_mode is ControlMode.LANE_FALLBACK
        and decision.selected_source == "lane_fallback"
        and decision.speed_profile == "fallback"
    )


def select_fresh_lane_fallback_candidate(
    *,
    decision: MissionDecision,
    steering_angle_deg: float | None,
    command_valid: bool,
    command_receive_sec: float | None,
    now_sec: float,
    config: LaneFallbackInputConfig,
) -> LaneFallbackDriveCandidate | None:
    """Return a fresh physical-angle candidate with shared fallback speed."""

    if decision.stop_required or not _decision_selects_lane_fallback(
        decision
    ):
        return None
    if steering_angle_deg is None:
        return None

    values_valid = lane_fallback_command_values_are_valid(
        steering_angle_deg=steering_angle_deg,
        command_valid=command_valid,
        max_steering_angle_deg=config.max_steering_angle_deg,
    )
    if not lane_fallback_source_is_fresh(
        now_sec=now_sec,
        last_receive_sec=command_receive_sec,
        timeout_sec=config.command_timeout_sec,
        source_values_valid=values_valid,
    ):
        return None

    return LaneFallbackDriveCandidate(
        steering_angle_deg=float(steering_angle_deg),
        requested_speed=config.fallback_speed,
    )


class LaneFallbackInputSelector:
    """Hold the last fallback steering while Mission Manager keeps the mode."""

    def __init__(self, config: LaneFallbackInputConfig):
        self.config = config
        self._last_valid_steering_angle_deg: float | None = None

    @property
    def last_valid_steering_angle_deg(self) -> float | None:
        return self._last_valid_steering_angle_deg

    def reset(self) -> None:
        self._last_valid_steering_angle_deg = None

    def select(
        self,
        *,
        decision: MissionDecision,
        steering_angle_deg: float | None,
        command_valid: bool,
        command_receive_sec: float | None,
        now_sec: float,
    ) -> LaneFallbackDriveCandidate | None:
        """Select a fresh command or hold the last physical steering angle."""

        if decision.stop_required or not _decision_selects_lane_fallback(
            decision
        ):
            self.reset()
            return None

        fresh_candidate = select_fresh_lane_fallback_candidate(
            decision=decision,
            steering_angle_deg=steering_angle_deg,
            command_valid=command_valid,
            command_receive_sec=command_receive_sec,
            now_sec=now_sec,
            config=self.config,
        )
        if fresh_candidate is not None:
            self._last_valid_steering_angle_deg = (
                fresh_candidate.steering_angle_deg
            )
            return fresh_candidate

        if self._last_valid_steering_angle_deg is None:
            return None

        return LaneFallbackDriveCandidate(
            steering_angle_deg=self._last_valid_steering_angle_deg,
            requested_speed=self.config.fallback_speed,
            steering_held=True,
        )
