"""Select a safe cone-driving candidate for the single Final Driver.

This module does not publish a motor command.  It converts the existing
``/my_rule/cone_cmd`` contract into a numeric steering and speed candidate
only while Mission Manager explicitly selects cone rule driving.
"""

from collections.abc import Sequence
from dataclasses import dataclass
import math

from track_drive.integration.lidar_cone_adapter import (
    parse_cone_command,
    source_sample_is_fresh,
)
from track_drive.mission.mission_types import MissionDecision
from track_drive.mission.states import ControlMode


@dataclass(frozen=True)
class ConeInputConfig:
    """Validation limits matching the vehicle-tested ``cone_node`` output."""

    source_timeout_sec: float = 0.2
    minimum_confidence: float = 0.2
    max_steering_angle_deg: float = 26.0
    minimum_requested_speed: float = 9.5
    maximum_requested_speed: float = 21.0

    def __post_init__(self) -> None:
        values = (
            self.source_timeout_sec,
            self.minimum_confidence,
            self.max_steering_angle_deg,
            self.minimum_requested_speed,
            self.maximum_requested_speed,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("cone input configuration must be finite")
        if self.source_timeout_sec < 0.0:
            raise ValueError("source_timeout_sec must be non-negative")
        if not 0.0 <= self.minimum_confidence < 1.0:
            raise ValueError("minimum_confidence must be in [0, 1)")
        if self.max_steering_angle_deg <= 0.0:
            raise ValueError("max_steering_angle_deg must be positive")
        if self.minimum_requested_speed <= 0.0:
            raise ValueError("minimum_requested_speed must be positive")
        if self.maximum_requested_speed < self.minimum_requested_speed:
            raise ValueError(
                "maximum_requested_speed must not be below the minimum"
            )


@dataclass(frozen=True)
class ConeDriveCandidate:
    """Numeric cone command that a single Final Driver may apply later."""

    steering_angle_deg: float
    requested_speed: float
    confidence: float
    steering_held: bool = False


def _decision_selects_cone(decision: MissionDecision) -> bool:
    return (
        decision.control_mode is ControlMode.CONE_DRIVE_RULE
        and decision.selected_source == "cone_rule"
        and decision.speed_profile == "cone"
    )


def select_cone_drive_candidate(
    *,
    decision: MissionDecision,
    command_values: Sequence[float] | None,
    command_receive_sec: float | None,
    now_sec: float,
    config: ConeInputConfig = ConeInputConfig(),
) -> ConeDriveCandidate | None:
    """Return a usable cone candidate, or ``None`` when it must not be used.

    ``requested_speed`` comes directly from ``/my_rule/cone_cmd.data[1]``.
    A positive value below the tested cone minimum is rejected instead of
    being increased.  Values above the configured maximum are capped.
    """

    if decision.stop_required or not _decision_selects_cone(decision):
        return None

    if not source_sample_is_fresh(
        now_sec=now_sec,
        last_receive_sec=command_receive_sec,
        timeout_sec=config.source_timeout_sec,
    ):
        return None

    if command_values is None:
        return None
    command = parse_cone_command(command_values)
    if command is None:
        return None

    values = (command.angle, command.speed, command.confidence)
    if not all(math.isfinite(value) for value in values):
        return None
    if abs(command.angle) > config.max_steering_angle_deg:
        return None
    if command.confidence <= config.minimum_confidence:
        return None
    if command.speed < config.minimum_requested_speed:
        return None

    return ConeDriveCandidate(
        steering_angle_deg=command.angle,
        requested_speed=min(
            command.speed,
            config.maximum_requested_speed,
        ),
        confidence=command.confidence,
    )


class ConeInputSelector:
    """Remember the last valid cone steering while cone mode stays selected.

    Invalid or stale input never updates the memory.  Once one valid command
    has been accepted, the selector keeps its steering indefinitely and asks
    for the configured minimum cone speed until a new valid command arrives.
    Leaving cone mode or receiving a stop decision clears that memory.
    """

    def __init__(self, config: ConeInputConfig = ConeInputConfig()):
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
        command_values: Sequence[float] | None,
        command_receive_sec: float | None,
        now_sec: float,
    ) -> ConeDriveCandidate | None:
        """Select a fresh candidate or hold the previous valid steering."""

        if decision.stop_required or not _decision_selects_cone(decision):
            self.reset()
            return None

        fresh_candidate = select_cone_drive_candidate(
            decision=decision,
            command_values=command_values,
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

        return ConeDriveCandidate(
            steering_angle_deg=self._last_valid_steering_angle_deg,
            requested_speed=self.config.minimum_requested_speed,
            confidence=0.0,
            steering_held=True,
        )
