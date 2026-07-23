"""Select NORMAL_IL steering and speed for the single Final Driver.

The module is ROS-independent and never publishes a motor command.  It reads
the vehicle-tested ``/il/policy_debug`` array contract:

``data[2]``: final Xycar steering command
``data[3]``: speed request calculated by the IL driving node
"""

from collections.abc import Sequence
from dataclasses import dataclass
import math

from track_drive.integration.drive_policy_adapter import (
    drive_policy_source_is_fresh,
    policy_debug_values_are_valid,
)
from track_drive.mission.mission_types import MissionDecision
from track_drive.mission.states import ControlMode


@dataclass(frozen=True)
class NormalIlInputConfig:
    """NORMAL_IL input limits and the required fallback speed.

    ``fallback_speed`` intentionally has no production default.  Its numeric
    value must be selected through vehicle testing before a ROS Final Driver
    is created.
    """

    fallback_speed: float
    source_timeout_sec: float = 0.5

    def __post_init__(self) -> None:
        if not math.isfinite(self.fallback_speed):
            raise ValueError("fallback_speed must be finite")
        if (
            not math.isfinite(self.source_timeout_sec)
            or self.source_timeout_sec < 0.0
        ):
            raise ValueError(
                "source_timeout_sec must be finite and non-negative"
            )


@dataclass(frozen=True)
class NormalIlDriveCandidate:
    """Numeric NORMAL_IL candidate for the single Final Driver."""

    steering_command: float
    requested_speed: float
    steering_held: bool = False


def _decision_selects_normal_il(
    decision: MissionDecision,
    *,
    allowed_speed_profiles: set[str],
) -> bool:
    return (
        decision.control_mode is ControlMode.NORMAL_IL
        and decision.selected_source == "drive_il"
        and decision.speed_profile in allowed_speed_profiles
    )


def select_fresh_normal_il_candidate(
    *,
    decision: MissionDecision,
    debug_values: Sequence[float] | None,
    debug_receive_sec: float | None,
    now_sec: float,
    config: NormalIlInputConfig,
) -> NormalIlDriveCandidate | None:
    """Return fresh ``data[2] + data[3]`` without speed recalculation.

    No numeric speed range or clamp is applied.  Only NaN/Inf is rejected at
    this final numeric-command boundary.
    """

    if decision.stop_required or not _decision_selects_normal_il(
        decision,
        allowed_speed_profiles={"normal"},
    ):
        return None
    if debug_values is None:
        return None

    values_valid = policy_debug_values_are_valid(debug_values)
    if not drive_policy_source_is_fresh(
        now_sec=now_sec,
        last_receive_sec=debug_receive_sec,
        timeout_sec=config.source_timeout_sec,
        source_values_valid=values_valid,
    ):
        return None

    try:
        steering_command = float(debug_values[2])
        requested_speed = float(debug_values[3])
    except (IndexError, TypeError, ValueError):
        return None
    if not math.isfinite(requested_speed):
        return None

    return NormalIlDriveCandidate(
        steering_command=steering_command,
        requested_speed=requested_speed,
    )


class NormalIlInputSelector:
    """Hold the last IL steering only while Mission Manager keeps IL selected."""

    def __init__(self, config: NormalIlInputConfig):
        self.config = config
        self._last_valid_steering_command: float | None = None

    @property
    def last_valid_steering_command(self) -> float | None:
        return self._last_valid_steering_command

    def reset(self) -> None:
        self._last_valid_steering_command = None

    def select(
        self,
        *,
        decision: MissionDecision,
        debug_values: Sequence[float] | None,
        debug_receive_sec: float | None,
        now_sec: float,
    ) -> NormalIlDriveCandidate | None:
        """Select a fresh IL pair or apply Mission Manager's fallback profile."""

        if decision.stop_required or not _decision_selects_normal_il(
            decision,
            allowed_speed_profiles={"normal", "fallback"},
        ):
            self.reset()
            return None

        fresh_candidate = None
        if decision.speed_profile == "normal":
            fresh_candidate = select_fresh_normal_il_candidate(
                decision=decision,
                debug_values=debug_values,
                debug_receive_sec=debug_receive_sec,
                now_sec=now_sec,
                config=self.config,
            )
        if fresh_candidate is not None:
            self._last_valid_steering_command = (
                fresh_candidate.steering_command
            )
            return fresh_candidate

        if self._last_valid_steering_command is None:
            return None

        return NormalIlDriveCandidate(
            steering_command=self._last_valid_steering_command,
            requested_speed=self.config.fallback_speed,
            steering_held=True,
        )
