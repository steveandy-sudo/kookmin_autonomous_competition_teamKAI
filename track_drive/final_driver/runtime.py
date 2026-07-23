"""Stateful runtime core used by the single ROS Final Driver node."""

from collections.abc import Sequence
from dataclasses import dataclass, field
import math

from track_drive.final_driver.input_selector import (
    DriveInputSelector,
    DriveInputSelectorConfig,
    DriveInputSnapshot,
)
from track_drive.final_driver.steering_converter import (
    FinalDriveCommandCandidate,
    SteeringCalibration,
    convert_drive_input,
)
from track_drive.mission.mission_types import MissionDecision
from track_drive.mission.states import ControlMode, MissionState


@dataclass(frozen=True)
class FinalDriverRuntimeConfig:
    """Final Driver freshness and steering-conversion configuration."""

    input_selector: DriveInputSelectorConfig
    decision_timeout_sec: float = 0.2
    steering_calibration: SteeringCalibration = field(
        default_factory=SteeringCalibration
    )

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.decision_timeout_sec)
            or self.decision_timeout_sec < 0.0
        ):
            raise ValueError(
                "decision_timeout_sec must be finite and non-negative"
            )


def stop_decision() -> MissionDecision:
    """Return the neutral decision used before or after a valid heartbeat."""

    return MissionDecision(
        mission_state=MissionState.WAIT_START_SIGNAL,
        control_mode=ControlMode.STOP,
        selected_source="none",
        speed_profile="stop",
        stop_required=True,
    )


def _receive_time_is_valid(receive_sec: float) -> bool:
    return math.isfinite(receive_sec) and receive_sec >= 0.0


class FinalDriverRuntime:
    """Cache direct controller outputs and select one without a ROS hop."""

    def __init__(self, config: FinalDriverRuntimeConfig):
        self.config = config
        self._selector = DriveInputSelector(config.input_selector)

        self._decision = stop_decision()
        self._decision_receive_sec: float | None = None

        self._il_debug_values: tuple[float, ...] | None = None
        self._il_debug_receive_sec: float | None = None

        self._lane_steering_angle_deg: float | None = None
        self._lane_command_valid = False
        self._lane_command_receive_sec: float | None = None

        self._cone_command_values: tuple[float, ...] | None = None
        self._cone_command_receive_sec: float | None = None

    def update_decision(
        self,
        decision: MissionDecision,
        *,
        receive_sec: float,
    ) -> None:
        if not _receive_time_is_valid(receive_sec):
            raise ValueError(
                "decision receive time must be finite and non-negative"
            )
        self._decision = decision
        self._decision_receive_sec = float(receive_sec)

    def update_il(
        self,
        values: Sequence[float],
        *,
        receive_sec: float,
    ) -> None:
        if not _receive_time_is_valid(receive_sec):
            raise ValueError("IL receive time must be finite and non-negative")
        self._il_debug_values = tuple(values)
        self._il_debug_receive_sec = float(receive_sec)

    def update_lane(
        self,
        *,
        steering_angle_deg: float,
        command_valid: bool,
        receive_sec: float,
    ) -> None:
        if not _receive_time_is_valid(receive_sec):
            raise ValueError(
                "lane receive time must be finite and non-negative"
            )
        self._lane_steering_angle_deg = steering_angle_deg
        self._lane_command_valid = bool(command_valid)
        self._lane_command_receive_sec = float(receive_sec)

    def update_cone(
        self,
        values: Sequence[float],
        *,
        receive_sec: float,
    ) -> None:
        if not _receive_time_is_valid(receive_sec):
            raise ValueError(
                "cone receive time must be finite and non-negative"
            )
        self._cone_command_values = tuple(values)
        self._cone_command_receive_sec = float(receive_sec)

    def _decision_is_fresh(self, now_sec: float) -> bool:
        if self._decision_receive_sec is None:
            return False
        if not math.isfinite(now_sec) or now_sec < 0.0:
            return False
        age_sec = now_sec - self._decision_receive_sec
        if age_sec < 0.0:
            return False
        return age_sec <= self.config.decision_timeout_sec or math.isclose(
            age_sec,
            self.config.decision_timeout_sec,
            rel_tol=1.0e-9,
            abs_tol=1.0e-9,
        )

    def command(self, *, now_sec: float) -> FinalDriveCommandCandidate:
        """Return the selected command, or a neutral invalid candidate."""

        decision = (
            self._decision
            if self._decision_is_fresh(now_sec)
            else stop_decision()
        )
        selection = self._selector.select(
            decision=decision,
            inputs=DriveInputSnapshot(
                now_sec=now_sec,
                il_debug_values=self._il_debug_values,
                il_debug_receive_sec=self._il_debug_receive_sec,
                lane_steering_angle_deg=self._lane_steering_angle_deg,
                lane_command_valid=self._lane_command_valid,
                lane_command_receive_sec=self._lane_command_receive_sec,
                cone_command_values=self._cone_command_values,
                cone_command_receive_sec=self._cone_command_receive_sec,
            ),
        )
        return convert_drive_input(
            selection,
            self.config.steering_calibration,
        )
