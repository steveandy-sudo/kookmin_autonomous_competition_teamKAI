"""Unify IL, Lane Fallback, and Cone candidates without driving the motor."""

from collections.abc import Sequence
from dataclasses import dataclass, field
from enum import Enum
import math

from track_drive.final_driver.cone_input import (
    ConeInputConfig,
    ConeInputSelector,
)
from track_drive.final_driver.lane_fallback_input import (
    LaneFallbackInputConfig,
    LaneFallbackInputSelector,
)
from track_drive.final_driver.normal_il_input import (
    NormalIlInputConfig,
    NormalIlInputSelector,
)
from track_drive.mission.mission_types import MissionDecision


class SteeringUnit(Enum):
    """Unit of the selected steering value before final conversion."""

    NONE = "none"
    XYCAR_COMMAND = "xycar_command"
    PHYSICAL_DEG = "physical_deg"


_INPUT_SOURCES = frozenset(
    {
        "none",
        "drive_il",
        "lane_fallback",
        "cone_rule",
        "fixed_obstacle_rule",
        "vehicle_rule",
        "shortcut",
    }
)


@dataclass(frozen=True)
class DriveInputSelection:
    """One source candidate selected for the future Final Driver."""

    steering_value: float
    steering_unit: SteeringUnit
    requested_speed: float
    selected_source: str
    steering_held: bool
    valid: bool

    def __post_init__(self) -> None:
        if self.selected_source not in _INPUT_SOURCES:
            raise ValueError("unsupported Final Driver input source")
        if not self.valid:
            if (
                self.steering_value != 0.0
                or self.steering_unit is not SteeringUnit.NONE
                or self.requested_speed != 0.0
                or self.steering_held
            ):
                raise ValueError("invalid selection must contain neutral values")
            return
        if self.selected_source == "none":
            raise ValueError("valid selection requires a driving source")
        if self.steering_unit is SteeringUnit.NONE:
            raise ValueError("valid selection requires a steering unit")
        if not all(
            math.isfinite(value)
            for value in (self.steering_value, self.requested_speed)
        ):
            raise ValueError("valid selection values must be finite")

    @classmethod
    def invalid(cls, selected_source: str = "none") -> "DriveInputSelection":
        return cls(
            steering_value=0.0,
            steering_unit=SteeringUnit.NONE,
            requested_speed=0.0,
            selected_source=selected_source,
            steering_held=False,
            valid=False,
        )


@dataclass(frozen=True)
class DriveInputSnapshot:
    """Latest raw values received from all three controller sources."""

    now_sec: float

    il_debug_values: Sequence[float] | None = None
    il_debug_receive_sec: float | None = None

    lane_steering_angle_deg: float | None = None
    lane_command_valid: bool = False
    lane_command_receive_sec: float | None = None

    cone_command_values: Sequence[float] | None = None
    cone_command_receive_sec: float | None = None


@dataclass(frozen=True)
class DriveInputSelectorConfig:
    """One configuration source shared by all input selectors."""

    fallback_speed: float
    il_source_timeout_sec: float = 0.5
    lane_command_timeout_sec: float = 0.2
    lane_max_steering_angle_deg: float = 26.0
    cone: ConeInputConfig = field(default_factory=ConeInputConfig)

    def __post_init__(self) -> None:
        NormalIlInputConfig(
            fallback_speed=self.fallback_speed,
            source_timeout_sec=self.il_source_timeout_sec,
        )
        LaneFallbackInputConfig(
            fallback_speed=self.fallback_speed,
            command_timeout_sec=self.lane_command_timeout_sec,
            max_steering_angle_deg=self.lane_max_steering_angle_deg,
        )


class DriveInputSelector:
    """Select exactly one source according to a MissionDecision."""

    def __init__(self, config: DriveInputSelectorConfig):
        self.config = config
        self._normal_il = NormalIlInputSelector(
            NormalIlInputConfig(
                fallback_speed=config.fallback_speed,
                source_timeout_sec=config.il_source_timeout_sec,
            )
        )
        self._lane_fallback = LaneFallbackInputSelector(
            LaneFallbackInputConfig(
                fallback_speed=config.fallback_speed,
                command_timeout_sec=config.lane_command_timeout_sec,
                max_steering_angle_deg=(
                    config.lane_max_steering_angle_deg
                ),
            )
        )
        self._cone = ConeInputSelector(config.cone)

    def select(
        self,
        *,
        decision: MissionDecision,
        inputs: DriveInputSnapshot,
    ) -> DriveInputSelection:
        """Return the only candidate allowed by ``decision``.

        All source selectors see every decision.  Non-selected selectors
        therefore clear their private last-valid steering memory immediately.
        """

        il_candidate = self._normal_il.select(
            decision=decision,
            debug_values=inputs.il_debug_values,
            debug_receive_sec=inputs.il_debug_receive_sec,
            now_sec=inputs.now_sec,
        )
        lane_candidate = self._lane_fallback.select(
            decision=decision,
            steering_angle_deg=inputs.lane_steering_angle_deg,
            command_valid=inputs.lane_command_valid,
            command_receive_sec=inputs.lane_command_receive_sec,
            now_sec=inputs.now_sec,
        )
        cone_candidate = self._cone.select(
            decision=decision,
            command_values=inputs.cone_command_values,
            command_receive_sec=inputs.cone_command_receive_sec,
            now_sec=inputs.now_sec,
        )

        candidates_present = sum(
            candidate is not None
            for candidate in (
                il_candidate,
                lane_candidate,
                cone_candidate,
            )
        )
        if candidates_present != 1:
            attempted_source = (
                "none"
                if decision.stop_required
                or decision.selected_source == "none"
                else decision.selected_source
            )
            return DriveInputSelection.invalid(attempted_source)

        if il_candidate is not None:
            return DriveInputSelection(
                steering_value=il_candidate.steering_command,
                steering_unit=SteeringUnit.XYCAR_COMMAND,
                requested_speed=il_candidate.requested_speed,
                selected_source="drive_il",
                steering_held=il_candidate.steering_held,
                valid=True,
            )
        if lane_candidate is not None:
            return DriveInputSelection(
                steering_value=lane_candidate.steering_angle_deg,
                steering_unit=SteeringUnit.PHYSICAL_DEG,
                requested_speed=lane_candidate.requested_speed,
                selected_source="lane_fallback",
                steering_held=lane_candidate.steering_held,
                valid=True,
            )
        return DriveInputSelection(
            steering_value=cone_candidate.steering_angle_deg,
            steering_unit=SteeringUnit.PHYSICAL_DEG,
            requested_speed=cone_candidate.requested_speed,
            selected_source="cone_rule",
            steering_held=cone_candidate.steering_held,
            valid=True,
        )
