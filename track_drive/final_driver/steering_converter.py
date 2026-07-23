"""Convert selected steering values into the Xycar command unit."""

from bisect import bisect_left
from dataclasses import dataclass
import math

from track_drive.final_driver.input_selector import (
    DriveInputSelection,
    SteeringUnit,
)


@dataclass(frozen=True)
class SteeringCalibration:
    """Piecewise-linear physical-degree to Xycar command calibration."""

    physical_angle_deg: tuple[float, ...] = (
        0.0,
        4.0,
        10.0,
        16.0,
        26.0,
    )
    xycar_command: tuple[float, ...] = (
        0.0,
        10.0,
        20.0,
        30.0,
        42.0,
    )
    physical_steering_sign: float = 1.0

    def __post_init__(self) -> None:
        if len(self.physical_angle_deg) != len(self.xycar_command):
            raise ValueError("steering calibration tables must have equal size")
        if len(self.physical_angle_deg) < 2:
            raise ValueError("steering calibration needs at least two points")
        if not all(
            math.isfinite(value)
            for value in self.physical_angle_deg + self.xycar_command
        ):
            raise ValueError("steering calibration points must be finite")
        if self.physical_angle_deg[0] != 0.0:
            raise ValueError("physical calibration must start at zero")
        if self.xycar_command[0] != 0.0:
            raise ValueError("Xycar calibration must start at zero")
        if any(
            right <= left
            for left, right in zip(
                self.physical_angle_deg,
                self.physical_angle_deg[1:],
            )
        ):
            raise ValueError("physical calibration points must increase")
        if any(
            right <= left
            for left, right in zip(
                self.xycar_command,
                self.xycar_command[1:],
            )
        ):
            raise ValueError("Xycar calibration points must increase")
        if self.physical_steering_sign not in (-1.0, 1.0):
            raise ValueError("physical_steering_sign must be -1.0 or 1.0")


@dataclass(frozen=True)
class FinalDriveCommandCandidate:
    """Unit-normalized command candidate; still not a motor publication."""

    steering_command: float
    requested_speed: float
    selected_source: str
    steering_held: bool
    valid: bool

    def __post_init__(self) -> None:
        if not self.valid:
            if (
                self.steering_command != 0.0
                or self.requested_speed != 0.0
                or self.steering_held
            ):
                raise ValueError(
                    "invalid final candidate must contain neutral values"
                )
            return
        if not all(
            math.isfinite(value)
            for value in (self.steering_command, self.requested_speed)
        ):
            raise ValueError("valid final candidate values must be finite")

    @classmethod
    def invalid(
        cls,
        selected_source: str = "none",
    ) -> "FinalDriveCommandCandidate":
        return cls(
            steering_command=0.0,
            requested_speed=0.0,
            selected_source=selected_source,
            steering_held=False,
            valid=False,
        )


def physical_deg_to_xycar_command(
    angle_deg: float,
    calibration: SteeringCalibration = SteeringCalibration(),
) -> float:
    """Convert one physical steering angle using clamped interpolation."""

    angle = float(angle_deg)
    if not math.isfinite(angle):
        raise ValueError("physical steering angle must be finite")

    magnitude = min(
        abs(angle),
        calibration.physical_angle_deg[-1],
    )
    if magnitude == 0.0:
        return 0.0

    upper_index = bisect_left(
        calibration.physical_angle_deg,
        magnitude,
    )
    if (
        calibration.physical_angle_deg[upper_index]
        == magnitude
    ):
        command_magnitude = calibration.xycar_command[upper_index]
    else:
        lower_index = upper_index - 1
        lower_angle = calibration.physical_angle_deg[lower_index]
        upper_angle = calibration.physical_angle_deg[upper_index]
        lower_command = calibration.xycar_command[lower_index]
        upper_command = calibration.xycar_command[upper_index]
        ratio = (magnitude - lower_angle) / (
            upper_angle - lower_angle
        )
        command_magnitude = lower_command + ratio * (
            upper_command - lower_command
        )

    input_sign = -1.0 if angle < 0.0 else 1.0
    return (
        input_sign
        * calibration.physical_steering_sign
        * command_magnitude
    )


def convert_drive_input(
    selection: DriveInputSelection,
    calibration: SteeringCalibration = SteeringCalibration(),
) -> FinalDriveCommandCandidate:
    """Normalize a selected source without smoothing or rate limiting."""

    if not selection.valid:
        return FinalDriveCommandCandidate.invalid(
            selection.selected_source
        )

    if selection.steering_unit is SteeringUnit.XYCAR_COMMAND:
        steering_command = selection.steering_value
    elif selection.steering_unit is SteeringUnit.PHYSICAL_DEG:
        steering_command = physical_deg_to_xycar_command(
            selection.steering_value,
            calibration,
        )
    else:
        return FinalDriveCommandCandidate.invalid(
            selection.selected_source
        )

    return FinalDriveCommandCandidate(
        steering_command=steering_command,
        requested_speed=selection.requested_speed,
        selected_source=selection.selected_source,
        steering_held=selection.steering_held,
        valid=True,
    )
