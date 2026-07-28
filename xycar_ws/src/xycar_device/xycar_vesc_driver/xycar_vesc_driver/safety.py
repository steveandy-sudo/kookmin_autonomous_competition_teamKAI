"""Pure control and safety helpers for the native VESC node."""

from __future__ import annotations

from dataclasses import dataclass
import math


NORMAL = "normal"
LIMITED = "low_voltage_limited"
LATCHED = "fault_latched"


@dataclass
class VoltageGuard:
    limit_voltage: float = 7.5
    stop_voltage: float = 6.0
    recovery_voltage: float = 8.0
    recovery_stable_sec: float = 2.0
    auto_recover: bool = False
    state: str = NORMAL
    voltage: float = math.nan
    fault_code: int = 0
    recovery_since: float | None = None

    def update(self, voltage: float, fault_code: int, now: float) -> str:
        self.voltage = voltage
        self.fault_code = fault_code
        if fault_code != 0 or (
            math.isfinite(voltage) and voltage <= self.stop_voltage
        ):
            self.state = LATCHED
            self.recovery_since = None
            return self.state

        if self.state == LATCHED:
            if math.isfinite(voltage) and voltage >= self.recovery_voltage:
                if self.recovery_since is None:
                    self.recovery_since = now
                elif (
                    self.auto_recover
                    and now - self.recovery_since >= self.recovery_stable_sec
                ):
                    self.state = NORMAL
                    self.recovery_since = None
            else:
                self.recovery_since = None
            return self.state

        if math.isfinite(voltage) and voltage <= self.limit_voltage:
            self.state = LIMITED
        else:
            self.state = NORMAL
        return self.state

    def clear(self, now: float) -> tuple[bool, str]:
        if self.state != LATCHED:
            return True, "motor fault latch is already clear"
        if self.fault_code != 0:
            return False, f"VESC fault_code is still {self.fault_code}"
        if not math.isfinite(self.voltage):
            return False, "no valid VESC voltage is available"
        if self.voltage < self.recovery_voltage:
            return (
                False,
                f"voltage {self.voltage:.2f} V is below recovery threshold "
                f"{self.recovery_voltage:.2f} V",
            )
        if self.recovery_since is None:
            self.recovery_since = now
            return False, "recovery stability timer started"
        remaining = self.recovery_stable_sec - (now - self.recovery_since)
        if remaining > 0.0:
            return False, f"wait {remaining:.2f} s for stable recovery voltage"
        self.state = NORMAL
        self.recovery_since = None
        return True, "motor fault latch cleared"

    @property
    def output_allowed(self) -> bool:
        return self.state != LATCHED

    @property
    def acceleration_allowed(self) -> bool:
        return self.state == NORMAL

    @property
    def output_scale(self) -> float:
        """Scale propulsion linearly through the configured cutoff range."""
        if self.state == LATCHED:
            return 0.0
        if not math.isfinite(self.voltage) or self.voltage >= self.limit_voltage:
            return 1.0
        voltage_range = self.limit_voltage - self.stop_voltage
        if voltage_range <= 0.0:
            return 0.0
        return max(
            0.0,
            min(1.0, (self.voltage - self.stop_voltage) / voltage_range),
        )


def slew(
    current: float,
    target: float,
    dt: float,
    acceleration_limit: float,
    deceleration_limit: float,
) -> float:
    """Move current toward target with separate acceleration/deceleration."""
    if dt <= 0.0:
        return current
    increasing_magnitude = abs(target) > abs(current) and (
        current == 0.0 or math.copysign(1.0, target) == math.copysign(1.0, current)
    )
    limit = acceleration_limit if increasing_magnitude else deceleration_limit
    maximum_change = max(0.0, limit) * dt
    difference = target - current
    if abs(difference) <= maximum_change:
        return target
    return current + math.copysign(maximum_change, difference)
