"""Sequential LiDAR-gate selection between rule and RL commands."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class CandidateSource(str, Enum):
    RL = "RL"
    RULE = "RULE"


class HybridState(str, Enum):
    START_DELAY = "START_DELAY"
    RUNNING = "RUNNING"
    SENSOR_STOP = "SENSOR_STOP"


@dataclass(frozen=True)
class GateDefinition:
    name: str
    next_source: CandidateSource
    primary_sector_index: int
    primary_min_m: float
    primary_max_m: float
    secondary_sector_index: int = -1
    secondary_min_m: float = 0.0
    secondary_max_m: float = 100.0
    rule_abs_angle_max: float = float("inf")

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("gate name must not be empty")
        if self.primary_sector_index < 0:
            raise ValueError("primary sector index must be non-negative")
        if self.primary_min_m > self.primary_max_m:
            raise ValueError("primary gate range is reversed")
        if self.secondary_min_m > self.secondary_max_m:
            raise ValueError("secondary gate range is reversed")
        if self.rule_abs_angle_max < 0.0:
            raise ValueError("rule angle limit must be non-negative")


@dataclass(frozen=True)
class SequentialHybridConfig:
    gates: tuple[GateDefinition, ...]
    initial_source: CandidateSource = CandidateSource.RL
    start_delay_sec: float = 3.0
    minimum_stage_sec: float = 0.8
    required_gate_frames: int = 3
    candidate_fresh_sec: float = 0.35
    candidate_hold_sec: float = 0.75
    transition_blend_sec: float = 0.30
    minimum_speed_command: float = 3.0
    maximum_speed_command: float = 30.0
    maximum_abs_angle_command: float = 42.0
    repeat_sequence: bool = True

    def __post_init__(self) -> None:
        if not self.gates:
            raise ValueError("at least one gate is required")
        if self.required_gate_frames < 1:
            raise ValueError("required gate frames must be positive")
        if self.candidate_fresh_sec < 0.0:
            raise ValueError("candidate fresh timeout must be non-negative")
        if self.candidate_hold_sec < self.candidate_fresh_sec:
            raise ValueError("candidate hold must cover the fresh timeout")
        if self.maximum_speed_command < self.minimum_speed_command:
            raise ValueError("speed command range is reversed")


@dataclass(frozen=True)
class SequentialHybridInput:
    scan_fresh: bool
    scan_sample_is_new: bool
    sector_distances_m: tuple[float, ...]
    rl_command_age_sec: float
    rl_angle_command: float
    rl_speed_command: float
    rule_command_age_sec: float
    rule_angle_command: float
    rule_speed_command: float
    gate_advancement_enabled: bool = True


@dataclass(frozen=True)
class SequentialHybridOutput:
    state: HybridState
    source: CandidateSource
    next_gate_number: int
    next_gate_name: str
    lap_count: int
    gate_streak: int
    angle_command: float
    speed_command: float
    reason: str


class SequentialHybridController:
    """Advance ordered physical gates and select one command source."""

    def __init__(self, config: SequentialHybridConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.state = HybridState.START_DELAY
        self.source = self.config.initial_source
        self.gate_index = 0
        self.lap_count = 0
        self.state_elapsed_sec = 0.0
        self.stage_elapsed_sec = 0.0
        self.gate_streak = 0
        self.last_angle_command = 0.0
        self.last_speed_command = 0.0
        self.transition_elapsed_sec = self.config.transition_blend_sec
        self.transition_start_angle = 0.0
        self.transition_start_speed = 0.0

    def advance_gate(self) -> None:
        self._complete_gate()

    def select_start_waypoint(self, waypoint_number: int) -> None:
        """Reset at a user-selected next gate and infer its incoming source."""
        number = int(waypoint_number)
        if number < 1 or number > len(self.config.gates):
            raise ValueError(
                f"start waypoint must be 1..{len(self.config.gates)}"
            )
        self.reset()
        self.gate_index = number - 1
        source = self.config.initial_source
        for gate in self.config.gates[: self.gate_index]:
            source = gate.next_source
        self.source = source

    def _complete_gate(self) -> GateDefinition:
        gate = self.config.gates[self.gate_index]
        self.transition_start_angle = self.last_angle_command
        self.transition_start_speed = self.last_speed_command
        self.transition_elapsed_sec = 0.0
        self.source = gate.next_source
        self.gate_index += 1
        if self.gate_index >= len(self.config.gates):
            self.lap_count += 1
            if self.config.repeat_sequence:
                self.gate_index = 0
            else:
                self.gate_index = len(self.config.gates) - 1
        self.stage_elapsed_sec = 0.0
        self.gate_streak = 0
        return gate

    def _distance(self, inputs: SequentialHybridInput, index: int) -> float:
        if index < 0 or index >= len(inputs.sector_distances_m):
            return float("inf")
        return float(inputs.sector_distances_m[index])

    def _gate_matches(
        self,
        gate: GateDefinition,
        inputs: SequentialHybridInput,
    ) -> bool:
        primary = self._distance(inputs, gate.primary_sector_index)
        if not math.isfinite(primary):
            return False
        if not gate.primary_min_m <= primary <= gate.primary_max_m:
            return False
        if gate.secondary_sector_index < 0:
            secondary_matches = True
        else:
            secondary = self._distance(inputs, gate.secondary_sector_index)
            secondary_matches = bool(
                math.isfinite(secondary)
                and gate.secondary_min_m
                <= secondary
                <= gate.secondary_max_m
            )
        return bool(
            secondary_matches
            and abs(inputs.rule_angle_command) <= gate.rule_abs_angle_max
        )

    def _candidate(
        self,
        inputs: SequentialHybridInput,
    ) -> tuple[float, float, float]:
        if self.source == CandidateSource.RL:
            return (
                float(inputs.rl_angle_command),
                float(inputs.rl_speed_command),
                float(inputs.rl_command_age_sec),
            )
        return (
            float(inputs.rule_angle_command),
            float(inputs.rule_speed_command),
            float(inputs.rule_command_age_sec),
        )

    def _limit(self, angle: float, speed: float) -> tuple[float, float]:
        angle = min(
            max(float(angle), -self.config.maximum_abs_angle_command),
            self.config.maximum_abs_angle_command,
        )
        speed = float(speed)
        if speed > 0.0:
            speed = min(
                max(speed, self.config.minimum_speed_command),
                self.config.maximum_speed_command,
            )
        return angle, speed

    def _output(
        self,
        angle: float,
        speed: float,
        reason: str,
    ) -> SequentialHybridOutput:
        gate = self.config.gates[self.gate_index]
        self.last_angle_command = float(angle)
        self.last_speed_command = float(speed)
        return SequentialHybridOutput(
            state=self.state,
            source=self.source,
            next_gate_number=self.gate_index + 1,
            next_gate_name=gate.name,
            lap_count=self.lap_count,
            gate_streak=self.gate_streak,
            angle_command=float(angle),
            speed_command=float(speed),
            reason=reason,
        )

    def step(
        self,
        inputs: SequentialHybridInput,
        *,
        dt_sec: float,
    ) -> SequentialHybridOutput:
        dt = max(0.0, min(0.25, float(dt_sec)))
        self.state_elapsed_sec += dt
        self.stage_elapsed_sec += dt
        self.transition_elapsed_sec += dt

        if not inputs.scan_fresh:
            self.state = HybridState.SENSOR_STOP
            return self._output(0.0, 0.0, "LiDAR scan stale")

        angle, speed, candidate_age = self._candidate(inputs)
        if candidate_age > self.config.candidate_hold_sec:
            self.state = HybridState.SENSOR_STOP
            return self._output(0.0, 0.0, f"{self.source.value} command stale")
        if candidate_age > self.config.candidate_fresh_sec:
            return self._output(
                self.last_angle_command,
                self.last_speed_command,
                f"holding last {self.source.value} command",
            )

        if self.state == HybridState.START_DELAY:
            if self.state_elapsed_sec < self.config.start_delay_sec:
                return self._output(0.0, 0.0, "start delay")
            self.state = HybridState.RUNNING
            self.state_elapsed_sec = 0.0
        elif self.state == HybridState.SENSOR_STOP:
            self.state = HybridState.RUNNING
            self.state_elapsed_sec = 0.0

        gate = self.config.gates[self.gate_index]
        matched = bool(
            inputs.gate_advancement_enabled
            and self._gate_matches(gate, inputs)
        )
        if not inputs.gate_advancement_enabled:
            self.gate_streak = 0
        elif inputs.scan_sample_is_new:
            self.gate_streak = self.gate_streak + 1 if matched else 0
        transition_reason = ""
        if (
            self.stage_elapsed_sec >= self.config.minimum_stage_sec
            and self.gate_streak >= self.config.required_gate_frames
        ):
            completed = self._complete_gate()
            angle, speed, candidate_age = self._candidate(inputs)
            transition_reason = (
                f"gate {completed.name} -> {self.source.value}"
            )

        angle, speed = self._limit(angle, speed)
        blend = self.config.transition_blend_sec
        if blend > 0.0 and self.transition_elapsed_sec < blend:
            alpha = max(0.0, min(1.0, self.transition_elapsed_sec / blend))
            angle = self.transition_start_angle + alpha * (
                angle - self.transition_start_angle
            )
            speed = self.transition_start_speed + alpha * (
                speed - self.transition_start_speed
            )
        reason = transition_reason or (
            f"{self.source.value}; waiting for gate {gate.name}"
        )
        return self._output(angle, speed, reason)
