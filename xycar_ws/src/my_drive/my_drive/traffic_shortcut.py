"""Pure state machine for four-lamp traffic signals and one shortcut."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class SignalState(str, Enum):
    WAIT_GREEN = "WAIT_GREEN"
    WAIT_START_CLEAR = "WAIT_START_CLEAR"
    RUNNING = "RUNNING"
    RED_HOLD = "RED_HOLD"
    SHORTCUT_LEFT = "SHORTCUT_LEFT"


@dataclass(frozen=True)
class SignalGateConfig:
    candidate_timeout_sec: float = 0.40
    signal_required_frames: int = 2
    start_clear_sec: float = 1.0
    shortcut_green_sync_sec: float = 0.75
    shortcut_duration_sec: float = 1.8
    shortcut_left_command: float = -18.0
    shortcut_lane_command_weight: float = 0.35
    shortcut_speed_command: float = 8.0
    maximum_abs_angle_command: float = 42.0


class SignalGateController:
    """Deterministic, sensor-time-based signal and shortcut state machine."""

    def __init__(self, config: SignalGateConfig) -> None:
        self.config = config
        self.state = SignalState.WAIT_GREEN
        self.red_frames = 0
        self.green_frames = 0
        self.last_red_sec = float("-inf")
        self.last_green_sec = float("-inf")
        self.last_left_sec = float("-inf")
        self.last_signal_sec = float("-inf")
        self.shortcut_started_sec = float("-inf")
        self.shortcut_taken = False
        self.reencounter_armed = False

    def observe_start_green(self, now_sec: float) -> None:
        if self.state == SignalState.WAIT_GREEN:
            self.state = SignalState.WAIT_START_CLEAR
            self.last_green_sec = float(now_sec)
            self.last_signal_sec = float(now_sec)

    def observe_detections(
        self,
        *,
        now_sec: float,
        red: bool,
        green: bool,
        yellow: bool,
        left: bool,
    ) -> None:
        now = float(now_sec)
        self.red_frames = self.red_frames + 1 if red else 0
        self.green_frames = self.green_frames + 1 if green else 0
        if red:
            self.last_red_sec = now
        if green:
            self.last_green_sec = now
        if left:
            self.last_left_sec = now
        if red or green or yellow or left:
            self.last_signal_sec = now

        required = max(1, int(self.config.signal_required_frames))
        if self.reencounter_armed and self.state == SignalState.RUNNING:
            if self.red_frames >= required:
                self.state = SignalState.RED_HOLD
            elif (
                not self.shortcut_taken
                and left
                and now - self.last_green_sec
                <= self.config.shortcut_green_sync_sec
            ):
                self.state = SignalState.SHORTCUT_LEFT
                self.shortcut_started_sec = now
                self.shortcut_taken = True
        elif (
            self.state == SignalState.RED_HOLD
            and self.green_frames >= required
        ):
            self.state = SignalState.RUNNING

    def update(self, now_sec: float) -> SignalState:
        now = float(now_sec)
        if self.state == SignalState.WAIT_START_CLEAR:
            if now - self.last_signal_sec >= self.config.start_clear_sec:
                self.state = SignalState.RUNNING
                self.reencounter_armed = True
        elif self.state == SignalState.SHORTCUT_LEFT:
            if now - self.shortcut_started_sec >= self.config.shortcut_duration_sec:
                self.state = SignalState.RUNNING
        return self.state

    def command(
        self,
        *,
        now_sec: float,
        candidate_age_sec: float,
        candidate_angle: float,
        candidate_speed: float,
    ) -> tuple[float, float, str]:
        state = self.update(now_sec)
        if state in {SignalState.WAIT_GREEN, SignalState.RED_HOLD}:
            return 0.0, 0.0, state.value
        if candidate_age_sec > self.config.candidate_timeout_sec:
            return 0.0, 0.0, "CANDIDATE_STALE"
        if state == SignalState.SHORTCUT_LEFT:
            angle = (
                self.config.shortcut_left_command
                + self.config.shortcut_lane_command_weight
                * float(candidate_angle)
            )
            limit = abs(float(self.config.maximum_abs_angle_command))
            angle = min(max(angle, -limit), limit)
            speed = min(
                max(0.0, float(candidate_speed)),
                max(0.0, float(self.config.shortcut_speed_command)),
            )
            return angle, speed, state.value
        return float(candidate_angle), float(candidate_speed), state.value
