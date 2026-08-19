"""Sensor-presence latch for cone driving."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class ConeModeEvent(str, Enum):
    NONE = "NONE"
    STARTED = "STARTED"
    FINISHED = "FINISHED"


@dataclass(frozen=True)
class ConeModeConfig:
    entry_confidence: float = 0.35
    entry_frames: int = 3
    exit_frames: int = 1
    exit_absence_sec: float = 0.0
    entry_distance_enabled: bool = False
    entry_distance_m: float = 3.0


class ConeModeLatch:
    """Enter on confirmed cone sensors and release only after both disappear."""

    def __init__(self, config: ConeModeConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.active = False
        self.entry_streak = 0
        self.exit_streak = 0
        self.absence_started_sec: float | None = None

    def observe_command(
        self,
        *,
        confidence: float,
        speed_command: float,
        yolo_confirmed: bool,
        lidar_distance_m: float,
    ) -> ConeModeEvent:
        distance_valid = not self.config.entry_distance_enabled or (
            math.isfinite(float(lidar_distance_m))
            and float(lidar_distance_m) <= self.config.entry_distance_m
        )
        valid_entry = (
            bool(yolo_confirmed)
            and distance_valid
            and float(confidence) >= self.config.entry_confidence
            and float(speed_command) > 0.0
        )
        if self.active:
            return ConeModeEvent.NONE
        self.entry_streak = self.entry_streak + 1 if valid_entry else 0
        if self.entry_streak < max(1, int(self.config.entry_frames)):
            return ConeModeEvent.NONE
        self.active = True
        self.entry_streak = 0
        self.exit_streak = 0
        self.absence_started_sec = None
        return ConeModeEvent.STARTED

    def update_presence(
        self,
        *,
        sensor_present: bool,
        now_sec: float | None = None,
    ) -> ConeModeEvent:
        if not self.active:
            return ConeModeEvent.NONE
        if bool(sensor_present):
            self.exit_streak = 0
            self.absence_started_sec = None
            return ConeModeEvent.NONE

        self.exit_streak += 1
        if now_sec is not None and self.absence_started_sec is None:
            self.absence_started_sec = float(now_sec)
        if self.exit_streak < max(1, int(self.config.exit_frames)):
            return ConeModeEvent.NONE
        if now_sec is not None:
            now = float(now_sec)
            absence_sec = max(0.0, now - self.absence_started_sec)
            if absence_sec < max(0.0, float(self.config.exit_absence_sec)):
                return ConeModeEvent.NONE

        self.active = False
        self.exit_streak = 0
        self.absence_started_sec = None
        return ConeModeEvent.FINISHED
