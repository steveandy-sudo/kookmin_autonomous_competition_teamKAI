"""Detect a cone-driving episode before the first sequential waypoint."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class ConeBypassEvent(str, Enum):
    NONE = "NONE"
    STARTED = "STARTED"
    FINISHED = "FINISHED"


@dataclass(frozen=True)
class ConeBypassConfig:
    entry_confidence: float = 0.35
    exit_confidence: float = 0.20
    entry_frames: int = 3
    exit_frames: int = 5
    entry_distance_m: float = 1.0


class PreWaypointConeBypass:
    """Track one cone episode per lap while waypoint 1 is still pending."""

    def __init__(self, config: ConeBypassConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.active = False
        self.entry_streak = 0
        self.exit_streak = 0
        self.completed_lap = -1

    def cancel(self) -> None:
        self.active = False
        self.entry_streak = 0
        self.exit_streak = 0

    def update(
        self,
        *,
        lap_count: int,
        target_waypoint_index: int,
        confidence: float,
        speed_command: float,
        yolo_confirmed: bool = True,
        lidar_distance_m: float = 0.0,
    ) -> ConeBypassEvent:
        eligible = (
            int(target_waypoint_index) == 0
            and self.completed_lap != int(lap_count)
        )
        if not eligible:
            self.cancel()
            return ConeBypassEvent.NONE

        valid_entry = (
            bool(yolo_confirmed)
            and math.isfinite(float(lidar_distance_m))
            and float(lidar_distance_m) <= self.config.entry_distance_m
            and
            float(confidence) >= self.config.entry_confidence
            and float(speed_command) > 0.0
        )
        if not self.active:
            self.entry_streak = self.entry_streak + 1 if valid_entry else 0
            if self.entry_streak < max(1, int(self.config.entry_frames)):
                return ConeBypassEvent.NONE
            self.active = True
            self.entry_streak = 0
            self.exit_streak = 0
            return ConeBypassEvent.STARTED

        cone_still_valid = (
            float(confidence) > self.config.exit_confidence
            and float(speed_command) > 0.0
        )
        self.exit_streak = 0 if cone_still_valid else self.exit_streak + 1
        if self.exit_streak < max(1, int(self.config.exit_frames)):
            return ConeBypassEvent.NONE

        self.active = False
        self.exit_streak = 0
        self.completed_lap = int(lap_count)
        return ConeBypassEvent.FINISHED
