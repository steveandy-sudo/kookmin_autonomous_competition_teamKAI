from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class DriveMode(IntEnum):
    MODEL_STRAIGHT = 0
    LANE_RULE_CURVE = 1
    CONE_RULE = 2


@dataclass(frozen=True)
class ModeManagerConfig:
    curve_entry_per_m: float = 0.16
    curve_exit_per_m: float = 0.10
    curve_entry_angle_command: float = 10.0
    curve_exit_angle_command: float = 6.0
    curve_entry_frames: int = 1
    curve_exit_frames: int = 3
    cone_entry_confidence: float = 0.35
    cone_entry_frames: int = 3
    cone_min_duration_sec: float = 1.0
    cone_exit_timeout_sec: float = 0.8
    cone_lane_recovery_frames: int = 4


@dataclass(frozen=True)
class ModeDecision:
    mode: DriveMode
    changed: bool
    reason: str


class HybridModeManager:
    """Priority state machine: cone rule, curve rule, then straight model."""

    def __init__(self, config: ModeManagerConfig) -> None:
        self.config = config
        self.mode = DriveMode.MODEL_STRAIGHT
        self.mode_started_sec = 0.0
        self.curve_frames = 0
        self.straight_frames = 0
        self.cone_frames = 0
        self.cone_lane_recovery_count = 0
        self.last_canonical_generation = -1
        self.last_cone_generation = -1

    def update(
        self,
        *,
        now_sec: float,
        canonical_generation: int,
        cone_generation: int,
        lane_available: bool,
        model_available: bool,
        path_curvature_per_m: float,
        rule_angle_command: float,
        cone_confidence: float,
        cone_age_sec: float,
        cone_cluster_count: int,
    ) -> ModeDecision:
        cfg = self.config
        new_cone_frame = cone_generation != self.last_cone_generation
        if new_cone_frame:
            self.last_cone_generation = cone_generation
            cone_entry = (
                cone_confidence >= cfg.cone_entry_confidence
                and cone_age_sec <= cfg.cone_exit_timeout_sec
                and cone_cluster_count >= 2
            )
            self.cone_frames = self.cone_frames + 1 if cone_entry else 0

        cone_recent = (
            cone_confidence > 0.20
            and cone_age_sec <= cfg.cone_exit_timeout_sec
            and cone_cluster_count >= 2
        )
        if self.mode != DriveMode.CONE_RULE and (
            self.cone_frames >= max(1, cfg.cone_entry_frames)
        ):
            return self._change(
                DriveMode.CONE_RULE,
                now_sec,
                "consecutive LiDAR cone-corridor paths",
            )

        new_canonical_frame = (
            canonical_generation != self.last_canonical_generation
        )
        if new_canonical_frame:
            self.last_canonical_generation = canonical_generation

        if self.mode == DriveMode.CONE_RULE:
            if cone_recent:
                self.cone_lane_recovery_count = 0
                return ModeDecision(self.mode, False, "fresh cone path")
            if now_sec - self.mode_started_sec < cfg.cone_min_duration_sec:
                return ModeDecision(self.mode, False, "minimum cone hold")
            if new_canonical_frame and lane_available:
                self.cone_lane_recovery_count += 1
            elif new_canonical_frame:
                self.cone_lane_recovery_count = 0
            if self.cone_lane_recovery_count < max(
                1, cfg.cone_lane_recovery_frames
            ):
                return ModeDecision(self.mode, False, "waiting for lane recovery")
            self.cone_frames = 0
            self.cone_lane_recovery_count = 0
            target = self._lane_mode(
                path_curvature_per_m,
                rule_angle_command,
                model_available,
                lane_available,
            )
            return self._change(target, now_sec, "cone ended; lane recovered")

        if not new_canonical_frame:
            return ModeDecision(self.mode, False, "holding current frame")

        curve_entry = lane_available and (
            path_curvature_per_m >= cfg.curve_entry_per_m
            or abs(rule_angle_command) >= cfg.curve_entry_angle_command
        )
        straight_exit = lane_available and (
            path_curvature_per_m <= cfg.curve_exit_per_m
            and abs(rule_angle_command) <= cfg.curve_exit_angle_command
        )
        self.curve_frames = self.curve_frames + 1 if curve_entry else 0
        self.straight_frames = self.straight_frames + 1 if straight_exit else 0

        if self.mode == DriveMode.MODEL_STRAIGHT:
            if not model_available and lane_available:
                return self._change(
                    DriveMode.LANE_RULE_CURVE,
                    now_sec,
                    "model unavailable",
                )
            if self.curve_frames >= max(1, cfg.curve_entry_frames):
                self.straight_frames = 0
                return self._change(
                    DriveMode.LANE_RULE_CURVE,
                    now_sec,
                    "curve preview threshold",
                )
            return ModeDecision(self.mode, False, "straight model")

        if self.mode == DriveMode.LANE_RULE_CURVE:
            if (
                model_available
                and self.straight_frames >= max(1, cfg.curve_exit_frames)
            ):
                self.curve_frames = 0
                return self._change(
                    DriveMode.MODEL_STRAIGHT,
                    now_sec,
                    "stable straight confirmed",
                )
            return ModeDecision(self.mode, False, "curve rule")

        return ModeDecision(self.mode, False, "unchanged")

    def _lane_mode(
        self,
        curvature_per_m: float,
        rule_angle_command: float,
        model_available: bool,
        lane_available: bool,
    ) -> DriveMode:
        curve = lane_available and (
            curvature_per_m >= self.config.curve_exit_per_m
            or abs(rule_angle_command) >= self.config.curve_exit_angle_command
        )
        if curve or not model_available:
            return DriveMode.LANE_RULE_CURVE
        return DriveMode.MODEL_STRAIGHT

    def _change(
        self,
        mode: DriveMode,
        now_sec: float,
        reason: str,
    ) -> ModeDecision:
        changed = mode != self.mode
        self.mode = mode
        if changed:
            self.mode_started_sec = float(now_sec)
        return ModeDecision(mode, changed, reason)
