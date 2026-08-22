"""YOLO-armed, LiDAR-tracked local obstacle avoidance state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from .lidar_obstacle import LidarPathObstacle


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(float(minimum), min(float(maximum), float(value)))


class YoloLidarAvoidanceMode(str, Enum):
    IDLE = "IDLE"
    YOLO_TRACKING = "YOLO_TRACKING"
    WAIT_SIDE_CLEAR = "WAIT_SIDE_CLEAR"
    AVOID_LEFT = "AVOID_LEFT"
    AVOID_RIGHT = "AVOID_RIGHT"
    RETURN_CENTER = "RETURN_CENTER"


@dataclass(frozen=True)
class YoloLidarAvoidanceConfig:
    yolo_min_confidence: float = 0.45
    yolo_required_frames: int = 2
    yolo_timeout_sec: float = 1.00
    red_car_yolo_timeout_sec: float = 0.30
    entry_distance_m: float = 1.20
    minimum_side_clearance_m: float = 0.70
    left_offset_m: float = 0.20
    right_offset_m: float = 0.20
    offset_rate_mps: float = 0.50
    speed_limit_command: float = 4.0
    minimum_avoid_sec: float = 0.50
    clear_hold_sec: float = 0.50
    return_hold_sec: float = 0.30
    return_deadband_m: float = 0.02
    immediate_on_yolo: bool = False
    preferred_side_required_frames: int = 2
    active_side_reselection_required_frames: int = 2


@dataclass(frozen=True)
class YoloLidarAvoidanceState:
    mode: YoloLidarAvoidanceMode
    lateral_offset_m: float
    speed_limit_command: float | None
    tracked_distance_m: float
    yolo_confidence: float
    target_class_name: str

    @property
    def controls_vehicle(self) -> bool:
        return self.mode in {
            YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR,
            YoloLidarAvoidanceMode.AVOID_LEFT,
            YoloLidarAvoidanceMode.AVOID_RIGHT,
            YoloLidarAvoidanceMode.RETURN_CENTER,
        }


@dataclass(frozen=True)
class ShortcutAvoidanceSuppressionConfig:
    enabled: bool = True
    release_left_angle_command: float = -8.0
    release_required_frames: int = 2


class ShortcutAvoidanceSuppression:
    """Keep vehicle avoidance disabled until explicit S-curve handoff."""

    def __init__(self, config: ShortcutAvoidanceSuppressionConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.active = False
        self.rule_handoff = False
        self.left_frames = 0

    def start_shortcut(self) -> None:
        self.active = bool(self.config.enabled)
        self.rule_handoff = False
        self.left_frames = 0

    def start_rule_handoff(self) -> None:
        if not self.active:
            return
        self.rule_handoff = True
        self.left_frames = 0

    def release(self) -> bool:
        """Release suppression only when the mission sequencer confirms it."""
        if not self.active:
            return False
        self.reset()
        return True

    def observe_rule_angle(self, angle_command: float) -> bool:
        """Retain legacy diagnostics without releasing mission suppression."""
        if not self.active or not self.rule_handoff:
            return False
        threshold = min(0.0, float(self.config.release_left_angle_command))
        if float(angle_command) <= threshold:
            self.left_frames += 1
        else:
            self.left_frames = 0
        return False


class YoloLidarAvoidanceController:
    """Track a YOLO vehicle and apply the configured avoidance entry policy."""

    def __init__(self, config: YoloLidarAvoidanceConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.mode = YoloLidarAvoidanceMode.IDLE
        self.current_offset_m = 0.0
        self.yolo_frames = 0
        self.yolo_time = float("-inf")
        self.yolo_confidence = 0.0
        self.tracked_distance_m = float("inf")
        self.mode_started_sec = 0.0
        self.clear_started_sec: float | None = None
        self.preferred_mode: YoloLidarAvoidanceMode | None = None
        self.preferred_candidate: YoloLidarAvoidanceMode | None = None
        self.preferred_candidate_frames = 0
        self.active_speed_limit_command = float(
            self.config.speed_limit_command
        )
        self.target_class_name = ""

    def observe_yolo(
        self,
        *,
        now_sec: float,
        detected: bool,
        confidence: float,
        lidar_distance_m: float,
        preferred_mode: YoloLidarAvoidanceMode | None = None,
        side_decision_allowed: bool = True,
        target_class_name: str = "",
        speed_limit_command: float | None = None,
    ) -> None:
        valid = (
            bool(detected)
            and float(confidence) >= self.config.yolo_min_confidence
        )
        self.yolo_frames = self.yolo_frames + 1 if valid else 0
        if not valid:
            return
        self.yolo_time = float(now_sec)
        self.yolo_confidence = float(confidence)
        if str(target_class_name).strip():
            self.target_class_name = str(target_class_name).strip().lower()
        if (
            speed_limit_command is not None
            and math.isfinite(float(speed_limit_command))
        ):
            self.active_speed_limit_command = max(
                0.0,
                float(speed_limit_command),
            )
        if math.isfinite(float(lidar_distance_m)):
            self.tracked_distance_m = float(lidar_distance_m)
        active_avoidance = self.mode in {
            YoloLidarAvoidanceMode.AVOID_LEFT,
            YoloLidarAvoidanceMode.AVOID_RIGHT,
            YoloLidarAvoidanceMode.RETURN_CENTER,
        }
        if not bool(side_decision_allowed) and not active_avoidance:
            self.preferred_mode = None
            self.preferred_candidate = None
            self.preferred_candidate_frames = 0
        if bool(side_decision_allowed) and preferred_mode in {
            YoloLidarAvoidanceMode.AVOID_LEFT,
            YoloLidarAvoidanceMode.AVOID_RIGHT,
        }:
            if preferred_mode == self.preferred_candidate:
                self.preferred_candidate_frames += 1
            else:
                self.preferred_candidate = preferred_mode
                self.preferred_candidate_frames = 1
            active_side_change = (
                self.mode
                in {
                    YoloLidarAvoidanceMode.AVOID_LEFT,
                    YoloLidarAvoidanceMode.AVOID_RIGHT,
                }
                and preferred_mode != self.mode
            )
            required_frames = (
                self.config.active_side_reselection_required_frames
                if active_side_change
                else self.config.preferred_side_required_frames
            )
            if self.preferred_candidate_frames >= max(1, int(required_frames)):
                self.preferred_mode = preferred_mode
                if active_side_change:
                    self._transition(preferred_mode, float(now_sec))
        if (
            self.mode == YoloLidarAvoidanceMode.IDLE
            and self.yolo_frames >= max(1, self.config.yolo_required_frames)
        ):
            if (
                self.config.immediate_on_yolo
                and bool(side_decision_allowed)
                and self.preferred_mode
                in {
                    YoloLidarAvoidanceMode.AVOID_LEFT,
                    YoloLidarAvoidanceMode.AVOID_RIGHT,
                }
            ):
                # Immediate mode is armed by camera semantics alone. LiDAR is
                # telemetry for distance/clearance and never delays entry.
                self._transition(self.preferred_mode, float(now_sec))
            else:
                self._transition(
                    YoloLidarAvoidanceMode.YOLO_TRACKING,
                    float(now_sec),
                )

    def update_lidar_distance(self, distance_m: float) -> None:
        if math.isfinite(float(distance_m)):
            self.tracked_distance_m = float(distance_m)

    def step(
        self,
        *,
        now_sec: float,
        dt_sec: float,
        obstacle: LidarPathObstacle | None,
        cone_active: bool,
    ) -> YoloLidarAvoidanceState:
        now = float(now_sec)
        dt = clamp(dt_sec, 0.0, 0.25)
        if cone_active:
            self.reset()
            return self.state()

        yolo_fresh = now - self.yolo_time <= self._yolo_timeout_sec()
        if self.mode == YoloLidarAvoidanceMode.YOLO_TRACKING:
            if not yolo_fresh:
                self.reset()
                return self.state()
            if self.config.immediate_on_yolo:
                next_mode = self._choose_avoidance_side(obstacle)
                if next_mode in {
                    YoloLidarAvoidanceMode.AVOID_LEFT,
                    YoloLidarAvoidanceMode.AVOID_RIGHT,
                }:
                    self._transition(next_mode, now)
            elif self.tracked_distance_m <= self.config.entry_distance_m:
                self._transition(self._choose_avoidance_side(obstacle), now)
        elif self.mode == YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR:
            if not yolo_fresh and obstacle is None:
                self.reset()
                return self.state()
            next_mode = self._choose_avoidance_side(obstacle)
            if next_mode != YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR:
                self._transition(next_mode, now)
        elif self.mode in {
            YoloLidarAvoidanceMode.AVOID_LEFT,
            YoloLidarAvoidanceMode.AVOID_RIGHT,
        }:
            minimum_elapsed = (
                now - self.mode_started_sec >= self.config.minimum_avoid_sec
            )
            clear = obstacle is None and not yolo_fresh
            if minimum_elapsed and clear:
                if self.clear_started_sec is None:
                    self.clear_started_sec = now
                elif (
                    now - self.clear_started_sec
                    >= self.config.clear_hold_sec
                ):
                    self._transition(
                        YoloLidarAvoidanceMode.RETURN_CENTER,
                        now,
                    )
            else:
                self.clear_started_sec = None
        elif self.mode == YoloLidarAvoidanceMode.RETURN_CENTER:
            if yolo_fresh:
                if self.config.immediate_on_yolo:
                    next_mode = self._choose_avoidance_side(obstacle)
                    if next_mode in {
                        YoloLidarAvoidanceMode.AVOID_LEFT,
                        YoloLidarAvoidanceMode.AVOID_RIGHT,
                    }:
                        self._transition(next_mode, now)
                elif self.tracked_distance_m <= self.config.entry_distance_m:
                    self._transition(self._choose_avoidance_side(obstacle), now)

        if self.mode == YoloLidarAvoidanceMode.AVOID_LEFT:
            target_offset_m = self.config.left_offset_m
        elif self.mode == YoloLidarAvoidanceMode.AVOID_RIGHT:
            target_offset_m = -self.config.right_offset_m
        else:
            target_offset_m = 0.0
        maximum_change = max(0.0, self.config.offset_rate_mps * dt)
        self.current_offset_m += clamp(
            target_offset_m - self.current_offset_m,
            -maximum_change,
            maximum_change,
        )
        if (
            self.mode == YoloLidarAvoidanceMode.RETURN_CENTER
            and abs(self.current_offset_m) <= self.config.return_deadband_m
            and now - self.mode_started_sec >= self.config.return_hold_sec
        ):
            self.reset()
        return self.state()

    def _yolo_timeout_sec(self) -> float:
        if self.target_class_name == "red_car":
            return max(0.0, float(self.config.red_car_yolo_timeout_sec))
        return max(0.0, float(self.config.yolo_timeout_sec))

    def state(self) -> YoloLidarAvoidanceState:
        if self.mode == YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR:
            speed_limit = 0.0
        elif self.mode in {
            YoloLidarAvoidanceMode.AVOID_LEFT,
            YoloLidarAvoidanceMode.AVOID_RIGHT,
            YoloLidarAvoidanceMode.RETURN_CENTER,
        }:
            speed_limit = self.active_speed_limit_command
        else:
            speed_limit = None
        return YoloLidarAvoidanceState(
            mode=self.mode,
            lateral_offset_m=self.current_offset_m,
            speed_limit_command=speed_limit,
            tracked_distance_m=self.tracked_distance_m,
            yolo_confidence=self.yolo_confidence,
            target_class_name=self.target_class_name,
        )

    def _choose_avoidance_side(
        self,
        obstacle: LidarPathObstacle | None,
    ) -> YoloLidarAvoidanceMode:
        if (
            self.config.immediate_on_yolo
            and self.preferred_mode
            in {
                YoloLidarAvoidanceMode.AVOID_LEFT,
                YoloLidarAvoidanceMode.AVOID_RIGHT,
            }
        ):
            return self.preferred_mode
        if obstacle is None:
            return YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR
        left = float(obstacle.left_clearance_m)
        right = float(obstacle.right_clearance_m)
        minimum = self.config.minimum_side_clearance_m
        if self.preferred_mode is None:
            return YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR
        if self.preferred_mode == YoloLidarAvoidanceMode.AVOID_LEFT:
            return (
                YoloLidarAvoidanceMode.AVOID_LEFT
                if left >= minimum
                else YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR
            )
        if self.preferred_mode == YoloLidarAvoidanceMode.AVOID_RIGHT:
            return (
                YoloLidarAvoidanceMode.AVOID_RIGHT
                if right >= minimum
                else YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR
            )
        return YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR

    def _transition(
        self,
        mode: YoloLidarAvoidanceMode,
        now_sec: float,
    ) -> None:
        if mode == self.mode:
            return
        self.mode = mode
        self.mode_started_sec = float(now_sec)
        self.clear_started_sec = None
