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
    yolo_timeout_sec: float = 0.75
    entry_distance_m: float = 1.20
    minimum_side_clearance_m: float = 0.70
    left_offset_m: float = 0.28
    right_offset_m: float = 0.28
    offset_rate_mps: float = 0.35
    speed_limit_command: float = 4.0
    minimum_avoid_sec: float = 0.80
    clear_hold_sec: float = 1.00
    return_hold_sec: float = 0.30
    return_deadband_m: float = 0.02
    immediate_on_yolo: bool = False


@dataclass(frozen=True)
class YoloLidarAvoidanceState:
    mode: YoloLidarAvoidanceMode
    lateral_offset_m: float
    speed_limit_command: float | None
    tracked_distance_m: float
    yolo_confidence: float

    @property
    def controls_vehicle(self) -> bool:
        return self.mode in {
            YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR,
            YoloLidarAvoidanceMode.AVOID_LEFT,
            YoloLidarAvoidanceMode.AVOID_RIGHT,
            YoloLidarAvoidanceMode.RETURN_CENTER,
        }


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

    def observe_yolo(
        self,
        *,
        now_sec: float,
        detected: bool,
        confidence: float,
        lidar_distance_m: float,
        preferred_mode: YoloLidarAvoidanceMode | None = None,
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
        if math.isfinite(float(lidar_distance_m)):
            self.tracked_distance_m = float(lidar_distance_m)
        if preferred_mode in {
            YoloLidarAvoidanceMode.AVOID_LEFT,
            YoloLidarAvoidanceMode.AVOID_RIGHT,
        }:
            self.preferred_mode = preferred_mode
        if (
            self.mode == YoloLidarAvoidanceMode.IDLE
            and self.yolo_frames >= max(1, self.config.yolo_required_frames)
        ):
            immediate_mode = (
                self.preferred_mode
                if self.config.immediate_on_yolo
                and math.isfinite(self.tracked_distance_m)
                else None
            )
            self._transition(
                immediate_mode or YoloLidarAvoidanceMode.YOLO_TRACKING,
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

        yolo_fresh = now - self.yolo_time <= self.config.yolo_timeout_sec
        if self.mode == YoloLidarAvoidanceMode.YOLO_TRACKING:
            if not yolo_fresh:
                self.reset()
                return self.state()
            if self.tracked_distance_m <= self.config.entry_distance_m:
                next_mode = self._choose_avoidance_side(obstacle)
                self._transition(next_mode, now)
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
            if (
                yolo_fresh
                and self.tracked_distance_m <= self.config.entry_distance_m
            ):
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

    def state(self) -> YoloLidarAvoidanceState:
        if self.mode == YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR:
            speed_limit = 0.0
        elif self.mode in {
            YoloLidarAvoidanceMode.AVOID_LEFT,
            YoloLidarAvoidanceMode.AVOID_RIGHT,
            YoloLidarAvoidanceMode.RETURN_CENTER,
        }:
            speed_limit = self.config.speed_limit_command
        else:
            speed_limit = None
        return YoloLidarAvoidanceState(
            mode=self.mode,
            lateral_offset_m=self.current_offset_m,
            speed_limit_command=speed_limit,
            tracked_distance_m=self.tracked_distance_m,
            yolo_confidence=self.yolo_confidence,
        )

    def _choose_avoidance_side(
        self,
        obstacle: LidarPathObstacle | None,
    ) -> YoloLidarAvoidanceMode:
        if obstacle is None:
            return YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR
        left = float(obstacle.left_clearance_m)
        right = float(obstacle.right_clearance_m)
        minimum = self.config.minimum_side_clearance_m
        if left < minimum and right < minimum:
            return YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR
        if left >= right:
            return YoloLidarAvoidanceMode.AVOID_LEFT
        return YoloLidarAvoidanceMode.AVOID_RIGHT

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
