"""Pure mission and localization logic for deterministic parking tests."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise ValueError("pose values must be finite")


@dataclass(frozen=True)
class TransitPassAssessment:
    passed: bool
    reason: str
    distance_m: float
    along_past_m: float
    lateral_error_m: float


def assess_transit_waypoint_pass(
    *,
    current: Pose2D,
    previous: Pose2D,
    target: Pose2D,
    radius_m: float,
    maximum_miss_distance_m: float,
    lateral_tolerance_m: float,
) -> TransitPassAssessment:
    """Accept a loose transit point by radius or by crossing its gate."""
    limits = (
        float(radius_m),
        float(maximum_miss_distance_m),
        float(lateral_tolerance_m),
    )
    if not all(math.isfinite(value) and value > 0.0 for value in limits):
        raise ValueError("transit pass limits must be finite and positive")

    distance = math.hypot(current.x - target.x, current.y - target.y)
    if distance <= limits[0]:
        return TransitPassAssessment(True, "radius", distance, 0.0, 0.0)

    segment_x = target.x - previous.x
    segment_y = target.y - previous.y
    segment_length = math.hypot(segment_x, segment_y)
    if segment_length <= 1.0e-6:
        return TransitPassAssessment(False, "degenerate", distance, 0.0, distance)
    unit_x = segment_x / segment_length
    unit_y = segment_y / segment_length
    offset_x = current.x - target.x
    offset_y = current.y - target.y
    along_past = offset_x * unit_x + offset_y * unit_y
    lateral = abs(-offset_x * unit_y + offset_y * unit_x)
    passed = (
        along_past >= 0.0
        and distance <= limits[1]
        and lateral <= limits[2]
    )
    return TransitPassAssessment(
        passed,
        "crossed_gate" if passed else "not_passed",
        distance,
        along_past,
        lateral,
    )


@dataclass(frozen=True)
class MissionStep:
    name: str
    reference_pose: Pose2D
    hold_sec: float = 0.0
    parking_goal: bool = False
    allow_reverse: bool = False
    precise_goal: bool = False
    maximum_retries: int = 2
    reverse_only: bool = False

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("mission step name must not be empty")
        if self.hold_sec < 0.0:
            raise ValueError("hold_sec must be non-negative")
        if self.maximum_retries < 0:
            raise ValueError("maximum_retries must be non-negative")
        if self.reverse_only and not self.allow_reverse:
            raise ValueError("reverse_only requires allow_reverse")


def is_reverse_fallback_candidate(step: MissionStep) -> bool:
    """Return whether a failed forward transit may retry bidirectionally.

    Precise cusp/parking poses retain their explicitly designed direction. The
    fallback is only for ordinary transit waypoints that a forward-only Dubins
    planner can make unreachable after the vehicle has passed them.
    """

    return not (step.allow_reverse or step.precise_goal or step.parking_goal)


class ForwardProgressWatchdog:
    """Detect a forward goal whose remaining distance is not decreasing."""

    def __init__(self, timeout_sec: float, minimum_improvement_m: float) -> None:
        values = (float(timeout_sec), float(minimum_improvement_m))
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("forward progress limits must be finite and positive")
        self.timeout_sec = values[0]
        self.minimum_improvement_m = values[1]
        self.best_distance_m: float | None = None
        self.last_progress_sec: float | None = None

    def reset(self) -> None:
        self.best_distance_m = None
        self.last_progress_sec = None

    def update(self, *, distance_m: float, now_sec: float) -> bool:
        distance = float(distance_m)
        now = float(now_sec)
        if not math.isfinite(distance) or distance < 0.0 or not math.isfinite(now):
            return False
        if self.best_distance_m is None or self.last_progress_sec is None:
            self.best_distance_m = distance
            self.last_progress_sec = now
            return False
        if distance <= self.best_distance_m - self.minimum_improvement_m:
            self.best_distance_m = distance
            self.last_progress_sec = now
            return False
        return now - self.last_progress_sec >= self.timeout_sec


@dataclass(frozen=True)
class MissionTimeAssessment:
    elapsed_sec: float
    remaining_sec: float
    expired: bool
    warning: bool


def assess_mission_time(
    *,
    started_at_sec: float | None,
    now_sec: float,
    limit_sec: float,
    warning_remaining_sec: float,
    finished_at_sec: float | None = None,
) -> MissionTimeAssessment:
    """Assess the competition clock, including pauses and a frozen finish time."""

    values = (now_sec, limit_sec, warning_remaining_sec)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("mission timing values must be finite")
    if limit_sec <= 0.0:
        raise ValueError("mission time limit must be positive")
    if not 0.0 <= warning_remaining_sec < limit_sec:
        raise ValueError("mission warning must be within the time limit")
    if started_at_sec is None:
        return MissionTimeAssessment(0.0, float(limit_sec), False, False)
    if not math.isfinite(float(started_at_sec)):
        raise ValueError("mission start time must be finite")

    effective_now = float(now_sec)
    if finished_at_sec is not None:
        if not math.isfinite(float(finished_at_sec)):
            raise ValueError("mission finish time must be finite")
        effective_now = min(effective_now, float(finished_at_sec))
    elapsed = max(0.0, effective_now - float(started_at_sec))
    remaining = max(0.0, float(limit_sec) - elapsed)
    return MissionTimeAssessment(
        elapsed_sec=elapsed,
        remaining_sec=remaining,
        expired=elapsed >= float(limit_sec),
        warning=remaining <= float(warning_remaining_sec),
    )


def reference_pose_to_base(pose: Pose2D, base_from_reference_x_m: float) -> Pose2D:
    """Move a geometric-center target to the calibrated base-frame origin.

    The real vehicle currently defines ``base_footprint`` at the front wheel
    center. Competition targets are marked at the vehicle rectangle center.
    A positive offset therefore moves the target forward along target yaw.
    """

    offset = float(base_from_reference_x_m)
    if not math.isfinite(offset):
        raise ValueError("base reference offset must be finite")
    return Pose2D(
        x=pose.x + offset * math.cos(pose.yaw),
        y=pose.y + offset * math.sin(pose.yaw),
        yaw=normalize_angle(pose.yaw),
    )


def pose_error(current: Pose2D, target: Pose2D) -> tuple[float, float]:
    return (
        math.hypot(current.x - target.x, current.y - target.y),
        abs(normalize_angle(current.yaw - target.yaw)),
    )


@dataclass(frozen=True)
class DirectReverseCommand:
    """Closed-loop reverse command for the final straight parking segment."""

    linear_x: float
    angular_z: float
    distance_m: float
    yaw_error_rad: float
    lateral_error_m: float
    reached: bool


def direct_reverse_parking_command(
    *,
    current: Pose2D,
    target: Pose2D,
    speed_mps: float,
    position_tolerance_m: float,
    yaw_tolerance_rad: float,
    heading_gain: float,
    maximum_curvature: float,
) -> DirectReverseCommand:
    """Track a target behind the vehicle without asking Nav2 to replan.

    The position term is reverse pure pursuit.  A separate signed heading
    term keeps the body aligned with the surveyed parking-pose yaw.  The
    caller still owns localization, obstacle and actuator safety gates.
    """

    values = (
        float(speed_mps),
        float(position_tolerance_m),
        float(yaw_tolerance_rad),
        float(heading_gain),
        float(maximum_curvature),
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("direct reverse controller limits must be positive")

    delta_x = target.x - current.x
    delta_y = target.y - current.y
    distance = math.hypot(delta_x, delta_y)
    yaw_error = normalize_angle(target.yaw - current.yaw)
    lateral = -math.sin(current.yaw) * delta_x + math.cos(current.yaw) * delta_y
    reached = (
        distance <= position_tolerance_m
        and abs(yaw_error) <= yaw_tolerance_rad
    )
    if reached:
        return DirectReverseCommand(
            linear_x=0.0,
            angular_z=0.0,
            distance_m=distance,
            yaw_error_rad=yaw_error,
            lateral_error_m=lateral,
            reached=True,
        )

    # Pure-pursuit curvature stays finite near the target.  The heading term
    # is expressed as yaw rate, so its sign remains intuitive with v < 0.
    squared_distance = max(distance * distance, 0.01)
    path_curvature = max(
        -maximum_curvature,
        min(maximum_curvature, 2.0 * lateral / squared_distance),
    )
    linear_x = -abs(speed_mps)
    angular_z = linear_x * path_curvature + heading_gain * yaw_error
    maximum_yaw_rate = abs(linear_x) * maximum_curvature
    angular_z = max(-maximum_yaw_rate, min(maximum_yaw_rate, angular_z))
    return DirectReverseCommand(
        linear_x=linear_x,
        angular_z=angular_z,
        distance_m=distance,
        yaw_error_rad=yaw_error,
        lateral_error_m=lateral,
        reached=False,
    )


@dataclass(frozen=True)
class LocalizationGateConfig:
    maximum_xy_variance: float = 0.0625
    maximum_yaw_variance: float = 0.12
    maximum_pose_age_sec: float = 4.00
    maximum_position_jump_m: float = 0.60
    maximum_yaw_jump_rad: float = 0.80
    required_stable_samples: int = 6

    def __post_init__(self) -> None:
        positive = (
            self.maximum_xy_variance,
            self.maximum_yaw_variance,
            self.maximum_pose_age_sec,
            self.maximum_position_jump_m,
            self.maximum_yaw_jump_rad,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("localization gate limits must be finite and positive")
        if self.required_stable_samples <= 0:
            raise ValueError("required_stable_samples must be positive")


@dataclass(frozen=True)
class LocalizationAssessment:
    ready: bool
    reason: str
    stable_samples: int


class LocalizationGate:
    """Reject stale, uncertain, or discontinuous AMCL estimates."""

    def __init__(self, config: LocalizationGateConfig) -> None:
        self.config = config
        self.stable_samples = 0
        self.last_pose: Pose2D | None = None
        self.last_stamp_sec: float | None = None
        self.last_reason = "no_pose"

    def update(
        self,
        *,
        pose: Pose2D,
        covariance: Sequence[float],
        stamp_sec: float,
        now_sec: float,
    ) -> LocalizationAssessment:
        reason = self._sample_reason(
            pose=pose,
            covariance=covariance,
            stamp_sec=stamp_sec,
            now_sec=now_sec,
        )
        if reason == "ok":
            self.stable_samples += 1
        else:
            self.stable_samples = 0

        self.last_pose = pose
        self.last_stamp_sec = stamp_sec
        self.last_reason = reason
        return LocalizationAssessment(
            ready=(
                reason == "ok"
                and self.stable_samples >= self.config.required_stable_samples
            ),
            reason=reason,
            stable_samples=self.stable_samples,
        )

    def age_assessment(self, now_sec: float) -> LocalizationAssessment:
        if self.last_stamp_sec is None:
            return LocalizationAssessment(False, "no_pose", self.stable_samples)
        if now_sec - self.last_stamp_sec > self.config.maximum_pose_age_sec:
            self.stable_samples = 0
            self.last_reason = "stale_pose"
            return LocalizationAssessment(False, self.last_reason, 0)
        return LocalizationAssessment(
            self.stable_samples >= self.config.required_stable_samples,
            self.last_reason,
            self.stable_samples,
        )

    def _sample_reason(
        self,
        *,
        pose: Pose2D,
        covariance: Sequence[float],
        stamp_sec: float,
        now_sec: float,
    ) -> str:
        if len(covariance) < 36:
            return "bad_covariance"
        checked = (covariance[0], covariance[7], covariance[35], stamp_sec, now_sec)
        if not all(math.isfinite(float(value)) for value in checked):
            return "non_finite"
        if stamp_sec <= 0.0 or now_sec < stamp_sec:
            return "bad_stamp"
        if now_sec - stamp_sec > self.config.maximum_pose_age_sec:
            return "stale_pose"
        if max(float(covariance[0]), float(covariance[7])) > self.config.maximum_xy_variance:
            return "xy_uncertain"
        if float(covariance[35]) > self.config.maximum_yaw_variance:
            return "yaw_uncertain"
        if self.last_pose is not None and self.last_stamp_sec is not None:
            if stamp_sec <= self.last_stamp_sec:
                return "non_monotonic_stamp"
            position_jump, yaw_jump = pose_error(pose, self.last_pose)
            if position_jump > self.config.maximum_position_jump_m:
                return "position_jump"
            if yaw_jump > self.config.maximum_yaw_jump_rad:
                return "yaw_jump"
        return "ok"


def mission_steps_from_dicts(items: Iterable[dict]) -> list[MissionStep]:
    steps = []
    for item in items:
        steps.append(
            MissionStep(
                name=str(item["name"]),
                reference_pose=Pose2D(
                    float(item["x"]),
                    float(item["y"]),
                    float(item["yaw"]),
                ),
                hold_sec=float(item.get("hold_sec", 0.0)),
                parking_goal=bool(item.get("parking_goal", False)),
                allow_reverse=bool(item.get("allow_reverse", False)),
                precise_goal=bool(item.get("precise_goal", False)),
                maximum_retries=int(item.get("maximum_retries", 2)),
                reverse_only=bool(item.get("reverse_only", False)),
            )
        )
    if not steps:
        raise ValueError("mission must contain at least one step")
    names = [step.name for step in steps]
    if len(names) != len(set(names)):
        raise ValueError("mission step names must be unique")
    return steps
