"""Reject discontinuous map-to-odometry localization corrections."""

from __future__ import annotations

from dataclasses import dataclass
import math


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class PlanarTransform:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GuardResult:
    transform: PlanarTransform
    state: str
    translation_residual_m: float
    yaw_residual_rad: float
    outlier_age_sec: float

    @property
    def faulted(self) -> bool:
        return self.state == "FAULT"


def compose_planar(
    parent_to_child: PlanarTransform,
    child_to_base: PlanarTransform,
) -> PlanarTransform:
    cosine = math.cos(parent_to_child.yaw)
    sine = math.sin(parent_to_child.yaw)
    return PlanarTransform(
        x=(
            parent_to_child.x
            + cosine * child_to_base.x
            - sine * child_to_base.y
        ),
        y=(
            parent_to_child.y
            + sine * child_to_base.x
            + cosine * child_to_base.y
        ),
        yaw=normalize_angle(
            parent_to_child.yaw + child_to_base.yaw
        ),
    )


class LocalizationJumpGuard:
    """Hold a trusted transform through transient scan-matching jumps."""

    def __init__(
        self,
        *,
        maximum_translation_jump_m: float,
        maximum_yaw_jump_rad: float,
        fault_after_sec: float,
    ) -> None:
        self.maximum_translation_jump_m = max(
            0.0, float(maximum_translation_jump_m)
        )
        self.maximum_yaw_jump_rad = max(
            0.0, float(maximum_yaw_jump_rad)
        )
        self.fault_after_sec = max(0.0, float(fault_after_sec))
        self.accepted: PlanarTransform | None = None
        self.outlier_since_sec: float | None = None
        self.faulted = False

    def reset(self) -> None:
        self.accepted = None
        self.outlier_since_sec = None
        self.faulted = False

    def update(
        self,
        raw: PlanarTransform,
        *,
        now_sec: float,
        enabled: bool,
        child_to_base: PlanarTransform | None = None,
    ) -> GuardResult:
        if not enabled or self.accepted is None:
            self.accepted = raw
            self.outlier_since_sec = None
            self.faulted = False
            return GuardResult(
                transform=raw,
                state="DISABLED" if not enabled else "TRACKING",
                translation_residual_m=0.0,
                yaw_residual_rad=0.0,
                outlier_age_sec=0.0,
            )

        raw_pose = (
            compose_planar(raw, child_to_base)
            if child_to_base is not None
            else raw
        )
        accepted_pose = (
            compose_planar(self.accepted, child_to_base)
            if child_to_base is not None
            else self.accepted
        )
        translation = math.hypot(
            raw_pose.x - accepted_pose.x,
            raw_pose.y - accepted_pose.y,
        )
        yaw_residual = abs(
            normalize_angle(raw_pose.yaw - accepted_pose.yaw)
        )
        within_limits = (
            translation <= self.maximum_translation_jump_m
            and yaw_residual <= self.maximum_yaw_jump_rad
        )

        if within_limits and not self.faulted:
            self.accepted = raw
            self.outlier_since_sec = None
            return GuardResult(
                transform=raw,
                state="TRACKING",
                translation_residual_m=translation,
                yaw_residual_rad=yaw_residual,
                outlier_age_sec=0.0,
            )

        if self.outlier_since_sec is None:
            self.outlier_since_sec = float(now_sec)
        outlier_age = max(0.0, float(now_sec) - self.outlier_since_sec)
        if outlier_age >= self.fault_after_sec:
            self.faulted = True
        return GuardResult(
            transform=self.accepted,
            state="FAULT" if self.faulted else "HOLDING",
            translation_residual_m=translation,
            yaw_residual_rad=yaw_residual,
            outlier_age_sec=outlier_age,
        )
