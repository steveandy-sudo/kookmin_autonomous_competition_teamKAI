"""Pure state classification helpers for the cone-entry monitor."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ConeEntryThresholds:
    yolo_confidence: float = 0.50
    yolo_frames: int = 2
    lidar_distance_m: float = 3.0
    path_confidence: float = 0.35
    command_frames: int = 3


@dataclass(frozen=True)
class ConeEntrySnapshot:
    scan_fresh: bool = False
    object_stream_fresh: bool = False
    selector_active: bool = False
    yolo_confidence: float = 0.0
    yolo_frames: int = 0
    yolo_fresh: bool = False
    lidar_distance_m: float = float("inf")
    lidar_count: int = 0
    lidar_fresh: bool = False
    command_confidence: float = 0.0
    command_speed: float = 0.0
    command_fresh: bool = False
    command_frames: int = 0


def entry_checks(
    snapshot: ConeEntrySnapshot,
    thresholds: ConeEntryThresholds,
) -> tuple[bool, bool, bool]:
    """Return the three gates used by ``ConeModeLatch`` at entry."""
    yolo_ok = bool(
        snapshot.yolo_fresh
        and snapshot.yolo_frames >= max(1, thresholds.yolo_frames)
        and snapshot.yolo_confidence >= thresholds.yolo_confidence
    )
    lidar_ok = bool(
        snapshot.lidar_fresh
        and snapshot.lidar_count > 0
        and math.isfinite(snapshot.lidar_distance_m)
        and snapshot.lidar_distance_m <= thresholds.lidar_distance_m
    )
    command_ok = bool(
        snapshot.command_fresh
        and snapshot.command_confidence >= thresholds.path_confidence
        and snapshot.command_speed > 0.0
    )
    return yolo_ok, lidar_ok, command_ok


def classify_entry_stage(
    snapshot: ConeEntrySnapshot,
    thresholds: ConeEntryThresholds,
) -> str:
    """Return a concise Korean label for the next unmet entry condition."""
    if snapshot.selector_active:
        return "진입 완료 (CONE_RULE 작동 중)"
    if not snapshot.scan_fresh and not snapshot.object_stream_fresh:
        return "카메라·LiDAR 데이터 대기"
    if not snapshot.scan_fresh:
        return "LiDAR 데이터 대기"
    if not snapshot.object_stream_fresh:
        return "YOLO 데이터 대기"

    yolo_ok, lidar_ok, command_ok = entry_checks(snapshot, thresholds)
    if not yolo_ok:
        if snapshot.yolo_confidence >= thresholds.yolo_confidence:
            return (
                "YOLO 라바콘 연속 확인 중 "
                f"({min(snapshot.yolo_frames, thresholds.yolo_frames)}/"
                f"{thresholds.yolo_frames})"
            )
        return "라바콘 미검출 (YOLO 기준 미달)"
    if not lidar_ok:
        if (
            snapshot.lidar_fresh
            and math.isfinite(snapshot.lidar_distance_m)
            and snapshot.lidar_distance_m > thresholds.lidar_distance_m
        ):
            return "라바콘 진입 거리까지 접근 중"
        return "YOLO 확인됨 · LiDAR 라바콘 후보 대기"
    if not command_ok:
        if snapshot.command_fresh:
            return "경로 confidence/속도 조건 대기"
        return "YOLO+LiDAR 확인됨 · 라바콘 경로 생성 대기"
    if snapshot.command_frames < max(1, thresholds.command_frames):
        return (
            "진입 확정 연속 명령 확인 중 "
            f"({snapshot.command_frames}/{thresholds.command_frames})"
        )
    return "모든 진입 조건 충족 · 통합 선택기 전환 대기"


def direction_label(angle: float) -> str:
    if not math.isfinite(float(angle)):
        return "미확인"
    if angle < -0.5:
        return "좌조향"
    if angle > 0.5:
        return "우조향"
    return "직진"


def physical_angle_to_command(angle_deg: float) -> float:
    """Mirror the integrated selector's calibrated cone steering mapping."""
    actual = (0.0, 4.0, 10.0, 16.0, 26.0)
    command = (0.0, 10.0, 20.0, 30.0, 42.0)
    raw_value = float(angle_deg)
    if not math.isfinite(raw_value):
        return float("nan")
    value = max(-42.0, min(42.0, raw_value))
    sign = -1.0 if value < 0.0 else 1.0
    magnitude = abs(value)
    if magnitude >= actual[-1]:
        return sign * command[-1]
    for index in range(1, len(actual)):
        if magnitude <= actual[index]:
            ratio = (
                (magnitude - actual[index - 1])
                / (actual[index] - actual[index - 1])
            )
            mapped = command[index - 1] + ratio * (
                command[index] - command[index - 1]
            )
            return sign * mapped
    return 0.0
