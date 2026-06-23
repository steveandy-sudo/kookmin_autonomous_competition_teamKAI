"""차량 제어 스타터 로직."""

from __future__ import annotations

from track_drive.mission_state import MissionState
from track_drive.utils import clamp


def compute_lane_center_steering(
    lane_center_x: float | None,
    image_width: int | None,
    kp: float = 0.3,
) -> float:
    """차선 중심 오차에 대한 단순 비례(P) 조향값을 계산한다."""
    # 차선 중심 오차를 조향각으로 변환한다.
    if lane_center_x is None or image_width is None or image_width <= 0:
        return 0.0

    image_center = image_width / 2.0
    normalized_error = (lane_center_x - image_center) / image_center
    return -kp * normalized_error * 50.0


def select_speed_by_state(state: MissionState) -> float:
    """미션 상태에 따라 기본 목표 속도를 선택한다(안전 우선)."""
    # 현재 미션 상태에 맞는 목표 속도를 선택한다.
    if state == MissionState.WAIT_TRAFFIC_LIGHT:
        return 0.0
    if state in (MissionState.PEDESTRIAN_AVOID, MissionState.SCHOOL_ZONE):
        return 2.0
    if state == MissionState.FINISH:
        return 0.0
    return 4.0


def clamp_steering(angle: float) -> float:
    """조향각을 안전 범위로 제한한다."""
    # 조향각을 차량이 허용하는 범위로 제한한다.
    return clamp(angle, -50.0, 50.0)


def clamp_speed(speed: float) -> float:
    """속도를 안전 범위로 제한한다."""
    # 속도 명령을 차량이 허용하는 범위로 제한한다.
    return clamp(speed, 0.0, 8.0)
