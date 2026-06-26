"""LiDAR 데이터 처리 유틸리티."""

from __future__ import annotations

import math
from typing import Dict, List, Optional

from sensor_msgs.msg import LaserScan


# 설명: LaserScan 거리 배열에서 NaN, Inf, 0 이하 값을 제거하고 유효 거리만 모은다.
def _clean_ranges(scan: Optional[LaserScan]) -> List[float]:
    """LaserScan ranges에서 유효한 거리값만 추출한다."""
    # LiDAR 유틸리티의 clean ranges 로직을 수행한다.
    if scan is None or not scan.ranges:
        return []

    cleaned: List[float] = []
    for value in scan.ranges:
        if value is None:
            continue
        if math.isnan(value) or math.isinf(value):
            continue
        if value <= 0.0:
            continue
        cleaned.append(float(value))
    return cleaned


# 설명: 스캔 배열의 지정된 비율 구간에서 유효한 최소 거리를 계산한다.
def _sector_min(scan: Optional[LaserScan], start_ratio: float, end_ratio: float) -> float:
    """스캔 배열의 비율 구간에서 최소 거리를 계산한다."""
    # LiDAR 유틸리티의 sector min 로직을 수행한다.
    if scan is None or not scan.ranges:
        return float('inf')

    total = len(scan.ranges)
    start_idx = max(0, int(total * start_ratio))
    end_idx = min(total, int(total * end_ratio))
    if end_idx <= start_idx:
        return float('inf')

    values = []
    for value in scan.ranges[start_idx:end_idx]:
        if value is None or math.isnan(value) or math.isinf(value) or value <= 0.0:
            continue
        values.append(float(value))

    if not values:
        return float('inf')
    return min(values)


# 설명: 점, 경로, 객체 사이의 거리를 계산한다.
def get_front_obstacle_distance(scan: Optional[LaserScan]) -> float:
    """전방 장애물 최소 거리(정면 중심 기준)를 반환한다."""
    # get 전방 장애물 거리 값을 현재 상태에서 계산하거나 조회한다.
    return _sector_min(scan, 0.45, 0.55)


# 설명: 점, 경로, 객체 사이의 거리를 계산한다.
def get_left_right_obstacle_distance(scan: Optional[LaserScan]) -> Dict[str, float]:
    """좌/우 측면 장애물 최소 거리를 반환한다."""
    # get 왼쪽/좌회전 right 장애물 거리 값을 현재 상태에서 계산하거나 조회한다.
    return {
        'left': _sector_min(scan, 0.70, 0.90),
        'right': _sector_min(scan, 0.10, 0.30),
    }


# 설명: 센서 입력에서 대상 상태나 객체를 감지한다.
def detect_cone_like_objects(scan: Optional[LaserScan]) -> Dict[str, bool]:
    """라바콘 유사 패턴 감지 Placeholder.

    TODO: 클러스터링 기반으로 라바콘 폭/거리 특성을 추정하도록 고도화.
    """
    # 입력 데이터에서 detect 콘 like objects 조건을 감지한다.
    valid = _clean_ranges(scan)
    return {
        'cone_like_detected': len(valid) > 10,
    }


# 설명: 센서 입력에서 대상 상태나 객체를 감지한다.
def detect_pedestrian_vehicle_obstacles(scan: Optional[LaserScan]) -> Dict[str, bool]:
    """보행자/차량 장애물 감지 Placeholder.

    TODO: 시간축 추적 + 거리/속도 기반 분류 로직으로 개선.
    """
    # 입력 데이터에서 detect pedestrian 차량 obstacles 조건을 감지한다.
    front = get_front_obstacle_distance(scan)
    return {
        'pedestrian_detected': front < 1.2,
        'vehicle_detected': front < 2.0,
    }
