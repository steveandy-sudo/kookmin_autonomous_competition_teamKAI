#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LiDAR LaserScan을 차량 기준 각도/좌표로 다루는 공통 유틸입니다.

여기서는 0°=정면, 90°=왼쪽, 180°=후방, 270°=오른쪽 규약을 기본으로 사용합니다.
긴급 정지와 경찰차 판단 stub을 실제 구현으로 확장할 때 이 파일을 먼저 보면 됩니다.
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from sensor_msgs.msg import LaserScan

from . import config

LidarPoint = Tuple[float, float, float, float, int]


def normalize_deg_180(degree: float) -> float:
    """각도를 -180 이상 180 미만으로 정규화합니다."""
    return (float(degree) + 180.0) % 360.0 - 180.0


def normalize_deg_360(degree: float) -> float:
    """각도를 0 이상 360 미만으로 정규화합니다."""
    return float(degree) % 360.0


def _range_max(scan_msg: LaserScan) -> float:
    """LaserScan의 range_max가 비정상일 때 안전한 기본값을 반환합니다."""
    value = float(getattr(scan_msg, "range_max", 100.0))
    if not math.isfinite(value) or value <= 0.0:
        return 100.0
    return value


def _range_min(scan_msg: LaserScan) -> float:
    """LaserScan의 range_min과 최소 유효 거리 중 큰 값을 반환합니다."""
    value = float(getattr(scan_msg, "range_min", 0.05))
    if not math.isfinite(value) or value <= 0.0:
        return 0.05
    return max(value, 0.05)


def is_valid_range(value: float, scan_msg: Optional[LaserScan] = None) -> bool:
    """nan, inf, 0 이하, 센서 범위 밖 값을 무효 거리로 판단합니다."""
    try:
        distance = float(value)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(distance):
        return False
    if scan_msg is None:
        return distance > 0.05
    return _range_min(scan_msg) <= distance <= _range_max(scan_msg)


def raw_degree_at_index(scan_msg: LaserScan, index: int) -> float:
    """LaserScan index가 나타내는 원시 각도를 degree로 반환합니다."""
    angle_min = math.degrees(float(getattr(scan_msg, "angle_min", 0.0)))
    angle_increment = float(getattr(scan_msg, "angle_increment", 0.0))
    if math.isfinite(angle_increment) and abs(angle_increment) > 1.0e-12:
        inc_deg = math.degrees(angle_increment)
    else:
        inc_deg = 360.0 / max(1, len(scan_msg.ranges))
    return angle_min + float(index) * inc_deg


def vehicle_degree_at_index(scan_msg: LaserScan, index: int) -> float:
    """원시 LaserScan 각도를 차량 규약 각도로 변환합니다."""
    return normalize_deg_360(raw_degree_at_index(scan_msg, index))


def is_angle_in_sector(angle_deg: float, start_deg: float, end_deg: float) -> bool:
    """wrap-around을 고려해 angle_deg가 start~end 섹터 안에 있는지 확인합니다."""
    angle = normalize_deg_360(angle_deg)
    start = normalize_deg_360(start_deg)
    end = normalize_deg_360(end_deg)
    if start <= end:
        return start <= angle <= end
    return angle >= start or angle <= end


def sector_ranges(scan_msg: Optional[LaserScan], start_deg: float, end_deg: float) -> List[float]:
    """특정 각도 섹터의 유효 거리 리스트를 반환합니다."""
    if scan_msg is None or not getattr(scan_msg, "ranges", None):
        return []

    values: List[float] = []
    for i, raw in enumerate(scan_msg.ranges):
        if not is_valid_range(raw, scan_msg):
            continue
        deg = vehicle_degree_at_index(scan_msg, i)
        if is_angle_in_sector(deg, start_deg, end_deg):
            values.append(float(raw))
    return values


def sector_stats(scan_msg: Optional[LaserScan], start_deg: float, end_deg: float) -> Dict[str, float]:
    """특정 각도 섹터의 min/mean/count 통계를 계산합니다."""
    values = sector_ranges(scan_msg, start_deg, end_deg)
    if not values:
        return {"min": float("inf"), "mean": float("inf"), "count": 0.0}
    return {"min": min(values), "mean": sum(values) / len(values), "count": float(len(values))}


def front_min_distance(scan_msg: Optional[LaserScan]) -> float:
    """긴급 정지용 정면 섹터 최소 거리를 반환합니다."""
    stats = sector_stats(scan_msg, *config.EMERGENCY_FRONT_SECTOR)
    return float(stats["min"])


def scan_to_xy_points(scan_msg: Optional[LaserScan], max_distance: Optional[float] = None) -> List[LidarPoint]:
    """LaserScan을 차량 기준 x-y 점 리스트로 변환합니다.

    반환 형식은 (x, y, distance, degree, index)입니다.
    x는 차량 전방, y는 차량 왼쪽이 +입니다.
    """
    if scan_msg is None or not getattr(scan_msg, "ranges", None):
        return []

    limit = _range_max(scan_msg) if max_distance is None else min(float(max_distance), _range_max(scan_msg))
    points: List[LidarPoint] = []
    for i, raw in enumerate(scan_msg.ranges):
        if not is_valid_range(raw, scan_msg):
            continue
        dist = float(raw)
        if dist > limit:
            continue
        deg = vehicle_degree_at_index(scan_msg, i)
        rad = math.radians(deg)
        x = dist * math.cos(rad)
        y = dist * math.sin(rad)
        points.append((x, y, dist, deg, i))
    return points


def filter_points_in_roi(
    points: Sequence[LidarPoint],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> List[LidarPoint]:
    """차량 기준 x-y ROI 박스 안에 들어온 점만 반환합니다."""
    return [p for p in points if x_min <= p[0] <= x_max and y_min <= p[1] <= y_max]


def simple_cluster_count(points: Sequence[LidarPoint], gap_m: float = 0.35) -> int:
    """xy 거리만 이용해 아주 가볍게 cluster 개수를 셉니다."""
    if not points:
        return 0
    pts = sorted(points, key=lambda p: p[4])
    count = 1
    last = pts[0]
    for p in pts[1:]:
        if math.hypot(p[0] - last[0], p[1] - last[1]) > gap_m:
            count += 1
        last = p
    return count


def pedestrian_in_front(
    scan_msg: Optional[LaserScan],
    x_min: float,
    x_max: float,
    y_abs: float,
    self_ignore_m: float,
    point_threshold: int,
) -> Tuple[bool, int, float]:
    """전방 ROI(x=전방 [x_min,x_max], |y|<=y_abs) 안에 self_ignore_m보다 먼 LiDAR 점이
    point_threshold개 이상이면 보행자(전방 장애물)로 판단한다.

    친구의 LidarAvoidanceDebug.analyze_stop을 우리 좌표 규약(x=전방, y=왼쪽+)으로 옮긴 것.
    반환: (detected, point_count, min_distance). 점이 없으면 min_distance는 -1.0.
    """
    points = scan_to_xy_points(scan_msg, max_distance=x_max + 1.0)
    count = 0
    min_dist = float("inf")
    for (x, y, dist, _deg, _idx) in points:
        if dist < self_ignore_m:
            continue
        if x_min <= x <= x_max and -y_abs <= y <= y_abs:
            count += 1
            if dist < min_dist:
                min_dist = dist
    detected = count >= int(point_threshold)
    return detected, count, (min_dist if count > 0 else -1.0)


def is_emergency_collision(scan_msg: Optional[LaserScan]) -> Tuple[bool, float]:
    """정면 가까운 장애물이 있으면 긴급 정지 여부와 최소 거리를 반환합니다."""
    front_min = front_min_distance(scan_msg)
    return front_min <= config.EMERGENCY_STOP_DIST_M, front_min


def police_blocking_left_stub(scan_msg: Optional[LaserScan]) -> Tuple[bool, Dict[str, float]]:
    """경찰차 좌회전 차단 판단 자리입니다.

    현재 기본값은 stub이라 항상 False를 반환합니다. 나중에 실제 구현을 켤 때는
    config.ENABLE_POLICE_LIDAR_STUB=True로 바꾸고 아래 ROI/cluster 조건을 튜닝하면 됩니다.
    """
    stats = sector_stats(scan_msg, *config.POLICE_LEFT_SECTOR)
    debug = {
        "left_min": float(stats["min"]),
        "left_mean": float(stats["mean"]),
        "left_count": float(stats["count"]),
    }

    if not config.ENABLE_POLICE_LIDAR_STUB:
        return False, debug

    points = scan_to_xy_points(scan_msg, max_distance=config.POLICE_CLUSTER_MAX_DIST_M)
    left_points = [
        p for p in points
        if is_angle_in_sector(p[3], *config.POLICE_LEFT_SECTOR) and p[2] <= config.POLICE_CLUSTER_MAX_DIST_M
    ]
    cluster_count = simple_cluster_count(left_points)
    debug["cluster_count"] = float(cluster_count)
    blocked = len(left_points) >= config.POLICE_CLUSTER_MIN_POINTS and cluster_count >= 1
    return blocked, debug


def compact_scan_debug(scan_msg: Optional[LaserScan]) -> Dict[str, float]:
    """디버그 화면에 표시하기 좋은 LiDAR 요약값을 반환합니다."""
    front = sector_stats(scan_msg, -8.0, 8.0)
    left = sector_stats(scan_msg, 45.0, 100.0)
    right = sector_stats(scan_msg, 260.0, 315.0)
    return {
        "front_min": float(front["min"]),
        "front_count": float(front["count"]),
        "left_min": float(left["min"]),
        "right_min": float(right["min"]),
    }
