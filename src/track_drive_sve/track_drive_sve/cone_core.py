#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""완성된 라바콘 주행 코드를 Node에서 분리한 순수 core입니다.

원본 cone_drive.py의 라바콘 알고리즘과 파라미터는 그대로 유지하고,
ROS 구독/발행/타이머를 가진 단독 테스트 노드만 제거했습니다.
"""

import math
from typing import Any, Dict, Iterable, List, MutableMapping, Optional, Sequence, Tuple

import numpy as np
from sensor_msgs.msg import LaserScan

class P:
    """cone_drive.py 안에서만 쓰는 라바콘 튜닝 파라미터 모음."""
    # =====================================================================
    # 방향 보정
    # =====================================================================

    STEER_SIGN = 1.0
    LIDAR_ANGLE_OFFSET_DEG = 0.0
    LIDAR_LEFT_RIGHT_SWAP = False

    # =====================================================================
    # 기본 주행
    # =====================================================================

    DEFAULT_MODE = "FULL_RUN"
    CONTROL_PERIOD = 0.02

    MAX_STEER = 90.0
    CONE_SPEED = 23.0
    CONE_SLOW_SPEED = 11.0
    CONE_STOP_SPEED = 0.0
    CONE_TEST_START_DELAY_SEC = 3.0
    STOP_ON_CONE_LOST = False

    STEER_SMOOTH_ALPHA = 0.78
    TURN_SLOW_ANGLE = 28.0

    FRONT_SLOW_DIST = 0.55
    FRONT_STOP_DIST = 0.18

    # =====================================================================
    # 기본 LiDAR
    # =====================================================================

    CONE_DETECT_DIST = 7.0
    MIN_VALID_RANGE = 0.05
    DEFAULT_RANGE_MAX = 100.0
    MIN_CONE_POINTS = 2
    CONE_LOST_LIMIT = 120
    CONE_MIN_RANGE = 0.45

    # 조향 민감도. 약하면 70~80, 강하면 50~60.
    PATH_STEER_GAIN = 204.0
    PATH_TARGET_X = 2.30
    PATH_MIN_TARGET_X = 0.80
    PATH_MAX_TARGET_X = 4.20
    PATH_MAX_TARGET_Y = 4.3
    PATH_TARGET_SMOOTH_ALPHA = 0.35
    PATH_MAX_TARGET_Y_STEP = 0.28
    PATH_SIGN_FLIP_HOLD_Y = 0.35
    PATH_LOST_KEEP_FRAMES = 30

    # 기존 이름 호환
    CONE_CENTER_GAIN = 32.0
    WALL_FOLLOW_GAIN = 10.0
    CONE_WALL_TARGET_DIST = 1.55

    # =====================================================================
    # raw sector debug용
    # =====================================================================

    RAW_FRONT_SECTOR = (-8.0, 8.0)
    RAW_LEFT_SECTOR = (45.0, 100.0)
    RAW_RIGHT_SECTOR = (-100.0, -45.0)

    # =====================================================================
    # cluster 후보 영역
    # =====================================================================

    CLUSTER_X_MIN = 0.45
    CLUSTER_X_MAX = 6.50
    CLUSTER_Y_ABS_MIN = 0.10
    CLUSTER_Y_ABS_MAX = 4.20
    CLUSTER_MIN_DIST = 0.45
    CLUSTER_MAX_DIST = 7.0
    CLUSTER_INDEX_GAP = 3
    CLUSTER_EUCLIDEAN_GAP = 0.38
    CLUSTER_MIN_POINTS = 1
    CLUSTER_MAX_RADIUS = 0.48

    # =====================================================================
    # gap center planner
    # =====================================================================

    GAP_X_MIN = 0.60
    GAP_X_MAX = 6.00
    GAP_Y_ABS_MIN = 0.10
    GAP_Y_ABS_MAX = 4.20
    GAP_BAND_WIDTH = 2.20
    GAP_BAND_STEP = 0.45
    GAP_MIN_Y_GAP = 0.30

    # 라바콘 두 줄 사이 폭. 노이즈 억제 핵심값.
    # 나무와 라바콘을 엮는 큰 gap이 생기면 GAP_MAX_WIDTH를 낮춰라.
    GAP_MIN_WIDTH = 1.20
    GAP_MAX_WIDTH = 5.80
    GAP_EXPECTED_WIDTH = 4.40

    # 이전 path와 너무 멀리 튄 gap은 reject.
    GAP_MAX_CENTER_DEVIATION = 2.20
    GAP_DEVIATION_GROWTH = 0.25
    GAP_SCORE_LIMIT = 9.00
    GAP_MIN_PATH_POINTS = 1
    GAP_PATH_MAX_SLOPE = 1.60
    GAP_PATH_BASE_JUMP = 0.90

    GAP_SCORE_CENTER_WEIGHT = 2.8
    GAP_SCORE_WIDTH_WEIGHT = 0.25
    GAP_SCORE_X_WEIGHT = 0.25

    # 기존 centerline 이름 호환
    CENTERLINE_X_MIN = GAP_X_MIN
    CENTERLINE_X_MAX = GAP_X_MAX
    CENTERLINE_Y_ABS_MIN = GAP_Y_ABS_MIN
    CENTERLINE_Y_ABS_MAX = GAP_Y_ABS_MAX
    CENTERLINE_BAND_WIDTH = GAP_BAND_WIDTH
    CENTERLINE_BAND_STEP = GAP_BAND_STEP
    CENTERLINE_MIN_Y_GAP = GAP_MIN_Y_GAP
    CENTERLINE_MIN_WIDTH = GAP_MIN_WIDTH
    CENTERLINE_MAX_WIDTH = GAP_MAX_WIDTH
    PATH_EXPECTED_WIDTH = GAP_EXPECTED_WIDTH

    # =====================================================================
    # 기존 left/right ROI fallback/visual용
    # =====================================================================

    CONE_LOOKAHEAD_X_MIN = 0.80
    CONE_LOOKAHEAD_X_MAX = 4.50
    CONE_NEAR_POINT_LIMIT = 20

    LEFT_CONE_X_MIN = 0.80
    LEFT_CONE_X_MAX = 5.50
    LEFT_CONE_Y_MIN = 0.30
    LEFT_CONE_Y_MAX = 4.20

    RIGHT_CONE_X_MIN = 0.80
    RIGHT_CONE_X_MAX = 5.50
    RIGHT_CONE_Y_MIN = -4.20
    RIGHT_CONE_Y_MAX = -0.30

    # =====================================================================
    # side guard
    # =====================================================================
    # 노이즈 때문에 기본 OFF. 정말 충돌 직전만 쓰고 싶으면 True.

    ENABLE_SIDE_GUARD = False
    SIDE_GUARD_X_MIN = -0.05
    SIDE_GUARD_X_MAX = 0.85
    SIDE_GUARD_Y_MIN = 0.12
    SIDE_GUARD_Y_MAX = 0.30
    SIDE_HARD_GUARD_Y = 0.18
    SIDE_GUARD_GAIN = 160.0
    SIDE_EMERGENCY_STEER = 45.0

    USE_RAW_SIDE_CONTROL = False
    RAW_SIDE_MIN_VALID = 0.45
    RAW_SIDE_TARGET_DIST = 0.75
    RAW_SIDE_CENTER_GAIN = 90.0
    RAW_SIDE_DEADBAND = 0.12
    RIGHT_GUARD_Y = 0.30
    LEFT_GUARD_Y = 0.30
    RIGHT_GUARD_DIST = RIGHT_GUARD_Y
    LEFT_GUARD_DIST = LEFT_GUARD_Y
    RIGHT_GUARD_GAIN = 260.0
    LEFT_GUARD_GAIN = 220.0
    SIDE_DANGER_DIST = 0.70
    SIDE_SLOW_DIST = 0.0
    SIDE_STOP_DIST = 0.0
    SIDE_CRAWL_SPEED = 0.55

    # =====================================================================
    # front safety ROI
    # =====================================================================

    FRONT_SAFE_X_MIN = 0.50
    FRONT_SAFE_X_MAX = 2.80
    FRONT_SAFE_Y_MIN = -0.12
    FRONT_SAFE_Y_MAX = 0.12

    # =====================================================================
    # 시각화 범위
    # =====================================================================

    VISUAL_X_MIN = -1.0
    VISUAL_X_MAX = 9.0
    VISUAL_Y_MIN = -4.8
    VISUAL_Y_MAX = 4.8
    VISUAL_MAX_RANGE = 12.0

    # =====================================================================
    # VFH 보조 옵션
    # =====================================================================

    USE_VFH_FOR_CONE = False
    VFH_RADIUS = 0.35
    VFH_ALPHA = 1.6
    VFH_BIN_DEG = 5.0
    VFH_RANGE_LIMIT = 8.0
    VFH_ANGLE_CLAMP_DEG = 55.0
    VFH_SMOOTH_SIGMA = 1.2
    VFH_STEER_SCALE = 1.0
    VFH_CENTER_WEIGHT = 0.12
    VFH_ROI_X_MIN = 0.30
    VFH_ROI_X_MAX = 7.50
    VFH_ROI_Y_MIN = -4.30
    VFH_ROI_Y_MAX = 4.30

    # =====================================================================
    # 로그/CSV
    # =====================================================================

    DEBUG_LOG_INTERVAL = 0.5
    ENABLE_CSV_LOG = False
    CSV_LOG_PATH = "/tmp/xycar_lidar_cone_debug.csv"


    # =====================================================================
    # 오른쪽 라바콘만 남는 후반 구간용 one-side boundary follow
    # =====================================================================
    # 두 줄 gap이 정상일 때는 gap_centerline을 사용하고,
    # gap이 나무/장식물과 잘못 엮이거나 한쪽 줄만 남으면 아래 one-side 모드로 전환합니다.
    ENABLE_ONE_SIDE_FOLLOW = True
    ONE_SIDE_PREFERRED_BOUNDARY = "lower"   # 오른쪽 라바콘 줄은 보통 y_low 경계입니다.
    ONE_SIDE_REQUIRE_MEMORY = True          # 두 줄을 본 뒤에만 한 줄 추종 허용
    ONE_SIDE_MIN_PATH_POINTS = 1

    # 오른쪽 단독 라바콘을 따라갈 때 사용할 반폭. 너무 왼쪽으로 붙으면 낮추고, 너무 오른쪽으로 붙으면 올립니다.
    ONE_SIDE_TARGET_HALF_WIDTH = 2.70
    ONE_SIDE_MIN_HALF_WIDTH = 2.20
    ONE_SIDE_MAX_HALF_WIDTH = 4.00

    # 한 줄 후보를 찾는 범위. 나무가 섞이면 X/Y_MAX를 낮추세요.
    ONE_SIDE_X_MIN = 0.70
    ONE_SIDE_X_MAX = 4.80
    ONE_SIDE_Y_ABS_MIN = 0.15
    ONE_SIDE_Y_ABS_MAX = 3.60
    ONE_SIDE_BAND_WIDTH = 1.40
    ONE_SIDE_BAND_STEP = 0.45

    # 이전에 보던 오른쪽 경계와 너무 멀어진 cluster는 나무/장식물로 보고 버립니다.
    ONE_SIDE_MAX_BOUNDARY_DEVIATION = 1.00
    ONE_SIDE_DEVIATION_GROWTH = 0.18
    ONE_SIDE_SCORE_LIMIT = 3.20
    ONE_SIDE_CLUSTER_COUNT_BONUS = 0.05

    # gap이 잡혔더라도 이전 좌/우 경계 기억과 안 맞으면 나무와 pair된 것으로 보고 one-side로 넘깁니다.
    GAP_MEMORY_CHECK = True
    GAP_REJECT_BOUNDARY_DEV = 1.25
    GAP_REJECT_WIDTH_DELTA = 1.70

    # 한 줄 모드에서 target이 갑자기 튀는 것을 더 강하게 막습니다.
    ONE_SIDE_TARGET_SMOOTH_ALPHA = 0.25
    ONE_SIDE_MAX_TARGET_Y_STEP = 0.45

    # =====================================================================
    # One-side 후반부 안전 거리 보정
    # =====================================================================
    # 오른쪽 라바콘 한 줄만 남았을 때, 두 줄 구간의 센터라인처럼 더 떨어져 주행하기 위한 값입니다.
    # ONE_SIDE_EXTRA_OFFSET은 기억된 반폭에 추가로 더하는 여유 거리입니다.
    ONE_SIDE_EXTRA_OFFSET = 0.35

    # 후반부에는 오른쪽 경계(lower)를 따라가는 것이 목적이므로, upper fallback은 기본적으로 막습니다.
    # True로 바꾸면 upper도 시도하지만 나무/장식물을 경계로 착각할 가능성이 커집니다.
    ONE_SIDE_ALLOW_UPPER_FALLBACK = False


    # =====================================================================
    # 차선주행 기본 파라미터
    # =====================================================================
    # lane_detector.py 초기 뼈대용 값입니다. 실제 시뮬 화면을 보면서 조정하세요.
    LANE_SPEED = 12.0
    LANE_SLOW_SPEED = 8.0
    LANE_LOST_SPEED = 4.0
    LANE_STEER_GAIN = 0.13
    LANE_STEER_SMOOTH_ALPHA = 0.65
    LANE_TURN_SLOW_ANGLE = 25.0
    LANE_MIN_CONFIDENCE = 0.15


    # =====================================================================
    # 신호등/미션 FSM 파라미터
    # =====================================================================
    # 기본 실행은 예선 과제 흐름에 맞춰 파란불/녹색 신호를 기다린 뒤 라바콘으로 넘어갑니다.
    # 미션별 단독 테스트는 실행 시 ROS 파라미터로 선택합니다.
    #   ros2 run track_drive_sve track_drive_sve --ros-args -p mode:=CONE_TEST
    #   ros2 run track_drive_sve track_drive_sve --ros-args -p mode:=LANE_TEST
    #   ros2 run track_drive_sve track_drive_sve --ros-args -p mode:=TRAFFIC_TEST
    TRAFFIC_START_SPEED = 0.0
    TRAFFIC_START_HOLD_GREEN = 2
    TRAFFIC_COLOR_THRESHOLD = 500
    TRAFFIC_ROI_Y1_RATIO = 0.0
    TRAFFIC_ROI_Y2_RATIO = 0.5
    TRAFFIC_ROI_X1_RATIO = 0.1
    TRAFFIC_ROI_X2_RATIO = 0.9
    TRAFFIC_DEBUG_SHOW_WINDOW = False
    TRAFFIC_DEBUG_PUBLISH = True

    # 팀원이 준 신호등 코드의 HSV 범위. 원리는 유지하되 파라미터로 분리했습니다.
    TRAFFIC_GREEN_LOWER = (50, 150, 150)
    TRAFFIC_GREEN_UPPER = (85, 255, 255)
    TRAFFIC_RED_LOWER = (0, 150, 150)
    TRAFFIC_RED_UPPER = (10, 255, 255)
    TRAFFIC_YELLOW_LOWER = (20, 200, 60)
    TRAFFIC_YELLOW_UPPER = (35, 255, 255)
    TRAFFIC_WHITE_LOWER = (0, 0, 220)
    TRAFFIC_WHITE_UPPER = (179, 20, 255)

    # 라바콘에서 차선으로 넘어갈 때 cone_lost 하나만 보지 않고 차선 confidence도 같이 볼 수 있게 둔 값.
    # 현재는 라바콘 안정성을 우선해서 cone_lost 발생 시 LANE_ENTRY로 넘깁니다.
    CONE_TO_LANE_MIN_FRAMES = 20
    LANE_ENTRY_CONFIRM_FRAMES = 5

    # 최종 command 안전 처리. clamp만 수행하므로 미션 로직 원리는 바꾸지 않습니다.
    COMMAND_FILTER_ENABLED = True

LidarPoint = Tuple[float, float, float, float, int]


def clamp(value: float, low: float, high: float) -> float:
    """값을 low~high 범위 안으로 제한합니다."""
    return max(low, min(high, value))


def normalize_degree(degree: float) -> float:
    """각도를 -180 이상 180 미만 범위로 정규화합니다."""
    return (degree + 180.0) % 360.0 - 180.0


def _range_max(scan_msg: LaserScan) -> float:
    """LaserScan의 range_max가 비정상일 때 기본값으로 보정합니다."""
    value = float(getattr(scan_msg, "range_max", P.DEFAULT_RANGE_MAX))
    if not math.isfinite(value) or value <= 0.0:
        return P.DEFAULT_RANGE_MAX
    return value


def _range_min(scan_msg: LaserScan) -> float:
    """LaserScan의 range_min과 사용자 최소 유효 거리를 함께 고려합니다."""
    value = float(getattr(scan_msg, "range_min", P.MIN_VALID_RANGE))
    if not math.isfinite(value) or value <= 0.0:
        value = P.MIN_VALID_RANGE
    return max(value, P.MIN_VALID_RANGE)


def is_valid_range(value: float, scan_msg: Optional[LaserScan] = None) -> bool:
    """inf, nan, 0 이하, range_min 미만, range_max 초과 값을 무효로 판단합니다."""
    if value is None:
        return False
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(value):
        return False
    if scan_msg is None:
        return value > P.MIN_VALID_RANGE
    return _range_min(scan_msg) <= value <= _range_max(scan_msg)


def clean_range(value: float, range_max: float) -> float:
    """단일 range 값을 안전하게 보정합니다.

    무효값은 range_max로 치환합니다. 실제 라바콘 판단에서는 무효값을
    점으로 만들지 않으므로, 이 함수는 주로 거리 조회/로그용입니다.
    """
    try:
        value = float(value)
    except (TypeError, ValueError):
        return float(range_max)
    if not math.isfinite(value) or value <= 0.0:
        return float(range_max)
    return float(value)


def clean_ranges(scan_msg: LaserScan) -> List[float]:
    """LaserScan.ranges 전체를 보정한 리스트로 반환합니다."""
    rmax = _range_max(scan_msg)
    return [clean_range(v, rmax) for v in scan_msg.ranges]


def _angle_min_deg(scan_msg: LaserScan) -> float:
    return math.degrees(float(getattr(scan_msg, "angle_min", 0.0)))


def _angle_increment_deg(scan_msg: LaserScan) -> float:
    inc = float(getattr(scan_msg, "angle_increment", 0.0))
    if math.isfinite(inc) and abs(inc) > 1.0e-12:
        return math.degrees(inc)
    n = max(1, len(scan_msg.ranges))
    return 360.0 / float(n)


def raw_degree_at_index(scan_msg: LaserScan, index: int) -> float:
    """LaserScan 원시 index의 각도를 degree로 반환합니다."""
    return _angle_min_deg(scan_msg) + float(index) * _angle_increment_deg(scan_msg)


def vehicle_degree_at_index(scan_msg: LaserScan, index: int) -> float:
    """원시 LiDAR 각도를 차량 기준 각도로 변환합니다.

    params.LIDAR_ANGLE_OFFSET_DEG를 더한 뒤 -180~180 범위로 정규화합니다.
    y 좌우 반전 옵션은 좌표 변환에서 반영하므로 여기서는 각도 offset만 반영합니다.
    """
    return normalize_degree(raw_degree_at_index(scan_msg, index) + P.LIDAR_ANGLE_OFFSET_DEG)


def degree_to_index(scan_msg: LaserScan, degree: float) -> int:
    """차량 기준 degree에 가장 가까운 LaserScan index를 찾습니다.

    angle_min이 0도인지, -180도인지에 관계없이 모든 index의 각도 차이를 비교합니다.
    360개 스캔 기준으로 비용이 작아 디버그/제어 주기에 충분히 사용할 수 있습니다.
    """
    n = len(scan_msg.ranges)
    if n == 0:
        return 0

    target_raw = normalize_degree(float(degree) - P.LIDAR_ANGLE_OFFSET_DEG)
    best_idx = 0
    best_diff = 9999.0
    for i in range(n):
        raw = normalize_degree(raw_degree_at_index(scan_msg, i))
        diff = abs(normalize_degree(raw - target_raw))
        if diff < best_diff:
            best_diff = diff
            best_idx = i
    return best_idx


def get_distance_at_degree(scan_msg: LaserScan, degree: float) -> float:
    """차량 기준 특정 각도에서의 거리 값을 반환합니다."""
    if scan_msg is None or len(scan_msg.ranges) == 0:
        return float("inf")
    idx = degree_to_index(scan_msg, degree)
    return clean_range(scan_msg.ranges[idx], _range_max(scan_msg))


def _is_angle_in_sector(angle: float, start_degree: float, end_degree: float) -> bool:
    """시작/끝 각도 사이에 angle이 들어가는지 확인합니다.

    start <= end 이면 일반 구간, start > end 이면 180도 경계를 넘는 wrap 구간입니다.
    """
    a = normalize_degree(angle)
    s = normalize_degree(start_degree)
    e = normalize_degree(end_degree)
    if s <= e:
        return s <= a <= e
    return a >= s or a <= e


def sector_stats(scan_msg: LaserScan, start_degree: float, end_degree: float) -> Dict[str, float]:
    """차량 기준 각도 구간의 유효 거리 통계를 계산합니다."""
    if scan_msg is None or len(scan_msg.ranges) == 0:
        return {"min": float("inf"), "mean": float("inf"), "count": 0, "start": start_degree, "end": end_degree}

    values: List[float] = []
    for i, raw in enumerate(scan_msg.ranges):
        if not is_valid_range(raw, scan_msg):
            continue
        vehicle_deg = vehicle_degree_at_index(scan_msg, i)
        if P.LIDAR_LEFT_RIGHT_SWAP:
            vehicle_deg = -vehicle_deg
        if _is_angle_in_sector(vehicle_deg, start_degree, end_degree):
            values.append(float(raw))

    if not values:
        return {"min": float("inf"), "mean": float("inf"), "count": 0, "start": start_degree, "end": end_degree}
    return {"min": min(values), "mean": sum(values) / len(values), "count": len(values), "start": start_degree, "end": end_degree}


def scan_to_xy_points(scan_msg: LaserScan, max_distance: Optional[float] = None) -> List[LidarPoint]:
    """LaserScan을 차량 기준 x-y 점 리스트로 변환합니다.

    반환되는 각 점은 (x, y, distance, vehicle_degree, index)입니다.
    x는 차량 전방, y는 차량 왼쪽이 + 입니다. params.LIDAR_LEFT_RIGHT_SWAP이
    True이면 y와 각도 부호를 반전해 좌우를 쉽게 교정합니다.
    """
    if scan_msg is None or len(scan_msg.ranges) == 0:
        return []

    points: List[LidarPoint] = []
    limit = _range_max(scan_msg) if max_distance is None else min(float(max_distance), _range_max(scan_msg))

    for i, raw in enumerate(scan_msg.ranges):
        if not is_valid_range(raw, scan_msg):
            continue
        d = float(raw)
        if d > limit:
            continue

        vehicle_deg = vehicle_degree_at_index(scan_msg, i)
        if P.LIDAR_LEFT_RIGHT_SWAP:
            vehicle_deg = -vehicle_deg

        rad = math.radians(vehicle_deg)
        x = d * math.cos(rad)
        y = d * math.sin(rad)
        points.append((x, y, d, vehicle_deg, i))

    return points


def filter_points_in_roi(
    points: Sequence[LidarPoint],
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> List[LidarPoint]:
    """x-y ROI 박스 안에 있는 점만 반환합니다."""
    return [p for p in points if x_min <= p[0] <= x_max and y_min <= p[1] <= y_max]


def nearest_distance(points: Sequence[LidarPoint]) -> float:
    """점들 중 차량과 가장 가까운 거리입니다. 점이 없으면 inf를 반환합니다."""
    if not points:
        return float("inf")
    return min(math.hypot(p[0], p[1]) for p in points)


def mean_distance(points: Sequence[LidarPoint]) -> float:
    """점들의 평균 거리입니다. 점이 없으면 inf를 반환합니다."""
    if not points:
        return float("inf")
    return sum(math.hypot(p[0], p[1]) for p in points) / len(points)


def mean_xy(points: Sequence[LidarPoint]) -> Tuple[float, float]:
    """점들의 평균 x,y를 반환합니다. 점이 없으면 nan을 반환합니다."""
    if not points:
        return float("nan"), float("nan")
    return sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points)


def median(values: Iterable[float]) -> float:
    """표준 라이브러리만 사용한 median 계산입니다."""
    data = sorted(float(v) for v in values)
    if not data:
        return float("nan")
    mid = len(data) // 2
    if len(data) % 2 == 1:
        return data[mid]
    return 0.5 * (data[mid - 1] + data[mid])


# =====================================================================
# 공통 유틸
# =====================================================================


def _safe_getattr(name: str, default: Any) -> Any:
    return getattr(P, name, default)


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _raw_sector_debug(scan_msg: LaserScan) -> Dict[str, Any]:
    return {
        "front": sector_stats(scan_msg, *P.RAW_FRONT_SECTOR),
        "left": sector_stats(scan_msg, *P.RAW_LEFT_SECTOR),
        "right": sector_stats(scan_msg, *P.RAW_RIGHT_SECTOR),
    }


def _smooth_and_limit_angle(raw_angle: float, state: MutableMapping[str, Any]) -> float:
    """조향 부호 보정, clamp, EMA smoothing."""
    signed = P.STEER_SIGN * float(raw_angle)
    signed = clamp(signed, -P.MAX_STEER, P.MAX_STEER)

    prev = float(state.get("prev_angle", 0.0))
    alpha = clamp(P.STEER_SMOOTH_ALPHA, 0.0, 1.0)
    angle = alpha * signed + (1.0 - alpha) * prev
    angle = clamp(angle, -P.MAX_STEER, P.MAX_STEER)
    state["prev_angle"] = angle
    return angle


def _speed_for_safety(angle: float, front_min: float) -> float:
    if front_min <= P.FRONT_STOP_DIST:
        return P.CONE_STOP_SPEED
    if front_min <= P.FRONT_SLOW_DIST:
        return P.CONE_SLOW_SPEED
    if abs(angle) >= P.TURN_SLOW_ANGLE:
        return P.CONE_SLOW_SPEED
    return P.CONE_SPEED


def _front_min(points: Sequence[LidarPoint]) -> float:
    fpts = filter_points_in_roi(
        points,
        P.FRONT_SAFE_X_MIN,
        P.FRONT_SAFE_X_MAX,
        P.FRONT_SAFE_Y_MIN,
        P.FRONT_SAFE_Y_MAX,
    )
    fpts = [p for p in fpts if p[2] >= 0.30]
    return nearest_distance(fpts)


# =====================================================================
# 후보점 / cluster
# =====================================================================


def _filter_candidate_points(points: Sequence[LidarPoint]) -> List[LidarPoint]:
    """
    cluster 대상 후보점 필터링.

    넓게 보되 너무 가까운 차체/바닥 잡점과 너무 먼 배경은 제거한다.
    """
    x_min = float(_safe_getattr("CLUSTER_X_MIN", 0.45))
    x_max = float(_safe_getattr("CLUSTER_X_MAX", 5.50))
    y_abs_min = float(_safe_getattr("CLUSTER_Y_ABS_MIN", 0.12))
    y_abs_max = float(_safe_getattr("CLUSTER_Y_ABS_MAX", 4.20))
    min_dist = float(_safe_getattr("CLUSTER_MIN_DIST", _safe_getattr("CONE_MIN_RANGE", 0.45)))
    max_dist = float(_safe_getattr("CLUSTER_MAX_DIST", P.CONE_DETECT_DIST))

    out: List[LidarPoint] = []
    for p in points:
        x, y, dist, _deg, _idx = p
        if dist < min_dist or dist > max_dist:
            continue
        if not (x_min <= x <= x_max):
            continue
        ay = abs(y)
        if not (y_abs_min <= ay <= y_abs_max):
            continue
        out.append(p)
    return out


def _cluster_points(points: Sequence[LidarPoint]) -> List[Dict[str, Any]]:
    """
    LaserScan index 순서와 xy 거리 기반의 가벼운 clustering.

    scipy/DBSCAN 없이 동작한다. 나무/콘 모두 cluster가 될 수 있으므로,
    이후 gap planner에서 한 번 더 강하게 거른다.
    """
    if not points:
        return []

    idx_gap = int(_safe_getattr("CLUSTER_INDEX_GAP", 3))
    xy_gap = float(_safe_getattr("CLUSTER_EUCLIDEAN_GAP", 0.38))
    min_pts = int(_safe_getattr("CLUSTER_MIN_POINTS", 1))
    max_radius = float(_safe_getattr("CLUSTER_MAX_RADIUS", 0.48))

    pts = sorted(points, key=lambda p: p[4])
    raw_clusters: List[List[LidarPoint]] = []
    cur: List[LidarPoint] = [pts[0]]

    for p in pts[1:]:
        prev = cur[-1]
        idx_diff = abs(int(p[4]) - int(prev[4]))
        xy_diff = math.hypot(p[0] - prev[0], p[1] - prev[1])

        if idx_diff <= idx_gap and xy_diff <= xy_gap:
            cur.append(p)
        else:
            raw_clusters.append(cur)
            cur = [p]
    raw_clusters.append(cur)

    clusters: List[Dict[str, Any]] = []
    for c in raw_clusters:
        if len(c) < min_pts:
            continue

        xs = [p[0] for p in c]
        ys = [p[1] for p in c]
        ds = [p[2] for p in c]
        idxs = [p[4] for p in c]

        cx = float(median(xs))
        cy = float(median(ys))
        cd = float(median(ds))
        cidx = int(round(median(idxs)))
        radius = max(math.hypot(p[0] - cx, p[1] - cy) for p in c)

        if radius > max_radius:
            continue

        clusters.append(
            {
                "x": cx,
                "y": cy,
                "dist": cd,
                "idx": cidx,
                "count": len(c),
                "radius": float(radius),
            }
        )

    return clusters


# =====================================================================
# gap center planner
# =====================================================================


def _clusters_for_gap(clusters: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    x_min = float(_safe_getattr("GAP_X_MIN", 0.60))
    x_max = float(_safe_getattr("GAP_X_MAX", 5.20))
    y_abs_min = float(_safe_getattr("GAP_Y_ABS_MIN", 0.10))
    y_abs_max = float(_safe_getattr("GAP_Y_ABS_MAX", 4.20))

    out: List[Dict[str, Any]] = []
    for c in clusters:
        x = float(c["x"])
        y = float(c["y"])
        ay = abs(y)
        if x_min <= x <= x_max and y_abs_min <= ay <= y_abs_max:
            out.append(c)
    return out


def _interp_path_y(path: Sequence[Dict[str, Any]], x: float) -> Optional[float]:
    """이전 path에서 x 위치의 예상 center_y를 선형보간한다."""
    if not path:
        return None

    pts = sorted(path, key=lambda d: float(d["x"]))
    if len(pts) == 1:
        return float(pts[0]["y"])

    x = float(x)
    if x <= float(pts[0]["x"]):
        return float(pts[0]["y"])
    if x >= float(pts[-1]["x"]):
        return float(pts[-1]["y"])

    for a, b in zip(pts[:-1], pts[1:]):
        x0 = float(a["x"])
        x1 = float(b["x"])
        if x0 <= x <= x1:
            y0 = float(a["y"])
            y1 = float(b["y"])
            if abs(x1 - x0) < 1.0e-6:
                return y0
            t = (x - x0) / (x1 - x0)
            return (1.0 - t) * y0 + t * y1

    return None


def _predict_center_y(state: MutableMapping[str, Any], x: float) -> float:
    """
    현재 band에서 기대하는 center_y.

    이전 path가 있으면 그것을 우선 사용한다.
    없으면 이전 target_y를 target_x 기준으로 완만하게 확장한다.
    그것도 없으면 0.0이다.
    """
    last_path = state.get("last_gap_path", [])
    pred = _interp_path_y(last_path, x) if isinstance(last_path, list) else None
    if pred is not None and _finite(pred):
        return float(pred)

    target_y = state.get("target_y", 0.0)
    target_x = max(float(state.get("target_x", _safe_getattr("PATH_TARGET_X", 2.20))), 0.6)
    if _finite(target_y):
        # 멀수록 살짝 더 커질 수 있게 하되 과하지 않게 제한.
        scale = clamp(float(x) / target_x, 0.4, 1.4)
        return float(target_y) * scale

    return 0.0


def _gap_candidate_from_pair(c_low: Dict[str, Any], c_high: Dict[str, Any], x_mid: float, source: str) -> Optional[Dict[str, Any]]:
    y_low = float(c_low["y"])
    y_high = float(c_high["y"])
    if y_low > y_high:
        y_low, y_high = y_high, y_low

    width = y_high - y_low
    min_w = float(_safe_getattr("GAP_MIN_WIDTH", 0.70))
    max_w = float(_safe_getattr("GAP_MAX_WIDTH", 2.80))
    if width < min_w or width > max_w:
        return None

    center_y = 0.5 * (y_low + y_high)
    x = 0.5 * (float(c_low["x"]) + float(c_high["x"]))

    # band 대표 x와 실제 pair x를 섞어 target이 튀지 않게 한다.
    x = 0.55 * x + 0.45 * float(x_mid)

    return {
        "x": x,
        "y": center_y,
        "y_low": y_low,
        "y_high": y_high,
        "width": width,
        "source": source,
        "count": int(c_low.get("count", 1)) + int(c_high.get("count", 1)),
    }


def _score_gap(c: Dict[str, Any], state: MutableMapping[str, Any]) -> float:
    x = float(c["x"])
    y = float(c["y"])
    width = float(c["width"])
    pred_y = _predict_center_y(state, x)

    expected_width = float(state.get("gap_width", _safe_getattr("GAP_EXPECTED_WIDTH", 1.55)))
    target_x = float(_safe_getattr("PATH_TARGET_X", 2.20))

    center_w = float(_safe_getattr("GAP_SCORE_CENTER_WEIGHT", 2.8))
    width_w = float(_safe_getattr("GAP_SCORE_WIDTH_WEIGHT", 1.2))
    x_w = float(_safe_getattr("GAP_SCORE_X_WEIGHT", 0.25))

    score = 0.0
    score += center_w * abs(y - pred_y)
    score += width_w * abs(width - expected_width)
    score += x_w * abs(x - target_x)

    # 먼 후보는 나무/장식물일 가능성이 커서 약하게 패널티.
    far_start = float(_safe_getattr("PATH_MAX_TARGET_X", 4.20))
    if x > far_start:
        score += 2.0 * (x - far_start)

    return score


# =====================================================================
# 경계 기억 / 한 줄 라바콘 추종
# =====================================================================

def _median_or_nan(values: Sequence[float]) -> float:
    vals = [float(v) for v in values if _finite(v)]
    if not vals:
        return float("nan")
    return median(vals)


def _path_width_median(path: Sequence[Dict[str, Any]]) -> float:
    return _median_or_nan([float(p.get("width", float("nan"))) for p in path])


def _store_boundary_memory(path: Sequence[Dict[str, Any]], state: MutableMapping[str, Any]) -> None:
    """
    두 줄 gap이 정상적으로 보일 때 y_low/y_high 경계를 기억한다.

    후반에 오른쪽 라바콘 한 줄만 남으면, 이 기억을 기준으로
    visible boundary가 원래 y_low 쪽인지 y_high 쪽인지 판단한다.
    """
    lower_path = []
    upper_path = []
    widths = []

    for p in path:
        if "y_low" not in p or "y_high" not in p:
            continue
        x = float(p["x"])
        y_low = float(p["y_low"])
        y_high = float(p["y_high"])
        width = float(p.get("width", y_high - y_low))
        if not (_finite(x) and _finite(y_low) and _finite(y_high) and _finite(width)):
            continue
        lower_path.append({"x": x, "y": y_low})
        upper_path.append({"x": x, "y": y_high})
        widths.append(width)

    if lower_path and upper_path:
        state["last_lower_boundary_path"] = sorted(lower_path, key=lambda d: float(d["x"]))
        state["last_upper_boundary_path"] = sorted(upper_path, key=lambda d: float(d["x"]))
        w = _median_or_nan(widths)
        if _finite(w) and 0.3 < w < 8.0:
            state["gap_width"] = w
            state["last_half_width"] = 0.5 * w
        state["boundary_memory_valid"] = True


def _interp_simple_path(path: Sequence[Dict[str, Any]], x: float) -> Optional[float]:
    if not path:
        return None
    pts = sorted(path, key=lambda d: float(d["x"]))
    if len(pts) == 1:
        return float(pts[0]["y"])
    x = float(x)
    if x <= float(pts[0]["x"]):
        return float(pts[0]["y"])
    if x >= float(pts[-1]["x"]):
        return float(pts[-1]["y"])
    for a, b in zip(pts[:-1], pts[1:]):
        x0 = float(a["x"])
        x1 = float(b["x"])
        if x0 <= x <= x1:
            y0 = float(a["y"])
            y1 = float(b["y"])
            if abs(x1 - x0) < 1.0e-6:
                return y0
            t = (x - x0) / (x1 - x0)
            return (1.0 - t) * y0 + t * y1
    return None


def _expected_boundary_y(state: MutableMapping[str, Any], side: str, x: float) -> Optional[float]:
    """이전 프레임 기억에서 lower/upper 경계 y를 예측한다."""
    key = "last_lower_boundary_path" if side == "lower" else "last_upper_boundary_path"
    path = state.get(key, [])
    if isinstance(path, list):
        y = _interp_simple_path(path, x)
        if y is not None and _finite(y):
            return float(y)

    # boundary path가 없으면 마지막 center와 half width로 예측한다.
    center = _predict_center_y(state, x)
    half = float(state.get("last_half_width", _safe_getattr("ONE_SIDE_TARGET_HALF_WIDTH", 2.15)))
    if _finite(center) and _finite(half):
        return float(center - half if side == "lower" else center + half)
    return None


def _gap_path_memory_ok(path: Sequence[Dict[str, Any]], state: MutableMapping[str, Any]) -> bool:
    """
    gap path가 이전 경계 기억과 너무 다르면 나무/장식물과 잘못 pair된 것으로 본다.

    특히 후반에 오른쪽 라바콘만 남았을 때, 오른쪽 라바콘 + 나무가 gap으로 잡히면
    y_low는 그럭저럭 맞고 y_high가 크게 튀는 경우가 많다. 이 경우 gap_centerline을 버리고
    one-side boundary follow로 넘긴다.
    """
    if not bool(_safe_getattr("GAP_MEMORY_CHECK", True)):
        return True
    if not path:
        return False
    if not bool(state.get("boundary_memory_valid", False)):
        return True

    lower_devs = []
    upper_devs = []
    widths = []
    for p in path:
        if "y_low" not in p or "y_high" not in p:
            continue
        x = float(p["x"])
        y_low = float(p["y_low"])
        y_high = float(p["y_high"])
        exp_low = _expected_boundary_y(state, "lower", x)
        exp_high = _expected_boundary_y(state, "upper", x)
        if exp_low is not None and _finite(exp_low):
            lower_devs.append(abs(y_low - exp_low))
        if exp_high is not None and _finite(exp_high):
            upper_devs.append(abs(y_high - exp_high))
        if _finite(p.get("width", float("nan"))):
            widths.append(float(p["width"]))

    max_dev = float(_safe_getattr("GAP_REJECT_BOUNDARY_DEV", 1.25))
    width_jump = float(_safe_getattr("GAP_REJECT_WIDTH_DELTA", 1.70))
    prev_w = float(state.get("gap_width", _safe_getattr("GAP_EXPECTED_WIDTH", 4.4)))
    cur_w = _median_or_nan(widths)

    low_dev = _median_or_nan(lower_devs)
    high_dev = _median_or_nan(upper_devs)

    # 폭이 갑자기 크게 튀면 reject.
    if _finite(cur_w) and _finite(prev_w) and abs(cur_w - prev_w) > width_jump:
        state["last_gap_reject_reason"] = "width_jump"
        return False

    # 한쪽 경계만 기억과 맞고 다른 쪽이 크게 튀면 나무와 pair된 경우로 본다.
    if _finite(low_dev) and _finite(high_dev):
        if low_dev <= max_dev and high_dev > max_dev:
            state["last_gap_reject_reason"] = "upper_boundary_jump"
            return False
        if high_dev <= max_dev and low_dev > max_dev:
            state["last_gap_reject_reason"] = "lower_boundary_jump"
            return False
        if low_dev > max_dev and high_dev > max_dev:
            state["last_gap_reject_reason"] = "both_boundary_jump"
            return False

    state["last_gap_reject_reason"] = "ok"
    return True


def _half_width_for_one_side(state: MutableMapping[str, Any]) -> float:
    """
    오른쪽 라바콘 한 줄만 남았을 때 사용할 중심선 반폭을 계산한다.

    기존에는 last_half_width를 그대로 써서 후반부에 오른쪽 라바콘에 너무 붙는 문제가 있었다.
    여기서는 두 줄 구간에서 기억한 gap_width/last_half_width를 우선 사용하되,
    최소 반폭과 추가 여유 거리(ONE_SIDE_EXTRA_OFFSET)를 강제로 적용한다.
    """
    default_half = float(_safe_getattr("ONE_SIDE_TARGET_HALF_WIDTH", 2.70))
    min_half = float(_safe_getattr("ONE_SIDE_MIN_HALF_WIDTH", 2.20))
    max_half = float(_safe_getattr("ONE_SIDE_MAX_HALF_WIDTH", 4.00))
    extra = float(_safe_getattr("ONE_SIDE_EXTRA_OFFSET", 0.35))

    candidates: List[float] = []

    last_half = state.get("last_half_width", float("nan"))
    if _finite(last_half):
        candidates.append(float(last_half))

    gap_width = state.get("gap_width", float("nan"))
    if _finite(gap_width):
        candidates.append(0.5 * float(gap_width))

    candidates.append(default_half)

    # 너무 작게 기억된 반폭 때문에 오른쪽 라바콘에 붙지 않도록 가장 큰 후보를 사용한다.
    half = max(candidates)
    half = half + extra

    return clamp(half, min_half, max_half)


def _clusters_for_one_side(clusters: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    x_min = float(_safe_getattr("ONE_SIDE_X_MIN", _safe_getattr("GAP_X_MIN", 0.70)))
    x_max = float(_safe_getattr("ONE_SIDE_X_MAX", _safe_getattr("GAP_X_MAX", 5.20)))
    y_min = float(_safe_getattr("ONE_SIDE_Y_ABS_MIN", 0.15))
    y_max = float(_safe_getattr("ONE_SIDE_Y_ABS_MAX", 3.80))
    out = []
    for c in clusters:
        x = float(c["x"])
        y = float(c["y"])
        ay = abs(y)
        if x_min <= x <= x_max and y_min <= ay <= y_max:
            out.append(c)
    return out


def _make_one_side_path(clusters: Sequence[Dict[str, Any]], state: MutableMapping[str, Any]) -> Dict[str, Any]:
    """
    오른쪽 라바콘만 남는 후반 구간용 path 생성.

    기본은 lower boundary 추종이다.
    왼쪽 커브에서 오른쪽 라바콘 줄이 차량 기준 왼쪽에 보여도, 두 줄이 있을 때 저장한
    y_low 경계 기억과 이어지는 cluster를 고르면 계속 오른쪽 경계로 추적할 수 있다.
    """
    if not bool(_safe_getattr("ENABLE_ONE_SIDE_FOLLOW", True)):
        return {"detected": False, "path": [], "candidates": [], "reject_reason": "one_side_disabled"}

    if bool(_safe_getattr("ONE_SIDE_REQUIRE_MEMORY", True)) and not bool(state.get("boundary_memory_valid", False)):
        return {"detected": False, "path": [], "candidates": [], "reject_reason": "no_boundary_memory"}

    valid = _clusters_for_one_side(clusters)
    if not valid:
        return {"detected": False, "path": [], "candidates": [], "reject_reason": "no_one_side_clusters"}

    x_min = float(_safe_getattr("ONE_SIDE_X_MIN", _safe_getattr("GAP_X_MIN", 0.70)))
    x_max = float(_safe_getattr("ONE_SIDE_X_MAX", _safe_getattr("GAP_X_MAX", 5.20)))
    band_w = float(_safe_getattr("ONE_SIDE_BAND_WIDTH", 1.40))
    band_step = float(_safe_getattr("ONE_SIDE_BAND_STEP", 0.45))
    max_dev = float(_safe_getattr("ONE_SIDE_MAX_BOUNDARY_DEVIATION", 1.10))
    dev_growth = float(_safe_getattr("ONE_SIDE_DEVIATION_GROWTH", 0.18))
    score_limit = float(_safe_getattr("ONE_SIDE_SCORE_LIMIT", 3.80))
    half = _half_width_for_one_side(state)
    target_x = float(_safe_getattr("PATH_TARGET_X", 2.20))
    count_bonus = float(_safe_getattr("ONE_SIDE_CLUSTER_COUNT_BONUS", 0.05))

    preferred = str(_safe_getattr("ONE_SIDE_PREFERRED_BOUNDARY", "lower")).lower()
    # 후반부는 오른쪽 라바콘(lower boundary)을 따라가는 것이 목적이다.
    # upper fallback을 허용하면 나무/장식물을 왼쪽 경계로 착각해서 다시 centerline이 튈 수 있다.
    if bool(_safe_getattr("ONE_SIDE_ALLOW_UPPER_FALLBACK", False)):
        sides = [preferred, "upper" if preferred == "lower" else "lower"]
    else:
        sides = [preferred]

    best_result: Optional[Dict[str, Any]] = None

    for side in sides:
        path: List[Dict[str, Any]] = []
        candidates: List[Dict[str, Any]] = []
        x0 = x_min
        while x0 + band_w <= x_max + 1.0e-6:
            x1 = x0 + band_w
            x_mid = 0.5 * (x0 + x1)
            band = [c for c in valid if x0 <= float(c["x"]) <= x1]
            scored = []
            for c in band:
                x = float(c["x"])
                y = float(c["y"])
                exp_y = _expected_boundary_y(state, side, x)
                if exp_y is None or not _finite(exp_y):
                    exp_y = y
                dev = abs(y - exp_y)
                allowed = max_dev + dev_growth * max(x - 1.0, 0.0)
                if dev > allowed:
                    continue

                center_y = y + half if side == "lower" else y - half
                if abs(center_y) > float(_safe_getattr("PATH_MAX_TARGET_Y", 3.0)) + 0.8:
                    continue

                score = dev + 0.18 * abs(x - target_x) - count_bonus * int(c.get("count", 1))
                item = {
                    "x": 0.65 * x + 0.35 * x_mid,
                    "y": center_y,
                    "boundary_y": y,
                    "y_low": y if side == "lower" else center_y - half,
                    "y_high": center_y + half if side == "lower" else y,
                    "width": 2.0 * half,
                    "source": f"one_side_{side}",
                    "boundary_side": side,
                    "score": score,
                    "pred_boundary_y": exp_y,
                    "boundary_dev": dev,
                    "count": int(c.get("count", 1)),
                }
                scored.append(item)
                candidates.append(item)

            if scored:
                best = min(scored, key=lambda d: float(d["score"]))
                if float(best["score"]) <= score_limit:
                    path.append(best)
            x0 += band_step

        # x 중복 제거 및 연속성 필터.
        deduped: List[Dict[str, Any]] = []
        for c in sorted(path, key=lambda d: (float(d["x"]), float(d["score"]))):
            if deduped and abs(float(c["x"]) - float(deduped[-1]["x"])) < 0.25:
                if float(c["score"]) < float(deduped[-1]["score"]):
                    deduped[-1] = c
            else:
                deduped.append(c)

        continuous: List[Dict[str, Any]] = []
        max_slope = float(_safe_getattr("GAP_PATH_MAX_SLOPE", 1.60))
        base_jump = float(_safe_getattr("GAP_PATH_BASE_JUMP", 0.90))
        for c in deduped:
            if not continuous:
                continuous.append(c)
                continue
            prev = continuous[-1]
            dx = max(float(c["x"]) - float(prev["x"]), 0.05)
            dy = abs(float(c["y"]) - float(prev["y"]))
            if dy <= base_jump + max_slope * dx:
                continuous.append(c)

        min_pts = int(_safe_getattr("ONE_SIDE_MIN_PATH_POINTS", 1))
        detected = len(continuous) >= min_pts
        result = {
            "detected": detected,
            "path": continuous,
            "candidates": candidates,
            "reject_reason": "ok" if detected else f"no_one_side_path_{side}",
            "boundary_side": side,
        }
        if detected:
            total_score = sum(float(p.get("score", 0.0)) for p in continuous) / max(len(continuous), 1)
            # preferred side에 약간 보너스.
            if side != preferred:
                total_score += 0.35
            result["mean_score"] = total_score
            if best_result is None or total_score < float(best_result.get("mean_score", 999.0)):
                best_result = result
        elif best_result is None:
            best_result = result

    if best_result is None:
        return {"detected": False, "path": [], "candidates": [], "reject_reason": "one_side_failed"}
    return best_result


def _make_gap_path(clusters: Sequence[Dict[str, Any]], state: MutableMapping[str, Any]) -> Dict[str, Any]:
    """
    band별로 가장 그럴듯한 lateral gap center를 선택해 path를 만든다.

    이 함수가 노이즈 억제의 핵심이다.
    - 각 band에서 모든 pair를 쓰지 않고 하나만 고른다.
    - 선택 기준은 이전 path와의 연속성 + 예상 통로 폭이다.
    - 이전 path와 너무 멀리 튄 gap은 reject한다.
    """
    valid = _clusters_for_gap(clusters)
    if len(valid) < 2:
        return {
            "detected": False,
            "path": [],
            "candidates": [],
            "reject_reason": "not_enough_clusters",
        }

    x_min = float(_safe_getattr("GAP_X_MIN", 0.60))
    x_max = float(_safe_getattr("GAP_X_MAX", 5.20))
    band_w = float(_safe_getattr("GAP_BAND_WIDTH", 1.20))
    band_step = float(_safe_getattr("GAP_BAND_STEP", 0.35))
    min_y_gap = float(_safe_getattr("GAP_MIN_Y_GAP", 0.45))
    max_center_dev = float(_safe_getattr("GAP_MAX_CENTER_DEVIATION", 1.15))
    dev_growth = float(_safe_getattr("GAP_DEVIATION_GROWTH", 0.18))
    score_limit = float(_safe_getattr("GAP_SCORE_LIMIT", 4.20))

    path_initialized = bool(state.get("path_initialized", False))
    selected_path: List[Dict[str, Any]] = []
    all_candidates: List[Dict[str, Any]] = []

    x0 = x_min
    while x0 + band_w <= x_max + 1.0e-6:
        x1 = x0 + band_w
        x_mid = 0.5 * (x0 + x1)
        band = [c for c in valid if x0 <= float(c["x"]) <= x1]
        band = sorted(band, key=lambda c: float(c["y"]))

        band_candidates: List[Dict[str, Any]] = []
        for i in range(len(band) - 1):
            gap = float(band[i + 1]["y"]) - float(band[i]["y"])
            if gap < min_y_gap:
                continue

            cand = _gap_candidate_from_pair(band[i], band[i + 1], x_mid, "gap")
            if cand is None:
                continue

            pred_y = _predict_center_y(state, cand["x"])
            allowed_dev = max_center_dev + dev_growth * max(float(cand["x"]) - 1.0, 0.0)

            # 이전 path가 있는 경우, 갑자기 멀리 튄 gap은 나무/장식물일 가능성이 크다.
            if path_initialized and abs(float(cand["y"]) - pred_y) > allowed_dev:
                continue

            cand["pred_y"] = pred_y
            cand["score"] = _score_gap(cand, state)
            cand["band_x0"] = x0
            cand["band_x1"] = x1
            band_candidates.append(cand)
            all_candidates.append(cand)

        if band_candidates:
            best = min(band_candidates, key=lambda d: float(d["score"]))
            if not path_initialized or float(best["score"]) <= score_limit:
                selected_path.append(best)

        x0 += band_step

    # 중복 band 중심 제거: x가 비슷한 것 중 score가 좋은 것만 유지.
    deduped: List[Dict[str, Any]] = []
    for c in sorted(selected_path, key=lambda d: (float(d["x"]), float(d["score"]))):
        if deduped and abs(float(c["x"]) - float(deduped[-1]["x"])) < 0.25:
            if float(c["score"]) < float(deduped[-1]["score"]):
                deduped[-1] = c
        else:
            deduped.append(c)

    # path 연속성 필터: y가 너무 급격히 튀는 뒷점 제거.
    continuous: List[Dict[str, Any]] = []
    max_slope = float(_safe_getattr("GAP_PATH_MAX_SLOPE", 1.15))
    base_jump = float(_safe_getattr("GAP_PATH_BASE_JUMP", 0.55))
    for c in deduped:
        if not continuous:
            continuous.append(c)
            continue
        prev = continuous[-1]
        dx = max(float(c["x"]) - float(prev["x"]), 0.05)
        dy = abs(float(c["y"]) - float(prev["y"]))
        if dy <= base_jump + max_slope * dx:
            continuous.append(c)

    min_path_points = int(_safe_getattr("GAP_MIN_PATH_POINTS", 1))
    if len(continuous) < min_path_points:
        return {
            "detected": False,
            "path": continuous,
            "candidates": all_candidates,
            "reject_reason": "no_continuous_path",
        }

    return {
        "detected": True,
        "path": continuous,
        "candidates": all_candidates,
        "reject_reason": "ok",
    }


def _select_target_from_path(path: Sequence[Dict[str, Any]], state: MutableMapping[str, Any]) -> Dict[str, Any]:
    if not path:
        return {
            "detected": False,
            "target_x": float("nan"),
            "target_y": float("nan"),
            "raw_target_y": float("nan"),
            "width": float("nan"),
            "reject_reason": "empty_path",
        }

    target_x = float(_safe_getattr("PATH_TARGET_X", 2.20))
    max_target_x = float(_safe_getattr("PATH_MAX_TARGET_X", 4.20))

    near = [p for p in path if float(p["x"]) <= max_target_x]
    if not near:
        return {
            "detected": False,
            "target_x": float("nan"),
            "target_y": float("nan"),
            "raw_target_y": float("nan"),
            "width": float("nan"),
            "reject_reason": "no_near_path",
        }

    # target_x에 가장 가까운 path point 선택.
    best = min(near, key=lambda p: abs(float(p["x"]) - target_x))
    raw_x = float(best["x"])
    raw_y = float(best["y"])
    is_one_side_target = str(best.get("source", "")).startswith("one_side")

    # target_y temporal filtering.
    target_y = raw_y
    last_y = state.get("target_y", None)
    if last_y is not None and _finite(last_y):
        last_y = float(last_y)
        diff = raw_y - last_y

        max_step = float(_safe_getattr("ONE_SIDE_MAX_TARGET_Y_STEP", _safe_getattr("PATH_MAX_TARGET_Y_STEP", 0.28))) if is_one_side_target else float(_safe_getattr("PATH_MAX_TARGET_Y_STEP", 0.28))
        flip_hold_y = float(_safe_getattr("PATH_SIGN_FLIP_HOLD_Y", 0.35))

        # 큰 부호 반전은 한 프레임에 허용하지 않는다.
        if abs(last_y) > flip_hold_y and abs(raw_y) > flip_hold_y and last_y * raw_y < 0.0:
            raw_y = last_y + clamp(diff, -0.35 * max_step, 0.35 * max_step)
        else:
            raw_y = last_y + clamp(diff, -max_step, max_step)

        alpha = float(_safe_getattr("ONE_SIDE_TARGET_SMOOTH_ALPHA", _safe_getattr("PATH_TARGET_SMOOTH_ALPHA", 0.35))) if is_one_side_target else float(_safe_getattr("PATH_TARGET_SMOOTH_ALPHA", 0.35))
        target_y = alpha * last_y + (1.0 - alpha) * raw_y

    target_y = clamp(target_y, -float(_safe_getattr("PATH_MAX_TARGET_Y", 3.0)), float(_safe_getattr("PATH_MAX_TARGET_Y", 3.0)))

    width = float(best.get("width", float("nan")))
    if _finite(width) and width > 0.3:
        state["gap_width"] = width

    _store_boundary_memory(path, state)

    state["target_x"] = raw_x
    state["target_y"] = target_y
    state["last_gap_path"] = [dict(p) for p in path]
    state["path_initialized"] = True
    state["path_lost_count"] = 0
    state["last_path_source"] = str(best.get("source", "gap"))

    return {
        "detected": True,
        "target_x": raw_x,
        "target_y": target_y,
        "raw_target_y": float(best["y"]),
        "width": width,
        "reject_reason": "ok",
        "source": str(best.get("source", "gap")),
        "score": float(best.get("score", 0.0)),
    }


# =====================================================================
# side guard / fallback debug
# =====================================================================


def _side_clearance(points: Sequence[LidarPoint]) -> Tuple[float, float]:
    x_min = float(_safe_getattr("SIDE_GUARD_X_MIN", -0.05))
    x_max = float(_safe_getattr("SIDE_GUARD_X_MAX", 0.85))
    y_min = float(_safe_getattr("SIDE_GUARD_Y_MIN", 0.12))
    y_max = float(_safe_getattr("SIDE_GUARD_Y_MAX", 0.30))
    min_dist = float(_safe_getattr("CONE_MIN_RANGE", 0.45))

    left_vals: List[float] = []
    right_vals: List[float] = []

    for x, y, dist, _deg, _idx in points:
        if dist < min_dist:
            continue
        if not (x_min <= x <= x_max):
            continue
        ay = abs(y)
        if not (y_min <= ay <= y_max):
            continue
        if y > 0:
            left_vals.append(ay)
        elif y < 0:
            right_vals.append(ay)

    left_clear = min(left_vals) if left_vals else float("inf")
    right_clear = min(right_vals) if right_vals else float("inf")
    return left_clear, right_clear


def _feature_from_clusters(clusters: Sequence[Dict[str, Any]], sign: int) -> Dict[str, Any]:
    vals = [c for c in clusters if (float(c["y"]) > 0 if sign > 0 else float(c["y"]) < 0)]
    if not vals:
        return {
            "detected": False,
            "count": 0,
            "dist": float("inf"),
            "mean_x": float("nan"),
            "mean_y": float("nan"),
            "lat_dist": float("inf"),
        }
    near = sorted(vals, key=lambda c: math.hypot(float(c["x"]), float(c["y"])))[: min(len(vals), 8)]
    xs = [float(c["x"]) for c in near]
    ys = [float(c["y"]) for c in near]
    ds = [float(c["dist"]) for c in near]
    return {
        "detected": len(near) >= P.MIN_CONE_POINTS,
        "count": len(vals),
        "dist": min(ds),
        "mean_x": median(xs),
        "mean_y": median(ys),
        "lat_dist": abs(median(ys)),
    }


# =====================================================================
# 메인 주행 함수
# =====================================================================


def cone_center_drive(scan_msg: LaserScan, state: Optional[MutableMapping[str, Any]] = None) -> Dict[str, Any]:
    if state is None:
        state = {}

    if scan_msg is None:
        return {
            "active": False,
            "angle": 0.0,
            "speed": 0.0,
            "cone_lost": True,
            "reason": "no_scan",
            "debug": {"mode": "CONE_CENTER", "reason": "no_scan", "angle": 0.0, "speed": 0.0, "cone_lost": True},
        }

    max_range = max(float(P.CONE_DETECT_DIST), float(_safe_getattr("VFH_RANGE_LIMIT", 8.0)))
    points = scan_to_xy_points(scan_msg, max_distance=max_range)
    candidate_points = _filter_candidate_points(points)
    clusters = _cluster_points(candidate_points)

    raw_gap = _make_gap_path(clusters, state)
    planner_source = "gap"

    # gap이 잡혀도 이전 경계 기억과 안 맞으면 나무/장식물과 잘못 pair된 것으로 보고 버린다.
    gap = dict(raw_gap)
    if gap.get("detected", False) and not _gap_path_memory_ok(gap.get("path", []), state):
        gap["detected"] = False
        gap["reject_reason"] = "memory_reject_" + str(state.get("last_gap_reject_reason", "unknown"))

    # 두 줄 gap이 없거나 reject되면, 오른쪽 라바콘 한 줄만 남은 후반 구간으로 보고 one-side follow를 시도한다.
    one_side = {"detected": False, "path": [], "candidates": [], "reject_reason": "not_tried"}
    if not gap.get("detected", False):
        one_side = _make_one_side_path(clusters, state)
        if one_side.get("detected", False):
            gap = one_side
            planner_source = str(one_side.get("boundary_side", "one_side"))

    target = (
        _select_target_from_path(gap["path"], state)
        if gap.get("detected", False)
        else {
            "detected": False,
            "reject_reason": gap.get("reject_reason", "no_gap"),
            "target_x": float("nan"),
            "target_y": float("nan"),
            "raw_target_y": float("nan"),
            "width": float("nan"),
            "source": "none",
        }
    )

    front_min = _front_min(points)
    raw_sector = _raw_sector_debug(scan_msg)
    left_debug = _feature_from_clusters(clusters, +1)
    right_debug = _feature_from_clusters(clusters, -1)
    left_clear, right_clear = _side_clearance(points)

    raw_angle = 0.0
    cone_error = 0.0
    reason = "no_gap_path"

    if target["detected"]:
        tx = max(float(target["target_x"]), float(_safe_getattr("PATH_MIN_TARGET_X", 0.80)))
        ty = float(target["target_y"])
        bearing = math.atan2(ty, tx)

        raw_angle = -float(_safe_getattr("PATH_STEER_GAIN", 62.0)) * bearing
        cone_error = ty
        state["cone_lost_count"] = 0
        target_source = str(target.get("source", "gap"))
        if target_source.startswith("one_side_lower") or planner_source == "lower":
            reason = "one_side_right_boundary"
        elif target_source.startswith("one_side_upper") or planner_source == "upper":
            reason = "one_side_left_boundary"
        else:
            reason = "gap_centerline"
    else:
        lost = int(state.get("path_lost_count", 0)) + 1
        state["path_lost_count"] = lost
        keep_frames = int(_safe_getattr("PATH_LOST_KEEP_FRAMES", 8))

        if lost <= keep_frames and _finite(state.get("target_y", float("nan"))):
            tx = max(float(state.get("target_x", _safe_getattr("PATH_TARGET_X", 2.20))), 0.8)
            ty = float(state.get("target_y", 0.0))
            raw_angle = -0.55 * float(_safe_getattr("PATH_STEER_GAIN", 62.0)) * math.atan2(ty, tx)
            cone_error = ty
            state["cone_lost_count"] = 0
            reason = "keep_last_gap"
        else:
            state["cone_lost_count"] = int(state.get("cone_lost_count", 0)) + 1
            raw_angle = 0.0
            cone_error = 0.0
            reason = "gap_lost"

    # side guard는 기본 OFF 권장. 켜더라도 hard guard만 매우 좁게 작동한다.
    if bool(_safe_getattr("ENABLE_SIDE_GUARD", False)):
        hard_y = float(_safe_getattr("SIDE_HARD_GUARD_Y", 0.18))
        gain = float(_safe_getattr("SIDE_GUARD_GAIN", 160.0))
        emergency = float(_safe_getattr("SIDE_EMERGENCY_STEER", 45.0))
        if _finite(right_clear) and right_clear < hard_y:
            need = hard_y - right_clear
            guard_angle = -clamp(gain * need, 8.0, emergency)
            if guard_angle < raw_angle:
                raw_angle = guard_angle
                cone_error = -need
                reason = "right_hard_guard"
        elif _finite(left_clear) and left_clear < hard_y:
            need = hard_y - left_clear
            guard_angle = clamp(gain * need, 8.0, emergency)
            if guard_angle > raw_angle:
                raw_angle = guard_angle
                cone_error = need
                reason = "left_hard_guard"

    cone_lost_count = int(state.get("cone_lost_count", 0))
    cone_lost = cone_lost_count >= P.CONE_LOST_LIMIT

    angle = _smooth_and_limit_angle(raw_angle, state)
    speed = _speed_for_safety(angle, front_min)

    if front_min <= P.FRONT_STOP_DIST:
        reason += "|front_stop"
    elif front_min <= P.FRONT_SLOW_DIST:
        reason += "|front_slow"

    if cone_lost:
        reason = "cone_lost"
        if P.STOP_ON_CONE_LOST:
            speed = P.CONE_STOP_SPEED

    path_points = [(float(p["x"]), float(p["y"])) for p in gap.get("path", [])]
    cluster_centers = [(float(c["x"]), float(c["y"])) for c in clusters]

    debug = {
        "mode": "CONE_CENTER",
        "reason": reason,
        "raw_sector": raw_sector,
        "front_min": front_min,
        "left_cone_count": left_debug["count"],
        "right_cone_count": right_debug["count"],
        "left_cone_dist": left_debug["dist"],
        "right_cone_dist": right_debug["dist"],
        "left_cone_y": left_debug["mean_y"],
        "right_cone_y": right_debug["mean_y"],
        "left_cone_lat_dist": left_debug["lat_dist"],
        "right_cone_lat_dist": right_debug["lat_dist"],
        "left_side_clear": left_clear,
        "right_side_clear": right_clear,
        "candidate_point_count": len(candidate_points),
        "cluster_count": len(clusters),
        "candidate_count": len(gap.get("candidates", [])),
        "path_count": len(gap.get("path", [])),
        "path_detected": bool(gap.get("detected", False)),
        "path_reject_reason": str(gap.get("reject_reason", "")),
        "target_detected": bool(target.get("detected", False)),
        "target_x": target.get("target_x", float("nan")),
        "target_y": target.get("target_y", float("nan")),
        "raw_target_y": target.get("raw_target_y", float("nan")),
        "path_width": target.get("width", float("nan")),
        "target_reject_reason": str(target.get("reject_reason", "")),
        "planner_source": planner_source,
        "raw_gap_detected": bool(raw_gap.get("detected", False)),
        "raw_gap_reject_reason": str(raw_gap.get("reject_reason", "")),
        "one_side_detected": bool(one_side.get("detected", False)),
        "one_side_reject_reason": str(one_side.get("reject_reason", "")),
        "one_side_boundary": str(one_side.get("boundary_side", "")),
        "cluster_centers": cluster_centers,
        "path_points": path_points,
        "cone_error": cone_error,
        "raw_angle": raw_angle,
        "angle": angle,
        "speed": speed,
        "cone_lost": cone_lost,
        "cone_lost_count": cone_lost_count,
    }

    return {"active": not cone_lost, "angle": angle, "speed": speed, "cone_lost": cone_lost, "reason": reason, "debug": debug}


# =====================================================================
# VFH 보조 방식 유지
# =====================================================================


def _make_gaussian_kernel(sigma: float) -> np.ndarray:
    sigma = max(0.1, float(sigma))
    half = int(math.ceil(3.0 * sigma))
    x = np.arange(-half, half + 1, dtype=np.float32)
    kernel = np.exp(-0.5 * (x / sigma) ** 2)
    s = float(kernel.sum())
    if s <= 0.0:
        return np.array([1.0], dtype=np.float32)
    return kernel / s


def _vfh_histogram(points: Sequence[LidarPoint]) -> Tuple[np.ndarray, np.ndarray]:
    bin_deg = float(P.VFH_BIN_DEG)
    nbins = max(1, int(round(180.0 / bin_deg)))
    centers = np.linspace(-90.0 + bin_deg / 2.0, 90.0 - bin_deg / 2.0, nbins, dtype=np.float32)
    hist = np.zeros(nbins, dtype=np.float32)
    for x, y, dist, _deg, _idx in points:
        if x <= 0.0:
            continue
        if dist <= P.MIN_VALID_RANGE or dist > P.VFH_RANGE_LIMIT:
            continue
        angle_deg = math.degrees(math.atan2(y, x))
        if abs(angle_deg) > 90.0:
            continue
        ratio = clamp(P.VFH_RADIUS / max(dist, P.MIN_VALID_RANGE), -1.0, 1.0)
        spread = math.degrees(math.asin(ratio))
        weight = 1.0 / (dist ** P.VFH_ALPHA)
        start = clamp(angle_deg - spread, -90.0, 90.0)
        end = clamp(angle_deg + spread, -90.0, 90.0)
        j0 = int(clamp(math.floor((start + 90.0) / bin_deg), 0, nbins - 1))
        j1 = int(clamp(math.floor((end + 90.0) / bin_deg), 0, nbins - 1))
        hist[j0 : j1 + 1] = np.maximum(hist[j0 : j1 + 1], weight)
    return hist, centers


def cone_vfh_drive(scan_msg: LaserScan, state: Optional[MutableMapping[str, Any]] = None) -> Dict[str, Any]:
    if state is None:
        state = {}
    if scan_msg is None:
        return {"active": False, "angle": 0.0, "speed": 0.0, "cone_lost": True, "reason": "no_scan", "debug": {"mode": "CONE_VFH", "reason": "no_scan"}}

    points = scan_to_xy_points(scan_msg, max_distance=max(P.CONE_DETECT_DIST, P.VFH_RANGE_LIMIT))
    vfh_points = filter_points_in_roi(points, P.VFH_ROI_X_MIN, P.VFH_ROI_X_MAX, P.VFH_ROI_Y_MIN, P.VFH_ROI_Y_MAX)
    hist, centers = _vfh_histogram(vfh_points)
    kernel = _make_gaussian_kernel(P.VFH_SMOOTH_SIGMA)
    pad = len(kernel) // 2
    if pad > 0:
        padded = np.pad(hist, (pad, pad), mode="edge")
        smooth = np.convolve(padded, kernel, mode="same")[pad:-pad]
    else:
        smooth = hist.copy()
    center_cost = P.VFH_CENTER_WEIGHT * (np.abs(centers) / 90.0)
    risk = smooth + center_cost
    outer = np.abs(centers) > P.VFH_ANGLE_CLAMP_DEG
    risk[outer] = np.maximum(risk[outer], risk.max(initial=0.0) + 0.1)
    best_idx = int(np.argmin(risk)) if len(risk) else 0
    selected_deg = float(centers[best_idx]) if len(risk) else 0.0
    raw_angle = -selected_deg * P.VFH_STEER_SCALE
    angle = _smooth_and_limit_angle(raw_angle, state)
    front_min = _front_min(points)
    speed = _speed_for_safety(angle, front_min)
    reason = "vfh_roi"
    debug = {"mode": "CONE_VFH", "reason": reason, "raw_sector": _raw_sector_debug(scan_msg), "front_min": front_min, "left_cone_count": 0, "right_cone_count": 0, "left_cone_dist": float("inf"), "right_cone_dist": float("inf"), "cone_error": 0.0, "raw_angle": raw_angle, "angle": angle, "speed": speed, "cone_lost": False, "cone_lost_count": 0, "vfh_point_count": len(vfh_points), "vfh_selected_deg": selected_deg}
    return {"active": True, "angle": angle, "speed": speed, "cone_lost": False, "reason": reason, "debug": debug}


def cone_drive(scan_msg: LaserScan, state: Optional[MutableMapping[str, Any]] = None) -> Dict[str, Any]:
    if P.USE_VFH_FOR_CONE:
        return cone_vfh_drive(scan_msg, state)
    return cone_center_drive(scan_msg, state)


# =====================================================================
# Mission wrapper
# =====================================================================

class ConeMission:
    """라바콘 미션 어댑터.

    기존 cone_drive(scan_msg, state) 함수는 그대로 유지하되, 메인 FSM에서는
    모든 미션을 같은 형태인 mission.update(sensor_data, context)로 호출하기 위해
    이 얇은 래퍼 클래스를 사용합니다.
    """

    name = "CONE"

    def __init__(self):
        self.state = {}

    def reset(self) -> None:
        self.state.clear()

    def update(self, sensor_data, context=None):
        scan_msg = getattr(sensor_data, "scan", None)
        if scan_msg is None:
            return {
                "angle": 0.0,
                "speed": 0.0,
                "done": False,
                "confidence": 0.0,
                "debug": {"reason": "no_scan"},
                "cone_lost": False,
            }

        result = cone_drive(scan_msg, self.state)
        result.setdefault("done", bool(result.get("cone_lost", False)))
        result.setdefault("confidence", 0.0 if result.get("cone_lost", False) else 1.0)
        return result

# =====================================================================
# 미션 단위 wrapper
# =====================================================================


class ConeDrive:
    """최종 track_drive.py에서 호출하기 좋은 라바콘 주행 wrapper."""

    def __init__(self):
        self.state: Dict[str, Any] = {}

    def reset(self) -> None:
        """라바콘 구간에 새로 진입할 때 내부 기억값을 초기화한다."""
        self.state.clear()

    def update(self, scan_msg: Optional[LaserScan]) -> Dict[str, Any]:
        """LaserScan을 받아 angle/speed/debug dict를 반환한다."""
        return cone_drive(scan_msg, self.state)




# =====================================================================
# 통합 노드용 core wrapper
# =====================================================================

class ConeCore:
    """LaserScan을 받아 라바콘 통과 angle/speed를 계산하는 통합용 wrapper입니다."""

    def __init__(self):
        """원본 ConeDrive와 같은 내부 state를 한 번만 만들고 계속 재사용합니다."""
        self.driver = ConeDrive()
        self.last_result: Dict[str, Any] = {
            "angle": 0.0,
            "speed": 0.0,
            "cone_lost": False,
            "debug": {"reason": "초기화"},
        }

    def reset(self) -> None:
        """라바콘 구간 재진입 시 내부 기억값을 초기화합니다."""
        self.driver.reset()
        self.last_result = {
            "angle": 0.0,
            "speed": 0.0,
            "cone_lost": False,
            "debug": {"reason": "reset"},
        }

    def compute(self, scan_msg: Optional[LaserScan]) -> Tuple[float, float]:
        """LaserScan을 받아 라바콘 주행 angle/speed를 반환합니다."""
        result = self.driver.update(scan_msg)
        self.last_result = result
        return float(result.get("angle", 0.0)), float(result.get("speed", 0.0))

    def debug_summary(self) -> Dict[str, Any]:
        """최근 라바콘 계산 디버그 정보를 반환합니다."""
        return dict(self.last_result.get("debug", {}))

    def cone_missing_now(self) -> bool:
        """현재 프레임에서 라바콘 path/target이 사라졌는지 판단합니다."""
        debug = self.last_result.get("debug", {})
        if bool(self.last_result.get("cone_lost", False)):
            return True
        target_detected = bool(debug.get("target_detected", False))
        path_detected = bool(debug.get("path_detected", False))
        return not (target_detected or path_detected)

    def cone_lost_count(self) -> int:
        """원본 라바콘 코드의 cone_lost_count를 반환합니다."""
        debug = self.last_result.get("debug", {})
        return int(debug.get("cone_lost_count", 0))
