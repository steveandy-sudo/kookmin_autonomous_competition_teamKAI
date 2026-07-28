#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""국민대 예선 통합 주행 노드에서 공통으로 쓰는 설정값입니다.

초보자는 우선 이 파일만 열어 임계값과 시간값을 바꾸면 됩니다.
차선/라바콘 알고리즘의 게인과 내부 로직은 각 core 파일에 그대로 보존되어 있습니다.
"""

# =====================================================================
# ROS 토픽 / 실행 주기
# =====================================================================
CAMERA_TOPIC = "/image_raw"
SCAN_TOPIC = "/scan"
IMU_TOPIC = "/imu"
MOTOR_TOPIC = "/xycar_motor"
LANE_DEBUG_TOPIC = "/lane_detection/debug_image"

CONTROL_PERIOD_SEC = 0.05
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# drive_mode 파라미터 기본값입니다. shadow에서는 어떤 경우에도 /xycar_motor를 발행하지 않습니다.
DEFAULT_DRIVE_MODE = "shadow"  # "shadow" 또는 "auto"

# Gazebo 트랙 단독 주행에서는 신호등/라바콘 미션 없이 바로 차선 주행부터 시작합니다.
GAZEBO_START_IN_LANE = True

# =====================================================================
# 센서 stale 판정
# =====================================================================
IMAGE_STALE_SEC = 0.50
SCAN_STALE_SEC = 0.50
IMU_STALE_SEC = 1.00
STALE_REQUIRE_IMU = False

# =====================================================================
# FSM 디바운스 / 래치
# =====================================================================
START_CONFIRM_FRAMES = 1
CONE_MISSING_CONFIRM_FRAMES = 8
LANE_STABLE_CONFIRM_FRAMES = 5
CONE_EXIT_LANE_ONLY_FRAMES = 12   # 차선만으로 콘 구간 탈출(안전망): 차선 12프레임(~0.6초) 연속 안정 시 콘 무관 LANE
# 콘 -> 차선 전환 직후에는 lane_core 조향을 유지하고 속도만 잠깐 제한한다.
# 안정된 차선을 확인하고 LANE으로 전환하므로 조향을 0으로 덮어쓰지 않는다.
CONE_EXIT_DELAY_SEC = 0.8         # 전환 후 속도 제한 유지 시간
CONE_EXIT_STRAIGHT_SPEED = 1.2    # 전환 직후 lane_core 속도 상한
DECISION_ANCHOR_CONFIRM_FRAMES = 3

# DECISION(4구 신호 대기)에서 경찰이 없어 좌회전 화살표를 기다리며 정지할 때,
# 진입 직후 잠깐 후진해 정지선과 거리를 벌린다(너무 붙어 정지하면 좌회전 각이 안 나옴).
# speed 음수가 시뮬에서 후진으로 동작해야 의미가 있다(테스트로 확인 필요).
DECISION_REVERSE_SEC = 0.5      # 후진 지속 시간(초)
DECISION_REVERSE_SPEED = -0.8   # 후진 속도(음수=뒤로, 작게 시작)
DECISION_REVERSE_ANGLE = 0.0    # 후진 시 조향(0=직선 후진)

# 라바콘 진입 직후 오검출로 바로 LANE으로 빠지는 것을 막는 최소 시간입니다.
CONE_MIN_SEC_BEFORE_EXIT = 1.0

# DECISION에서 좌회전 화살표를 기다리는 시간입니다. 현재 화살표 인식은 stub입니다.
DECISION_TIMEOUT_SEC = 5.0
# 도착선 통과 후 이 시간(초)간 정지선(DECISION) 감지 무시 (도착선 흑백무늬 오인 방지)
DECISION_AFTER_FINISH_COOLDOWN_SEC = 4.0

# 같은 lap에서 4구 분기 판단이 여러 번 반복되지 않도록 막습니다.
DECISION_REARM_AFTER_FINISH_LINE = True

# =====================================================================
# TURN_LEFT stub 기본값
# =====================================================================
# TODO: 실제 교차로에서 튜닝해야 합니다. angle은 좌회전이므로 음수입니다.
TURN_LEFT_ANGLE = -55.0
TURN_LEFT_SPEED = 1.0
TURN_LEFT_SEC = 1.70
TURN_LEFT_MIN_SEC_BEFORE_LANE_EXIT = 0.70

# True로 바꾸면 IMU yaw 변화량 기반 종료 조건을 함께 사용합니다. 기본은 시간 방식입니다.
TURN_LEFT_USE_IMU_YAW = False
TURN_LEFT_TARGET_YAW_DEG = 85.0
TURN_LEFT_YAW_TOLERANCE_DEG = 12.0

# 지름길(SHORTCUT) 진입 좌회전 하드코딩 값
SHORTCUT_ENTER_ANGLE = -100.0
SHORTCUT_ENTER_SPEED = 1.0
SHORTCUT_ENTER_SEC = 3.2

# =====================================================================
# LiDAR 각도 규약 / 안전 ROI
# =====================================================================
# 문제 조건: 0°=정면, 90°=왼쪽, 180°=후방, 270°=오른쪽입니다.
LIDAR_FRONT_DEG = 0.0
LIDAR_LEFT_DEG = 90.0
LIDAR_REAR_DEG = 180.0
LIDAR_RIGHT_DEG = 270.0

EMERGENCY_FRONT_SECTOR = (-8.0, 8.0)
EMERGENCY_STOP_DIST_M = 0.28
EMERGENCY_WARN_DIST_M = 0.45

# 경찰차 판단 stub을 실제 LiDAR 판정으로 바꿀 때 쓰는 기본 ROI입니다.
POLICE_LEFT_SECTOR = (45.0, 90.0)
POLICE_CLUSTER_MIN_POINTS = 8
POLICE_CLUSTER_MAX_DIST_M = 2.8
ENABLE_POLICE_LIDAR_STUB = False

# =====================================================================
# 보행자(LiDAR) 정지 설정
# =====================================================================
# 친구의 LidarAvoidanceDebug 판정(전방 ROI 안 점 개수)을 우리 좌표(x=전방, y=왼쪽+)로
# 옮긴 것. ROI는 오검출 방지를 위해 좁게 시작한다(전방 ~2m, 좌우 ±0.8m). MAIN 주행 중에만 arm.
ENABLE_PEDESTRIAN_LIDAR = True
PED_ROI_X_MIN = 0.3        # 전방 근거리 시작(m)
PED_ROI_X_MAX = 2.0        # 전방 한계(m)
PED_ROI_Y_ABS = 0.8        # 좌우 ±(m)
PED_SELF_IGNORE_M = 0.45   # 이보다 가까운 점은 무시(차체/마운트)
PED_POINT_THRESHOLD = 5    # ROI 안 점 개수 >= 이 값이면 보행자로 판단
PED_STOP_HOLD_SEC = 1.5    # 감지 시 정지 유지(초, 고정). 감지가 사라졌다 다시 들어오면 재발동.
LANE_POST_PED_WINDING_SPEED_CAP = 1.2  # 보행자 통과 후 차량 회피 시작 전 S자 구간 상한

# 보행자 정지 후 재출발 안정화.
# 정지 중 조향을 0으로 펴면(직진 강제) 곡선에서 멈췄다 출발할 때 바퀴가 0→곡선각으로
# 다시 꺾이는 동안 바깥으로 밀려 차선이탈한다. 그래서 정지 중에도 조향은 lane_core 값을
# 유지(바퀴를 미리 꺾어둠)하고, 속도만 0으로 막는다.
PED_KEEP_LANE_STEER_WHILE_STOPPED = True

# =====================================================================
# 차선 조향/곡선 게인 (GPT 재튜닝 값으로 교체 — 곡선 조향 부족 해결)
# =====================================================================
# lane_core는 이 값들을 getattr로 읽는다. 알고리즘은 그대로이고 게인만 공격적으로 올렸다.
# 곡선에서 더 강하게(HEADING_KP↑), 더 빨리(MAX_STEER_DELTA↑), 더 가까이 보고(FORWARD_PX↓) 꺾는다.
LANE_MAX_STEER_ANGLE = 112.0      # S자 바깥 밀림 방지: 필요할 때 조금 더 꺾을 수 있게 여유
LANE_MAX_STEER_DELTA = 38.0       # 한 프레임 조향각 변화 한계
LANE_PID_KP = 0.86
LANE_PID_KI = 0.0
LANE_PID_KD = 0.24
LANE_HEADING_KP = 40.0
LANE_PREVIEW_HEADING_KP = 18.0
LANE_PID_INTEGRAL_LIMIT = 100.0
LANE_IMAGE_WIDTH = 640
LANE_IMAGE_HEIGHT = 480
LANE_WARP_SRC_POINTS = (
    (130.0, 280.0),
    (500.0, 280.0),
    (640.0, 385.0),
    (0.0, 385.0),
)
LANE_WARP_DST_LEFT_RATIO = 0.10
LANE_WARP_DST_RIGHT_RATIO = 0.90
LANE_WHITE_HSV_LOWER = (0, 0, 185)
LANE_WHITE_HSV_UPPER = (180, 70, 255)
LANE_YELLOW_HSV_LOWER = (22, 130, 130)
LANE_YELLOW_HSV_UPPER = (35, 255, 255)
LANE_MASK_OPEN_KERNEL = (3, 3)
LANE_MASK_CLOSE_KERNEL = (9, 5)
LANE_MASK_OPEN_ITERATIONS = 1
LANE_YELLOW_CLOSE_ITERATIONS = 2
LANE_PEAK_HISTOGRAM_Y_START_RATIO = 2.0 / 3.0
LANE_PEAK_THRESHOLD_RATIO = 0.38
LANE_PEAK_CLUSTER_GAP_PX = 50
LANE_TARGET_FORWARD_PX = 120.0
LANE_CURVE_TARGET_FORWARD_PX = 105.0
LANE_PREVIEW_FORWARD_PX = 190.0
LANE_FAR_PREVIEW_FORWARD_PX = 240.0
LANE_CURVE_HEADING_THRESHOLD = 0.12
LANE_CURVE_AHEAD_PREVIEW_HEADING_TH = 0.085
LANE_CURVE_AHEAD_FAR_HEADING_TH = 0.065
LANE_CURVE_HOLD_FRAMES = 18           # (10→18)
LANE_CURVE_HOLD_MIN_ANGLE = 10.0      # (12→10)
LANE_STRAIGHT_SMOOTH_WINDOW = 4       # (5→4)
LANE_CURVE_SMOOTH_WINDOW = 2          # (3→2)
LANE_MAX_REUSE_FRAMES = 50
LANE_CURVE_REUSE_FRAMES = 12
LANE_YELLOW_CENTER_SWITCH_GATE = 120
LANE_YELLOW_CENTER_PREV_WEIGHT = 0.80
LANE_YELLOW_CENTER_IMAGE_WEIGHT = 0.20
LANE_PATH_SMOOTHING_ALPHA = 0.66
LANE_PATH_RESAMPLE_POINTS = 40
LANE_PATH_FIT_MIN_POINTS = 5
LANE_DEFAULT_HALF_WIDTH_PX = 115.0
LANE_PATH_DUPLICATE_X_GAP = 1.0
LANE_PATH_HEADING_WINDOW = 3
LANE_PATH_MIN_HEADING_DX = 1.0e-3
LANE_CURVATURE_PERCENTILE = 65
LANE_CURVATURE_FILTER_RISE_OLD_WEIGHT = 0.45
LANE_CURVATURE_FILTER_RISE_NEW_WEIGHT = 0.55
LANE_CURVATURE_FILTER_FALL_OLD_WEIGHT = 0.64
LANE_CURVATURE_FILTER_FALL_NEW_WEIGHT = 0.36
LANE_GAUSSIAN_SMOOTH_SIGMA = 0.8
LANE_MIN_CENTER_LINE_POINTS = 4
LANE_SLIDING_WINDOWS = 15
LANE_SLIDING_MARGIN = 50
LANE_SLIDING_MINPIX = 30

# 속도 계획: Gazebo 디버깅용 저속 명령값입니다.
# xycar_motor_bridge 기준 speed_cmd 1.5 ~= 0.12m/s 입니다.
LANE_BASE_SPEED = 1.5
LANE_SPEED_UP_DELTA = 0.12
LANE_SPEED_DOWN_DELTA = 0.20
LANE_FAST_STRAIGHT_RATIO = 1.15
LANE_SHORT_STRAIGHT_RATIO = 1.05
LANE_MILD_CURVE_RATIO = 0.95
LANE_CURVE_RATIO = 0.85
LANE_HARD_CURVE_RATIO = 0.75
LANE_EXTREME_CURVE_RATIO = 0.65
LANE_STRAIGHT_CURVATURE_TH = 0.012
LANE_MID_CURVATURE_TH = 0.016
LANE_HARD_CURVATURE_TH = 0.024
LANE_EXTREME_CURVATURE_TH = 0.032
# legacy: 조향각 기반 감속은 직선 오실레이션 감속 방지를 위해 현재 speed plan에서 쓰지 않는다.
LANE_MILD_CURVE_ANGLE_TH = 25.0
LANE_HARD_CURVE_ANGLE_TH = 45.0
LANE_EXTREME_CURVE_ANGLE_TH = 65.0
LANE_FAST_STRAIGHT_CONFIRM_FRAMES = 2
LANE_FAST_STRAIGHT_ANGLE_TH = 14.0
LANE_FAST_STRAIGHT_HEADING_TH = 0.075
LANE_FAST_STRAIGHT_PREVIEW_TH = 0.070
LANE_FAST_STRAIGHT_FAR_TH = 0.065
LANE_FAST_STRAIGHT_SPEED_UP_DELTA = 0.18
LANE_SCHOOL_ZONE_SPEED_LIMIT = 1.4
LANE_LOST_CURVE_SPEED_RATIO = 0.45
LANE_REUSED_CURVE_SPEED_RATIO = 0.65
LANE_SHORT_LANE_LENGTH_MIN = 1
LANE_SHORT_LANE_LENGTH_MAX = 8
LANE_SHORT_LANE_LENGTH_SCALE = 0.10
LANE_SHORT_LANE_MIN_FACTOR = 0.75
LANE_CURVE_SHORT_LANE_MIN_FACTOR = 0.95
LANE_SCHOOL_ZONE_ENTER_FRAMES = 3
LANE_SCHOOL_ZONE_COUNTER_MAX = 24
LANE_SCHOOL_ZONE_HOLD_FRAMES = 40
LANE_SCHOOL_ZONE_WIDE_SPAN_RATIO = 0.45
LANE_SCHOOL_ZONE_SELECT_LEFT_RATIO = 0.45
LANE_SCHOOL_ZONE_SELECT_RIGHT_RATIO = 0.55
LANE_SCHOOL_ZONE_LEFT_EDGE_RATIO = 0.32
LANE_SCHOOL_ZONE_RIGHT_EDGE_RATIO = 0.68
LANE_EXTERNAL_COMMAND_TIMEOUT = 0.5
LANE_EXTERNAL_MIN_PATH_OFFSET = 1.0
LANE_SYNC_STOP_SPEED_EPS = 0.05
LANE_CURVE_HOLD_BLEND = 0.80
LANE_CURVE_HOLD_RAW_BLEND = 0.20
LANE_S_CURVE_SIGN_HEADING_TH = 0.05
LANE_S_CURVE_MAX_HEADING_TH = 0.08
LANE_S_CURVE_HEADING_SUM_TH = 0.18

# 직선 오실레이션 감속 억제 / 코너 선제 감속.
# 속도 감속 판단은 조향각(angle)이 아니라 경로 curvature와 전방 heading으로 한다.
# 그래서 직선에서 오실레이션 때문에 조향각만 커져도 감속하지 않고,
# far/preview heading이 커지는 코너 진입 전에는 미리 speed guard가 걸린다.
LANE_PRE_CORNER_TARGET_HEADING_TH = 0.10
LANE_PRE_CORNER_PREVIEW_HEADING_TH = 0.085
LANE_PRE_CORNER_FAR_HEADING_TH = 0.065
LANE_PRE_CORNER_NEAR_SHARP_HEADING_TH = 0.17
LANE_PRE_CORNER_PREVIEW_SHARP_HEADING_TH = 0.15
LANE_PRE_CORNER_FAR_SHARP_HEADING_TH = 0.13
LANE_PRE_CORNER_SOFT_HOLD_FRAMES = 22
LANE_PRE_CORNER_HARD_HOLD_FRAMES = 38
LANE_PRE_CORNER_SOFT_CAP_RATIO = 0.95
LANE_PRE_CORNER_HARD_CAP_RATIO = 0.80
LANE_PRE_CORNER_SPEED_UP_DELTA = 0.08
LANE_PRE_CORNER_SPEED_DOWN_DELTA = 0.20

# =====================================================================
# 도착선 검출(흑백 패턴) 설정
# =====================================================================
FINISH_ROI_Y1_RATIO = 0.60
FINISH_ROI_Y2_RATIO = 0.92
FINISH_ROI_X1_RATIO = 0.12
FINISH_ROI_X2_RATIO = 0.88
FINISH_MIN_BLACK_RATIO = 0.08
FINISH_MIN_WHITE_RATIO = 0.08
FINISH_MIN_CONTRAST = 55.0
FINISH_MIN_TRANSITIONS = 5
FINISH_CONFIRM_FRAMES = 2
FINISH_RELEASE_FRAMES = 5
FINISH_MIN_INTERVAL_SEC = 2.0
TOTAL_LAPS = 3
# 마지막 랩 도착선 감지 후, 즉시 멈추지 않고 이 시간(초)만큼 더 차선주행한 뒤 종료한다.
# 도착선이 차 앞쪽 ROI에서 감지되므로, 선을 확실히 통과해 랩 기록이 완료되도록 한다.
FINISH_EXTRA_DRIVE_SEC = 5.0

# =====================================================================
# 디버그 표시 / 로그
# =====================================================================
SHOW_DEBUG_WINDOW = True
SHOW_LANE_BIRDVIEW_WINDOW = True
SHOW_SHADOW_GUIDE = True
DEBUG_WINDOW_NAME = "TrackDrive FSM Debug"
DEBUG_WINDOW_WIDTH = 960
DEBUG_WINDOW_HEIGHT = 720
SUMMARY_LOG_INTERVAL_SEC = 0.70

# shadow 검증 편의용 키입니다. 실제 auto에서도 발행 동작만 다르고 키 처리는 같습니다.
# d: DECISION 강제 진입, l: 도착선 1회 카운트, c: 라바콘 종료 조건 강제, a: 좌회전 결정, s: 직진 결정, t: TURN_LEFT 완료
ENABLE_DEBUG_KEYS = True

# =====================================================================
# 인터럽트 arm 조건
# =====================================================================
# 보행자/차량 인터럽트는 route_plan==MAIN이고 lap<TOTAL_LAPS일 때만 arm합니다.
ARM_MAIN_ROUTE_INTERRUPTS_ONLY = True
