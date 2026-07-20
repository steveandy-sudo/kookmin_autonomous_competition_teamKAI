"""Mission Manager V0.2 input, memory, output, and configuration models."""

from dataclasses import dataclass

from track_drive.mission.states import (
    ControlMode,
    MissionState,
    StartSignal,
)


@dataclass(frozen=True)
class MissionObservation:
    """현재 한 update 주기에 들어온 의미 기반 입력만 담는다."""

    now_sec: float

    safety_stop_required: bool = False

    start_signal: StartSignal = StartSignal.UNKNOWN
    start_signal_valid: bool = False
    safety_ready: bool = True

    drive_policy_valid: bool = False
    lane_fallback_valid: bool = False

    camera_cone_valid: bool = False
    camera_cone_count: int = 0
    lidar_cone_valid: bool = False
    lidar_cone_detected: bool = False

    fixed_obstacle_detected: bool = False
    vehicle_detected: bool = False
    shortcut_signal_detected: bool = False
    lap_crossing_detected: bool = False


@dataclass
class MissionContext:
    """여러 update 주기 사이에서 유지되는 Mission Manager의 영속 메모리.

    mission_state는 현재 코스 미션을, control_mode는 그 미션에서
    현재 선택된 제어기를 기억한다.
    `cone_seen_since`는 유효한 콘 검출이 시작된 시각을 기억하고,
    `mode_enter_sec`는 최소 모드 유지 시간을 검사할 때 사용한다.
    `manual_override`는 AUTO 또는 강제 시험 모드가 활성인지 기억한다.
    `lap_count`와 `shortcut_used`는 후속 대회 로직을 위한 예약 필드이며
    V0.1에서는 자동 상태 전이에 사용하지 않는다.
    """

    mission_state: MissionState = MissionState.WAIT_START_SIGNAL
    control_mode: ControlMode = ControlMode.STOP

    state_enter_sec: float = 0.0
    mode_enter_sec: float = 0.0

    red_signal_seen_since: float | None = None
    start_signal_seen_since: float | None = None
    start_signal_armed: bool = False

    cone_seen_since: float | None = None
    cone_missing_since: float | None = None
    cone_last_seen_sec: float | None = None

    drive_valid_since: float | None = None
    lane_fallback_ready_since: float | None = None

    manual_override: str = "AUTO"

    lap_count: int = 0
    shortcut_used: bool = False


_SELECTED_SOURCES = {
    "none",
    "drive_il",
    "cone_rule",
    "lane_fallback",
    "fixed_obstacle_rule",
    "vehicle_rule",
    "shortcut",
}

_SPEED_PROFILES = {
    "stop",
    "normal",
    "cone",
    "fallback",
    "obstacle",
    "vehicle_follow",
    "vehicle_overtake",
    "shortcut",
}


@dataclass(frozen=True)
class MissionDecision:
    """현재 주기에 후단 selector가 따라야 할 상징적 선택 결과."""

    mission_state: MissionState
    control_mode: ControlMode

    selected_source: str
    speed_profile: str
    stop_required: bool

    def __post_init__(self) -> None:
        if self.selected_source not in _SELECTED_SOURCES:
            raise ValueError("unsupported selected_source")
        if self.speed_profile not in _SPEED_PROFILES:
            raise ValueError("unsupported speed_profile")
        if not isinstance(self.stop_required, bool):
            raise TypeError("stop_required must be bool")


@dataclass(frozen=True)
class MissionManagerConfig:
    start_signal_red_hold_sec: float = 0.3
    start_signal_green_hold_sec: float = 0.3

    cone_enter_hold_sec: float = 0.25
    cone_exit_hold_sec: float = 0.7
    cone_min_dwell_sec: float = 1.0
    cone_reenter_cooldown_sec: float = 1.0

    drive_recover_hold_sec: float = 0.4
    lane_fallback_ready_hold_sec: float = 0.2

    minimum_camera_cone_count: int = 4
    maximum_camera_cone_count_for_exit: int = 1

    status_log_period_sec: float = 1.0

    def __post_init__(self) -> None:
        durations = (
            self.start_signal_red_hold_sec,
            self.start_signal_green_hold_sec,
            self.cone_enter_hold_sec,
            self.cone_exit_hold_sec,
            self.cone_min_dwell_sec,
            self.cone_reenter_cooldown_sec,
            self.drive_recover_hold_sec,
            self.lane_fallback_ready_hold_sec,
            self.status_log_period_sec,
        )
        if any(value < 0.0 for value in durations):
            raise ValueError("time configuration values must be non-negative")
        if self.minimum_camera_cone_count < 0:
            raise ValueError(
                "minimum_camera_cone_count must be non-negative"
            )
        if self.maximum_camera_cone_count_for_exit < 0:
            raise ValueError(
                "maximum_camera_cone_count_for_exit must be non-negative"
            )
        if (
            self.maximum_camera_cone_count_for_exit
            >= self.minimum_camera_cone_count
        ):
            raise ValueError(
                "camera cone exit count must be lower than entry count"
            )
