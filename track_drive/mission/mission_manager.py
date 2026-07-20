"""ROS에 의존하지 않는 Mission Manager 전이 코어."""

import math
from typing import Callable, Optional

from track_drive.mission.mission_types import (
    MissionDecision,
    MissionManagerConfig,
    MissionObservation,
    MissionContext,
)
from track_drive.mission.states import (
    ControlMode,
    MissionState,
    StartSignal,
)


StatusLogger = Callable[[str], None]

_SUPPORTED_OVERRIDES = {
    "AUTO",
    "START",
    "NORMAL_IL",
    "CONE_DRIVE_RULE",
    "LANE_FALLBACK",
    "STOP",
}

_FORCED_MODES = {
    "NORMAL_IL",
    "CONE_DRIVE_RULE",
    "LANE_FALLBACK",
    "STOP",
}

_DECISION_BY_MODE = {
    ControlMode.STOP: ("none", "stop", True),
    ControlMode.NORMAL_IL: ("drive_il", "normal", False),
    ControlMode.CONE_DRIVE_RULE: ("cone_rule", "cone", False),
    ControlMode.LANE_FALLBACK: ("lane_fallback", "fallback", False),
    ControlMode.FIXED_OBSTACLE_RULE: (
        "fixed_obstacle_rule",
        "obstacle",
        False,
    ),
    ControlMode.VEHICLE_FOLLOW: (
        "vehicle_rule",
        "vehicle_follow",
        False,
    ),
    ControlMode.VEHICLE_OVERTAKE: (
        "vehicle_rule",
        "vehicle_overtake",
        False,
    ),
    ControlMode.SHORTCUT_RULE: ("shortcut", "shortcut", False),
}

_ALLOWED_MODES_BY_STATE = {
    MissionState.WAIT_START_SIGNAL: {ControlMode.STOP},
    MissionState.LANE_DRIVING: {
        ControlMode.STOP,
        ControlMode.NORMAL_IL,
        ControlMode.LANE_FALLBACK,
    },
    MissionState.CONE_SECTION: {
        ControlMode.STOP,
        ControlMode.CONE_DRIVE_RULE,
    },
    MissionState.FIXED_OBSTACLE_SECTION: {
        ControlMode.STOP,
        ControlMode.FIXED_OBSTACLE_RULE,
    },
    MissionState.OVERTAKE_SECTION: {
        ControlMode.STOP,
        ControlMode.NORMAL_IL,
        ControlMode.LANE_FALLBACK,
        ControlMode.VEHICLE_FOLLOW,
        ControlMode.VEHICLE_OVERTAKE,
    },
    MissionState.ROUTE_SELECTION: {
        ControlMode.STOP,
        ControlMode.NORMAL_IL,
        ControlMode.LANE_FALLBACK,
    },
    MissionState.SHORTCUT_SECTION: {
        ControlMode.STOP,
        ControlMode.NORMAL_IL,
        ControlMode.SHORTCUT_RULE,
    },
}


class MissionManager:
    """현재 관측과 영속 context로 상징적인 주행 모드만 결정한다."""

    def __init__(self, config: MissionManagerConfig):
        self.config = config
        self.context = MissionContext()

        self._logger: Optional[StatusLogger] = None
        self._last_logged_state_mode = None
        self._last_status_log_sec: float | None = None

        # 요구된 MissionContext 스키마에는 별도 exit 시각 필드가 없으므로
        # 재진입 cooldown의 기준 시각 하나만 manager 내부에 보존한다.
        self._cone_exit_sec: float | None = None

        # Action 명령은 20 Hz update 전에 다음 명령이 와도 소실되지 않는다.
        self._pending_start = False

    def set_logger(self, logger: Optional[StatusLogger]) -> None:
        """ROS logger 또는 테스트 callback을 연결한다."""

        self._logger = logger
        self._last_logged_state_mode = None
        self._last_status_log_sec = None

    def set_manual_override(self, command: str) -> bool:
        """통합시험 명령을 저장한다. 지원하지 않는 명령은 안전하게 무시한다."""

        normalized = command.strip().upper()
        if normalized not in _SUPPORTED_OVERRIDES:
            return False

        if normalized == "START":
            if self.context.mission_state is not MissionState.WAIT_START_SIGNAL:
                return False
            self._pending_start = True
            return True

        if normalized != self.context.manual_override:
            self._clear_confirmation_timers()
        self.context.manual_override = normalized
        return True

    def update(
        self,
        observation: MissionObservation,
    ) -> MissionDecision:
        """한 주기의 의미 기반 입력을 처리하고 현재 결정을 반환한다."""

        now_sec = float(observation.now_sec)
        if not math.isfinite(now_sec):
            raise ValueError("now_sec must be finite")

        if observation.safety_stop_required:
            self._pending_start = False
            self._clear_confirmation_timers()
            self._set_control_mode(ControlMode.STOP, now_sec)
        else:
            if self.context.mission_state is MissionState.WAIT_START_SIGNAL:
                self._update_wait_start(observation)
            elif self.context.mission_state in {
                MissionState.LANE_DRIVING,
                MissionState.CONE_SECTION,
                MissionState.FIXED_OBSTACLE_SECTION,
                MissionState.OVERTAKE_SECTION,
                MissionState.ROUTE_SELECTION,
                MissionState.SHORTCUT_SECTION,
            }:
                self._update_active_mission(observation)
            else:
                self._set_mission_state(
                    MissionState.WAIT_START_SIGNAL, now_sec
                )
                self._set_control_mode(ControlMode.STOP, now_sec)

        if not self._mode_is_allowed(
            self.context.mission_state,
            self.context.control_mode,
        ):
            self._set_control_mode(ControlMode.STOP, now_sec)

        decision = self._make_decision()
        self._emit_status(now_sec)
        return decision

    def _update_wait_start(self, observation: MissionObservation) -> None:
        now_sec = observation.now_sec
        self._set_control_mode(ControlMode.STOP, now_sec)

        if not observation.safety_ready:
            self._pending_start = False
            self._clear_start_signal_confirmation()
            return

        if self._pending_start:
            self._pending_start = False
            self._clear_start_signal_confirmation()
            self._enter_lane_driving(observation)
            return

        signal = (
            observation.start_signal
            if observation.start_signal_valid
            else StartSignal.UNKNOWN
        )

        if not self.context.start_signal_armed:
            self.context.start_signal_seen_since = None
            if signal is StartSignal.RED:
                self.context.red_signal_seen_since = (
                    self._start_or_rebase(
                        self.context.red_signal_seen_since,
                        now_sec,
                    )
                )
                if self._held_for(
                    self.context.red_signal_seen_since,
                    now_sec,
                    self.config.start_signal_red_hold_sec,
                ):
                    self.context.start_signal_armed = True
                    self.context.red_signal_seen_since = None
            else:
                self.context.red_signal_seen_since = None
            return

        self.context.red_signal_seen_since = None
        if signal is StartSignal.GO:
            self.context.start_signal_seen_since = self._start_or_rebase(
                self.context.start_signal_seen_since, now_sec
            )
        else:
            self.context.start_signal_seen_since = None

        if self._held_for(
            self.context.start_signal_seen_since,
            now_sec,
            self.config.start_signal_go_hold_sec,
        ):
            self._clear_start_signal_confirmation()
            self._enter_lane_driving(observation)

    def _enter_lane_driving(
        self,
        observation: MissionObservation,
    ) -> None:
        now_sec = observation.now_sec
        self._set_mission_state(MissionState.LANE_DRIVING, now_sec)

        override = self.context.manual_override
        if override in _FORCED_MODES:
            self._apply_forced_mode(override, observation)
        elif observation.drive_policy_valid:
            self._set_control_mode(ControlMode.NORMAL_IL, now_sec)
        elif observation.lane_fallback_valid:
            self._set_control_mode(ControlMode.LANE_FALLBACK, now_sec)
        else:
            self._set_control_mode(ControlMode.STOP, now_sec)

    def _update_active_mission(
        self,
        observation: MissionObservation,
    ) -> None:
        self.context.start_signal_seen_since = None

        self._pending_start = False

        override = self.context.manual_override
        if override in _FORCED_MODES:
            self._reset_automatic_mode_timers()
            self._apply_forced_mode(override, observation)
            return

        self._update_automatic_mode(observation)

    def _apply_forced_mode(
        self,
        override: str,
        observation: MissionObservation,
    ) -> None:
        now_sec = observation.now_sec
        if override == "NORMAL_IL":
            self._set_mission_state(MissionState.LANE_DRIVING, now_sec)
            mode = (
                ControlMode.NORMAL_IL
                if observation.drive_policy_valid
                else ControlMode.STOP
            )
        elif override == "CONE_DRIVE_RULE":
            # 현재 Observation에는 cone controller validity 입력이 없다.
            self._set_mission_state(MissionState.CONE_SECTION, now_sec)
            mode = ControlMode.CONE_DRIVE_RULE
        elif override == "LANE_FALLBACK":
            self._set_mission_state(MissionState.LANE_DRIVING, now_sec)
            mode = (
                ControlMode.LANE_FALLBACK
                if observation.lane_fallback_valid
                else ControlMode.STOP
            )
        else:
            mode = ControlMode.STOP
        self._set_control_mode(mode, now_sec)

    def _update_automatic_mode(
        self,
        observation: MissionObservation,
    ) -> None:
        state = self.context.mission_state
        mode = self.context.control_mode

        if state is MissionState.CONE_SECTION:
            if mode is not ControlMode.CONE_DRIVE_RULE:
                self._set_control_mode(
                    ControlMode.CONE_DRIVE_RULE,
                    observation.now_sec,
                )
            self._update_cone_drive(observation)
            return

        if state is not MissionState.LANE_DRIVING:
            # 후속 미션 상태는 V0.2 구조 자리만 정의한다. 실제 장애물,
            # 추월, 경로 선택, 지름길 로직은 아직 구현하지 않는다.
            self._set_control_mode(ControlMode.STOP, observation.now_sec)
            return

        # NORMAL_IL이 선택되어 있을 때도 YOLO lane source의 준비 시간을
        # 백그라운드에서 누적한다. 일반 모델이 hard-invalid가 되었을 때
        # 이미 준비된 fallback으로 즉시 전환하기 위한 메모리다.
        self._track_lane_fallback_readiness(observation)

        if self._cone_entry_confirmed(observation):
            self.context.lane_fallback_ready_since = None
            self._set_mission_state(
                MissionState.CONE_SECTION,
                observation.now_sec,
            )
            self._set_control_mode(
                ControlMode.CONE_DRIVE_RULE,
                observation.now_sec,
            )
            return

        if mode is ControlMode.NORMAL_IL:
            self._update_normal_il(observation)
        elif mode is ControlMode.LANE_FALLBACK:
            self._update_lane_fallback(observation)
        else:
            self._update_lane_stop(observation)

    def _update_normal_il(self, observation: MissionObservation) -> None:
        now_sec = observation.now_sec
        self.context.drive_valid_since = None

        if observation.drive_policy_valid:
            return

        # 입력이 사라진 NORMAL_IL을 확인 시간 동안 계속 선택하지 않는다.
        # 미리 준비된 fallback이 없으면 즉시 recoverable STOP으로 간다.
        self.context.cone_seen_since = None
        if self._lane_fallback_is_ready(now_sec):
            self._set_control_mode(ControlMode.LANE_FALLBACK, now_sec)
        else:
            self._set_control_mode(ControlMode.STOP, now_sec)

    def _update_lane_fallback(
        self,
        observation: MissionObservation,
    ) -> None:
        now_sec = observation.now_sec
        self.context.cone_missing_since = None
        self._track_drive_recovery(observation)

        if self._drive_is_recovered(now_sec):
            self.context.drive_valid_since = None
            self._set_control_mode(ControlMode.NORMAL_IL, now_sec)
            return

        if not observation.lane_fallback_valid:
            # 선택된 fallback source가 hard-invalid이면 한 tick도 계속 쓰지 않는다.
            self._set_control_mode(ControlMode.STOP, now_sec)

    def _update_cone_drive(
        self,
        observation: MissionObservation,
    ) -> None:
        now_sec = observation.now_sec
        self.context.cone_seen_since = None
        self.context.drive_valid_since = None
        self.context.lane_fallback_ready_since = None

        if self._any_valid_cone_evidence(observation):
            self.context.cone_last_seen_sec = now_sec

        exit_candidate = (
            observation.camera_cone_valid
            and observation.lidar_cone_valid
            and observation.camera_cone_count >= 0
            and observation.camera_cone_count
            <= self.config.maximum_camera_cone_count_for_exit
            and not observation.lidar_cone_detected
        )
        if exit_candidate:
            self.context.cone_missing_since = self._start_or_rebase(
                self.context.cone_missing_since, now_sec
            )
        else:
            self.context.cone_missing_since = None

        dwell_complete = self._held_for(
            self.context.mode_enter_sec,
            now_sec,
            self.config.cone_min_dwell_sec,
        )
        missing_complete = self._held_for(
            self.context.cone_missing_since,
            now_sec,
            self.config.cone_exit_hold_sec,
        )
        if not (
            dwell_complete
            and missing_complete
        ):
            return

        if observation.drive_policy_valid:
            next_mode = ControlMode.NORMAL_IL
        elif observation.lane_fallback_valid:
            next_mode = ControlMode.LANE_FALLBACK
        else:
            next_mode = ControlMode.STOP
        self._set_mission_state(MissionState.LANE_DRIVING, now_sec)
        self._set_control_mode(next_mode, now_sec)

    def _update_lane_stop(
        self,
        observation: MissionObservation,
    ) -> None:
        now_sec = observation.now_sec
        self._track_drive_recovery(observation)

        if self._drive_is_recovered(now_sec):
            self.context.drive_valid_since = None
            self.context.cone_seen_since = None
            self._set_control_mode(ControlMode.NORMAL_IL, now_sec)
        elif self._lane_fallback_is_ready(now_sec):
            self.context.cone_seen_since = None
            self._set_control_mode(ControlMode.LANE_FALLBACK, now_sec)

    def _track_lane_fallback_readiness(
        self,
        observation: MissionObservation,
    ) -> None:
        if observation.lane_fallback_valid:
            self.context.lane_fallback_ready_since = self._start_or_rebase(
                self.context.lane_fallback_ready_since,
                observation.now_sec,
            )
        else:
            self.context.lane_fallback_ready_since = None

    def _lane_fallback_is_ready(self, now_sec: float) -> bool:
        return self._held_for(
            self.context.lane_fallback_ready_since,
            now_sec,
            self.config.lane_fallback_ready_hold_sec,
        )

    def _track_drive_recovery(
        self,
        observation: MissionObservation,
    ) -> None:
        if observation.drive_policy_valid:
            self.context.drive_valid_since = self._start_or_rebase(
                self.context.drive_valid_since,
                observation.now_sec,
            )
        else:
            self.context.drive_valid_since = None

    def _drive_is_recovered(self, now_sec: float) -> bool:
        return self._held_for(
            self.context.drive_valid_since,
            now_sec,
            self.config.drive_recover_hold_sec,
        )

    def _cone_entry_confirmed(
        self,
        observation: MissionObservation,
    ) -> bool:
        now_sec = observation.now_sec

        if self._any_valid_cone_evidence(observation):
            self.context.cone_last_seen_sec = now_sec

        if self._cone_exit_sec is not None:
            cooldown_complete = self._held_for(
                self._cone_exit_sec,
                now_sec,
                self.config.cone_reenter_cooldown_sec,
            )
            if not cooldown_complete:
                self.context.cone_seen_since = None
                return False
            self._cone_exit_sec = None

        valid_cone_zone_entry = (
            observation.camera_cone_valid
            and observation.lidar_cone_valid
            and observation.camera_cone_count
            >= self.config.minimum_camera_cone_count
            and observation.lidar_cone_detected
        )
        if not valid_cone_zone_entry:
            self.context.cone_seen_since = None
            return False

        self.context.cone_seen_since = self._start_or_rebase(
            self.context.cone_seen_since, now_sec
        )
        return self._held_for(
            self.context.cone_seen_since,
            now_sec,
            self.config.cone_enter_hold_sec,
        )

    @staticmethod
    def _any_valid_cone_evidence(
        observation: MissionObservation,
    ) -> bool:
        camera_evidence = (
            observation.camera_cone_valid
            and observation.camera_cone_count > 0
        )
        lidar_evidence = (
            observation.lidar_cone_valid
            and observation.lidar_cone_detected
        )
        return camera_evidence or lidar_evidence

    def _set_mission_state(
        self,
        state: MissionState,
        now_sec: float,
    ) -> None:
        if self.context.mission_state is state:
            return
        self.context.mission_state = state
        self.context.state_enter_sec = now_sec
        if not self._mode_is_allowed(
            state,
            self.context.control_mode,
        ):
            self._set_control_mode(ControlMode.STOP, now_sec)

    def _set_control_mode(
        self,
        mode: ControlMode,
        now_sec: float,
    ) -> None:
        if not self._mode_is_allowed(
            self.context.mission_state,
            mode,
        ):
            mode = ControlMode.STOP

        previous = self.context.control_mode
        if previous is mode:
            return

        if (
            previous is ControlMode.CONE_DRIVE_RULE
            and mode is not ControlMode.CONE_DRIVE_RULE
            and self.context.mission_state is not MissionState.CONE_SECTION
        ):
            self._cone_exit_sec = now_sec
            self.context.cone_seen_since = None
            self.context.cone_missing_since = None

        if mode is ControlMode.CONE_DRIVE_RULE:
            self.context.cone_seen_since = None
            self.context.cone_missing_since = None

        self.context.control_mode = mode
        self.context.mode_enter_sec = now_sec

    def _make_decision(self) -> MissionDecision:
        state = self.context.mission_state
        mode = self.context.control_mode
        if not self._mode_is_allowed(state, mode):
            mode = ControlMode.STOP

        source, profile, stop_required = _DECISION_BY_MODE[mode]
        return MissionDecision(
            mission_state=state,
            control_mode=mode,
            selected_source=source,
            speed_profile=profile,
            stop_required=stop_required,
        )

    @staticmethod
    def _mode_is_allowed(
        state: MissionState,
        mode: ControlMode,
    ) -> bool:
        return mode in _ALLOWED_MODES_BY_STATE[state]

    def _emit_status(self, now_sec: float) -> None:
        if self._logger is None:
            return

        current = (
            self.context.mission_state,
            self.context.control_mode,
        )
        if current != self._last_logged_state_mode:
            self._logger(
                "[MISSION] state={} mode={}".format(
                    current[0].name, current[1].name
                )
            )
            self._last_logged_state_mode = current
            self._last_status_log_sec = now_sec
            return

        period = self.config.status_log_period_sec
        if period <= 0.0:
            return
        if self._last_status_log_sec is None or now_sec < self._last_status_log_sec:
            self._last_status_log_sec = now_sec
            return
        if now_sec - self._last_status_log_sec < period:
            return

        self._logger(
            "[MISSION] state={} mode={} lap={} shortcut_used={} override={}".format(
                self.context.mission_state.name,
                self.context.control_mode.name,
                self.context.lap_count,
                str(self.context.shortcut_used).lower(),
                self.context.manual_override,
            )
        )
        self._last_status_log_sec = now_sec

    def _reset_automatic_mode_timers(self) -> None:
        self.context.cone_seen_since = None
        self.context.cone_missing_since = None
        self.context.drive_valid_since = None
        self.context.lane_fallback_ready_since = None

    def _clear_confirmation_timers(self) -> None:
        self._clear_start_signal_confirmation()
        self._reset_automatic_mode_timers()

    def _clear_start_signal_confirmation(self) -> None:
        self.context.red_signal_seen_since = None
        self.context.start_signal_seen_since = None
        self.context.start_signal_armed = False

    @staticmethod
    def _start_or_rebase(
        since: float | None,
        now_sec: float,
    ) -> float:
        if since is None or now_sec < since:
            return now_sec
        return since

    @staticmethod
    def _held_for(
        since: float | None,
        now_sec: float,
        duration_sec: float,
    ) -> bool:
        return since is not None and now_sec >= since + duration_sec
