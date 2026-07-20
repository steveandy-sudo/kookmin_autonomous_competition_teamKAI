"""ROS에 의존하지 않는 Mission Manager V0.1 전이 코어."""

import math
from typing import Callable, Optional

from track_drive.mission.mission_types import (
    MissionDecision,
    MissionManagerConfig,
    MissionObservation,
    MissionContext,
)
from track_drive.mission.states import ControlMode, MissionState


StatusLogger = Callable[[str], None]

_SUPPORTED_OVERRIDES = {
    "AUTO",
    "START",
    "NORMAL_IL",
    "CONE_DRIVE_RULE",
    "LANE_FALLBACK",
    "STOP",
    "EMERGENCY_STOP",
    "RESET_EMERGENCY",
}

_FORCED_MODES = {
    "NORMAL_IL",
    "CONE_DRIVE_RULE",
    "LANE_FALLBACK",
    "STOP",
}

_ACTIVE_MODES = {
    ControlMode.STOP,
    ControlMode.NORMAL_IL,
    ControlMode.CONE_DRIVE_RULE,
    ControlMode.LANE_FALLBACK,
}

_DECISION_BY_MODE = {
    ControlMode.STOP: ("none", "stop", True),
    ControlMode.NORMAL_IL: ("drive_il", "normal", False),
    ControlMode.CONE_DRIVE_RULE: ("cone_rule", "cone", False),
    ControlMode.LANE_FALLBACK: ("lane_fallback", "fallback", False),
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
        self._pending_reset_emergency = False
        self._pending_manual_emergency = False

    def set_logger(self, logger: Optional[StatusLogger]) -> None:
        """ROS logger 또는 테스트 callback을 연결한다."""

        self._logger = logger
        self._last_logged_state_mode = None
        self._last_status_log_sec = None

    def set_manual_override(self, command: str) -> bool:
        """수동 시험 명령을 저장한다. 지원하지 않는 명령은 안전하게 무시한다."""

        normalized = command.strip().upper()
        if normalized not in _SUPPORTED_OVERRIDES:
            return False

        if self.context.mission_state is MissionState.EMERGENCY_STOP:
            if normalized == "RESET_EMERGENCY":
                self._pending_reset_emergency = True
                return True
            if normalized == "EMERGENCY_STOP":
                self._pending_reset_emergency = False
                return True
            return False

        # 아직 update되지 않은 수동 E-stop도 일반 명령으로 덮어쓸 수 없다.
        if self._pending_manual_emergency:
            return normalized == "EMERGENCY_STOP"

        if normalized == "START":
            self._pending_start = True
            return True
        if normalized == "RESET_EMERGENCY":
            self._pending_reset_emergency = True
            return True
        if normalized == "EMERGENCY_STOP":
            self._pending_manual_emergency = True
            self.context.manual_override = normalized
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

        if observation.emergency_stop:
            self._enter_emergency_stop(now_sec)
        elif self.context.mission_state is MissionState.EMERGENCY_STOP:
            if self._pending_reset_emergency:
                self._reset_emergency(now_sec)
            else:
                self._set_control_mode(ControlMode.STOP, now_sec)
        elif self._pending_manual_emergency:
            self._enter_emergency_stop(now_sec)
        else:
            self._pending_reset_emergency = False

            if self.context.mission_state is MissionState.WAIT_START_SIGNAL:
                self._update_wait_start(observation)
            elif self.context.mission_state is MissionState.RACING:
                self._update_racing(observation)
            else:
                self._set_mission_state(
                    MissionState.WAIT_START_SIGNAL, now_sec
                )
                self._set_control_mode(ControlMode.STOP, now_sec)

        if self.context.control_mode not in _ACTIVE_MODES:
            self._set_control_mode(ControlMode.STOP, now_sec)

        decision = self._make_decision()
        self._emit_status(now_sec)
        return decision

    def _update_wait_start(self, observation: MissionObservation) -> None:
        now_sec = observation.now_sec
        self._set_control_mode(ControlMode.STOP, now_sec)

        if self._pending_start:
            self._pending_start = False
            self.context.start_signal_seen_since = None
            self._enter_racing(observation)
            return

        if observation.start_signal_go:
            self.context.start_signal_seen_since = self._start_or_rebase(
                self.context.start_signal_seen_since, now_sec
            )
        else:
            self.context.start_signal_seen_since = None

        if self._held_for(
            self.context.start_signal_seen_since,
            now_sec,
            self.config.start_signal_hold_sec,
        ):
            self.context.start_signal_seen_since = None
            self._enter_racing(observation)

    def _enter_racing(self, observation: MissionObservation) -> None:
        now_sec = observation.now_sec
        self._set_mission_state(MissionState.RACING, now_sec)

        override = self.context.manual_override
        if override in _FORCED_MODES:
            self._apply_forced_mode(override, observation)
        elif observation.drive_policy_valid:
            self._set_control_mode(ControlMode.NORMAL_IL, now_sec)
        elif observation.lane_fallback_valid:
            self._set_control_mode(ControlMode.LANE_FALLBACK, now_sec)
        else:
            self._set_control_mode(ControlMode.STOP, now_sec)

    def _update_racing(self, observation: MissionObservation) -> None:
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
            mode = (
                ControlMode.NORMAL_IL
                if observation.drive_policy_valid
                else ControlMode.STOP
            )
        elif override == "CONE_DRIVE_RULE":
            # V0.1 Observation에는 cone controller validity 입력이 없다.
            mode = ControlMode.CONE_DRIVE_RULE
        elif override == "LANE_FALLBACK":
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
        mode = self.context.control_mode
        if mode is ControlMode.NORMAL_IL:
            self._update_normal_il(observation)
        elif mode is ControlMode.CONE_DRIVE_RULE:
            self._update_cone_drive(observation)
        elif mode is ControlMode.LANE_FALLBACK:
            self._update_lane_fallback(observation)
        else:
            self._update_racing_stop(observation)

    def _update_normal_il(self, observation: MissionObservation) -> None:
        now_sec = observation.now_sec
        self.context.drive_valid_since = None

        if self._cone_entry_confirmed(observation):
            self.context.lane_fallback_valid_since = None
            self._set_control_mode(ControlMode.CONE_DRIVE_RULE, now_sec)
            return

        fallback_condition = (
            not observation.drive_policy_valid
            and observation.lane_fallback_valid
        )
        if fallback_condition:
            self.context.lane_fallback_valid_since = self._start_or_rebase(
                self.context.lane_fallback_valid_since, now_sec
            )
            if self._held_for(
                self.context.lane_fallback_valid_since,
                now_sec,
                self.config.lane_fallback_enter_hold_sec,
            ):
                self.context.lane_fallback_valid_since = None
                self.context.cone_seen_since = None
                self._set_control_mode(
                    ControlMode.LANE_FALLBACK, now_sec
                )
            return

        self.context.lane_fallback_valid_since = None
        if not observation.drive_policy_valid:
            self.context.cone_seen_since = None
            self._set_control_mode(ControlMode.STOP, now_sec)

    def _update_lane_fallback(
        self,
        observation: MissionObservation,
    ) -> None:
        now_sec = observation.now_sec
        self.context.cone_seen_since = None
        self.context.cone_missing_since = None
        self.context.lane_fallback_valid_since = None

        if observation.drive_policy_valid:
            self.context.drive_valid_since = self._start_or_rebase(
                self.context.drive_valid_since, now_sec
            )
            if self._held_for(
                self.context.drive_valid_since,
                now_sec,
                self.config.drive_recover_hold_sec,
            ):
                self.context.drive_valid_since = None
                self._set_control_mode(ControlMode.NORMAL_IL, now_sec)
            return

        self.context.drive_valid_since = None
        if not observation.lane_fallback_valid:
            self._set_control_mode(ControlMode.STOP, now_sec)

    def _update_cone_drive(
        self,
        observation: MissionObservation,
    ) -> None:
        now_sec = observation.now_sec
        self.context.cone_seen_since = None
        self.context.drive_valid_since = None
        self.context.lane_fallback_valid_since = None

        if observation.cone_detected:
            self.context.cone_last_seen_sec = now_sec
            self.context.cone_missing_since = None
        else:
            self.context.cone_missing_since = self._start_or_rebase(
                self.context.cone_missing_since, now_sec
            )

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
            and observation.cone_exit_ready
            and missing_complete
        ):
            return

        if observation.drive_policy_valid:
            next_mode = ControlMode.NORMAL_IL
        elif observation.lane_fallback_valid:
            next_mode = ControlMode.LANE_FALLBACK
        else:
            next_mode = ControlMode.STOP
        self._set_control_mode(next_mode, now_sec)

    def _update_racing_stop(
        self,
        observation: MissionObservation,
    ) -> None:
        now_sec = observation.now_sec
        self.context.drive_valid_since = None
        self.context.lane_fallback_valid_since = None

        if self._cone_entry_confirmed(observation):
            self._set_control_mode(ControlMode.CONE_DRIVE_RULE, now_sec)
        elif observation.drive_policy_valid:
            self.context.cone_seen_since = None
            self._set_control_mode(ControlMode.NORMAL_IL, now_sec)
        elif observation.lane_fallback_valid:
            self.context.cone_seen_since = None
            self._set_control_mode(ControlMode.LANE_FALLBACK, now_sec)

    def _cone_entry_confirmed(
        self,
        observation: MissionObservation,
    ) -> bool:
        now_sec = observation.now_sec

        if observation.cone_detected:
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

        valid_cone = (
            observation.cone_detected
            and observation.cone_count >= self.config.minimum_cone_count
            and observation.cone_confidence
            >= self.config.minimum_cone_confidence
        )
        if not valid_cone:
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

    def _set_mission_state(
        self,
        state: MissionState,
        now_sec: float,
    ) -> None:
        if self.context.mission_state is state:
            return
        self.context.mission_state = state
        self.context.state_enter_sec = now_sec

    def _set_control_mode(
        self,
        mode: ControlMode,
        now_sec: float,
    ) -> None:
        previous = self.context.control_mode
        if previous is mode:
            return

        if (
            previous is ControlMode.CONE_DRIVE_RULE
            and mode is not ControlMode.CONE_DRIVE_RULE
        ):
            self._cone_exit_sec = now_sec
            self.context.cone_seen_since = None
            self.context.cone_missing_since = None

        if mode is ControlMode.CONE_DRIVE_RULE:
            self.context.cone_seen_since = None
            self.context.cone_missing_since = None

        self.context.control_mode = mode
        self.context.mode_enter_sec = now_sec

    def _enter_emergency_stop(self, now_sec: float) -> None:
        self.context.manual_override = "EMERGENCY_STOP"
        self._pending_start = False
        self._pending_reset_emergency = False
        self._pending_manual_emergency = False
        self._set_mission_state(MissionState.EMERGENCY_STOP, now_sec)
        self._set_control_mode(ControlMode.STOP, now_sec)
        self._clear_confirmation_timers()

    def _reset_emergency(self, now_sec: float) -> None:
        self.context.manual_override = "AUTO"
        self._pending_start = False
        self._pending_reset_emergency = False
        self._pending_manual_emergency = False
        self._set_mission_state(MissionState.WAIT_START_SIGNAL, now_sec)
        self._set_control_mode(ControlMode.STOP, now_sec)
        self._clear_confirmation_timers()
        self._cone_exit_sec = None

    def _make_decision(self) -> MissionDecision:
        state = self.context.mission_state
        mode = self.context.control_mode
        if state is not MissionState.RACING:
            mode = ControlMode.STOP

        source, profile, stop_required = _DECISION_BY_MODE[mode]
        return MissionDecision(
            mission_state=state,
            control_mode=mode,
            selected_source=source,
            speed_profile=profile,
            stop_required=stop_required,
        )

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
        self.context.lane_fallback_valid_since = None

    def _clear_confirmation_timers(self) -> None:
        self.context.start_signal_seen_since = None
        self._reset_automatic_mode_timers()

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
