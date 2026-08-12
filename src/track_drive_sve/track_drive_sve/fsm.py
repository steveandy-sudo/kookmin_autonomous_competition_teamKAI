#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""차선주행 중심 경량 FSM입니다.

상태는 WAIT_START → CONE → LANE을 기본 흐름으로 두고,
분기 판단 DECISION과 짧은 좌회전 실행 TURN_LEFT만 예외 상태로 둡니다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional

from . import config
from .world_model import RoutePlan, WorldModel


class MissionState(Enum):
    """통합 주행에서 사용하는 6개 상태입니다."""

    WAIT_START = "WAIT_START"
    CONE = "CONE"
    LANE = "LANE"
    DECISION = "DECISION"
    TURN_LEFT = "TURN_LEFT"
    FINISH = "FINISH"


@dataclass
class FsmEvents:
    """FSM 한 주기에 필요한 인지/수동 이벤트입니다."""

    start_allowed: bool = True
    cone_missing: bool = False
    lane_stable: bool = False
    decision_anchor: bool = False
    finish_line_crossed: bool = False
    shortcut_done: bool = False
    arrow_green: bool = False
    straight_green: bool = False
    police_blocked: bool = False
    current_yaw_deg: Optional[float] = None
    manual_decision: bool = False
    manual_cone_done: bool = False
    manual_left: bool = False
    manual_straight: bool = False
    manual_turn_done: bool = False


@dataclass
class Transition:
    """상태 전환 로그에 필요한 정보를 담습니다."""

    before: MissionState
    after: MissionState
    reason: str
    lap: int
    route: str


@dataclass
class FsmCounters:
    """디바운스 카운터를 모아 둡니다."""

    start_ok: int = 0
    cone_missing: int = 0
    lane_stable: int = 0
    decision_anchor: int = 0


@dataclass
class MissionFSM:
    """상태 전이와 TURN_LEFT stub 명령을 담당합니다."""

    state: MissionState = MissionState.WAIT_START
    counters: FsmCounters = field(default_factory=FsmCounters)
    state_enter_time: float = 0.0
    turn_left_start_yaw_deg: Optional[float] = None
    last_transition: Optional[Transition] = None
    final_lap_reached_time: float = -1.0   # 마지막 랩(lap==TOTAL_LAPS) 처음 감지 시각

    def reset(self, now_sec: float = 0.0) -> None:
        """FSM을 처음 상태로 되돌립니다."""
        self.state = MissionState.WAIT_START
        self.counters = FsmCounters()
        self.state_enter_time = float(now_sec)
        self.turn_left_start_yaw_deg = None
        self.last_transition = None
        self.final_lap_reached_time = -1.0

    def update(self, events: FsmEvents, world: WorldModel, now_sec: float) -> Optional[Transition]:
        """현재 상태와 이벤트를 보고 다음 상태로 전이합니다."""
        self.last_transition = None

        if world.lap >= config.TOTAL_LAPS and self.state != MissionState.FINISH:
            # 마지막 랩 도착선을 감지하면 바로 멈추지 않고, 일정 시간(FINISH_EXTRA_DRIVE_SEC)
            # 동안 차선주행을 계속해 도착선을 확실히 통과한 뒤 종료한다.
            # (도착선은 화면 하단 ROI에서 차 앞쪽에 감지되므로, 즉시 멈추면 선 앞에서 멈춰
            #  시뮬의 랩 통과 기록이 완료되지 않는다.)
            if self.final_lap_reached_time < 0.0:
                self.final_lap_reached_time = now_sec
            if now_sec - self.final_lap_reached_time >= config.FINISH_EXTRA_DRIVE_SEC:
                return self._transition(
                    MissionState.FINISH,
                    f"마지막 랩 감지 후 {config.FINISH_EXTRA_DRIVE_SEC:.0f}s 추가 주행 완료, 종료",
                    world,
                    now_sec,
                )
            # 아직 추가 주행 중 → 종료하지 않고 아래 상태 로직(LANE 차선주행)을 계속한다.

        if self.state == MissionState.WAIT_START:
            self._state_wait_start(events, world, now_sec)
        elif self.state == MissionState.CONE:
            self._state_cone(events, world, now_sec)
        elif self.state == MissionState.LANE:
            self._state_lane(events, world, now_sec)
        elif self.state == MissionState.DECISION:
            self._state_decision(events, world, now_sec)
        elif self.state == MissionState.TURN_LEFT:
            self._state_turn_left(events, world, now_sec)
        elif self.state == MissionState.FINISH:
            pass

        return self.last_transition

    # =====================================================================
    # 상태별 전이: WAIT_START
    # =====================================================================
    def _state_wait_start(self, events: FsmEvents, world: WorldModel, now_sec: float) -> None:
        """WAIT_START에서는 3구 신호 stub이 출발 허용하면 CONE으로 넘어갑니다."""
        if events.start_allowed:
            self.counters.start_ok += 1
        else:
            self.counters.start_ok = 0

        if self.counters.start_ok >= config.START_CONFIRM_FRAMES:
            if config.GAZEBO_START_IN_LANE:
                world.cone_done = True
                self._transition(
                    MissionState.LANE,
                    "Gazebo 단독 주행 설정: 신호/라바콘 생략 후 차선 주행 시작",
                    world,
                    now_sec,
                )
                return
            self._transition(
                MissionState.CONE,
                "3구 신호 stub 출발 허용 → 인트로 라바콘 진입",
                world,
                now_sec,
            )

    # =====================================================================
    # 상태별 전이: CONE
    # =====================================================================
    def _state_cone(self, events: FsmEvents, world: WorldModel, now_sec: float) -> None:
        """CONE에서는 콘 미검출+차선 안정으로 LANE 복귀하되,
        차선이 충분히 오래 안정되면 콘 판정과 무관하게 LANE으로 빠져나옵니다."""
        self.counters.cone_missing = self._count_if(
            events.cone_missing,
            self.counters.cone_missing,
            config.CONE_MISSING_CONFIRM_FRAMES,
        )
        self.counters.lane_stable = self._count_if(
            events.lane_stable,
            self.counters.lane_stable,
            config.CONE_EXIT_LANE_ONLY_FRAMES,
        )
        elapsed = now_sec - self.state_enter_time

        exit_by_sensor = (
            self.counters.cone_missing >= config.CONE_MISSING_CONFIRM_FRAMES
            and self.counters.lane_stable >= config.LANE_STABLE_CONFIRM_FRAMES
            and elapsed >= config.CONE_MIN_SEC_BEFORE_EXIT
        )
        exit_by_lane = (
            self.counters.lane_stable >= config.CONE_EXIT_LANE_ONLY_FRAMES
            and elapsed >= config.CONE_MIN_SEC_BEFORE_EXIT
        )

        if events.manual_cone_done or exit_by_sensor or exit_by_lane:
            world.cone_done = True
            if events.manual_cone_done:
                reason = "수동 c키로 라바콘 종료"
            elif exit_by_sensor:
                reason = (f"콘 미검출 {self.counters.cone_missing}/{config.CONE_MISSING_CONFIRM_FRAMES} + "
                          f"차선 안정 {self.counters.lane_stable} 충족")
            else:
                reason = f"차선 단독 안정 {self.counters.lane_stable}/{config.CONE_EXIT_LANE_ONLY_FRAMES} (콘 무관 탈출)"
            self._transition(MissionState.LANE, reason, world, now_sec)

    # =====================================================================
    # 상태별 전이: LANE
    # =====================================================================
    def _state_lane(self, events: FsmEvents, world: WorldModel, now_sec: float) -> None:
        """LANE에서는 차선주행을 유지하다가 4구 분기 anchor 또는 도착선을 처리합니다."""
        if world.lap >= config.TOTAL_LAPS:
            # 마지막 랩: 종료 타이밍은 update() 상단의 추가주행 타이머가 처리한다.
            # 여기서는 그냥 차선주행을 유지(분기/종료로 빠지지 않음)한다.
            return

        if events.manual_decision and world.can_enter_decision_this_lap(now_sec):
            self._transition(MissionState.DECISION, "수동 d키로 4구 분기 판단 진입", world, now_sec)
            return

        anchor = events.decision_anchor
        self.counters.decision_anchor = self._count_if(
            anchor,
            self.counters.decision_anchor,
            config.DECISION_ANCHOR_CONFIRM_FRAMES,
        )

        if world.can_enter_decision_this_lap(now_sec) and self.counters.decision_anchor >= config.DECISION_ANCHOR_CONFIRM_FRAMES:
            reason = f"4구 분기 anchor 안정 검출 {self.counters.decision_anchor}/{config.DECISION_ANCHOR_CONFIRM_FRAMES}"
            self._transition(MissionState.DECISION, reason, world, now_sec)

    # =====================================================================
    # 상태별 전이: DECISION
    # =====================================================================
    def _state_decision(self, events: FsmEvents, world: WorldModel, now_sec: float) -> None:
        """DECISION에서는 직진/좌회전만 결정하고 실제 주행은 다음 상태에 맡깁니다."""
        elapsed = now_sec - self.state_enter_time

        if world.lap == 0:
            world.set_route(RoutePlan.MAIN)
            world.mark_decision_done()
            self._transition(MissionState.LANE, "1바퀴째(lap0)는 규칙상 항상 직진", world, now_sec)
            return

        if events.manual_left:
            world.set_route(RoutePlan.SHORTCUT)
            world.mark_decision_done()
            self._transition(MissionState.TURN_LEFT, "수동 a키: 좌회전/지름길 선택", world, now_sec)
            return

        if events.manual_straight:
            world.set_route(RoutePlan.MAIN)
            world.mark_decision_done()
            self._transition(MissionState.LANE, "수동 s키: 직진/본선 선택", world, now_sec)
            return

        # 경찰이 좌회전 통로를 막으면 → 좌회전 불가
        if events.police_blocked:
            if events.straight_green:
                world.set_route(RoutePlan.MAIN)
                world.mark_decision_done()
                self._transition(MissionState.LANE, "경찰 차단 + 직진 신호 → 본선", world, now_sec)
            # green 아니면 DECISION 유지하며 정지 대기
            return

        # 경찰이 없으면 화살표가 이미 보여도 후진 시간을 먼저 확보한다.
        # 이 동안 DECISION을 유지해야 track_drive의 DECISION_REVERSE 명령이 실행된다.
        if elapsed < config.DECISION_REVERSE_SEC:
            return

        # 경찰 없음 → 좌회전 화살표 뜰 때까지 대기 (green 떠도 무시, 지름길 우선)
        if events.arrow_green:
            world.set_route(RoutePlan.SHORTCUT)
            world.mark_decision_done()
            self._transition(MissionState.TURN_LEFT, "좌회전 화살표 + 경찰 없음 → 지름길", world, now_sec)
            return
        # 화살표 아직 안 뜸 → DECISION 유지하며 대기 (타임아웃 없음)

    # =====================================================================
    # 상태별 전이: TURN_LEFT
    # =====================================================================
    def _state_turn_left(self, events: FsmEvents, world: WorldModel, now_sec: float) -> None:
        """지름길(shortcut_core)이 진입→직진→T자탈출을 모두 끝내고 done을 알리면 LANE으로 복귀한다.
        중간에 차선이 잡히거나 신호가 바뀌어도 빠져나가지 않고, shortcut이 끝날 때까지 유지한다."""
        if events.manual_turn_done or events.shortcut_done:
            reason = ("수동 t키: TURN_LEFT 완료"
                      if events.manual_turn_done
                      else "지름길 완료(진입→직진→T자탈출) → LANE 복귀")
            self._transition(MissionState.LANE, reason, world, now_sec)

    def turn_left_command(self) -> tuple[float, float]:
        """TURN_LEFT stub에서 사용할 임시 angle/speed 명령을 반환합니다."""
        return float(config.TURN_LEFT_ANGLE), float(config.TURN_LEFT_SPEED)

    def _turn_left_yaw_done(self, current_yaw_deg: Optional[float]) -> bool:
        """IMU yaw 기반 좌회전 완료 조건을 계산합니다."""
        if not config.TURN_LEFT_USE_IMU_YAW:
            return False
        if self.turn_left_start_yaw_deg is None or current_yaw_deg is None:
            return False
        delta = abs(self._angle_diff_deg(current_yaw_deg, self.turn_left_start_yaw_deg))
        target = float(config.TURN_LEFT_TARGET_YAW_DEG)
        tol = float(config.TURN_LEFT_YAW_TOLERANCE_DEG)
        return delta >= max(0.0, target - tol)

    # =====================================================================
    # 공통 유틸
    # =====================================================================
    def _transition(self, after: MissionState, reason: str, world: WorldModel, now_sec: float) -> Transition:
        """상태를 바꾸고 카운터/진입 시각/전환 정보를 갱신합니다."""
        before = self.state
        self.state = after
        self.state_enter_time = float(now_sec)
        self.counters = FsmCounters()

        if after == MissionState.TURN_LEFT:
            self.turn_left_start_yaw_deg = None
        elif before == MissionState.TURN_LEFT and after != MissionState.TURN_LEFT:
            self.turn_left_start_yaw_deg = None

        transition = Transition(before=before, after=after, reason=reason, lap=world.lap, route=world.route_text())
        self.last_transition = transition
        return transition

    @staticmethod
    def _count_if(flag: bool, current: int, limit: int) -> int:
        """조건이 참이면 카운터를 올리고 거짓이면 0으로 리셋합니다."""
        if flag:
            return min(current + 1, limit)
        return 0

    @staticmethod
    def _angle_diff_deg(a: float, b: float) -> float:
        """두 각도의 signed 차이를 -180~180 범위로 계산합니다."""
        return (float(a) - float(b) + 180.0) % 360.0 - 180.0

    def on_entered_turn_left_with_yaw(self, yaw_deg: Optional[float]) -> None:
        """TURN_LEFT 진입 직후 시작 yaw를 기록합니다."""
        self.turn_left_start_yaw_deg = yaw_deg

    def guidance_text(self, world: WorldModel) -> str:
        """현재 상태에서 다음 단계로 가려면 무엇이 필요한지 한 줄로 설명합니다."""
        if self.state == MissionState.WAIT_START:
            return f"to CONE: 3-light green start (stub) [{self.counters.start_ok}/{config.START_CONFIRM_FRAMES}]"
        if self.state == MissionState.CONE:
            return (
                "to LANE: cones gone + lane visible  "
                f"[콘 미검출 {self.counters.cone_missing}/{config.CONE_MISSING_CONFIRM_FRAMES}, "
                f"차선 {self.counters.lane_stable}/{config.LANE_STABLE_CONFIRM_FRAMES}]"
            )
        if self.state == MissionState.LANE:
            return (
                f"cruising lane | lap={world.lap} route={world.route_text()} | "
                "d-key/4-light anchor -> DECISION, finishline -> lap+1"
            )
        if self.state == MissionState.DECISION:
            lap_human = world.lap + 1
            return (
                f"좌회전 판단 중 | 이번 바퀴={lap_human} | "
                "turn cond: no-police[stub] + arrow-green[stub] | a=turnleft, s=straight"
            )
        if self.state == MissionState.TURN_LEFT:
            return (
                "좌회전 실행 stub | 시간+고정 조향 기본값 사용 | "
                f"{config.TURN_LEFT_SEC:.1f}s 경과 또는 차선 재검출/t키 → LANE"
            )
        return "주행 종료: FINISH, 정지 유지"

    def state_elapsed_hint(self) -> float:
        """디버그 문자열용 경과시간 자리입니다. 실제 시간은 overlay에서 별도로 보정하지 않습니다."""
        return 0.0

    def counters_debug(self) -> Dict[str, int]:
        """디버그 표시용 카운터 dict를 반환합니다."""
        return {
            "start_ok": self.counters.start_ok,
            "cone_missing": self.counters.cone_missing,
            "lane_stable": self.counters.lane_stable,
            "decision_anchor": self.counters.decision_anchor,
        }
