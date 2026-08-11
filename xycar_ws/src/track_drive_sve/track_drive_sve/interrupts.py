#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""차선/라바콘 기본 명령 위에 얹는 미션 인터럽트와 arbitration입니다.

현재 보행자, 차량 회피, 신호등, 경찰차 판단은 stub입니다. 항상 감지 안 됨을 반환하지만,
나중에 실제 판정 코드를 끼우기 쉽도록 함수와 우선순위 구조는 완성해 두었습니다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from sensor_msgs.msg import LaserScan

from . import config
from .fsm import MissionState
from . import lidar_utils
from .world_model import RoutePlan, WorldModel


@dataclass
class ControlCommand:
    """angle/speed 명령과 출처를 함께 담습니다."""

    angle: float = 0.0
    speed: float = 0.0
    source: str = "NONE"


@dataclass
class InterruptAction:
    """하나의 인터럽트 개입 결과입니다."""

    priority: int
    name: str
    active: bool = False
    speed_cap: Optional[float] = None
    angle_override: Optional[float] = None
    reason: str = ""
    debug_text: str = ""


@dataclass
class ArbitrationResult:
    """인터럽트 병합 후 최종 명령과 디버그 정보를 담습니다."""

    final: ControlCommand
    active_actions: List[InterruptAction] = field(default_factory=list)
    interrupt_text: str = "interrupt: none"
    reason: str = "기본 명령 유지"
    debug: Dict[str, object] = field(default_factory=dict)


@dataclass
class DecisionSignals:
    """DECISION 상태에서 필요한 신호등/경찰 판단 결과입니다."""

    arrow_green: bool = False
    police_blocked: bool = False
    debug_text: str = "좌회전 신호/경찰 판단 stub"
    debug: Dict[str, object] = field(default_factory=dict)


class InterruptManager:
    """우선순위 인터럽트 판정과 명령 병합을 담당합니다."""

    def __init__(self) -> None:
        """stub 로그가 너무 자주 나오지 않도록 내부 플래그를 초기화합니다."""
        self.stub_notice_done: Dict[str, bool] = {}
        # 보행자(LiDAR) 정지 상태: 정지 종료 시각과 직전 프레임 감지 여부(엣지 트리거용)
        self._ped_stop_until: float = -1.0
        self._ped_was_detected: bool = False

    # =====================================================================
    # 외부에서 호출하는 함수
    # =====================================================================
    def start_signal_allowed_stub(self, image_bgr) -> bool:
        """3구 신호등 출발 판정 자리입니다. 현재는 바로 출발 허용입니다."""
        _ = image_bgr
        self._mark_stub_seen("3구 출발 신호")
        return True

    def decision_signals_stub(self, image_bgr, scan_msg: Optional[LaserScan]) -> DecisionSignals:
        """4구 좌회전 화살표와 경찰차 차단 판단 자리입니다."""
        _ = image_bgr
        self._mark_stub_seen("4구 좌회전 화살표")
        self._mark_stub_seen("경찰차 차단")

        police_blocked, police_debug = lidar_utils.police_blocking_left_stub(scan_msg)
        return DecisionSignals(
            arrow_green=False,
            police_blocked=police_blocked,
            debug_text="화살표초록=False(stub), 경찰차=False(stub 기본)",
            debug={"police": police_debug},
        )

    def arbitrate(
        self,
        base: ControlCommand,
        state: MissionState,
        world: WorldModel,
        scan_msg: Optional[LaserScan],
        now_sec: float,
    ) -> ArbitrationResult:
        """기본 주행 명령 위에 인터럽트를 우선순위대로 병합합니다."""
        actions: List[InterruptAction] = []
        actions.append(self._sensor_stale_action(world, now_sec))
        actions.append(self._emergency_collision_action(scan_msg))
        actions.append(self._traffic_action_stub(state))
        actions.append(self._pedestrian_action(world, scan_msg, now_sec))
        actions.append(self._vehicle_avoid_action_stub(world))

        active = [a for a in actions if a.active]
        final_speed = float(base.speed)
        final_angle = float(base.angle)

        # 속도는 모든 cap의 최솟값을 적용합니다.
        speed_caps = [a.speed_cap for a in active if a.speed_cap is not None]
        if speed_caps:
            final_speed = min(final_speed, min(float(v) for v in speed_caps))

        # 조향 override는 우선순위가 가장 높은 하나만 사용합니다.
        steer_actions = [a for a in active if a.angle_override is not None]
        if steer_actions:
            top = min(steer_actions, key=lambda a: a.priority)
            final_angle = float(top.angle_override)

        final = ControlCommand(angle=final_angle, speed=final_speed, source="ARBITRATED")
        if active:
            top_active = min(active, key=lambda a: a.priority)
            text = f"인터럽트: {top_active.debug_text or top_active.name}(P{top_active.priority})"
            reason = top_active.reason or top_active.name
        else:
            text = "interrupt: none"
            reason = "기본 명령 유지"

        return ArbitrationResult(
            final=final,
            active_actions=active,
            interrupt_text=text,
            reason=reason,
            debug={
                "base_angle": float(base.angle),
                "base_speed": float(base.speed),
                "final_angle": final_angle,
                "final_speed": final_speed,
                "active_names": [a.name for a in active],
            },
        )

    # =====================================================================
    # 우선순위별 인터럽트 구현 / stub
    # =====================================================================
    def _sensor_stale_action(self, world: WorldModel, now_sec: float) -> InterruptAction:
        """P1 센서 stale이면 정지합니다."""
        health = world.sensor_health(now_sec)
        stale = world.has_critical_stale(now_sec)
        return InterruptAction(
            priority=1,
            name="SENSOR_STALE",
            active=stale,
            speed_cap=0.0 if stale else None,
            angle_override=0.0 if stale else None,
            reason=health.stale_reason if stale else "센서 정상",
            debug_text="센서 끊김 정지" if stale else "",
        )

    def _emergency_collision_action(self, scan_msg: Optional[LaserScan]) -> InterruptAction:
        """P2 정면 근접 장애물이 있으면 긴급 정지합니다."""
        emergency, front_min = lidar_utils.is_emergency_collision(scan_msg)
        return InterruptAction(
            priority=2,
            name="EMERGENCY_COLLISION",
            active=emergency,
            speed_cap=0.0 if emergency else None,
            angle_override=0.0 if emergency else None,
            reason=f"정면 {front_min:.2f}m 장애물" if emergency else f"정면 {front_min:.2f}m",
            debug_text="긴급 충돌 정지" if emergency else "",
        )

    def _traffic_action_stub(self, state: MissionState) -> InterruptAction:
        """P3 신호등 정지/대기 인터럽트 자리입니다. 현재는 no-op입니다."""
        _ = state
        self._mark_stub_seen("신호등 인터럽트")
        return InterruptAction(priority=3, name="TRAFFIC_LIGHT_STUB", active=False)

    def _pedestrian_action(self, world: WorldModel, scan_msg, now_sec: float) -> InterruptAction:
        """P4 보행자 정지. 전방 ROI(LiDAR)에 보행자가 들어오면 1.5초 고정 정지한다.
        - MAIN 주행 중에만 arm(오검출 구간 축소).
        - 엣지 트리거: 감지가 새로 들어올 때만 1.5초 타이머를 시작한다(정지 후 GO,
          같은 보행자가 계속 보여도 재정지하지 않음. 사라졌다 다시 들어오면 재발동).
        """
        armed = config.ENABLE_PEDESTRIAN_LIDAR and self._main_route_interrupt_armed(world)
        if not armed:
            # arm 해제 구간에서는 감지/타이머를 리셋해 다음 MAIN 진입 시 깨끗하게 동작
            self._ped_was_detected = False
            return InterruptAction(priority=4, name="PEDESTRIAN", active=False)

        self._mark_stub_seen("보행자 인터럽트")
        detected, count, min_dist = lidar_utils.pedestrian_in_front(
            scan_msg,
            config.PED_ROI_X_MIN,
            config.PED_ROI_X_MAX,
            config.PED_ROI_Y_ABS,
            config.PED_SELF_IGNORE_M,
            config.PED_POINT_THRESHOLD,
        )

        # 새로 감지된 순간(엣지)이고 현재 정지 중이 아니면 1.5초 정지 타이머 시작
        if detected and not self._ped_was_detected and now_sec >= self._ped_stop_until:
            self._ped_stop_until = now_sec + config.PED_STOP_HOLD_SEC
        self._ped_was_detected = detected

        active = now_sec < self._ped_stop_until
        remain = max(0.0, self._ped_stop_until - now_sec)
        # 정지 중 조향: 0으로 펴면 곡선 재출발 시 바퀴가 0→곡선각으로 꺾이는 동안 이탈한다.
        # PED_KEEP_LANE_STEER_WHILE_STOPPED이면 조향은 override하지 않고(lane_core 값 유지)
        # 속도만 0으로 막아, 바퀴를 곡선각으로 미리 꺾어둔 채 정지한다.
        keep_steer = getattr(config, "PED_KEEP_LANE_STEER_WHILE_STOPPED", True)
        angle_override = None if (active and keep_steer) else (0.0 if active else None)
        return InterruptAction(
            priority=4,
            name="PEDESTRIAN",
            active=active,
            speed_cap=0.0 if active else None,
            angle_override=angle_override,
            reason=f"보행자 정지 {remain:.1f}s (pts={count}, d={min_dist:.2f})" if active else "보행자 없음",
            debug_text="보행자 정지" if active else "",
        )

    def _vehicle_avoid_action_stub(self, world: WorldModel) -> InterruptAction:
        """P5 차량 회피/감속 인터럽트 자리입니다. 현재는 no-op입니다."""
        armed = self._main_route_interrupt_armed(world)
        if armed:
            self._mark_stub_seen("차량 회피 인터럽트")
        return InterruptAction(priority=5, name="VEHICLE_AVOID_STUB", active=False)

    # =====================================================================
    # 공통 helper
    # =====================================================================
    def _main_route_interrupt_armed(self, world: WorldModel) -> bool:
        """보행자/차량 인터럽트가 현재 route에서 arm되어야 하는지 계산합니다."""
        if not config.ARM_MAIN_ROUTE_INTERRUPTS_ONLY:
            return world.lap < config.TOTAL_LAPS
        return world.route_plan == RoutePlan.MAIN and world.lap < config.TOTAL_LAPS

    def _mark_stub_seen(self, name: str) -> None:
        """stub이 호출되었다는 사실만 내부에 기록합니다."""
        self.stub_notice_done[name] = True

    def stub_summary(self) -> str:
        """현재 no-op으로 남아 있는 stub 목록을 사람이 읽는 문자열로 반환합니다."""
        if not self.stub_notice_done:
            return "stub 호출 전"
        return ", ".join(sorted(self.stub_notice_done.keys()))
