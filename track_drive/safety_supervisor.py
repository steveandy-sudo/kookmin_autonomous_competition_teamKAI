#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 주행 중 즉시 정지 또는 안전 대기가 필요한 상황을 한곳에서 판단하는 안전 감독 모듈이다.
# 정지선, 신호등, 보행자, 차량 상태를 종합해 메인 제어 루프를 중단할지 결정한다.
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
# 안전 정지가 필요한지와 그 이유를 함께 전달하는 결과 구조체이다.
class SafetyStopDecision:
    should_stop: bool
    reason: str = ''


# 보행자, 신호등, 정지선 상태를 모아 안전 정지 여부를 판단하는 보조 클래스이다.
class SafetySupervisor:
    """신호등, 정지선, 보행자, 차량 상태를 종합해 안전 정지 여부를 결정한다."""

    def __init__(self, node):
        # SafetySupervisor 객체를 초기화하고 필요한 파라미터와 내부 상태를 준비한다.
        self.node = node
        self.next_stop_line_update_sec = 0.0
        self.next_school_zone_update_sec = 0.0
        self.last_stop_line_image_id = None

    # 현재 프레임의 신호등/정지선 인식 결과를 갱신한다.
    def update_perception(self, image: Optional[np.ndarray]):
        # 최신 입력을 기준으로 update perception 관련 캐시와 상태를 갱신한다.
        # 이미지가 끊기면 이전 검출 결과를 믿지 않고 각 검출기의 현재 상태를 비운다.
        if image is None:
            self.last_stop_line_image_id = None
            self.node.stop_line_detector.update(image)
            # 신호등은 정지/좌회전 판단의 직접 조건이므로 매 perception 주기마다 최신화한다.
            self.node.traffic_light_detector.update(image)
            self.node.school_zone_detector.update(image)
            self.node.publish_ai_speed_limit()
            return

        now = time.monotonic()
        # 정지선은 계산 비용이 있으므로 새 프레임 또는 설정 주기가 되었을 때만 갱신한다.
        if self._stop_line_update_due(now, image):
            self.node.stop_line_detector.update(image)
            self.node.traffic_light_detector.update(image)
        # 어린이보호구역 검출은 노면 표식 기반이라 일정 주기로 갱신해 연산량을 조절한다.
        if self._period_due(now, 'school_zone_update_period_sec', 'next_school_zone_update_sec'):
            self.node.school_zone_detector.update(image)
        self.node.publish_ai_speed_limit()

    # 무거운 인식 연산이 매 프레임 돌지 않도록 갱신 주기를 제한한다.
    def _period_due(self, now: float, parameter_name: str, next_attr: str) -> bool:
        # 안전 판단의 period due 로직을 수행한다.
        # period가 0이면 매 주기 실행, 양수이면 다음 실행 시각까지 결과를 유지한다.
        period = max(float(self.node.get_parameter(parameter_name).value), 0.0)
        next_sec = float(getattr(self, next_attr))
        if period <= 0.0 or now >= next_sec:
            setattr(self, next_attr, now + period)
            return True
        return False

    def _stop_line_update_due(self, now: float, image: np.ndarray) -> bool:
        # 정지선 처리에서 정지 line update due 조건이나 값을 계산한다.
        period = max(float(self.node.get_parameter('stop_line_update_period_sec').value), 0.0)
        # 같은 이미지 객체를 반복 처리하지 않도록 객체 id로 새 프레임 여부를 구분한다.
        image_id = id(image)
        new_image = image_id != self.last_stop_line_image_id
        if period <= 0.0 or new_image or now >= self.next_stop_line_update_sec:
            self.next_stop_line_update_sec = now + period
            self.last_stop_line_image_id = image_id
            return True
        return False

    # 최신 인식 결과를 기반으로 차량을 멈춰야 하는지 최종 판단한다.
    def detect_safety_stop(self) -> SafetyStopDecision:
        # 현재 미션 상태에서 즉시 정지가 필요한지 우선순위 순서대로 판단한다.
        # 입력 데이터에서 detect safety 정지 조건을 감지한다.
        should_stop, reason = self.node._detect_safety_stop()
        return SafetyStopDecision(should_stop=should_stop, reason=reason)

    def startup_light_check_pending(self) -> bool:
        # 안전 판단의 startup 신호등 check pending 로직을 수행한다.
        return self.node.traffic_light_detector.startup_check_pending()
