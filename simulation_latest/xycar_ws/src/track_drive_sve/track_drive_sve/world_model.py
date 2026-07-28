#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""센서 상태, lap, route_plan, 도착선 latch를 관리하는 월드 모델입니다."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional, Set

import cv2
import numpy as np

from . import config


class RoutePlan(Enum):
    """이번 lap에서 본선/지름길 중 어느 경로를 타는지 나타냅니다."""

    MAIN = "MAIN"
    SHORTCUT = "SHORTCUT"


@dataclass
class SensorHealth:
    """센서별 마지막 수신 시각과 stale 여부를 담습니다."""

    image_age: float = float("inf")
    scan_age: float = float("inf")
    imu_age: float = float("inf")
    image_ok: bool = False
    scan_ok: bool = False
    imu_ok: bool = False
    stale_reason: str = "아직 센서 수신 없음"


@dataclass
class ManualEvents:
    """shadow 검증용 키 입력 이벤트를 한 프레임 동안만 보관합니다."""

    force_decision: bool = False
    force_finish_line: bool = False
    force_cone_done: bool = False
    choose_left: bool = False
    choose_straight: bool = False
    finish_turn_left: bool = False

    def clear(self) -> None:
        """프레임 단위 수동 이벤트를 모두 지웁니다."""
        self.force_decision = False
        self.force_finish_line = False
        self.force_cone_done = False
        self.choose_left = False
        self.choose_straight = False
        self.finish_turn_left = False


@dataclass
class WorldModel:
    """트랙 진행 상황과 센서 health를 한곳에 모아 둡니다."""

    lap: int = 0
    route_plan: RoutePlan = RoutePlan.MAIN
    cone_done: bool = False
    finish_line_latched: bool = False
    finish_detect_count: int = 0
    start_line_skipped: bool = False
    finish_release_count: int = 0
    last_finish_time: float = -999.0
    finish_line_visible: bool = False
    finish_line_crossed: bool = False
    last_image_time: Optional[float] = None
    last_scan_time: Optional[float] = None
    last_imu_time: Optional[float] = None
    decision_done_laps: Set[int] = field(default_factory=set)
    manual: ManualEvents = field(default_factory=ManualEvents)
    last_finish_debug: Dict[str, float] = field(default_factory=dict)

    def update_image_time(self, now_sec: float) -> None:
        """카메라 프레임 수신 시각을 갱신합니다."""
        self.last_image_time = float(now_sec)

    def update_scan_time(self, now_sec: float) -> None:
        """LiDAR scan 수신 시각을 갱신합니다."""
        self.last_scan_time = float(now_sec)

    def update_imu_time(self, now_sec: float) -> None:
        """IMU 수신 시각을 갱신합니다."""
        self.last_imu_time = float(now_sec)

    def sensor_health(self, now_sec: float) -> SensorHealth:
        """현재 시각 기준 센서 stale 여부를 계산합니다."""
        image_age = float("inf") if self.last_image_time is None else now_sec - self.last_image_time
        scan_age = float("inf") if self.last_scan_time is None else now_sec - self.last_scan_time
        imu_age = float("inf") if self.last_imu_time is None else now_sec - self.last_imu_time

        image_ok = image_age <= config.IMAGE_STALE_SEC
        scan_ok = scan_age <= config.SCAN_STALE_SEC
        imu_ok = imu_age <= config.IMU_STALE_SEC

        reasons = []
        if not image_ok:
            reasons.append(f"카메라 끊김 {image_age:.2f}s")
        if not scan_ok:
            reasons.append(f"LiDAR 끊김 {scan_age:.2f}s")
        if config.STALE_REQUIRE_IMU and not imu_ok:
            reasons.append(f"IMU 끊김 {imu_age:.2f}s")

        return SensorHealth(
            image_age=image_age,
            scan_age=scan_age,
            imu_age=imu_age,
            image_ok=image_ok,
            scan_ok=scan_ok,
            imu_ok=imu_ok,
            stale_reason=", ".join(reasons) if reasons else "ok",
        )

    def has_critical_stale(self, now_sec: float) -> bool:
        """주행에 꼭 필요한 센서가 stale인지 확인합니다."""
        health = self.sensor_health(now_sec)
        if config.STALE_REQUIRE_IMU:
            return not (health.image_ok and health.scan_ok and health.imu_ok)
        return not (health.image_ok and health.scan_ok)

    def mark_decision_done(self) -> None:
        """현재 lap에서 4구 분기 판단을 끝냈다고 표시합니다."""
        self.decision_done_laps.add(int(self.lap))

    def can_enter_decision_this_lap(self, now_sec: float = 0.0) -> bool:
        """현재 lap에서 아직 DECISION 상태에 들어갈 수 있는지 확인합니다.
        도착선 통과 직후 쿨다운 동안은 도착선 흑백무늬를 정지선으로 오인하지 않도록 막습니다."""
        if now_sec - self.last_finish_time < config.DECISION_AFTER_FINISH_COOLDOWN_SEC:
            return False
        return int(self.lap) not in self.decision_done_laps and self.lap < config.TOTAL_LAPS
        
    def set_route(self, route: RoutePlan) -> None:
        """이번 lap의 route_plan을 갱신합니다."""
        self.route_plan = route

    def reset_route_for_new_lap(self) -> None:
        """새 lap 시작 시 route_plan을 기본 본선으로 되돌립니다."""
        self.route_plan = RoutePlan.MAIN

    def update_finish_line(self, image_bgr: Optional[np.ndarray], now_sec: float) -> bool:
        """흑백 도착선 검출과 latch를 갱신하고, 새로 통과하면 True를 반환합니다."""
        self.finish_line_crossed = False

        detected, debug = self.detect_finish_line(image_bgr)
        self.finish_line_visible = detected
        self.last_finish_debug = debug

        if self.manual.force_finish_line:
            detected = True
            debug["manual"] = 1.0
            self.finish_detect_count = max(self.finish_detect_count, config.FINISH_CONFIRM_FRAMES - 1)

        if detected:
            self.finish_detect_count += 1
            self.finish_release_count = 0
        else:
            self.finish_detect_count = 0
            self.finish_release_count += 1

        if self.finish_line_latched and self.finish_release_count >= config.FINISH_RELEASE_FRAMES:
            self.finish_line_latched = False

        enough_interval = (now_sec - self.last_finish_time) >= config.FINISH_MIN_INTERVAL_SEC
        if (
            detected
            and not self.finish_line_latched
            and self.finish_detect_count >= config.FINISH_CONFIRM_FRAMES
            and enough_interval
        ):
            self.finish_line_latched = True
            self.last_finish_time = now_sec
            if not self.start_line_skipped:
                self.start_line_skipped = True
                return False
            self.lap = min(config.TOTAL_LAPS, self.lap + 1)
            self.finish_line_crossed = True
            self.reset_route_for_new_lap()
            return True
 
        return False

    def detect_finish_line(self, image_bgr: Optional[np.ndarray]) -> tuple[bool, Dict[str, float]]:
        """카메라 하단 ROI에서 흑백 교차 패턴을 찾아 도착선 후보를 판정합니다."""
        if image_bgr is None:
            return False, {"reason": 0.0, "transitions": 0.0, "contrast": 0.0}

        h, w = image_bgr.shape[:2]
        y1 = int(h * config.FINISH_ROI_Y1_RATIO)
        y2 = int(h * config.FINISH_ROI_Y2_RATIO)
        x1 = int(w * config.FINISH_ROI_X1_RATIO)
        x2 = int(w * config.FINISH_ROI_X2_RATIO)
        roi = image_bgr[y1:y2, x1:x2]
        if roi.size == 0:
            return False, {"reason": 0.0, "transitions": 0.0, "contrast": 0.0}

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        black = blur < 70
        white = blur > 185
        black_ratio = float(np.mean(black))
        white_ratio = float(np.mean(white))
        contrast = float(np.percentile(blur, 90) - np.percentile(blur, 10))

        # 세로 방향 stripe보다 좌우 흑백 반복을 먼저 봅니다.
        col_mean = np.mean(blur, axis=0)
        if col_mean.size < 8:
            transitions = 0
        else:
            low = np.percentile(col_mean, 30)
            high = np.percentile(col_mean, 70)
            states = np.zeros_like(col_mean, dtype=np.int8)
            states[col_mean <= low] = -1
            states[col_mean >= high] = 1
            compact = [int(v) for v in states if int(v) != 0]
            transitions = 0
            for a, b in zip(compact[:-1], compact[1:]):
                if a != b:
                    transitions += 1

        detected = (
            black_ratio >= config.FINISH_MIN_BLACK_RATIO
            and white_ratio >= config.FINISH_MIN_WHITE_RATIO
            and contrast >= config.FINISH_MIN_CONTRAST
            and transitions >= config.FINISH_MIN_TRANSITIONS
        )

        debug = {
            "black_ratio": black_ratio,
            "white_ratio": white_ratio,
            "contrast": contrast,
            "transitions": float(transitions),
            "detected": 1.0 if detected else 0.0,
        }
        return detected, debug

    def decision_anchor_detected_stub(self, image_bgr: Optional[np.ndarray]) -> bool:
        """4구 신호등/분기 anchor 검출 자리입니다.

        현재는 실제 인식 코드가 없으므로 항상 False입니다. shadow 검증에서는 d 키로
        DECISION 진입을 강제할 수 있습니다. 나중에 4구 신호등 ROI 검출 코드는 여기에 넣으면 됩니다.
        """
        _ = image_bgr
        return False

    def apply_debug_key(self, key_code: int) -> Optional[str]:
        """OpenCV 창에서 들어온 검증용 키를 ManualEvents로 변환합니다."""
        if not config.ENABLE_DEBUG_KEYS or key_code < 0:
            return None

        key = chr(key_code & 0xFF).lower() if 0 <= (key_code & 0xFF) <= 255 else ""
        if key == "d":
            self.manual.force_decision = True
            return "DECISION 강제 진입"
        if key == "l":
            self.manual.force_finish_line = True
            return "도착선 수동 카운트"
        if key == "c":
            self.manual.force_cone_done = True
            return "라바콘 종료 강제"
        if key == "a":
            self.manual.choose_left = True
            return "좌회전 선택 강제"
        if key == "s":
            self.manual.choose_straight = True
            return "직진 선택 강제"
        if key == "t":
            self.manual.finish_turn_left = True
            return "TURN_LEFT 완료 강제"
        return None

    def route_text(self) -> str:
        """현재 route_plan을 디버그 표시용 문자열로 반환합니다."""
        return self.route_plan.value
