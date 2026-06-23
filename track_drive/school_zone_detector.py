#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class SchoolZoneBEVResult:
    active: bool = False
    candidate: bool = False
    speed_limit_active: bool = False
    left_ratio: float = 0.0
    right_ratio: float = 0.0
    pair_row_ratio: float = 0.0
    bottom_pair_row_ratio: float = 0.0
    separation_ratio: float = 0.0
    left_rows: int = 0
    right_rows: int = 0
    pair_rows: int = 0
    hold_remaining_sec: float = 0.0
    roi_debug_image: Optional[np.ndarray] = None
    bev_image: Optional[np.ndarray] = None
    mask_image: Optional[np.ndarray] = None
    debug_image: Optional[np.ndarray] = None


def declare_school_zone_bev_parameters(node):
    # 어린이 보호구역 검출에서 사용하는 ROS 파라미터 기본값을 선언한다.
    node.declare_parameter('camera_topic', '/usb_cam/image_raw/front')
    node.declare_parameter('school_zone_speed', 5.5)
    node.declare_parameter('school_zone_speed_limit_enabled', True)
    node.declare_parameter('school_zone_speed_limit_hold_sec', 1.0)
    node.declare_parameter('school_zone_hold_sec', 1.5)
    node.declare_parameter('school_zone_debug_topic_prefix', '/track_drive/school_zone_debug')
    node.declare_parameter('school_zone_debug_log_period_sec', 0.5)
    node.declare_parameter('school_zone_bev_width', 320)
    node.declare_parameter('school_zone_bev_height', 240)
    node.declare_parameter('school_zone_bev_src_top_ratio', 0.50)
    node.declare_parameter('school_zone_bev_src_bottom_ratio', 0.90)
    node.declare_parameter('school_zone_bev_src_top_half_width_ratio', 0.12)
    node.declare_parameter('school_zone_bev_src_bottom_half_width_ratio', 0.50)
    node.declare_parameter('school_zone_bev_center_shift_ratio', 0.0)
    node.declare_parameter('school_zone_bev_front_top_ratio', 0.38)
    node.declare_parameter('school_zone_bev_front_bottom_ratio', 1.00)
    node.declare_parameter('school_zone_bev_left_edge_max_ratio', 0.42)
    node.declare_parameter('school_zone_bev_right_edge_min_ratio', 0.58)
    node.declare_parameter('school_zone_yellow_h_min', 15)
    node.declare_parameter('school_zone_yellow_h_max', 40)
    node.declare_parameter('school_zone_yellow_s_min', 70)
    node.declare_parameter('school_zone_yellow_v_min', 80)
    node.declare_parameter('school_zone_bev_open_kernel', 3)
    node.declare_parameter('school_zone_bev_close_kernel', 5)
    node.declare_parameter('school_zone_bev_min_pixels', 45)
    node.declare_parameter('school_zone_bev_min_row_pixels', 3)
    node.declare_parameter('school_zone_bev_max_row_width_ratio', 0.16)
    node.declare_parameter('school_zone_bev_min_left_ratio', 0.0015)
    node.declare_parameter('school_zone_bev_min_right_ratio', 0.0015)
    node.declare_parameter('school_zone_bev_min_pair_row_ratio', 0.085)
    node.declare_parameter('school_zone_bev_min_bottom_pair_row_ratio', 0.05)
    node.declare_parameter('school_zone_bev_min_pair_rows', 4)
    node.declare_parameter('school_zone_bev_min_separation_ratio', 0.38)
    node.declare_parameter('school_zone_bev_preslow_ratio', 0.50)


class BEVSchoolZoneDetector:
    """Detect school-zone yellow side lane markings in bird's-eye view."""

    def __init__(self, node):
        # BEVSchoolZoneDetector 객체를 초기화하고 필요한 파라미터와 내부 상태를 준비한다.
        self.node = node
        self.active_until_sec = 0.0
        self.candidate_until_sec = 0.0

    def detect(self, image: Optional[np.ndarray]) -> SchoolZoneBEVResult:
        # 입력 데이터에서 detect 조건을 감지한다.
        if image is None or image.size == 0:
            return SchoolZoneBEVResult()

        height, width = image.shape[:2]
        bev_width = max(int(self._param('school_zone_bev_width', 320)), 16)
        bev_height = max(int(self._param('school_zone_bev_height', 240)), 16)
        src = self._source_points(width, height)
        dst = np.float32([
            [0.0, 0.0],
            [bev_width - 1.0, 0.0],
            [bev_width - 1.0, bev_height - 1.0],
            [0.0, bev_height - 1.0],
        ])

        roi_debug = image.copy()
        cv2.polylines(roi_debug, [src.astype(np.int32)], True, (0, 255, 255), 3)

        try:
            matrix = cv2.getPerspectiveTransform(src, dst)
            bev = cv2.warpPerspective(image, matrix, (bev_width, bev_height))
        except cv2.error:
            return SchoolZoneBEVResult(roi_debug_image=roi_debug)

        yellow_mask = self._yellow_mask(bev)
        result = self._evaluate_mask(yellow_mask, bev)
        result.roi_debug_image = roi_debug
        result.bev_image = bev
        result.mask_image = yellow_mask
        return result

    def _source_points(self, width: int, height: int) -> np.ndarray:
        # 원본 카메라 좌표와 BEV 변환에 필요한 기준점을 계산한다.
        top_ratio = float(np.clip(self._param('school_zone_bev_src_top_ratio', 0.50), 0.05, 0.95))
        bottom_ratio = float(np.clip(
            self._param('school_zone_bev_src_bottom_ratio', 0.90),
            top_ratio + 0.03,
            1.0,
        ))
        top_half = float(np.clip(self._param('school_zone_bev_src_top_half_width_ratio', 0.12), 0.02, 0.50))
        bottom_half = float(np.clip(
            self._param('school_zone_bev_src_bottom_half_width_ratio', 0.50),
            top_half,
            0.70,
        ))
        center_shift = float(np.clip(self._param('school_zone_bev_center_shift_ratio', 0.0), -0.25, 0.25))
        center_x = (0.5 + center_shift) * float(width)
        points = np.float32([
            [center_x - top_half * width, top_ratio * height],
            [center_x + top_half * width, top_ratio * height],
            [center_x + bottom_half * width, bottom_ratio * height],
            [center_x - bottom_half * width, bottom_ratio * height],
        ])
        points[:, 0] = np.clip(points[:, 0], 0.0, max(float(width - 1), 0.0))
        points[:, 1] = np.clip(points[:, 1], 0.0, max(float(height - 1), 0.0))
        return points

    def _yellow_mask(self, bev: np.ndarray) -> np.ndarray:
        # 어린이 보호구역 검출의 yellow 마스크 로직을 수행한다.
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        h_min = int(np.clip(self._param('school_zone_yellow_h_min', 15), 0, 179))
        h_max = int(np.clip(self._param('school_zone_yellow_h_max', 40), h_min, 179))
        s_min = int(np.clip(self._param('school_zone_yellow_s_min', 70), 0, 255))
        v_min = int(np.clip(self._param('school_zone_yellow_v_min', 80), 0, 255))
        mask = cv2.inRange(
            hsv,
            np.array([h_min, s_min, v_min], dtype=np.uint8),
            np.array([h_max, 255, 255], dtype=np.uint8),
        )
        open_size = max(int(self._param('school_zone_bev_open_kernel', 3)), 1)
        close_size = max(int(self._param('school_zone_bev_close_kernel', 5)), 1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_size, open_size), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((close_size, close_size), np.uint8))
        return mask

    def _evaluate_mask(self, mask: np.ndarray, bev: np.ndarray) -> SchoolZoneBEVResult:
        # 어린이 보호구역 검출의 evaluate 마스크 로직을 수행한다.
        height, width = mask.shape[:2]
        front_top = float(np.clip(self._param('school_zone_bev_front_top_ratio', 0.38), 0.0, 0.98))
        front_bottom = float(np.clip(
            self._param('school_zone_bev_front_bottom_ratio', 1.00),
            front_top + 0.01,
            1.0,
        ))
        y0 = int(front_top * height)
        y1 = max(y0 + 1, int(front_bottom * height))
        band = mask[y0:y1, :]
        debug = bev.copy()
        cv2.rectangle(debug, (0, y0), (width - 1, y1 - 1), (255, 0, 0), 2)

        left_edge_max = float(np.clip(self._param('school_zone_bev_left_edge_max_ratio', 0.42), 0.05, 0.49))
        right_edge_min = float(np.clip(self._param('school_zone_bev_right_edge_min_ratio', 0.58), 0.51, 0.95))
        left_end = max(int(width * left_edge_max), 1)
        right_start = min(int(width * right_edge_min), width - 1)
        left = band[:, :left_end]
        right = band[:, right_start:]
        left_pixels = int(np.count_nonzero(left))
        right_pixels = int(np.count_nonzero(right))
        left_ratio = left_pixels / float(max(left.size, 1))
        right_ratio = right_pixels / float(max(right.size, 1))

        min_row_pixels = max(int(self._param('school_zone_bev_min_row_pixels', 3)), 1)
        max_row_pixels = max(
            min_row_pixels,
            int(width * float(np.clip(self._param('school_zone_bev_max_row_width_ratio', 0.16), 0.01, 0.45))),
        )
        left_counts = np.count_nonzero(left, axis=1)
        right_counts = np.count_nonzero(right, axis=1)
        left_rows_mask = (left_counts >= min_row_pixels) & (left_counts <= max_row_pixels)
        right_rows_mask = (right_counts >= min_row_pixels) & (right_counts <= max_row_pixels)
        paired_rows = np.nonzero(left_rows_mask & right_rows_mask)[0]

        valid_pair_rows: List[int] = []
        separations: List[float] = []
        min_separation = float(np.clip(self._param('school_zone_bev_min_separation_ratio', 0.38), 0.10, 0.90))
        for row_idx in paired_rows:
            left_xs = np.flatnonzero(left[row_idx])
            right_xs = np.flatnonzero(right[row_idx])
            if len(left_xs) < min_row_pixels or len(right_xs) < min_row_pixels:
                continue
            left_x = float(np.median(left_xs))
            right_x = float(right_start + np.median(right_xs))
            separation = (right_x - left_x) / float(max(width, 1))
            if separation < min_separation:
                continue
            valid_pair_rows.append(int(row_idx))
            separations.append(separation)

        pair_rows = len(valid_pair_rows)
        pair_row_ratio = pair_rows / float(max(band.shape[0], 1))
        bottom_start = int(band.shape[0] * 0.55)
        bottom_pair_rows = sum(1 for row_idx in valid_pair_rows if row_idx >= bottom_start)
        bottom_pair_row_ratio = bottom_pair_rows / float(max(band.shape[0] - bottom_start, 1))
        separation_ratio = float(np.median(separations)) if separations else 0.0

        raw_active = self._active(
            left_pixels,
            right_pixels,
            left_ratio,
            right_ratio,
            pair_rows,
            pair_row_ratio,
            bottom_pair_row_ratio,
            separation_ratio,
        )
        raw_candidate = raw_active or self._candidate(
            left_pixels,
            right_pixels,
            pair_rows,
            pair_row_ratio,
            bottom_pair_row_ratio,
            separation_ratio,
        )
        active, candidate, hold_remaining_sec = self._apply_hold(raw_active, raw_candidate)

        cv2.rectangle(debug, (0, y0), (left_end, y1 - 1), (0, 255, 255), 2)
        cv2.rectangle(debug, (right_start, y0), (width - 1, y1 - 1), (0, 255, 255), 2)
        for row_idx in valid_pair_rows:
            y = y0 + row_idx
            cv2.line(debug, (left_end, y), (right_start, y), (0, 180, 255), 1)

        held = hold_remaining_sec > 0.0 and not raw_active and not raw_candidate
        state = 'ACTIVE' if active else ('CANDIDATE' if candidate else 'none')
        if held:
            state = f'{state} HOLD'
        color = (0, 255, 0) if active else ((0, 180, 255) if candidate else (0, 0, 255))
        cv2.rectangle(debug, (6, 6), (min(width - 6, 318), 92), (0, 0, 0), thickness=-1)
        cv2.putText(debug, f'SCHOOL ZONE: {state}', (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.60, color, 2)
        cv2.putText(
            debug,
            f'L={left_ratio:.3f} R={right_ratio:.3f} pair={pair_row_ratio:.2f}',
            (12, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (230, 230, 230),
            1,
        )
        cv2.putText(
            debug,
            f'bottom={bottom_pair_row_ratio:.2f} sep={separation_ratio:.2f} rows={pair_rows} hold={hold_remaining_sec:.1f}s',
            (12, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (230, 230, 230),
            1,
        )

        return SchoolZoneBEVResult(
            active=active,
            candidate=candidate,
            speed_limit_active=raw_active,
            left_ratio=left_ratio,
            right_ratio=right_ratio,
            pair_row_ratio=pair_row_ratio,
            bottom_pair_row_ratio=bottom_pair_row_ratio,
            separation_ratio=separation_ratio,
            left_rows=int(np.count_nonzero(left_rows_mask)),
            right_rows=int(np.count_nonzero(right_rows_mask)),
            pair_rows=pair_rows,
            hold_remaining_sec=hold_remaining_sec,
            debug_image=debug,
        )

    def _apply_hold(self, raw_active: bool, raw_candidate: bool) -> Tuple[bool, bool, float]:
        # apply hold 조건을 현재 명령이나 상태에 적용한다.
        now = time.monotonic()
        hold_sec = max(float(self._param('school_zone_hold_sec', 1.5)), 0.0)
        if raw_active:
            self.active_until_sec = now + hold_sec
            self.candidate_until_sec = max(self.candidate_until_sec, self.active_until_sec)
        elif raw_candidate:
            self.candidate_until_sec = now + hold_sec

        active = raw_active or now <= self.active_until_sec
        candidate = raw_candidate or active or now <= self.candidate_until_sec
        remaining = 0.0
        if active:
            remaining = max(self.active_until_sec - now, 0.0)
        elif candidate:
            remaining = max(self.candidate_until_sec - now, 0.0)
        return active, candidate, remaining

    def _active(
        self,
        left_pixels: int,
        right_pixels: int,
        left_ratio: float,
        right_ratio: float,
        pair_rows: int,
        pair_row_ratio: float,
        bottom_pair_row_ratio: float,
        separation_ratio: float,
    ) -> bool:
        # 어린이 보호구역 검출의 active 로직을 수행한다.
        min_pixels = max(int(self._param('school_zone_bev_min_pixels', 45)), 1)
        return (
            left_pixels >= min_pixels
            and right_pixels >= min_pixels
            and left_ratio >= float(self._param('school_zone_bev_min_left_ratio', 0.0015))
            and right_ratio >= float(self._param('school_zone_bev_min_right_ratio', 0.0015))
            and pair_rows >= max(int(self._param('school_zone_bev_min_pair_rows', 4)), 1)
            and pair_row_ratio >= float(self._param('school_zone_bev_min_pair_row_ratio', 0.085))
            and bottom_pair_row_ratio >= float(self._param('school_zone_bev_min_bottom_pair_row_ratio', 0.05))
            and separation_ratio >= float(self._param('school_zone_bev_min_separation_ratio', 0.38))
        )

    def _candidate(
        self,
        left_pixels: int,
        right_pixels: int,
        pair_rows: int,
        pair_row_ratio: float,
        bottom_pair_row_ratio: float,
        separation_ratio: float,
    ) -> bool:
        # candidate 동작을 수행할 수 있는 상태인지 확인한다.
        ratio = float(np.clip(self._param('school_zone_bev_preslow_ratio', 0.50), 0.25, 1.0))
        return (
            left_pixels >= int(max(int(self._param('school_zone_bev_min_pixels', 45)), 1) * ratio)
            and right_pixels >= int(max(int(self._param('school_zone_bev_min_pixels', 45)), 1) * ratio)
            and pair_rows >= max(2, int(max(int(self._param('school_zone_bev_min_pair_rows', 4)), 1) * ratio))
            and pair_row_ratio >= float(self._param('school_zone_bev_min_pair_row_ratio', 0.085)) * ratio
            and bottom_pair_row_ratio >= float(self._param('school_zone_bev_min_bottom_pair_row_ratio', 0.05)) * ratio
            and separation_ratio >= float(self._param('school_zone_bev_min_separation_ratio', 0.38))
        )

    def _param(self, name: str, default):
        # ROS 파라미터 값을 읽고 없으면 기본값을 사용한다.
        try:
            return self.node.get_parameter(name).value
        except Exception:
            return default


class SchoolZoneDetector:
    """School-zone perception and speed-limit boundary for TrackDriverNode."""

    def __init__(self, node):
        # SchoolZoneDetector 객체를 초기화하고 필요한 파라미터와 내부 상태를 준비한다.
        self.node = node
        self.bev_detector = BEVSchoolZoneDetector(node)
        self.last_result = SchoolZoneBEVResult()

    def update(self, image: Optional[np.ndarray]):
        # 최신 입력을 기준으로 update 관련 캐시와 상태를 갱신한다.
        if image is None or not bool(self.node.get_parameter('school_zone_enabled').value):
            self._reset_state()
            return

        if not bool(self._param('school_zone_bev_enabled', True)):
            self.node._update_school_zone_state(image)
            self._update_speed_limit_hold(bool(self.node.school_zone_active))
            self.last_result = SchoolZoneBEVResult(
                active=bool(self.node.school_zone_active),
                candidate=bool(self.node.school_zone_candidate_active),
                speed_limit_active=bool(self.node.school_zone_speed_limit_active),
                left_ratio=float(self.node.school_zone_yellow_left_ratio),
                right_ratio=float(self.node.school_zone_yellow_right_ratio),
                pair_row_ratio=float(self.node.school_zone_yellow_pair_row_ratio),
                bottom_pair_row_ratio=float(self.node.school_zone_yellow_bottom_pair_row_ratio),
                separation_ratio=float(self.node.school_zone_yellow_separation_ratio),
            )
            return

        result = self.bev_detector.detect(image)
        self.last_result = result
        self._apply_bev_result(result)

    def apply_speed_limit(self, speed: float) -> float:
        # apply 속도 limit 조건을 현재 명령이나 상태에 적용한다.
        speed = float(speed)
        if not bool(self.node.get_parameter('school_zone_speed_limit_enabled').value):
            return speed

        speed_limit_active = bool(getattr(self.node, 'school_zone_speed_limit_active', False))
        if not speed_limit_active:
            return speed

        limit = max(float(self.node.get_parameter('school_zone_speed').value), 0.0)
        if speed > 0.0:
            return min(speed, limit)
        return speed

    def takeover_requested(self) -> bool:
        # 어린이 보호구역 검출의 takeover requested 로직을 수행한다.
        return self.node._school_zone_takeover_requested()

    def log_text(self) -> str:
        # 로그 text 정보를 사람이 읽기 쉬운 로그 문자열로 만든다.
        return self.node._school_zone_log_text()

    def _apply_bev_result(self, result: SchoolZoneBEVResult):
        # apply BEV result 조건을 현재 명령이나 상태에 적용한다.
        now = time.monotonic()
        self.node.school_zone_active = bool(result.active)
        self.node.school_zone_candidate_active = bool(result.candidate)
        self._update_speed_limit_hold(bool(result.speed_limit_active), now)
        self.node.school_zone_yellow_left_ratio = float(result.left_ratio)
        self.node.school_zone_yellow_right_ratio = float(result.right_ratio)
        self.node.school_zone_yellow_pair_row_ratio = float(result.pair_row_ratio)
        self.node.school_zone_yellow_bottom_pair_row_ratio = float(result.bottom_pair_row_ratio)
        self.node.school_zone_yellow_separation_ratio = float(result.separation_ratio)

        if result.active:
            self.node.school_zone_confirm_count += 1
            self.node.school_zone_lost_count = 0
            self.node.school_zone_last_seen_sec = now
        elif result.candidate:
            self.node.school_zone_confirm_count = 0
            self.node.school_zone_lost_count = 0
            self.node.school_zone_candidate_last_seen_sec = now
        else:
            self.node.school_zone_confirm_count = 0
            self.node.school_zone_lost_count += 1
            self.node.school_zone_candidate_last_seen_sec = None

    def _update_speed_limit_hold(self, detected: bool, now: Optional[float] = None):
        # 최신 입력을 기준으로 update 속도 limit hold 관련 캐시와 상태를 갱신한다.
        if now is None:
            now = time.monotonic()
        hold_sec = max(float(self._param('school_zone_speed_limit_hold_sec', 1.0)), 0.0)
        if detected:
            self.node.school_zone_speed_limit_until_sec = now + hold_sec
        self.node.school_zone_speed_limit_active = bool(
            detected
            or now <= float(getattr(self.node, 'school_zone_speed_limit_until_sec', 0.0))
        )

    def _reset_state(self):
        # reset 상태 관련 내부 상태와 카운터를 초기화한다.
        self.last_result = SchoolZoneBEVResult()
        self.node.school_zone_active = False
        self.node.school_zone_candidate_active = False
        self.node.school_zone_speed_limit_active = False
        self.node.school_zone_speed_limit_until_sec = 0.0
        self.node.school_zone_yellow_left_ratio = 0.0
        self.node.school_zone_yellow_right_ratio = 0.0
        self.node.school_zone_yellow_pair_row_ratio = 0.0
        self.node.school_zone_yellow_bottom_pair_row_ratio = 0.0
        self.node.school_zone_yellow_separation_ratio = 0.0
        self.node.school_zone_confirm_count = 0
        self.node.school_zone_lost_count = 0
        self.node.school_zone_last_seen_sec = None
        self.node.school_zone_candidate_last_seen_sec = None

    def _param(self, name: str, default):
        # ROS 파라미터 값을 읽고 없으면 기본값을 사용한다.
        try:
            return self.node.get_parameter(name).value
        except Exception:
            return default
