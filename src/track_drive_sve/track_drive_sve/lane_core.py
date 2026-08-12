#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""검증된 차선 주행 코드를 Node에서 분리한 순수 core입니다.

새 버전 반영 사항(원본 LaneDetectionDriver 최신본과 동일 알고리즘):
- school zone hold/edge 모드 (스쿨존을 더 안정적으로 유지, 양쪽 노란 경계로 중심 추정)
- prev_lane_type 기억 (이전 프레임의 차선 종류 재사용)
- 외부 회피 명령 수신: path_offset / speed_limit
  (노드 콜백 대신 set_external_command() setter로 받는다. track_drive가 토픽 구독 후 매 프레임 전달)

ROS 구독/발행/타이머/drive는 제거했고, 검출 알고리즘과 PID 게인은 원본 값을 그대로 유지한다.
track_drive가 사용하는 인터페이스(compute, is_lane_stable, debug_summary, last_debug_image)는 보존한다.
"""

import math
import time
from typing import Callable, Dict, Optional, Tuple

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d

from . import config


class LaneCore:
    """카메라 이미지를 받아 검증된 차선 주행 angle/speed를 계산합니다."""

    def __init__(self, logger: Optional[Callable[[str], None]] = None):
        """원본 차선 노드의 상태 변수와 게인을 그대로 초기화합니다."""
        self.logger = logger
        self.image = None

        self.fix_speed = float(config.LANE_BASE_SPEED)
        self.lane_half_width = float(config.LANE_DEFAULT_HALF_WIDTH_PX)
        self.lane_detected = False
        self.angle_buffer = []
        self.last_angle = 0.0
        self.last_curve_angle = 0.0
        self.last_speed = 0.0
        self.last_valid_path = None
        self.lost_frames = 0
        self.max_reuse_frames = int(config.LANE_MAX_REUSE_FRAMES)
        self.curve_reuse_frames = int(config.LANE_CURVE_REUSE_FRAMES)
        self.curve_hold_frames = int(config.LANE_CURVE_HOLD_FRAMES)
        self.curve_hold_min_angle = float(config.LANE_CURVE_HOLD_MIN_ANGLE)
        self.prev_lane_base = {
            'left': None,
            'center': None,
            'right': None
        }
        self.prev_lane_type = {
            'left': None,
            'center': None,
            'right': None
        }
        self.yellow_center_switch_gate = int(config.LANE_YELLOW_CENTER_SWITCH_GATE)
        self.path_smoothing_alpha = float(config.LANE_PATH_SMOOTHING_ALPHA)
        self.target_forward_px = float(config.LANE_TARGET_FORWARD_PX)
        self.curve_target_forward_px = float(config.LANE_CURVE_TARGET_FORWARD_PX)
        self.preview_forward_px = float(config.LANE_PREVIEW_FORWARD_PX)
        self.far_preview_forward_px = float(config.LANE_FAR_PREVIEW_FORWARD_PX)
        self.curve_heading_threshold = float(config.LANE_CURVE_HEADING_THRESHOLD)
        self.curve_ahead_preview_heading_th = float(config.LANE_CURVE_AHEAD_PREVIEW_HEADING_TH)
        self.curve_ahead_far_heading_th = float(config.LANE_CURVE_AHEAD_FAR_HEADING_TH)
        self.preview_heading_kp = float(config.LANE_PREVIEW_HEADING_KP)
        self.speed_up_delta = float(config.LANE_SPEED_UP_DELTA)
        self.speed_down_delta = float(config.LANE_SPEED_DOWN_DELTA)
        self.straight_smooth_window = int(config.LANE_STRAIGHT_SMOOTH_WINDOW)
        self.curve_smooth_window = int(config.LANE_CURVE_SMOOTH_WINDOW)
        self.fast_straight_ratio = float(config.LANE_FAST_STRAIGHT_RATIO)
        self.short_straight_ratio = float(config.LANE_SHORT_STRAIGHT_RATIO)
        self.mild_curve_ratio = float(config.LANE_MILD_CURVE_RATIO)
        self.curve_ratio = float(config.LANE_CURVE_RATIO)
        self.hard_curve_ratio = float(config.LANE_HARD_CURVE_RATIO)
        self.extreme_curve_ratio = float(config.LANE_EXTREME_CURVE_RATIO)
        self.mild_curve_angle_th = float(config.LANE_MILD_CURVE_ANGLE_TH)
        self.hard_curve_angle_th = float(config.LANE_HARD_CURVE_ANGLE_TH)
        self.extreme_curve_angle_th = float(config.LANE_EXTREME_CURVE_ANGLE_TH)
        self.straight_confidence_frames = 0
        self.fast_straight_confirm_frames = int(config.LANE_FAST_STRAIGHT_CONFIRM_FRAMES)
        self.fast_straight_angle_th = float(config.LANE_FAST_STRAIGHT_ANGLE_TH)
        self.fast_straight_heading_th = float(config.LANE_FAST_STRAIGHT_HEADING_TH)
        self.fast_straight_preview_th = float(config.LANE_FAST_STRAIGHT_PREVIEW_TH)
        self.fast_straight_far_th = float(config.LANE_FAST_STRAIGHT_FAR_TH)
        self.fast_straight_speed_up_delta = float(config.LANE_FAST_STRAIGHT_SPEED_UP_DELTA)
        self._fast_straight_active = False
        self.lost_curve_speed_ratio = float(config.LANE_LOST_CURVE_SPEED_RATIO)
        self.reused_curve_speed_ratio = float(config.LANE_REUSED_CURVE_SPEED_RATIO)
        self.short_lane_length_min = int(config.LANE_SHORT_LANE_LENGTH_MIN)
        self.short_lane_length_max = int(config.LANE_SHORT_LANE_LENGTH_MAX)
        self.short_lane_length_scale = float(config.LANE_SHORT_LANE_LENGTH_SCALE)
        self.short_lane_min_factor = float(config.LANE_SHORT_LANE_MIN_FACTOR)
        self.curve_short_lane_min_factor = float(config.LANE_CURVE_SHORT_LANE_MIN_FACTOR)
        self.image_size = (
            int(config.LANE_IMAGE_WIDTH),
            int(config.LANE_IMAGE_HEIGHT)
        )
        self.warp_src_points = config.LANE_WARP_SRC_POINTS
        self.warp_dst_left_ratio = float(config.LANE_WARP_DST_LEFT_RATIO)
        self.warp_dst_right_ratio = float(config.LANE_WARP_DST_RIGHT_RATIO)
        self.white_hsv_lower = config.LANE_WHITE_HSV_LOWER
        self.white_hsv_upper = config.LANE_WHITE_HSV_UPPER
        self.yellow_hsv_lower = config.LANE_YELLOW_HSV_LOWER
        self.yellow_hsv_upper = config.LANE_YELLOW_HSV_UPPER
        self.mask_open_kernel = tuple(config.LANE_MASK_OPEN_KERNEL)
        self.mask_close_kernel = tuple(config.LANE_MASK_CLOSE_KERNEL)
        self.mask_open_iterations = int(config.LANE_MASK_OPEN_ITERATIONS)
        self.yellow_close_iterations = int(config.LANE_YELLOW_CLOSE_ITERATIONS)
        self.peak_histogram_y_start_ratio = float(config.LANE_PEAK_HISTOGRAM_Y_START_RATIO)
        self.peak_threshold_ratio = float(config.LANE_PEAK_THRESHOLD_RATIO)
        self.peak_cluster_gap_px = int(config.LANE_PEAK_CLUSTER_GAP_PX)
        self.yellow_center_prev_weight = float(config.LANE_YELLOW_CENTER_PREV_WEIGHT)
        self.yellow_center_image_weight = float(config.LANE_YELLOW_CENTER_IMAGE_WEIGHT)
        self.school_zone_wide_span_ratio = float(config.LANE_SCHOOL_ZONE_WIDE_SPAN_RATIO)
        self.school_zone_select_left_ratio = float(config.LANE_SCHOOL_ZONE_SELECT_LEFT_RATIO)
        self.school_zone_select_right_ratio = float(config.LANE_SCHOOL_ZONE_SELECT_RIGHT_RATIO)
        self.school_zone_left_edge_ratio = float(config.LANE_SCHOOL_ZONE_LEFT_EDGE_RATIO)
        self.school_zone_right_edge_ratio = float(config.LANE_SCHOOL_ZONE_RIGHT_EDGE_RATIO)
        self.external_min_path_offset = float(config.LANE_EXTERNAL_MIN_PATH_OFFSET)
        self.sync_stop_speed_eps = float(config.LANE_SYNC_STOP_SPEED_EPS)
        self.curve_hold_blend = float(config.LANE_CURVE_HOLD_BLEND)
        self.curve_hold_raw_blend = float(config.LANE_CURVE_HOLD_RAW_BLEND)
        self.s_curve_sign_heading_th = float(config.LANE_S_CURVE_SIGN_HEADING_TH)
        self.s_curve_max_heading_th = float(config.LANE_S_CURVE_MAX_HEADING_TH)
        self.s_curve_heading_sum_th = float(config.LANE_S_CURVE_HEADING_SUM_TH)
        # 짧은 직선-급커브 반복구간에서 직선 최고속이 순간적으로 들어가는 것을 막는다.
        # 직선 최고속 자체는 유지하고, 최근 곡선/전방 곡선이 있을 때만 잠깐 속도 상승을 제한한다.
        self.straight_curvature_th = float(config.LANE_STRAIGHT_CURVATURE_TH)
        self.mid_curvature_th = float(config.LANE_MID_CURVATURE_TH)
        self.hard_curvature_th = float(config.LANE_HARD_CURVATURE_TH)
        self.extreme_curvature_th = float(config.LANE_EXTREME_CURVATURE_TH)
        self.pre_corner_target_heading_th = float(config.LANE_PRE_CORNER_TARGET_HEADING_TH)
        self.pre_corner_preview_heading_th = float(config.LANE_PRE_CORNER_PREVIEW_HEADING_TH)
        self.pre_corner_far_heading_th = float(config.LANE_PRE_CORNER_FAR_HEADING_TH)
        self.pre_corner_near_sharp_heading_th = float(config.LANE_PRE_CORNER_NEAR_SHARP_HEADING_TH)
        self.pre_corner_preview_sharp_heading_th = float(config.LANE_PRE_CORNER_PREVIEW_SHARP_HEADING_TH)
        self.pre_corner_far_sharp_heading_th = float(config.LANE_PRE_CORNER_FAR_SHARP_HEADING_TH)
        self.curve_speed_guard_frames = 0
        self.curve_speed_guard_max = int(config.LANE_PRE_CORNER_HARD_HOLD_FRAMES)
        self.curve_speed_guard_soft_max = int(config.LANE_PRE_CORNER_SOFT_HOLD_FRAMES)
        self.curve_speed_guard_cap_ratio = float(config.LANE_PRE_CORNER_SOFT_CAP_RATIO)
        self.curve_speed_guard_sharp_cap_ratio = float(config.LANE_PRE_CORNER_HARD_CAP_RATIO)
        self.curve_speed_guard_up_delta = float(config.LANE_PRE_CORNER_SPEED_UP_DELTA)
        self.curve_speed_guard_down_delta = float(config.LANE_PRE_CORNER_SPEED_DOWN_DELTA)
        self.filtered_curvature = 0.0
        self.last_path_curvature = 0.0
        self.last_path_curve_ahead = False
        self.last_path_s_curve_like = False
        self.was_curve = False
        self.school_zone_active = False
        self.school_zone_counter = 0
        self.school_zone_enter_frames = int(config.LANE_SCHOOL_ZONE_ENTER_FRAMES)
        self.school_zone_counter_max = int(config.LANE_SCHOOL_ZONE_COUNTER_MAX)
        self.school_zone_hold_frames = int(config.LANE_SCHOOL_ZONE_HOLD_FRAMES)
        self.school_zone_hold_counter = 0
        self.use_school_zone_edges = False
        self.school_zone_speed_limit = float(config.LANE_SCHOOL_ZONE_SPEED_LIMIT)

        # 외부 회피 명령 (track_drive가 토픽 구독 후 set_external_command로 전달)
        self.external_path_offset = 0.0
        self.external_speed_limit = -1.0
        self.external_command_time = 0.0
        self.external_command_timeout = float(config.LANE_EXTERNAL_COMMAND_TIMEOUT)

        self.max_steer_angle = float(config.LANE_MAX_STEER_ANGLE)
        self.max_steer_delta = float(config.LANE_MAX_STEER_DELTA)

        self.path_resample_points = int(config.LANE_PATH_RESAMPLE_POINTS)
        self.path_fit_min_points = int(config.LANE_PATH_FIT_MIN_POINTS)
        self.path_duplicate_x_gap = float(config.LANE_PATH_DUPLICATE_X_GAP)
        self.path_heading_window = int(config.LANE_PATH_HEADING_WINDOW)
        self.path_min_heading_dx = float(config.LANE_PATH_MIN_HEADING_DX)
        self.curvature_percentile = float(config.LANE_CURVATURE_PERCENTILE)
        self.curvature_filter_rise_old_weight = float(config.LANE_CURVATURE_FILTER_RISE_OLD_WEIGHT)
        self.curvature_filter_rise_new_weight = float(config.LANE_CURVATURE_FILTER_RISE_NEW_WEIGHT)
        self.curvature_filter_fall_old_weight = float(config.LANE_CURVATURE_FILTER_FALL_OLD_WEIGHT)
        self.curvature_filter_fall_new_weight = float(config.LANE_CURVATURE_FILTER_FALL_NEW_WEIGHT)
        self.gaussian_smooth_sigma = float(config.LANE_GAUSSIAN_SMOOTH_SIGMA)
        self.min_center_line_points = int(config.LANE_MIN_CENTER_LINE_POINTS)
        self.sliding_windows = int(config.LANE_SLIDING_WINDOWS)
        self.sliding_margin = int(config.LANE_SLIDING_MARGIN)
        self.sliding_minpix = int(config.LANE_SLIDING_MINPIX)

        self.pid_kp = float(config.LANE_PID_KP)
        self.pid_ki = float(config.LANE_PID_KI)
        self.pid_kd = float(config.LANE_PID_KD)
        self.heading_kp = float(config.LANE_HEADING_KP)
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_integral_limit = float(config.LANE_PID_INTEGRAL_LIMIT)

        self.last_debug: Dict[str, object] = {}
        self.last_debug_image = None

    def _warn(self, message: str) -> None:
        """필요할 때만 외부 logger로 경고를 전달합니다."""
        if self.logger is not None:
            self.logger(message)

    # =====================================================================
    # 외부 회피 명령 수신 (노드 콜백 대신 setter)
    # =====================================================================
    def set_external_command(self, path_offset: float = None, speed_limit: float = None) -> None:
        """track_drive가 회피 노드 토픽을 구독해 매 프레임 전달한다.
        둘 중 받은 값만 갱신하고, 갱신 시각을 기록해 timeout으로 오래된 명령을 무시한다."""
        if path_offset is not None:
            self.external_path_offset = float(path_offset)
        if speed_limit is not None:
            self.external_speed_limit = float(speed_limit)
        self.external_command_time = time.time()

    def external_command_active(self) -> bool:
        """최근 external_command_timeout(초) 안에 외부 명령을 받았는지."""
        return (time.time() - self.external_command_time) <= self.external_command_timeout

    def apply_external_path_offset(self, path):
        """회피 offset이 활성 상태면 경로를 좌우로 민다 (원본 로직)."""
        if not path or not self.external_command_active():
            return path
        offset = self.external_path_offset
        if abs(offset) < self.external_min_path_offset:
            return path
        return [(x, y + offset) for x, y in path]

    def apply_external_speed_limit(self, speed):
        """회피 중 속도 제한이 활성이면 그 값으로 제한 (원본 로직)."""
        if self.external_command_active() and self.external_speed_limit > 0.0:
            return self.external_speed_limit
        return speed

    def sync_with_final_command(self, final_angle: float, final_speed: float) -> None:
        """인터럽트/회피 arbitration 후 최종 명령을 내부 속도 램프와 동기화한다.

        보행자 정지 중에도 lane_core는 매 프레임 계산되므로 내부 last_speed가 크게
        올라가 있을 수 있다. 최종 속도가 0/낮은 cap으로 막혔으면 내부 속도 램프도 그
        최종값까지 낮춰, 정지 해제 직후 속도가 튀어 곡선에서 이탈하는 것을 막는다.
        조향(final_angle)은 건드리지 않는다 — 정지 중 조향 유지는 인터럽트에서 처리.
        """
        _ = final_angle
        fs = max(0.0, float(final_speed))
        if fs <= self.sync_stop_speed_eps:
            self.last_speed = 0.0
        elif self.last_speed > fs:
            self.last_speed = fs

    def is_lane_stable(self) -> bool:
        """현재 프레임에서 재사용이 아닌 실제 차선 경로가 안정 검출되었는지 반환합니다."""
        return bool(self.last_debug.get('lane_stable', False))

    def debug_summary(self) -> Dict[str, object]:
        """최근 차선 계산 디버그 정보를 반환합니다."""
        return dict(self.last_debug)

    def warp_perspective(self, img):
        h, w = img.shape[:2]

        src = np.float32(self.warp_src_points)

        dst = np.float32([
            [w * self.warp_dst_left_ratio, 0],
            [w * self.warp_dst_right_ratio, 0],
            [w * self.warp_dst_right_ratio, h],
            [w * self.warp_dst_left_ratio, h]
        ])

        M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(img, M, (w, h))

    def preprocess_split(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        white_mask = cv2.inRange(
            hsv,
            self.white_hsv_lower,
            self.white_hsv_upper
        )

        yellow_mask = cv2.inRange(
            hsv,
            self.yellow_hsv_lower,
            self.yellow_hsv_upper
        )

        kernel_open = np.ones(self.mask_open_kernel, dtype=np.uint8)
        kernel_close = np.ones(self.mask_close_kernel, dtype=np.uint8)

        white_mask = cv2.morphologyEx(
            white_mask,
            cv2.MORPH_OPEN,
            kernel_open,
            iterations=self.mask_open_iterations
        )
        yellow_mask = cv2.morphologyEx(
            yellow_mask,
            cv2.MORPH_OPEN,
            kernel_open,
            iterations=self.mask_open_iterations
        )
        yellow_mask = cv2.morphologyEx(
            yellow_mask,
            cv2.MORPH_CLOSE,
            kernel_close,
            iterations=self.yellow_close_iterations
        )

        return white_mask, yellow_mask

    def find_lane_peaks(self, mask):
        h = mask.shape[0]
        histogram = np.sum(
            mask[int(h * self.peak_histogram_y_start_ratio):, :],
            axis=0
        )

        if np.max(histogram) == 0:
            return []

        threshold = self.peak_threshold_ratio * np.max(histogram)
        peaks = np.where(histogram > threshold)[0]

        clusters = []
        cluster = []

        for i in range(len(peaks)):
            if i == 0 or peaks[i] - peaks[i - 1] <= self.peak_cluster_gap_px:
                cluster.append(peaks[i])
            else:
                clusters.append(cluster)
                cluster = [peaks[i]]

        if cluster:
            clusters.append(cluster)

        return [int(np.mean(c)) for c in clusters]

    def classify_lanes_by_color_and_position(self, white_mask, yellow_mask, width):
        lanes = {
            'left': None,
            'center': None,
            'right': None
        }
        self.use_school_zone_edges = False

        white_peaks = self.find_lane_peaks(white_mask)
        yellow_peaks = self.find_lane_peaks(yellow_mask)
        school_zone_seen = self.update_school_zone_state(yellow_peaks, width)
        wide_yellow_pair = (
            len(yellow_peaks) >= 2 and
            max(yellow_peaks) - min(yellow_peaks) > width * self.school_zone_wide_span_ratio
        )
        school_zone_edge_mode = (
            school_zone_seen or
            (self.school_zone_active and wide_yellow_pair)
        )

        if school_zone_edge_mode:
            left_yellow, right_yellow = self.select_school_zone_yellow_edges(
                yellow_peaks,
                width
            )

            if left_yellow is not None and right_yellow is not None:
                lanes['left'] = ('yellow', left_yellow)
                lanes['right'] = ('yellow', right_yellow)
                self.use_school_zone_edges = True
                return lanes

        if yellow_peaks:
            center_x = self.select_yellow_center_peak(
                yellow_peaks,
                width
            )

            if center_x is not None:
                lanes['center'] = ('yellow', center_x)

        if white_peaks:
            white_peaks = sorted(white_peaks)

            lefts = [x for x in white_peaks if x < width // 2]
            rights = [x for x in white_peaks if x >= width // 2]

            if lefts:
                lanes['left'] = ('white', int(np.mean(lefts)))

            if rights:
                lanes['right'] = ('white', int(np.mean(rights)))

        return lanes

    def select_school_zone_yellow_edges(self, yellow_peaks, width):
        if len(yellow_peaks) < 2:
            return None, None

        sorted_peaks = sorted(yellow_peaks)
        left_candidates = [
            x for x in sorted_peaks
            if x < width * self.school_zone_select_left_ratio
        ]
        right_candidates = [
            x for x in sorted_peaks
            if x > width * self.school_zone_select_right_ratio
        ]

        if left_candidates and right_candidates:
            return int(left_candidates[0]), int(right_candidates[-1])

        if sorted_peaks[-1] - sorted_peaks[0] > width * self.school_zone_wide_span_ratio:
            return int(sorted_peaks[0]), int(sorted_peaks[-1])

        return None, None

    def select_yellow_center_peak(self, yellow_peaks, width):
        if not yellow_peaks:
            return None

        image_center = width // 2
        prev_center = self.prev_lane_base.get('center')

        if prev_center is None:
            return int(
                min(
                    yellow_peaks,
                    key=lambda x: abs(x - image_center)
                )
            )

        gated_peaks = [
            x for x in yellow_peaks
            if abs(x - prev_center) <= self.yellow_center_switch_gate
        ]

        if not gated_peaks:
            return None

        return int(
            min(
                gated_peaks,
                key=lambda x: (
                    self.yellow_center_prev_weight * abs(x - prev_center) +
                    self.yellow_center_image_weight * abs(x - image_center)
                )
            )
        )

    def update_school_zone_state(self, yellow_peaks, width):
        has_left_yellow_edge = any(
            x < width * self.school_zone_left_edge_ratio
            for x in yellow_peaks
        )
        has_right_yellow_edge = any(
            x > width * self.school_zone_right_edge_ratio
            for x in yellow_peaks
        )
        wide_yellow_span = (
            len(yellow_peaks) >= 2 and
            max(yellow_peaks) - min(yellow_peaks) > width * self.school_zone_wide_span_ratio
        )

        school_zone_seen = (
            has_left_yellow_edge and
            has_right_yellow_edge and
            wide_yellow_span
        )

        if school_zone_seen:
            self.school_zone_counter = min(
                self.school_zone_counter + 1,
                self.school_zone_counter_max
            )
            if self.school_zone_counter >= self.school_zone_enter_frames:
                self.school_zone_hold_counter = self.school_zone_hold_frames
        else:
            self.school_zone_counter = max(
                self.school_zone_counter - 1,
                0
            )
            self.school_zone_hold_counter = max(
                self.school_zone_hold_counter - 1,
                0
            )

        self.school_zone_active = (
            self.school_zone_counter >= self.school_zone_enter_frames or
            self.school_zone_hold_counter > 0
        )

        return school_zone_seen

    def apply_school_zone_speed_limit(self, speed):
        if self.school_zone_active:
            return min(speed, self.school_zone_speed_limit)

        return speed

    def convert_to_vehicle_coords(self, pts, shape):
        h, w = shape[:2]

        return [
            (h - py, -(px - w // 2))
            for (px, py) in pts
        ]

    def sliding_window_center_lane(
        self,
        binary_img,
        out_img,
        base_x=None,
        nwindows=None,
        margin=None,
        minpix=None,
        color=(0, 255, 0)
    ):
        nwindows = self.sliding_windows if nwindows is None else nwindows
        margin = self.sliding_margin if margin is None else margin
        minpix = self.sliding_minpix if minpix is None else minpix

        h, w = binary_img.shape
        histogram = np.sum(
            binary_img[int(h * self.peak_histogram_y_start_ratio):, :],
            axis=0
        )

        base = base_x if base_x is not None else np.argmax(histogram)
        window_height = h // nwindows

        nonzero = binary_img.nonzero()
        nonzeroy = nonzero[0]
        nonzerox = nonzero[1]

        x_current = base
        last_valid_x = base
        pts = []

        for window in range(nwindows):
            y_low = h - (window + 1) * window_height
            y_high = h - window * window_height

            x_low = x_current - margin
            x_high = x_current + margin

            cv2.rectangle(out_img, (x_low, y_low), (x_high, y_high), color, 2)

            good_inds = (
                (nonzeroy >= y_low) &
                (nonzeroy < y_high) &
                (nonzerox >= x_low) &
                (nonzerox < x_high)
            ).nonzero()[0]

            if len(good_inds) > minpix:
                x_current = int(np.mean(nonzerox[good_inds]))
                last_valid_x = x_current
                y_current = int(np.mean(nonzeroy[good_inds]))

                pts.append((x_current, y_current))
                cv2.circle(out_img, (x_current, y_current), 5, color, -1)
            else:
                x_current = last_valid_x

        return pts

    def infer_road_center(self, lane_coords):
        left = lane_coords['left']
        center = lane_coords['center']
        right = lane_coords['right']

        if self.use_school_zone_edges and left and right:
            min_len = min(len(left), len(right))

            return [
                (
                    (left[i][0] + right[i][0]) / 2.0,
                    (left[i][1] + right[i][1]) / 2.0
                )
                for i in range(min_len)
            ], 'YELLOW_EDGES_CENTER'

        if center:
            return center, 'YELLOW'

        if left and right:
            min_len = min(len(left), len(right))
            widths = [
                abs(left[i][1] - right[i][1]) * 0.5
                for i in range(min_len)
            ]
            if widths:
                self.lane_half_width = float(np.median(widths))

            return [
                (
                    (left[i][0] + right[i][0]) / 2.0,
                    (left[i][1] + right[i][1]) / 2.0
                )
                for i in range(min_len)
            ], 'WHITE_CENTER'

        if left:
            return [
                (x, y - self.lane_half_width)
                for (x, y) in left
            ], 'LEFT_ONLY'

        if right:
            return [
                (x, y + self.lane_half_width)
                for (x, y) in right
            ], 'RIGHT_ONLY'

        return [], 'NONE'

    def generate_resampled_path(self, center_line_pts):
        """
        기존 polyfit 방식은 꼬불꼬불한 길을 하나의 2차/3차 곡선으로 펴버릴 수 있다.
        그래서 여기서는 다항식 피팅을 제거하고,
        sliding window로 얻은 실제 차선 점들을 거리 기준으로 재샘플링한다.
        """

        if not center_line_pts or len(center_line_pts) < self.path_fit_min_points:
            return center_line_pts

        pts = sorted(center_line_pts, key=lambda p: p[0])

        filtered = []
        last_x = None

        for x, y in pts:
            if last_x is None or abs(x - last_x) > self.path_duplicate_x_gap:
                filtered.append((float(x), float(y)))
                last_x = x

        if len(filtered) < self.path_fit_min_points:
            return filtered

        xs = np.array([p[0] for p in filtered], dtype=np.float32)
        ys = np.array([p[1] for p in filtered], dtype=np.float32)

        dx = np.diff(xs)
        dy = np.diff(ys)
        ds = np.sqrt(dx ** 2 + dy ** 2)

        s = np.insert(np.cumsum(ds), 0, 0.0)

        if s[-1] < 1.0:
            return filtered

        s_new = np.linspace(0.0, s[-1], self.path_resample_points)

        x_new = np.interp(s_new, s, xs)
        y_new = np.interp(s_new, s, ys)

        return [
            (float(x), float(y))
            for x, y in zip(x_new, y_new)
        ]

    def stabilize_path(self, detected_path):
        if not detected_path or len(detected_path) < self.path_fit_min_points:
            return None

        path = sorted(
            [(float(x), float(y)) for x, y in detected_path if x >= 0.0],
            key=lambda p: p[0]
        )

        if len(path) < self.path_fit_min_points:
            return None

        if self.last_valid_path is not None and len(self.last_valid_path) >= 2:
            cur_x = np.array([p[0] for p in path], dtype=np.float32)
            cur_y = np.array([p[1] for p in path], dtype=np.float32)
            prev_x = np.array([p[0] for p in self.last_valid_path], dtype=np.float32)
            prev_y = np.array([p[1] for p in self.last_valid_path], dtype=np.float32)

            prev_interp_y = np.interp(
                cur_x,
                prev_x,
                prev_y,
                left=prev_y[0],
                right=prev_y[-1]
            )
            alpha = float(np.clip(self.path_smoothing_alpha, 0.0, 1.0))
            cur_y = alpha * cur_y + (1.0 - alpha) * prev_interp_y
            path = list(zip(cur_x, cur_y))

        self.last_valid_path = path
        return path

    def get_tracking_path(self, detected_path):
        stable_path = self.stabilize_path(detected_path)

        if stable_path is not None:
            self.lost_frames = 0
            self.lane_detected = True
            return stable_path, False

        self.lane_detected = False
        self.lost_frames += 1

        reuse_limit = self.max_reuse_frames
        if (
            self.is_curve_loss_context() or
            self.curve_speed_guard_frames > 0 or
            self.path_has_curve_ahead(self.last_valid_path)
        ):
            reuse_limit = min(self.max_reuse_frames, self.curve_reuse_frames)

        if self.last_valid_path is not None and self.lost_frames <= reuse_limit:
            return self.last_valid_path, True

        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        return None, False

    def get_steering_pid(self, path, return_debug=False):
        """
        PID 기반 차선 추종 제어
        """

        if path is None or len(path) < 2:
            if return_debug:
                return self.last_angle, {
                    'heading_error': 0.0,
                    'preview_heading': 0.0,
                    'far_preview_heading': 0.0,
                    'curve_ahead': False,
                    'target_forward_px': self.target_forward_px,
                    'target_y': 0.0,
                }
            return self.last_angle

        target_idx, target_x, target_y = self.find_target_point(
            path,
            self.target_forward_px
        )
        heading_error = self.path_heading_at(path, target_idx)
        preview_idx, _, _ = self.find_target_point(
            path,
            self.preview_forward_px
        )
        far_idx, _, _ = self.find_target_point(
            path,
            self.far_preview_forward_px
        )
        preview_heading = self.path_heading_at(path, preview_idx)
        far_preview_heading = self.path_heading_at(path, far_idx)

        curve_ahead = (
            abs(heading_error) > self.curve_heading_threshold or
            abs(preview_heading) > self.curve_ahead_preview_heading_th or
            abs(far_preview_heading) > self.curve_ahead_far_heading_th
        )

        if curve_ahead:
            target_idx, target_x, target_y = self.find_target_point(
                path,
                self.curve_target_forward_px
            )
            heading_error = self.path_heading_at(path, target_idx)

        error = target_y

        dt = 0.05

        self.pid_integral += error * dt
        self.pid_integral = float(
            np.clip(
                self.pid_integral,
                -self.pid_integral_limit,
                self.pid_integral_limit
            )
        )

        derivative = (error - self.pid_prev_error) / dt
        self.pid_prev_error = error

        control = (
            self.pid_kp * error +
            self.pid_ki * self.pid_integral +
            self.pid_kd * derivative +
            self.heading_kp * heading_error +
            self.preview_heading_kp * preview_heading
        )

        angle = -control

        if return_debug:
            return angle, {
                'heading_error': heading_error,
                'preview_heading': preview_heading,
                'far_preview_heading': far_preview_heading,
                'curve_ahead': bool(curve_ahead),
                'target_forward_px': (
                    self.curve_target_forward_px
                    if curve_ahead
                    else self.target_forward_px
                ),
                'target_y': target_y,
            }

        return angle

    def find_target_point(self, path, forward_px):
        target_idx = len(path) - 1
        target_x, target_y = path[-1]

        for i, (x, y) in enumerate(path):
            if x >= forward_px:
                target_idx = i
                target_x, target_y = x, y
                break

        return target_idx, target_x, target_y

    def path_heading_at(self, path, idx):
        if path is None or len(path) < 2:
            return 0.0

        i0 = max(idx - self.path_heading_window, 0)
        i1 = min(idx + self.path_heading_window, len(path) - 1)

        if i0 == i1:
            return 0.0

        dx = path[i1][0] - path[i0][0]
        dy = path[i1][1] - path[i0][1]

        return math.atan2(dy, max(dx, self.path_min_heading_dx))

    def path_heading_at_forward(self, path, forward_px):
        if path is None or len(path) < 2:
            return 0.0

        idx, _, _ = self.find_target_point(path, forward_px)
        return self.path_heading_at(path, idx)

    def path_has_curve_ahead(self, path):
        if path is None or len(path) < 2:
            return False

        heading = self.path_heading_at_forward(path, self.target_forward_px)
        preview_heading = self.path_heading_at_forward(
            path,
            self.preview_forward_px
        )
        far_heading = self.path_heading_at_forward(
            path,
            self.far_preview_forward_px
        )

        return (
            abs(heading) > self.curve_heading_threshold or
            abs(preview_heading) > self.curve_ahead_preview_heading_th or
            abs(far_heading) > self.curve_ahead_far_heading_th
        )

    def clamp_angle(self, angle):
        return float(
            np.clip(
                angle,
                -self.max_steer_angle,
                self.max_steer_angle
            )
        )

    def limit_angle_change(self, angle, last_angle):
        delta = angle - last_angle

        if abs(delta) > self.max_steer_delta:
            return float(
                last_angle + np.clip(
                    delta,
                    -self.max_steer_delta,
                    self.max_steer_delta
                )
            )

        return float(angle)

    def get_curve_hold_angle(self):
        if abs(self.last_curve_angle) >= self.curve_hold_min_angle:
            return self.last_curve_angle

        if abs(self.last_angle) >= self.curve_hold_min_angle:
            return self.last_angle

        return self.last_angle

    def keep_curve_angle_if_lost(self, raw_steer, reused_path):
        if not reused_path or self.lost_frames > self.curve_hold_frames:
            return raw_steer

        if not self.was_curve and abs(self.last_curve_angle) < self.curve_hold_min_angle:
            return raw_steer

        hold_angle = self.get_curve_hold_angle()

        if abs(hold_angle) < self.curve_hold_min_angle:
            return raw_steer

        if raw_steer * hold_angle <= 0.0 or abs(raw_steer) < abs(hold_angle):
            return (
                self.curve_hold_blend * hold_angle +
                self.curve_hold_raw_blend * raw_steer
            )

        return raw_steer

    def update_curve_memory(self, angle, curvature, curve_ahead=False, s_curve_like=False):
        curve_context = (
            curvature > self.straight_curvature_th or
            curve_ahead or
            s_curve_like
        )
        self.was_curve = curve_context

        if curve_context and abs(angle) >= self.curve_hold_min_angle:
            self.last_curve_angle = angle
        elif not self.was_curve:
            self.last_curve_angle = 0.0

    def is_curve_loss_context(self):
        return (
            self.was_curve or
            self.last_path_curve_ahead or
            self.last_path_s_curve_like or
            self.last_path_curvature > self.straight_curvature_th or
            self.path_has_curve_ahead(self.last_valid_path)
        )

    def get_straight_lane_speed(self):
        return self.get_speed_by_curvature(0.0, 0.0)

    def get_lost_lane_speed(self):
        if self.is_curve_loss_context():
            return self.fix_speed * self.lost_curve_speed_ratio

        return max(self.last_speed, self.get_straight_lane_speed())

    def smooth_angle(self, new_angle, window=None):
        if window is None:
            window = self.straight_smooth_window

        if self.angle_buffer:
            prev_angle = self.angle_buffer[-1]
            if (
                new_angle * prev_angle < 0.0 and
                abs(new_angle - prev_angle) >= 18.0
            ):
                self.angle_buffer = [new_angle]
                return float(new_angle)

        self.angle_buffer.append(new_angle)

        if len(self.angle_buffer) > window:
            self.angle_buffer.pop(0)

        return float(np.mean(self.angle_buffer))

    def estimate_curvature(self, path):
        if path is None or len(path) < 3:
            return 0.0

        xs, ys = zip(*path)

        xs = np.array(xs, dtype=np.float32)
        ys = np.array(ys, dtype=np.float32)

        dx = np.diff(xs)
        dy = np.diff(ys)

        d2x = np.diff(dx)
        d2y = np.diff(dy)

        denominator = (dx[:-1] ** 2 + dy[:-1] ** 2) ** 1.5
        valid = denominator > 1e-6

        if not np.any(valid):
            return 0.0

        curvature = np.abs(
            d2x[valid] * dy[:-1][valid] -
            d2y[valid] * dx[:-1][valid]
        ) / denominator[valid]

        if curvature.size == 0:
            return 0.0

        return float(np.percentile(curvature, self.curvature_percentile))

    def update_straight_confidence(self, curvature, angle, steering_debug):
        """긴 직선인지 누적 판정한다.

        S자 중간에서 잠깐 직선처럼 보이는 프레임에는 최고속을 열지 않고,
        near/preview/far heading이 조용한 상태가 이어질 때만 fast straight로 본다.
        """
        heading = abs(float(steering_debug.get('heading_error', 0.0)))
        preview = abs(float(steering_debug.get('preview_heading', 0.0)))
        far_preview = abs(float(steering_debug.get('far_preview_heading', 0.0)))
        abs_angle = abs(float(angle))

        straight_like = (
            curvature <= self.straight_curvature_th and
            abs_angle <= self.fast_straight_angle_th and
            heading <= self.fast_straight_heading_th and
            preview <= self.fast_straight_preview_th and
            far_preview <= self.fast_straight_far_th and
            self.curve_speed_guard_frames <= 0 and
            not self.last_path_curve_ahead and
            not self.last_path_s_curve_like and
            not bool(steering_debug.get('curve_ahead', False))
        )

        if straight_like:
            self.straight_confidence_frames = min(
                self.straight_confidence_frames + 1,
                self.fast_straight_confirm_frames
            )
        else:
            obvious_curve = (
                curvature > self.mid_curvature_th or
                abs_angle > self.mild_curve_angle_th or
                heading > self.fast_straight_heading_th * 1.8 or
                preview > self.fast_straight_preview_th * 1.8 or
                far_preview > self.fast_straight_far_th * 1.8 or
                self.curve_speed_guard_frames > 0 or
                self.last_path_curve_ahead or
                self.last_path_s_curve_like or
                bool(steering_debug.get('curve_ahead', False))
            )
            if obvious_curve:
                self.straight_confidence_frames = 0
            else:
                self.straight_confidence_frames = max(
                    self.straight_confidence_frames - 1,
                    0
                )

        self._fast_straight_active = (
            self.straight_confidence_frames >= self.fast_straight_confirm_frames
        )
        return self._fast_straight_active

    def get_straight_speed_ratio(self):
        if self.fast_straight_confirm_frames <= 0:
            return self.fast_straight_ratio

        effective_frames = self.straight_confidence_frames
        if effective_frames > 0:
            effective_frames += 1

        progress = float(np.clip(
            effective_frames / self.fast_straight_confirm_frames,
            0.0,
            1.0
        ))
        return (
            self.short_straight_ratio +
            (self.fast_straight_ratio - self.short_straight_ratio) * progress
        )

    def get_speed_by_curvature(self, curvature, angle, fast_straight=False):
        """곡선 속도 계획.

        직선 목표속도는 confidence에 따라 단계적으로 열린다.
        곡선 사이의 짧은 직선 조각은 낮은 ratio에 머물고, 긴 직선에서만 최고속까지 간다.
        """
        _ = fast_straight
        abs_angle = abs(angle)

        if (
            curvature <= self.straight_curvature_th and
            abs_angle <= self.fast_straight_angle_th
        ):
            speed_ratio = self.get_straight_speed_ratio()
        elif curvature > self.extreme_curvature_th or abs_angle > self.extreme_curve_angle_th:
            speed_ratio = self.extreme_curve_ratio
        elif curvature > self.hard_curvature_th or abs_angle > self.hard_curve_angle_th:
            speed_ratio = self.hard_curve_ratio
        elif curvature > self.mid_curvature_th or abs_angle > self.mild_curve_angle_th:
            speed_ratio = self.curve_ratio
        elif curvature > self.straight_curvature_th or abs_angle > self.fast_straight_angle_th:
            speed_ratio = self.mild_curve_ratio
        else:
            speed_ratio = self.short_straight_ratio

        return self.fix_speed * speed_ratio

    def update_curve_speed_guard(self, path, curvature, angle):
        """짧은 직선 뒤 급커브가 반복되는 구간용 속도 가드.

        현재 프레임만 보면 직선처럼 보여도 바로 직전/전방이 곡선이면 직선 최고속으로
        튀지 않게 몇 프레임 동안 곡선 속도 상한을 유지한다. 긴 직선에서는 countdown이
        끝난 뒤 기존 직선 최고속 로직으로 자동 복귀한다.
        """
        abs_angle = abs(angle)
        target_heading = 0.0
        near_heading = 0.0
        preview_heading = 0.0
        far_preview_heading = 0.0

        if path is not None and len(path) >= 2:
            target_heading = self.path_heading_at_forward(
                path,
                self.target_forward_px
            )
            near_heading = self.path_heading_at_forward(
                path,
                self.curve_target_forward_px
            )
            preview_heading = self.path_heading_at_forward(
                path,
                self.preview_forward_px
            )
            far_preview_heading = self.path_heading_at_forward(
                path,
                self.far_preview_forward_px
            )

        abs_target_heading = abs(target_heading)
        abs_near_heading = abs(near_heading)
        abs_preview_heading = abs(preview_heading)
        abs_far_preview_heading = abs(far_preview_heading)
        heading_values = [
            target_heading,
            near_heading,
            preview_heading,
            far_preview_heading
        ]
        sign_flip = any(
            a * b < 0.0
            and abs(a) > self.s_curve_sign_heading_th
            and abs(b) > self.s_curve_sign_heading_th
            for i, a in enumerate(heading_values)
            for b in heading_values[i + 1:]
        )

        curve_like = (
            curvature > self.straight_curvature_th or
            abs_angle > self.fast_straight_angle_th or
            abs_target_heading > self.pre_corner_target_heading_th or
            abs_preview_heading > self.pre_corner_preview_heading_th or
            abs_far_preview_heading > self.pre_corner_far_heading_th
        )
        sharp_like = (
            curvature > self.mid_curvature_th or
            abs_angle > self.mild_curve_angle_th or
            abs_near_heading > self.pre_corner_near_sharp_heading_th or
            abs_preview_heading > self.pre_corner_preview_sharp_heading_th or
            abs_far_preview_heading > self.pre_corner_far_sharp_heading_th
        )
        s_curve_like = (
            sign_flip and
            max(
                abs_target_heading,
                abs_near_heading,
                abs_preview_heading,
                abs_far_preview_heading
            ) > self.s_curve_max_heading_th and
            (
                abs_target_heading +
                abs_preview_heading +
                abs_far_preview_heading
            ) > self.s_curve_heading_sum_th
        )

        if sharp_like or s_curve_like:
            self.curve_speed_guard_frames = self.curve_speed_guard_max
        elif curve_like:
            self.curve_speed_guard_frames = max(
                self.curve_speed_guard_frames,
                self.curve_speed_guard_soft_max
            )
        elif self.curve_speed_guard_frames > 0:
            self.curve_speed_guard_frames -= 1

        return (
            abs_near_heading,
            abs_preview_heading,
            abs_far_preview_heading,
            sharp_like,
            curve_like,
            s_curve_like,
            self.curve_speed_guard_frames > 0
        )

    def apply_curve_speed_guard(self, speed, path, curvature, angle):
        (
            near_heading,
            preview_heading,
            far_preview_heading,
            sharp_like,
            curve_like,
            s_curve_like,
            active
        ) = self.update_curve_speed_guard(
            path,
            curvature,
            angle
        )

        if active and not self.school_zone_active:
            cap_ratio = (
                self.curve_speed_guard_sharp_cap_ratio
                if sharp_like or s_curve_like
                else self.curve_speed_guard_cap_ratio
            )
            speed = min(speed, self.fix_speed * cap_ratio)

        return (
            speed,
            near_heading,
            preview_heading,
            far_preview_heading,
            sharp_like,
            curve_like,
            s_curve_like,
            active
        )

    def limit_speed_change(self, speed):
        if self.last_speed <= 0.0:
            return float(speed)

        speed_up_delta = self.speed_up_delta
        speed_down_delta = self.speed_down_delta

        if self._fast_straight_active and self.curve_speed_guard_frames <= 0:
            speed_up_delta = max(speed_up_delta, self.fast_straight_speed_up_delta)

        if self.curve_speed_guard_frames > 0:
            speed_up_delta = min(speed_up_delta, self.curve_speed_guard_up_delta)
            speed_down_delta = max(speed_down_delta, self.curve_speed_guard_down_delta)

        delta = speed - self.last_speed

        if delta > speed_up_delta:
            return float(self.last_speed + speed_up_delta)

        if delta < -speed_down_delta:
            return float(self.last_speed - speed_down_delta)

        return float(speed)

    def compute(self, image) -> Tuple[float, float]:
        """카메라 BGR 이미지를 받아 차선 주행 angle/speed를 반환합니다.
        외부 회피 offset/speed_limit이 활성이면 경로를 밀고 속도를 제한한다."""
        if image is None:
            self.lane_detected = False
            self.last_debug = {
                'mode': 'NO_IMAGE',
                'lane_stable': False,
                'lane_detected': False,
                'lane_length': 0,
                'detected_lane_length': 0,
                'reused_path': False,
                'school_zone': self.school_zone_active,
                'preview_heading': 0.0,
                'far_preview_heading': 0.0,
                'curve_ahead': False,
                's_curve_like': False,
                'curve_speed_guard': self.curve_speed_guard_frames > 0,
                'curve_speed_guard_frames': self.curve_speed_guard_frames,
                'raw_angle': 0.0,
                'angle': 0.0,
                'speed': 0.0,
            }
            return 0.0, 0.0

        self.image = cv2.resize(image, self.image_size, interpolation=cv2.INTER_LINEAR)
        bird_view = self.warp_perspective(self.image)

        white_mask, yellow_mask = self.preprocess_split(bird_view)
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)

        debug_img = cv2.cvtColor(
            combined_mask,
            cv2.COLOR_GRAY2BGR
        )

        lanes = self.classify_lanes_by_color_and_position(
            white_mask,
            yellow_mask,
            bird_view.shape[1]
        )

        lane_vehicle_coords = {
            'left': [],
            'center': [],
            'right': []
        }

        path = None

        for key in ['left', 'center', 'right']:
            lane_candidate = lanes[key]

            if (
                lane_candidate is None and
                self.prev_lane_base[key] is not None and
                not (self.use_school_zone_edges and key == 'center')
            ):
                lane_candidate = (
                    self.prev_lane_type[key] or (
                        'yellow' if key == 'center' else 'white'
                    ),
                    self.prev_lane_base[key]
                )

            if lane_candidate:
                lane_type, x_base = lane_candidate

                mask = yellow_mask if lane_type == 'yellow' else white_mask

                color = {
                    'left': (255, 0, 0),
                    'center': (0, 255, 255),
                    'right': (0, 0, 255)
                }[key]

                pts = self.sliding_window_center_lane(
                    mask,
                    debug_img,
                    base_x=x_base,
                    color=color
                )

                vehicle_pts = self.convert_to_vehicle_coords(
                    pts,
                    bird_view.shape
                )

                lane_vehicle_coords[key] = vehicle_pts

                if pts:
                    self.prev_lane_base[key] = pts[0][0]
                    self.prev_lane_type[key] = lane_type

        center_line_pts, path_mode = self.infer_road_center(
            lane_vehicle_coords
        )

        detected_lane_length = len(center_line_pts) if center_line_pts else 0

        if center_line_pts and detected_lane_length > self.min_center_line_points:

            path = self.generate_resampled_path(center_line_pts)

            if path and len(path) > self.min_center_line_points:
                xs, ys = zip(*path)

                xs = gaussian_filter1d(xs, sigma=self.gaussian_smooth_sigma)
                ys = gaussian_filter1d(ys, sigma=self.gaussian_smooth_sigma)

                path = list(zip(xs, ys))

        path, reused_path = self.get_tracking_path(path)
        # 회피 offset 적용 (활성 시 경로를 좌우로 민다)
        path = self.apply_external_path_offset(path)
        lane_length = len(path) if path else 0

        if path:
            for x, y in path:
                px = int(-y + bird_view.shape[1] // 2)
                py = int(bird_view.shape[0] - x)

                if 0 <= px < bird_view.shape[1] and 0 <= py < bird_view.shape[0]:
                    cv2.circle(debug_img, (px, py), 4, (255, 255, 0), -1)

        self.last_debug_image = debug_img

        try:
            if config.SHOW_LANE_BIRDVIEW_WINDOW:
                cv2.imshow(
                    'BirdView + Sliding Window',
                    cv2.resize(debug_img, self.image_size)
                )
                cv2.waitKey(1)
        except Exception as e:
            self._warn(f'차선 디버그 창 표시 실패: {e}')

        if lane_length <= 0 or path is None:
            curve_loss_context = self.is_curve_loss_context()
            lost_mode = 'CURVE_LOST' if curve_loss_context else 'STRAIGHT_LOST'
            guard_active_before_decay = self.curve_speed_guard_frames > 0

            if curve_loss_context:
                search_speed = self.get_lost_lane_speed()
                hold_angle = self.get_curve_hold_angle()
            else:
                search_speed = max(self.last_speed, self.get_straight_lane_speed())
                hold_angle = 0.0
                self.pid_integral = 0.0
                self.pid_prev_error = 0.0
                self.was_curve = False
                self.last_curve_angle = 0.0
                self.last_path_curvature = 0.0
                self.last_path_curve_ahead = False
                self.last_path_s_curve_like = False

            search_speed = self.apply_school_zone_speed_limit(search_speed)
            search_speed = self.limit_speed_change(search_speed)
            search_speed = self.apply_school_zone_speed_limit(search_speed)
            search_speed = self.apply_external_speed_limit(search_speed)

            if self.curve_speed_guard_frames > 0:
                self.curve_speed_guard_frames -= 1

            self.last_angle = hold_angle
            self.last_speed = search_speed
            self.last_debug = {
                'mode': lost_mode,
                'path_mode': path_mode,
                'left_count': len(lane_vehicle_coords['left']),
                'yellow_count': len(lane_vehicle_coords['center']),
                'right_count': len(lane_vehicle_coords['right']),
                'detected_lane_length': detected_lane_length,
                'lane_length': lane_length,
                'path_points': 0,
                'curvature': 0.0,
                'filtered_curvature': self.filtered_curvature,
                'lost_frames': self.lost_frames,
                'reused_path': bool(reused_path),
                'lane_detected': bool(self.lane_detected),
                'lane_stable': False,
                'school_zone': bool(self.school_zone_active),
                'school_counter': self.school_zone_counter,
                'curve_hold_angle': self.last_curve_angle,
                'preview_heading': 0.0,
                'far_preview_heading': 0.0,
                'curve_ahead': bool(curve_loss_context),
                's_curve_like': bool(self.last_path_s_curve_like),
                'fast_straight': bool(self._fast_straight_active),
                'straight_confidence_frames': self.straight_confidence_frames,
                'curve_speed_guard': bool(guard_active_before_decay),
                'curve_speed_guard_frames': self.curve_speed_guard_frames,
                'raw_angle': hold_angle,
                'angle': hold_angle,
                'speed': search_speed,
            }
            return float(hold_angle), float(search_speed)

        curvature = self.estimate_curvature(path)
        if curvature > self.filtered_curvature:
            self.filtered_curvature = (
                self.curvature_filter_rise_old_weight * self.filtered_curvature +
                self.curvature_filter_rise_new_weight * curvature
            )
        else:
            self.filtered_curvature = (
                self.curvature_filter_fall_old_weight * self.filtered_curvature +
                self.curvature_filter_fall_new_weight * curvature
            )

        raw_steer, steering_debug = self.get_steering_pid(
            path,
            return_debug=True
        )

        if reused_path:
            raw_steer = self.keep_curve_angle_if_lost(
                raw_steer,
                reused_path
            )

        angle_smooth_window = (
            self.curve_smooth_window
            if (
                self.filtered_curvature > self.straight_curvature_th or
                steering_debug.get('curve_ahead', False)
            )
            else self.straight_smooth_window
        )
        smooth = self.smooth_angle(
            raw_steer,
            window=angle_smooth_window
        )

        limited = self.limit_angle_change(
            smooth,
            self.last_angle
        )

        angle = self.clamp_angle(limited)

        self.last_angle = angle
        self.update_curve_memory(
            angle,
            self.filtered_curvature,
            steering_debug.get('curve_ahead', False)
        )

        fast_straight = self.update_straight_confidence(
            self.filtered_curvature,
            angle,
            steering_debug
        )

        speed = self.get_speed_by_curvature(
            self.filtered_curvature,
            angle,
            fast_straight
        )

        if reused_path and self.is_curve_loss_context():
            speed = min(speed, self.fix_speed * self.reused_curve_speed_ratio)
        elif (
            detected_lane_length <= self.short_lane_length_max and
            detected_lane_length > self.short_lane_length_min
        ):
            # 곡선에서는 차선 점수가 짧게 잡히는 경우가 많아 기존 0.75배 보정이
            # 속도를 한 번 더 눌렀다. 실제 차선을 새로 잡은 상태이고 보호구역 cap이
            # 아닐 때만 보정을 완화한다. 직선/차선유실/보호구역은 기존처럼 보수적 유지.
            curve_short_lane_ok = (
                self.is_curve_loss_context()
                and not reused_path
                and detected_lane_length >= self.path_fit_min_points
                and not self.school_zone_active
            )
            min_factor = (
                self.curve_short_lane_min_factor
                if curve_short_lane_ok
                else self.short_lane_min_factor
            )
            speed *= max(
                detected_lane_length * self.short_lane_length_scale,
                min_factor
            )

        (
            speed,
            curve_guard_near_heading,
            curve_guard_preview_heading,
            curve_guard_far_preview_heading,
            curve_guard_sharp,
            curve_guard_like,
            s_curve_like,
            curve_guard_active
        ) = self.apply_curve_speed_guard(
            speed,
            path,
            self.filtered_curvature,
            angle
        )
        self.last_path_curvature = self.filtered_curvature
        self.last_path_curve_ahead = bool(
            steering_debug.get('curve_ahead', False) or curve_guard_like
        )
        self.last_path_s_curve_like = bool(s_curve_like)
        self.update_curve_memory(
            angle,
            self.filtered_curvature,
            steering_debug.get('curve_ahead', False),
            s_curve_like
        )

        speed = self.apply_school_zone_speed_limit(speed)
        speed = self.limit_speed_change(speed)
        speed = self.apply_school_zone_speed_limit(speed)
        speed = self.apply_external_speed_limit(speed)

        self.last_speed = speed
        self.last_debug = {
            'mode': path_mode,
            'path_mode': path_mode,
            'left_count': len(lane_vehicle_coords['left']),
            'yellow_count': len(lane_vehicle_coords['center']),
            'right_count': len(lane_vehicle_coords['right']),
            'detected_lane_length': detected_lane_length,
            'lane_length': lane_length,
            'path_points': len(path),
            'curvature': curvature,
            'filtered_curvature': self.filtered_curvature,
            'lost_frames': self.lost_frames,
            'reused_path': bool(reused_path),
            'lane_detected': bool(self.lane_detected),
            'lane_stable': bool(self.lane_detected and not reused_path and lane_length >= self.path_fit_min_points),
            'school_zone': bool(self.school_zone_active),
            'school_counter': self.school_zone_counter,
            'curve_hold_angle': self.last_curve_angle,
            'preview_heading': steering_debug.get('preview_heading', 0.0),
            'far_preview_heading': steering_debug.get('far_preview_heading', 0.0),
            'curve_ahead': bool(steering_debug.get('curve_ahead', False)),
            's_curve_like': bool(s_curve_like),
            'fast_straight': bool(fast_straight),
            'straight_confidence_frames': self.straight_confidence_frames,
            'curve_speed_guard': bool(curve_guard_active),
            'curve_speed_guard_frames': self.curve_speed_guard_frames,
            'curve_guard_near_heading': curve_guard_near_heading,
            'curve_guard_preview_heading': curve_guard_preview_heading,
            'curve_guard_far_heading': curve_guard_far_preview_heading,
            'curve_guard_sharp': bool(curve_guard_sharp),
            'curve_guard_like': bool(curve_guard_like),
            'raw_angle': raw_steer,
            'angle': angle,
            'speed': speed,
        }
        return float(angle), float(speed)
