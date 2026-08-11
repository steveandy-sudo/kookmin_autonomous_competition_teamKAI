#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""지름길(TURN_LEFT) 전용 주행 모듈.

phase 흐름:
  enter  : 진입 좌회전 하드코딩 (config.SHORTCUT_ENTER_*)
  cruise : 지름길 직진 = 차선주행 (원본 LaneDetectionDriver 알고리즘 그대로) + 노란 T자 검출
  exit   : 노란 T자 감지 시 탈출 좌회전 하드코딩 (원본 t_left_turn 값 그대로)
  done   : 탈출 완료 → FSM이 LANE으로 복귀

compute(image, now_sec) -> (angle, speed, done)
  done=True가 되면 지름길이 끝난 것이므로 호출 측(FSM)에서 LANE으로 전이한다.

주의: 차선주행/T자 검출 알고리즘은 팀원 검증 코드(LaneDetectionDriver)를 그대로 옮긴 것이며
      게인/임계값을 바꾸지 않았다. ROS 노드 껍데기와 publish만 제거하고 반환 방식으로 변경했다.
"""

import math
import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d

from . import config


class ShortcutCore:
    def __init__(self, logger=None):
        self.logger = logger

        # ===== phase 상태 =====
        self.phase = "idle"            # idle -> enter -> cruise -> exit -> done
        self.phase_start_sec = None

        # ===== 진입 좌회전 하드코딩 값 (config) =====
        self.enter_angle = config.SHORTCUT_ENTER_ANGLE
        self.enter_speed = config.SHORTCUT_ENTER_SPEED
        self.enter_duration = config.SHORTCUT_ENTER_SEC

        # ===== 아래는 원본 LaneDetectionDriver __init__ 그대로 =====
        self.fix_speed = 8.0
        self.lane_offset = 0.0
        self.lane_detected = False
        self.angle_buffer = []
        self.last_angle = 0.0
        self.last_curve_angle = 0.0
        self.last_speed = 0.0
        self.last_valid_path = None
        self.lost_frames = 0
        self.max_reuse_frames = 50
        self.curve_hold_frames = 10
        self.curve_hold_min_angle = 12.0
        self.prev_lane_base = {'left': None, 'center': None, 'right': None}
        self.yellow_center_switch_gate = 120
        self.path_smoothing_alpha = 0.58
        self.target_forward_px = 120.0
        self.curve_target_forward_px = 90.0
        self.curve_heading_threshold = 0.18
        self.speed_up_delta = 0.35
        self.speed_down_delta = 0.70
        self.filtered_curvature = 0.0
        self.was_curve = False
        self.school_zone_active = False
        self.school_zone_counter = 0
        self.school_zone_enter_frames = 3
        self.school_zone_counter_max = 24
        self.school_zone_speed_limit = float(config.LANE_SCHOOL_ZONE_SPEED_LIMIT)

        self.max_steer_angle = 100.0
        self.max_steer_delta = 20.0

        self.path_resample_points = 40
        self.path_fit_min_points = 5

        self.pid_kp = 0.75
        self.pid_ki = 0.0
        self.pid_kd = 0.22
        self.heading_kp = 32.0
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_integral_limit = 100.0

        # ===== Yellow T line detection parameters (원본 그대로) =====
        self.yellow_t_lower = (20, 80, 80)
        self.yellow_t_upper = (40, 255, 255)
        self.t_horizontal_kernel_width = 45
        self.t_vertical_kernel_height = 45
        self.t_min_yellow_area = 80
        self.t_min_horizontal_width = 70
        self.t_min_horizontal_height = 5
        self.t_min_vertical_height = 35
        self.t_min_vertical_width = 5
        self.t_center_tolerance_px = 120
        self.t_join_tolerance_px = 80
        self.t_detect_count = 0
        self.t_detect_threshold = 1

        # 탈출 좌회전 override 값 (원본 그대로)
        self.t_left_turn_angle = -100.0
        self.t_left_turn_speed = 10.0
        self.t_left_turn_duration = 2.7
        # cruise 시작 후 이 시간(초) 동안은 T자 검출 무시 (진입 직후 입구 노란무늬 오인 방지)
        self.t_ignore_after_enter_sec = 5.0
        self.t_left_turn_active = False
        self.t_left_turn_start_time = None

        self.debug_img = None  # 디버그 영상 보관(필요 시 외부에서 표시)

    # =====================================================================
    # 외부 인터페이스
    # =====================================================================
    def reset(self):
        """지름길 진입 직전에 호출. 모든 상태를 초기화한다."""
        self.phase = "idle"
        self.phase_start_sec = None
        self.angle_buffer = []
        self.last_angle = 0.0
        self.last_curve_angle = 0.0
        self.last_speed = 0.0
        self.last_valid_path = None
        self.lost_frames = 0
        self.prev_lane_base = {'left': None, 'center': None, 'right': None}
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.filtered_curvature = 0.0
        self.was_curve = False
        self.t_detect_count = 0
        self.t_left_turn_active = False
        self.t_left_turn_start_time = None

    def start_cruise(self, now_sec):
        """Start at the verified lane cruise without replaying timed entry.

        The semantic W1/Y1 controller owns the entry turn.  Once both selected
        boundaries are aligned with the vehicle forward axis, the integrated
        candidate uses this method to hand control to the already verified
        cruise/T-exit implementation.
        """
        self.reset()
        self.phase = "cruise"
        self.phase_start_sec = float(now_sec)
        self._log("[shortcut] semantic entry complete -> cruise")

    def compute(self, image, now_sec):
        """지름길 주행 명령을 반환. (angle, speed, done)"""
        if image is None:
            return 0.0, 0.0, False

        img = cv2.resize(image, (640, 480), interpolation=cv2.INTER_LINEAR)

        # 첫 호출이면 enter phase로 시작
        if self.phase in ("idle", "done"):
            self.phase = "enter"
            self.phase_start_sec = now_sec

        # ---------- phase: enter (진입 좌회전 하드코딩) ----------
        if self.phase == "enter":
            elapsed = now_sec - self.phase_start_sec
            if elapsed < self.enter_duration:
                self.last_angle = self.enter_angle
                self.last_speed = self.enter_speed
                self.last_curve_angle = self.enter_angle
                return self.enter_angle, self.enter_speed, False
            # 진입 끝 -> cruise로
            self.phase = "cruise"
            self.phase_start_sec = now_sec
            self.angle_buffer = []
            self.pid_integral = 0.0
            self.pid_prev_error = 0.0

        # ---------- phase: exit (T자 탈출 좌회전, 이미 시작됐다면) ----------
        if self.phase == "exit":
            angle, speed, done = self._run_exit_turn(now_sec)
            if done:
                self.phase = "done"
                return 0.0, 0.0, True
            return angle, speed, False

        # ---------- phase: cruise (지름길 직진 + T자 검출) ----------
        angle, speed, t_detected = self._run_cruise(img, now_sec)
        # 진입(enter) 직후에는 입구 근처 노란 무늬를 T자로 오인할 수 있으므로,
        # cruise 시작 후 t_ignore_after_enter_sec 동안은 T자 검출을 무시한다.
        cruise_elapsed = now_sec - self.phase_start_sec
        if t_detected and cruise_elapsed < self.t_ignore_after_enter_sec:
            t_detected = False
        if t_detected:
            # T자 감지 -> 탈출 좌회전 시작
            self._start_exit_turn(now_sec)
            self.phase = "exit"
            a, s, done = self._run_exit_turn(now_sec)
            return a, s, False
        return angle, speed, False

    def _log(self, msg):
        if self.logger is not None:
            self.logger(msg)

    # =====================================================================
    # exit (T자 탈출 좌회전) — 원본 t_left_turn 로직
    # =====================================================================
    def _start_exit_turn(self, now_sec):
        if self.t_left_turn_active:
            return
        self.t_left_turn_active = True
        self.t_left_turn_start_time = now_sec
        self.angle_buffer = []
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self._log(f"[shortcut] EXIT T-LEFT START angle={self.t_left_turn_angle} "
                  f"speed={self.t_left_turn_speed} dur={self.t_left_turn_duration}")

    def _run_exit_turn(self, now_sec):
        """(angle, speed, done) 반환. done=True면 탈출 완료."""
        if not self.t_left_turn_active:
            return 0.0, 0.0, True
        elapsed = now_sec - self.t_left_turn_start_time
        if elapsed <= self.t_left_turn_duration:
            self.last_angle = self.t_left_turn_angle
            self.last_speed = self.t_left_turn_speed
            self.last_curve_angle = self.t_left_turn_angle
            return self.t_left_turn_angle, self.t_left_turn_speed, False
        self.t_left_turn_active = False
        self.t_left_turn_start_time = None
        self._log("[shortcut] EXIT T-LEFT END -> done")
        return 0.0, 0.0, True

    # =====================================================================
    # cruise (지름길 직진 + T자 검출) — 원본 control_loop의 차선부분
    # =====================================================================
    def _run_cruise(self, image, now_sec):
        """(angle, speed, t_detected) 반환."""
        bird_view = self.warp_perspective(image)
        white_mask, yellow_mask = self.preprocess_split(bird_view)
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        debug_img = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

        # T자 검출
        raw_t_detected = self.detect_yellow_t_line(bird_view, debug_img)
        if raw_t_detected:
            self.t_detect_count += 1
        else:
            self.t_detect_count = 0
        stable_t_detected = self.t_detect_count >= self.t_detect_threshold

        self.debug_img = debug_img

        # 차선 주행 계산 (원본 그대로)
        lanes = self.classify_lanes_by_color_and_position(
            white_mask, yellow_mask, bird_view.shape[1])

        lane_vehicle_coords = {'left': [], 'center': [], 'right': []}
        path = None

        for key in ['left', 'center', 'right']:
            lane_candidate = lanes[key]
            if lane_candidate is None and self.prev_lane_base[key] is not None:
                lane_candidate = (
                    'yellow' if key == 'center' else 'white',
                    self.prev_lane_base[key])
            if lane_candidate:
                lane_type, x_base = lane_candidate
                mask = yellow_mask if lane_type == 'yellow' else white_mask
                color = {'left': (255, 0, 0), 'center': (0, 255, 255),
                         'right': (0, 0, 255)}[key]
                pts = self.sliding_window_center_lane(
                    mask, debug_img, base_x=x_base, color=color)
                vehicle_pts = self.convert_to_vehicle_coords(pts, bird_view.shape)
                lane_vehicle_coords[key] = vehicle_pts
                if pts:
                    self.prev_lane_base[key] = pts[0][0]

        center_line_pts, path_mode = self.infer_road_center(lane_vehicle_coords)
        detected_lane_length = len(center_line_pts) if center_line_pts else 0

        if center_line_pts and detected_lane_length > 4:
            path = self.generate_resampled_path(center_line_pts)
            if path and len(path) > 4:
                xs, ys = zip(*path)
                xs = gaussian_filter1d(xs, sigma=0.8)
                ys = gaussian_filter1d(ys, sigma=0.8)
                path = list(zip(xs, ys))

        path, reused_path = self.get_tracking_path(path)
        lane_length = len(path) if path else 0

        # 차선 소실 처리 (원본 그대로)
        if lane_length <= 0 or path is None:
            search_speed = self.apply_school_zone_speed_limit(self.get_lost_lane_speed())
            search_speed = self.limit_speed_change(search_speed)
            search_speed = self.apply_school_zone_speed_limit(search_speed)
            hold_angle = self.get_curve_hold_angle()
            self.last_angle = hold_angle
            self.last_speed = search_speed
            return hold_angle, search_speed, stable_t_detected

        curvature = self.estimate_curvature(path)
        self.filtered_curvature = 0.72 * self.filtered_curvature + 0.28 * curvature

        raw_steer = self.get_steering_pid(path)
        if reused_path:
            raw_steer = self.keep_curve_angle_if_lost(raw_steer, reused_path)

        angle_smooth_window = 3 if self.filtered_curvature > 0.012 else 5
        smooth = self.smooth_angle(raw_steer, window=angle_smooth_window)
        limited = self.limit_angle_change(smooth, self.last_angle)
        angle = self.clamp_angle(limited)

        self.last_angle = angle
        self.update_curve_memory(angle, self.filtered_curvature)

        speed = self.get_speed_by_curvature(self.filtered_curvature, angle)
        if reused_path and self.is_curve_loss_context():
            speed = min(speed, self.fix_speed * 0.65)
        elif detected_lane_length <= 8 and detected_lane_length > 1:
            speed *= max(detected_lane_length * 0.1, 0.75)
        speed = self.apply_school_zone_speed_limit(speed)
        speed = self.limit_speed_change(speed)
        speed = self.apply_school_zone_speed_limit(speed)

        self.last_speed = speed
        return angle, speed, stable_t_detected

    # =====================================================================
    # 아래는 원본 LaneDetectionDriver의 알고리즘 메서드들 (그대로)
    # =====================================================================
    def warp_perspective(self, img):
        h, w = img.shape[:2]
        src = np.float32([[130, 280], [500, 280], [640, 385], [0, 385]])
        dst = np.float32([[w * 0.1, 0], [w * 0.9, 0], [w * 0.9, h], [w * 0.1, h]])
        M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(img, M, (w, h))

    def preprocess_split(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, (0, 0, 220), (180, 30, 255))
        yellow_mask = cv2.inRange(hsv, (22, 130, 130), (35, 255, 255))
        kernel_open = np.ones((3, 3), dtype=np.uint8)
        kernel_close = np.ones((9, 5), dtype=np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
        return white_mask, yellow_mask

    def preprocess_yellow_t(self, bird_view):
        hsv = cv2.cvtColor(bird_view, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, self.yellow_t_lower, self.yellow_t_upper)
        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel_close, iterations=1)
        return yellow_mask

    def extract_t_line_components(self, yellow_mask):
        horizontal_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.t_horizontal_kernel_width, 3))
        vertical_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (3, self.t_vertical_kernel_height))
        horizontal_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, horizontal_kernel, iterations=1)
        vertical_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, vertical_kernel, iterations=1)
        return horizontal_mask, vertical_mask

    def get_contour_boxes(self, mask, min_area):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            boxes.append((x, y, w, h, area))
        return boxes

    def detect_yellow_t_line(self, bird_view, debug_img):
        yellow_mask = self.preprocess_yellow_t(bird_view)
        horizontal_mask, vertical_mask = self.extract_t_line_components(yellow_mask)
        horizontal_boxes = self.get_contour_boxes(horizontal_mask, self.t_min_yellow_area)
        vertical_boxes = self.get_contour_boxes(vertical_mask, self.t_min_yellow_area)
        valid_horizontal = []
        valid_vertical = []
        for x, y, w, h, area in horizontal_boxes:
            if w >= self.t_min_horizontal_width and h >= self.t_min_horizontal_height:
                valid_horizontal.append((x, y, w, h, area))
                cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 255), 2)
        for x, y, w, h, area in vertical_boxes:
            if h >= self.t_min_vertical_height and w >= self.t_min_vertical_width:
                valid_vertical.append((x, y, w, h, area))
                cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 165, 255), 2)
        if not valid_horizontal or not valid_vertical:
            return False
        for hx, hy, hw, hh, h_area in valid_horizontal:
            h_cx = hx + hw / 2.0
            h_cy = hy + hh / 2.0
            for vx, vy, vw, vh, v_area in valid_vertical:
                v_cx = vx + vw / 2.0
                v_top = vy
                v_bottom = vy + vh
                center_close = abs(h_cx - v_cx) <= self.t_center_tolerance_px
                vertical_under_horizontal = v_bottom > h_cy
                join_close = abs(v_top - h_cy) <= self.t_join_tolerance_px
                vertical_x_inside_horizontal = (
                    hx - self.t_center_tolerance_px <= v_cx <= hx + hw + self.t_center_tolerance_px)
                is_t_shape = (center_close and vertical_under_horizontal and
                              join_close and vertical_x_inside_horizontal)
                if is_t_shape:
                    cv2.rectangle(debug_img, (hx, hy), (hx + hw, hy + hh), (0, 0, 255), 3)
                    cv2.rectangle(debug_img, (vx, vy), (vx + vw, vy + vh), (0, 0, 255), 3)
                    return True
        return False

    def find_lane_peaks(self, mask):
        h = mask.shape[0]
        histogram = np.sum(mask[h * 2 // 3:, :], axis=0)
        if np.max(histogram) == 0:
            return []
        threshold = 0.38 * np.max(histogram)
        peaks = np.where(histogram > threshold)[0]
        clusters = []
        cluster = []
        for i in range(len(peaks)):
            if i == 0 or peaks[i] - peaks[i - 1] <= 50:
                cluster.append(peaks[i])
            else:
                clusters.append(cluster)
                cluster = [peaks[i]]
        if cluster:
            clusters.append(cluster)
        return [int(np.mean(c)) for c in clusters]

    def classify_lanes_by_color_and_position(self, white_mask, yellow_mask, width):
        lanes = {'left': None, 'center': None, 'right': None}
        white_peaks = self.find_lane_peaks(white_mask)
        yellow_peaks = self.find_lane_peaks(yellow_mask)
        self.update_school_zone_state(yellow_peaks, width)
        if yellow_peaks:
            center_x = self.select_yellow_center_peak(yellow_peaks, width)
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

    def select_yellow_center_peak(self, yellow_peaks, width):
        if not yellow_peaks:
            return None
        image_center = width // 2
        prev_center = self.prev_lane_base.get('center')
        if prev_center is None:
            return int(min(yellow_peaks, key=lambda x: abs(x - image_center)))
        gated_peaks = [x for x in yellow_peaks
                       if abs(x - prev_center) <= self.yellow_center_switch_gate]
        if not gated_peaks:
            return None
        return int(min(gated_peaks, key=lambda x: (
            0.80 * abs(x - prev_center) + 0.20 * abs(x - image_center))))

    def update_school_zone_state(self, yellow_peaks, width):
        has_left_yellow_edge = any(x < width * 0.32 for x in yellow_peaks)
        has_right_yellow_edge = any(x > width * 0.68 for x in yellow_peaks)
        wide_yellow_span = (len(yellow_peaks) >= 2 and
                            max(yellow_peaks) - min(yellow_peaks) > width * 0.45)
        school_zone_seen = has_left_yellow_edge and has_right_yellow_edge and wide_yellow_span
        if school_zone_seen:
            self.school_zone_counter = min(self.school_zone_counter + 1, self.school_zone_counter_max)
        else:
            self.school_zone_counter = max(self.school_zone_counter - 1, 0)
        self.school_zone_active = self.school_zone_counter >= self.school_zone_enter_frames

    def apply_school_zone_speed_limit(self, speed):
        if self.school_zone_active:
            return min(speed, self.school_zone_speed_limit)
        return speed

    def convert_to_vehicle_coords(self, pts, shape):
        h, w = shape[:2]
        return [(h - py, -(px - w // 2)) for (px, py) in pts]

    def sliding_window_center_lane(self, binary_img, out_img, base_x=None,
                                   nwindows=15, margin=50, minpix=30, color=(0, 255, 0)):
        h, w = binary_img.shape
        histogram = np.sum(binary_img[h * 2 // 3:, :], axis=0)
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
            good_inds = ((nonzeroy >= y_low) & (nonzeroy < y_high) &
                         (nonzerox >= x_low) & (nonzerox < x_high)).nonzero()[0]
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
        if center:
            return center, 'YELLOW'
        if left and right:
            min_len = min(len(left), len(right))
            return [((left[i][0] + right[i][0]) / 2.0,
                     (left[i][1] + right[i][1]) / 2.0) for i in range(min_len)], 'WHITE_CENTER'
        if left:
            return [(x, y + self.lane_offset) for (x, y) in left], 'LEFT_ONLY'
        if right:
            return [(x, y - self.lane_offset) for (x, y) in right], 'RIGHT_ONLY'
        return [], 'NONE'

    def generate_resampled_path(self, center_line_pts):
        if not center_line_pts or len(center_line_pts) < self.path_fit_min_points:
            return center_line_pts
        pts = sorted(center_line_pts, key=lambda p: p[0])
        filtered = []
        last_x = None
        for x, y in pts:
            if last_x is None or abs(x - last_x) > 1.0:
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
        return [(float(x), float(y)) for x, y in zip(x_new, y_new)]

    def stabilize_path(self, detected_path):
        if not detected_path or len(detected_path) < self.path_fit_min_points:
            return None
        path = sorted([(float(x), float(y)) for x, y in detected_path if x >= 0.0],
                      key=lambda p: p[0])
        if len(path) < self.path_fit_min_points:
            return None
        if self.last_valid_path is not None and len(self.last_valid_path) >= 2:
            cur_x = np.array([p[0] for p in path], dtype=np.float32)
            cur_y = np.array([p[1] for p in path], dtype=np.float32)
            prev_x = np.array([p[0] for p in self.last_valid_path], dtype=np.float32)
            prev_y = np.array([p[1] for p in self.last_valid_path], dtype=np.float32)
            prev_interp_y = np.interp(cur_x, prev_x, prev_y, left=prev_y[0], right=prev_y[-1])
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
        if self.last_valid_path is not None and self.lost_frames <= self.max_reuse_frames:
            return self.last_valid_path, True
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        return None, False

    def get_steering_pid(self, path):
        if path is None or len(path) < 2:
            return self.last_angle
        target_idx, target_x, target_y = self.find_target_point(path, self.target_forward_px)
        heading_error = self.path_heading_at(path, target_idx)
        if abs(heading_error) > self.curve_heading_threshold:
            target_idx, target_x, target_y = self.find_target_point(path, self.curve_target_forward_px)
            heading_error = self.path_heading_at(path, target_idx)
        error = target_y
        dt = 0.05
        self.pid_integral += error * dt
        self.pid_integral = float(np.clip(self.pid_integral,
                                          -self.pid_integral_limit, self.pid_integral_limit))
        derivative = (error - self.pid_prev_error) / dt
        self.pid_prev_error = error
        control = (self.pid_kp * error + self.pid_ki * self.pid_integral +
                   self.pid_kd * derivative + self.heading_kp * heading_error)
        return -control

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
        i0 = max(idx - 3, 0)
        i1 = min(idx + 3, len(path) - 1)
        if i0 == i1:
            return 0.0
        dx = path[i1][0] - path[i0][0]
        dy = path[i1][1] - path[i0][1]
        return math.atan2(dy, max(dx, 1.0e-3))

    def clamp_angle(self, angle):
        return float(np.clip(angle, -self.max_steer_angle, self.max_steer_angle))

    def limit_angle_change(self, angle, last_angle):
        delta = angle - last_angle
        if abs(delta) > self.max_steer_delta:
            return float(last_angle + np.clip(delta, -self.max_steer_delta, self.max_steer_delta))
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
            return 0.80 * hold_angle + 0.20 * raw_steer
        return raw_steer

    def update_curve_memory(self, angle, curvature):
        self.was_curve = (abs(angle) >= self.curve_hold_min_angle or curvature > 0.012)
        if abs(angle) >= self.curve_hold_min_angle:
            self.last_curve_angle = angle

    def is_curve_loss_context(self):
        return (self.was_curve or
                abs(self.last_curve_angle) >= self.curve_hold_min_angle or
                abs(self.last_angle) >= self.curve_hold_min_angle)

    def get_lost_lane_speed(self):
        if self.is_curve_loss_context():
            return self.fix_speed * 0.45
        return self.get_speed_by_curvature(0.0, 0.0)

    def smooth_angle(self, new_angle, window=5):
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
        curvature = np.abs(d2x[valid] * dy[:-1][valid] -
                           d2y[valid] * dx[:-1][valid]) / denominator[valid]
        if curvature.size == 0:
            return 0.0
        return float(np.percentile(curvature, 65))

    def get_speed_by_curvature(self, curvature, angle):
        if curvature <= 0.012 and abs(angle) <= 15:
            speed_ratio = 4.0
        else:
            speed_ratio = 1.0
        if curvature > 0.030:
            speed_ratio = 0.6
        elif curvature > 0.020:
            speed_ratio = 0.75
        elif curvature > 0.012:
            speed_ratio = 0.9
        elif abs(angle) > 25:
            speed_ratio = 1.0
        return self.fix_speed * speed_ratio

    def limit_speed_change(self, speed):
        if self.last_speed <= 0.0:
            return float(speed)
        delta = speed - self.last_speed
        if delta > self.speed_up_delta:
            return float(self.last_speed + self.speed_up_delta)
        if delta < -self.speed_down_delta:
            return float(self.last_speed - self.speed_down_delta)
        return float(speed)
