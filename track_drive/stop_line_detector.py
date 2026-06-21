#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class StopLineBEVResult:
    detected: bool = False
    row_ratio: float = 0.0
    bottom_row_ratio: float = 0.0
    width_ratio: float = 0.0
    solid_run_ratio: float = 0.0
    fill_ratio: float = 0.0
    distance_m: Optional[float] = None
    bbox: Optional[Tuple[int, int, int, int]] = None
    roi_debug_image: Optional[np.ndarray] = None
    bev_image: Optional[np.ndarray] = None
    mask_image: Optional[np.ndarray] = None
    debug_image: Optional[np.ndarray] = None


def declare_stop_line_bev_parameters(node):
    node.declare_parameter('camera_topic', '/usb_cam/image_raw/front')
    node.declare_parameter('stop_line_bev_width', 320)
    node.declare_parameter('stop_line_bev_height', 240)
    node.declare_parameter('stop_line_bev_src_top_ratio', 0.30)
    node.declare_parameter('stop_line_bev_src_bottom_ratio', 0.98)
    node.declare_parameter('stop_line_bev_src_top_half_width_ratio', 0.075)
    node.declare_parameter('stop_line_bev_src_bottom_half_width_ratio', 0.475)
    node.declare_parameter('stop_line_bev_center_shift_ratio', 0.0)
    node.declare_parameter('stop_line_bev_front_top_ratio', 0.08)
    node.declare_parameter('stop_line_bev_front_bottom_ratio', 1.00)
    node.declare_parameter('stop_line_bev_min_width_ratio', 0.38)
    node.declare_parameter('stop_line_bev_min_aspect_ratio', 5.0)
    node.declare_parameter('stop_line_bev_min_fill_ratio', 0.22)
    node.declare_parameter('stop_line_bev_min_row_run', 3)
    node.declare_parameter('stop_line_bev_min_solid_run_ratio', 0.70)
    node.declare_parameter('stop_line_bev_solid_col_min_fill_ratio', 0.55)
    node.declare_parameter('stop_line_detect_min_row_ratio', 0.08)
    node.declare_parameter('stop_line_detect_max_distance_m', 7.00)
    node.declare_parameter('stop_line_white_value_min', 200)
    node.declare_parameter('stop_line_white_sat_max', 80)
    node.declare_parameter('stop_line_bev_close_width_ratio', 0.025)
    node.declare_parameter('stop_line_bev_close_height', 3)
    node.declare_parameter('stop_line_bev_open_kernel', 3)
    node.declare_parameter('stop_line_distance_bottom_ratio', 1.0)
    node.declare_parameter('stop_line_distance_scale_m', 7.0)
    node.declare_parameter('stop_line_stop_row_ratio', 0.70)
    node.declare_parameter('stop_line_stop_bottom_row_ratio', 0.75)
    node.declare_parameter('stop_line_stop_distance_m', 3.0)
    node.declare_parameter('stop_line_confirm_frames', 2)
    node.declare_parameter('stop_on_light_requires_stop_line', True)


class BEVStopLineDetector:
    """Detect the white horizontal stop line in a bird's-eye front band."""

    def __init__(self, node):
        self.node = node

    def detect(self, image: Optional[np.ndarray]) -> StopLineBEVResult:
        if image is None or image.size == 0:
            return StopLineBEVResult()

        height, width = image.shape[:2]
        bev_width = max(int(self._param('stop_line_bev_width', 320)), 16)
        bev_height = max(int(self._param('stop_line_bev_height', 240)), 16)
        src = self._source_points(width, height)
        dst = np.float32([
            [0.0, 0.0],
            [bev_width - 1.0, 0.0],
            [bev_width - 1.0, bev_height - 1.0],
            [0.0, bev_height - 1.0],
        ])

        roi_debug = image.copy()
        cv2.polylines(roi_debug, [src.astype(np.int32)], True, (0, 255, 0), 3)

        try:
            matrix = cv2.getPerspectiveTransform(src, dst)
            bev = cv2.warpPerspective(image, matrix, (bev_width, bev_height))
        except cv2.error:
            return StopLineBEVResult(roi_debug_image=roi_debug)

        mask = self._white_mask(bev)
        debug = bev.copy()
        front_top = float(np.clip(self._param('stop_line_bev_front_top_ratio', 0.25), 0.0, 0.98))
        front_bottom = float(np.clip(
            self._param('stop_line_bev_front_bottom_ratio', 1.00),
            front_top + 0.01,
            1.0,
        ))
        y0 = int(front_top * bev_height)
        y1 = max(y0 + 1, int(front_bottom * bev_height))
        cv2.rectangle(debug, (0, y0), (bev_width - 1, y1 - 1), (255, 0, 0), 2)

        best = self._best_horizontal_line(mask[y0:y1, :], bev_width)
        if best is None:
            cv2.putText(debug, 'STOP LINE: none', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 180, 255), 2)
            return StopLineBEVResult(
                roi_debug_image=roi_debug,
                bev_image=bev,
                mask_image=mask,
                debug_image=debug,
            )

        x, y, box_w, box_h, width_ratio, solid_run_ratio, fill_ratio = best
        row_ratio = float((y0 + y + 0.5 * box_h) / float(bev_height))
        bottom_row_ratio = float((y0 + y + box_h) / float(bev_height))
        distance_m = self._estimate_distance(row_ratio)
        if not self._close_enough_to_detect(row_ratio, distance_m):
            cv2.putText(
                debug,
                f'STOP LINE: wait row={row_ratio:.2f} d={distance_m:.2f}m',
                (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 180, 255),
                2,
            )
            return StopLineBEVResult(
                row_ratio=row_ratio,
                bottom_row_ratio=bottom_row_ratio,
                width_ratio=width_ratio,
                solid_run_ratio=solid_run_ratio,
                fill_ratio=fill_ratio,
                distance_m=distance_m,
                bbox=(x, y0 + y, box_w, box_h),
                roi_debug_image=roi_debug,
                bev_image=bev,
                mask_image=mask,
                debug_image=debug,
            )

        cv2.rectangle(debug, (x, y0 + y), (x + box_w, y0 + y + box_h), (0, 0, 255), 2)
        cv2.putText(
            debug,
            f'STOP LINE row={row_ratio:.2f} w={width_ratio:.2f} solid={solid_run_ratio:.2f} d={distance_m:.2f}m',
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 0, 255),
            2,
        )
        return StopLineBEVResult(
            detected=True,
            row_ratio=row_ratio,
            bottom_row_ratio=bottom_row_ratio,
            width_ratio=width_ratio,
            solid_run_ratio=solid_run_ratio,
            fill_ratio=fill_ratio,
            distance_m=distance_m,
            bbox=(x, y0 + y, box_w, box_h),
            roi_debug_image=roi_debug,
            bev_image=bev,
            mask_image=mask,
            debug_image=debug,
        )

    def _source_points(self, width: int, height: int) -> np.ndarray:
        top_ratio = float(np.clip(self._param('stop_line_bev_src_top_ratio', 0.50), 0.05, 0.95))
        bottom_ratio = float(np.clip(
            self._param('stop_line_bev_src_bottom_ratio', 0.80),
            top_ratio + 0.03,
            1.0,
        ))
        top_half = float(np.clip(self._param('stop_line_bev_src_top_half_width_ratio', 0.075), 0.02, 0.50))
        bottom_half = float(np.clip(
            self._param('stop_line_bev_src_bottom_half_width_ratio', 0.475),
            top_half,
            0.70,
        ))
        center_shift = float(np.clip(self._param('stop_line_bev_center_shift_ratio', 0.0), -0.25, 0.25))
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

    def _white_mask(self, bev: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        value_min = int(np.clip(self._param('stop_line_white_value_min', 200), 0, 255))
        sat_max = int(np.clip(self._param('stop_line_white_sat_max', 80), 0, 255))
        mask = cv2.inRange(
            hsv,
            np.array([0, 0, value_min], dtype=np.uint8),
            np.array([180, sat_max, 255], dtype=np.uint8),
        )
        close_width = max(3, int(float(mask.shape[1]) * float(self._param('stop_line_bev_close_width_ratio', 0.025))))
        if close_width % 2 == 0:
            close_width += 1
        close_height = max(1, int(self._param('stop_line_bev_close_height', 3)))
        open_size = max(1, int(self._param('stop_line_bev_open_kernel', 3)))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((close_height, close_width), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_size, open_size), np.uint8))
        return mask

    def _best_horizontal_line(self, front_mask: np.ndarray, bev_width: int):
        if front_mask.size == 0:
            return None

        min_width_ratio = float(np.clip(self._param('stop_line_bev_min_width_ratio', 0.40), 0.05, 1.0))
        min_aspect_ratio = max(float(self._param('stop_line_bev_min_aspect_ratio', 5.0)), 1.0)
        min_fill_ratio = float(np.clip(self._param('stop_line_bev_min_fill_ratio', 0.22), 0.02, 1.0))
        min_row_run = max(int(self._param('stop_line_bev_min_row_run', 3)), 1)
        min_row_pixels = min_width_ratio * float(bev_width)

        row_counts = np.count_nonzero(front_mask, axis=1)
        candidate_rows = row_counts >= min_row_pixels
        if not np.any(candidate_rows):
            return None

        best = None
        start = None
        for idx, valid in enumerate(candidate_rows):
            if valid and start is None:
                start = idx
            if (not valid or idx == len(candidate_rows) - 1) and start is not None:
                end = idx + 1 if valid and idx == len(candidate_rows) - 1 else idx
                if end - start >= min_row_run:
                    candidate = self._score_row_band(front_mask, start, end, bev_width, min_aspect_ratio, min_fill_ratio)
                    if candidate is not None and (best is None or candidate[0] > best[0]):
                        best = candidate
                start = None

        if best is None:
            return None
        _, x, y, box_w, box_h, width_ratio, solid_run_ratio, fill_ratio = best
        return x, y, box_w, box_h, width_ratio, solid_run_ratio, fill_ratio

    def _score_row_band(
        self,
        mask: np.ndarray,
        y0: int,
        y1: int,
        bev_width: int,
        min_aspect_ratio: float,
        min_fill_ratio: float,
    ):
        band = mask[y0:y1, :]
        nonzero_y, nonzero_x = band.nonzero()
        if len(nonzero_x) == 0:
            return None

        x0 = int(np.min(nonzero_x))
        x1 = int(np.max(nonzero_x)) + 1
        box_w = max(x1 - x0, 1)
        box_h = max(y1 - y0, 1)
        aspect_ratio = box_w / float(box_h)
        if aspect_ratio < min_aspect_ratio:
            return None

        cropped = band[:, x0:x1]
        fill_ratio = float(np.count_nonzero(cropped)) / float(max(box_w * box_h, 1))
        if fill_ratio < min_fill_ratio:
            return None

        solid_run_ratio = self._longest_solid_column_run_ratio(band, bev_width)
        min_solid_run_ratio = float(np.clip(
            self._param('stop_line_bev_min_solid_run_ratio', 0.70),
            0.05,
            1.0,
        ))
        if solid_run_ratio < min_solid_run_ratio:
            return None

        width_ratio = box_w / float(max(bev_width, 1))
        center_bias = 1.0 - min(abs(((x0 + x1) * 0.5 / max(bev_width, 1)) - 0.5) * 0.6, 0.3)
        score = solid_run_ratio * aspect_ratio * fill_ratio * center_bias
        return score, x0, y0, box_w, box_h, width_ratio, solid_run_ratio, fill_ratio

    def _longest_solid_column_run_ratio(self, band: np.ndarray, bev_width: int) -> float:
        min_fill_ratio = float(np.clip(
            self._param('stop_line_bev_solid_col_min_fill_ratio', 0.55),
            0.05,
            1.0,
        ))
        min_col_pixels = max(1, int(np.ceil(float(max(band.shape[0], 1)) * min_fill_ratio)))
        column_valid = np.count_nonzero(band, axis=0) >= min_col_pixels
        longest = 0
        current = 0
        for valid in column_valid:
            if valid:
                current += 1
                longest = max(longest, current)
            else:
                current = 0
        return float(longest) / float(max(bev_width, 1))

    def _close_enough_to_detect(self, row_ratio: float, distance_m: float) -> bool:
        min_row_ratio = float(np.clip(
            self._param('stop_line_detect_min_row_ratio', 0.25),
            0.0,
            1.0,
        ))
        max_distance_m = float(self._param('stop_line_detect_max_distance_m', 7.00))
        if row_ratio < min_row_ratio:
            return False
        if max_distance_m > 0.0 and distance_m > max_distance_m:
            return False
        return True

    def _estimate_distance(self, row_ratio: float) -> float:
        bottom_ratio = float(np.clip(self._param('stop_line_distance_bottom_ratio', 1.0), 0.0, 1.0))
        scale_m = max(float(self._param('stop_line_distance_scale_m', 7.0)), 0.01)
        return max(0.0, (bottom_ratio - float(row_ratio)) * scale_m)

    def _param(self, name: str, default):
        try:
            return self.node.get_parameter(name).value
        except Exception:
            return default


class StopLineDetector:
    """Stop-line perception boundary for TrackDriverNode."""

    def __init__(self, node):
        self.node = node
        self.bev_detector = BEVStopLineDetector(node)
        self.last_result = StopLineBEVResult()

    def update(self, image: Optional[np.ndarray]):
        self._reset_current_state()
        if image is None or not bool(self.node.get_parameter('stop_on_light_requires_stop_line').value):
            self.node.stop_line_confirm_count = 0
            self.last_result = StopLineBEVResult()
            return

        result = self.bev_detector.detect(image)
        self.last_result = result
        self.node.stop_line_bev_detected = bool(result.detected)
        self.node.stop_line_bev_row_ratio = float(result.row_ratio)
        self.node.stop_line_bev_width_ratio = float(result.width_ratio)
        self.node.stop_line_best_row_ratio = float(result.width_ratio)

        if not result.detected:
            self.node.stop_line_confirm_count = 0
            return

        self.node.stop_line_detected = True
        self.node.stop_line_row_ratio = float(result.row_ratio)
        self.node.stop_line_bottom_row_ratio = float(result.bottom_row_ratio)
        self.node.stop_line_distance_m = result.distance_m
        self.node.stop_line_last_row_ratio = self.node.stop_line_row_ratio
        self.node.stop_line_last_bottom_row_ratio = self.node.stop_line_bottom_row_ratio
        self.node.stop_line_last_distance_m = result.distance_m
        self.node.stop_line_last_seen_sec = time.monotonic()
        self.node.stop_line_confirm_count += 1

    def ready_for_light_stop(self) -> bool:
        if not bool(self.node.get_parameter('stop_on_light_requires_stop_line').value):
            return True

        required_frames = max(int(self.node.get_parameter('stop_line_confirm_frames').value), 1)
        row_ratio = self.node.stop_line_row_ratio
        bottom_row_ratio = self.node.stop_line_bottom_row_ratio
        distance_m = self.node.stop_line_distance_m
        if self.node.stop_line_confirm_count < required_frames or not self.node.stop_line_detected:
            memory_sec = max(float(self.node.get_parameter('stop_line_memory_sec').value), 0.0)
            recently_seen = (
                self.node.stop_line_last_seen_sec is not None
                and time.monotonic() - self.node.stop_line_last_seen_sec <= memory_sec
            )
            if not recently_seen:
                return False
            row_ratio = self.node.stop_line_last_row_ratio
            bottom_row_ratio = self.node.stop_line_last_bottom_row_ratio
            distance_m = self.node.stop_line_last_distance_m
        if distance_m is None:
            return False

        stop_distance = float(self.node.get_parameter('stop_line_stop_distance_m').value)
        if stop_distance > 0.0:
            return distance_m <= stop_distance

        stop_row_ratio = float(np.clip(
            self.node.get_parameter('stop_line_stop_row_ratio').value, 0.0, 1.0))
        stop_bottom_row_ratio = float(np.clip(
            self.node.get_parameter('stop_line_stop_bottom_row_ratio').value, 0.0, 1.0))
        return (
            row_ratio >= stop_row_ratio
            and bottom_row_ratio >= stop_bottom_row_ratio
        )

    def log_text(self) -> str:
        return self.node._stop_line_log_text()

    def _reset_current_state(self):
        self.node.stop_line_detected = False
        self.node.stop_line_row_ratio = 0.0
        self.node.stop_line_bottom_row_ratio = 0.0
        self.node.stop_line_distance_m = None
        self.node.stop_line_best_row_ratio = 0.0
        self.node.stop_line_bev_detected = False
        self.node.stop_line_bev_row_ratio = 0.0
        self.node.stop_line_bev_width_ratio = 0.0
