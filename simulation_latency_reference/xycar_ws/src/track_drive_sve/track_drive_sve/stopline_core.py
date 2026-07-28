#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""정지선 검출 모듈 (카메라 하단 ROI에서 흰 가로선 검출).
전방 카메라 하단 영역에서 '한 행에 흰 픽셀이 가로로 넓게 퍼진' 행을 찾는다.
세로 차선(흰 실선)은 한 행에 픽셀이 적게 걸려서 걸러진다.
"""

import cv2
import numpy as np


class StopLineCore:
    def __init__(self):
        # ROI: 화면 하단 영역 (세로 비율). 카메라 시점에 맞춰 튜닝.
        self.roi_y_top = 0.60
        self.roi_y_bottom = 0.95
        # 흰색 = 채도 낮고(S<=60) 명도 높음(V>=200)
        self.white_sat_max = 60
        self.white_val_min = 200
        self.row_fill_ratio = 0.5
        self.min_contiguous_white_ratio = 0.60
        self.min_line_rows = 25
        self.last_line_rows = 0
        self.last_max_contiguous_ratio = 0.0
        self.last_detected = False

    def detect(self, cv_image):
        """정지선이 충분히 가까우면 True. 디버그용으로 line_rows도 저장."""
        if cv_image is None:
            self.last_line_rows = 0
            self.last_max_contiguous_ratio = 0.0
            self.last_detected = False
            return False

        h, w, _ = cv_image.shape
        y1 = int(h * self.roi_y_top)
        y2 = int(h * self.roi_y_bottom)
        roi = cv_image[y1:y2, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]
        white = (s <= self.white_sat_max) & (v >= self.white_val_min)  # 흰 픽셀 마스크

        # 체크무늬는 흰 픽셀 총량이 많아도 짧은 칸으로 끊겨 있다.
        # 실제 정지선처럼 한 덩어리로 길게 이어진 흰 영역이 있는 행만 센다.
        row_white = white.sum(axis=1).astype(np.float32)
        row_ratio = row_white / float(w)
        min_contiguous_width = int(round(w * self.min_contiguous_white_ratio))
        line_rows = 0
        max_contiguous_width = 0

        for row, fill_ratio in zip(white, row_ratio):
            if fill_ratio < self.row_fill_ratio:
                continue

            padded = np.pad(row.astype(np.int8), (1, 1), constant_values=0)
            edges = np.diff(padded)
            starts = np.flatnonzero(edges == 1)
            ends = np.flatnonzero(edges == -1)
            longest = int(np.max(ends - starts)) if starts.size else 0
            max_contiguous_width = max(max_contiguous_width, longest)

            if longest >= min_contiguous_width:
                line_rows += 1

        self.last_line_rows = line_rows
        self.last_max_contiguous_ratio = max_contiguous_width / float(w)
        self.last_detected = line_rows >= self.min_line_rows
        return self.last_detected
