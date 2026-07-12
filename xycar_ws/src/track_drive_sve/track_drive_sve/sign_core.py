#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""신호등 색 인식 모듈 (3구/4구 공용).
이미지를 받아 'green' / 'red' / 'left' / 'none' 중 하나를 반환합니다.
원본: 팀원 TrackDriver(signColor) 알고리즘에서 ROS 부분만 제거하고 가져옴.
"""

import cv2
import numpy as np


class SignCore:
    def __init__(self):
        # 초록 / 빨강 HSV 임계값 (원본 그대로)
        self.green_lower = np.array([50, 150, 150])
        self.green_upper = np.array([85, 255, 255])
        self.red_lower = np.array([0, 150, 150])
        self.red_upper = np.array([10, 255, 255])
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.color_threshold = 500          # green/red 판단 픽셀 기준
        self.left_green_min_threshold = 100  # 좌회전(작은 초록) 기준
        # 디버그용으로 마지막 카운트를 저장
        self.last_green = 0
        self.last_red = 0
        self.last_result = "none"

    def detect(self, cv_image):
        """이미지에서 신호 색을 판별해 문자열로 반환."""
        if cv_image is None:
            return "none"

        height, width, _ = cv_image.shape
        # ROI: 화면 상단 50%, 가로 전체 (원본과 동일)
        roi = cv_image[0:int(height * 0.5), 0:width]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        mask_green = cv2.inRange(hsv, self.green_lower, self.green_upper)
        mask_red = cv2.inRange(hsv, self.red_lower, self.red_upper)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, self.kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, self.kernel)

        green_count = cv2.countNonZero(mask_green)
        red_count = cv2.countNonZero(mask_red)

        # 판단 우선순위 (원본 그대로)
        if green_count >= self.color_threshold:
            result = "green"
        elif self.left_green_min_threshold <= green_count < self.color_threshold:
            result = "left"
        elif red_count >= self.color_threshold:
            result = "red"
        else:
            result = "none"

        self.last_green = green_count
        self.last_red = red_count
        self.last_result = result
        return result