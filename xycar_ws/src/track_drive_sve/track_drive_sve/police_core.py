#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""경찰차(좌회전 통로 차단) 인식 모듈.
LiDAR 왼쪽 섹터(정면-왼쪽~정측면)에 가까운 점이 충분히 많으면
좌회전 통로가 막혔다고 판단한다. 원본: 팀원 LidarVisualizer(left_turn_obstacle) 로직.
"""

from .lidar_utils import sector_ranges


class PoliceCore:
    def __init__(self):
        # 왼쪽 좌회전 통로 섹터 (0도=정면, 90도=왼쪽)
        self.left_start_deg = 45.0
        self.left_end_deg = 90.0
        self.max_dist = 6.0          # 이 거리(m) 안에 있어야 통로 차단으로 봄
        self.point_threshold = 15    # 이만큼 점이 있으면 경찰차 있음
        self.last_count = 0
        self.last_min_dist = -1.0
        self.last_detected = False

    def detect(self, scan_msg) -> bool:
        """왼쪽 통로에 장애물(경찰차)이 있으면 True."""
        if scan_msg is None:
            self.last_count = 0
            self.last_detected = False
            return False
        ranges = sector_ranges(scan_msg, self.left_start_deg, self.left_end_deg)
        close = [r for r in ranges if 0.1 < r <= self.max_dist]
        self.last_count = len(close)
        self.last_min_dist = min(close) if close else -1.0
        self.last_detected = self.last_count >= self.point_threshold
        return self.last_detected