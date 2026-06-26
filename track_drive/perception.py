"""카메라 기반 인지 스타터 코드."""

from __future__ import annotations

from typing import Any, Dict, Optional

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class PerceptionModule:
    """카메라 프레임에서 필요한 초기 인지 결과를 계산한다."""

    # 설명: 카메라 기반 단순 인지 모듈의 브리지와 내부 설정을 초기화한다.
    def __init__(self) -> None:
        # PerceptionModule 객체를 초기화하고 필요한 파라미터와 내부 상태를 준비한다.
        self.bridge = CvBridge()

    # 설명: ROS 이미지 메시지를 OpenCV BGR 이미지로 변환한다.
    def image_msg_to_bgr(self, image_msg: Optional[Image]) -> Optional[np.ndarray]:
        """ROS Image 메시지를 OpenCV BGR 이미지로 안전 변환한다."""
        # 카메라 인식의 이미지 msg to bgr 로직을 수행한다.
        if image_msg is None:
            return None

        try:
            return self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception:
            return None

    # 설명: 이미지에서 초록 신호등 후보가 보이는지 판정한다.
    def detect_green_traffic_light(self, bgr: Optional[np.ndarray]) -> bool:
        """HSV 임계값 기반 초록불 감지(스타터 버전)."""
        # 입력 데이터에서 detect 초록불 교통 신호등 조건을 감지한다.
        if bgr is None or bgr.size == 0:
            return False

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower = np.array([40, 60, 60], dtype=np.uint8)
        upper = np.array([90, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        green_ratio = float(np.count_nonzero(mask)) / float(mask.size)
        return green_ratio > 0.01

    # 설명: 이미지에서 차선 중심 위치를 추정한다.
    def detect_lane_center(self, bgr: Optional[np.ndarray]) -> Optional[float]:
        """하단 ROI에서 단순 차선 중심 x 좌표를 추정한다."""
        # 입력 데이터에서 detect 차선 center 조건을 감지한다.
        if bgr is None or bgr.size == 0:
            return None

        height, width = bgr.shape[:2]
        roi = bgr[int(height * 0.6):, :]
        if roi.size == 0:
            return None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        moments = cv2.moments(binary)

        if moments['m00'] == 0:
            return None

        cx = moments['m10'] / moments['m00']
        return float(cx)

    # 설명: 이미지에서 어린이보호구역 표식 여부를 판정한다.
    def detect_school_zone(self, bgr: Optional[np.ndarray]) -> bool:
        """스쿨존 텍스트/노면표시 감지 Placeholder."""
        # 입력 데이터에서 detect 어린이 보호구역 구역 조건을 감지한다.
        if bgr is None or bgr.size == 0:
            return False

        # TODO: OCR/특징 매칭 기반 스쿨존 감지로 교체
        return False

    # 설명: 이미지에서 좌회전 신호 여부를 판정한다.
    def detect_left_turn_signal(self, bgr: Optional[np.ndarray]) -> bool:
        """좌회전 신호/표지 감지 Placeholder."""
        # 입력 데이터에서 detect 왼쪽/좌회전 회전 signal 조건을 감지한다.
        if bgr is None or bgr.size == 0:
            return False

        # TODO: 좌회전 표지/신호등 규칙 기반 판단 로직 구현
        return False

    # 설명: 현재 이미지 프레임에서 신호등, 차선, 어린이보호구역, 좌회전 신호 결과를 묶어 반환한다.
    def run(self, image_msg: Optional[Image]) -> Dict[str, Any]:
        """현재 프레임으로 인지 결과를 계산한다."""
        # 카메라 인식 처리 파이프라인을 실행하고 결과를 반환한다.
        bgr = self.image_msg_to_bgr(image_msg)
        lane_center_x = self.detect_lane_center(bgr)

        return {
            'traffic_light_green': self.detect_green_traffic_light(bgr),
            'lane_center_x': lane_center_x,
            'school_zone_detected': self.detect_school_zone(bgr),
            'left_turn_signal': self.detect_left_turn_signal(bgr),
            'image_width': int(bgr.shape[1]) if bgr is not None else None,
        }
