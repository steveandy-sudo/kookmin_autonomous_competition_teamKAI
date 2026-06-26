# CNN 조향 모델 입력에 맞게 카메라 이미지를 가공하는 전처리 모듈이다.
# 원본 BGR 이미지를 ROI crop, resize, RGB 변환, 정규화 과정을 거쳐 모델 입력 텐서로 만든다.
import math
from typing import Optional

import cv2
import numpy as np


# 카메라 원본 이미지를 모델이 학습한 입력 크기와 정규화 방식에 맞게 변환한다.
def preprocess_image_bgr(
    image_bgr: np.ndarray,
    roi_top_ratio: float = 0.45,
    width: int = 160,
    height: int = 90,
) -> np.ndarray:
    """BGR 카메라 이미지를 CNN 입력 형식인 RGB CHW float32 텐서로 변환한다.

    학습과 주행 추론에서 같은 crop/resize/normalize 과정을 사용해 모델 입력 분포가 달라지지 않게 한다.
    """
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError('empty image')

    # 모델 학습 때 사용한 하단 도로 영역만 남겨 조향에 덜 필요한 하늘/배경 영향을 줄인다.
    h, w = image_bgr.shape[:2]
    roi_top = int(np.clip(roi_top_ratio, 0.0, 0.9) * h)
    roi = image_bgr[roi_top:h, :]
    if roi.size == 0:
        roi = image_bgr

    # 입력 크기를 고정해야 TorchScript CNN의 첫 레이어 shape와 실제 추론 입력이 일치한다.
    resized = cv2.resize(roi, (int(width), int(height)), interpolation=cv2.INTER_AREA)
    # OpenCV는 BGR 순서이므로, 학습 코드에서 사용한 RGB 순서로 채널을 맞춘다.
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    # 0~255 픽셀을 0~1 실수로 바꾸고, PyTorch CNN이 기대하는 채널 우선(CHW) 배열로 변환한다.
    chw = np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))
    return chw


# 전처리 결과를 사람이 확인할 수 있는 이미지 파일로 저장할 때 사용한다.
def save_preprocessed_image_bgr(
    image_bgr: np.ndarray,
    path: str,
    roi_top_ratio: float = 0.45,
    width: int = 160,
    height: int = 90,
) -> None:
    """모델 입력에 사용되는 crop/resize 결과를 이미지 파일로 저장한다."""
    h = image_bgr.shape[0]
    roi_top = int(np.clip(roi_top_ratio, 0.0, 0.9) * h)
    roi = image_bgr[roi_top:h, :]
    if roi.size == 0:
        roi = image_bgr
    resized = cv2.resize(roi, (int(width), int(height)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(path, resized)


# 저장된 전처리 이미지를 다시 모델 입력 텐서 형태로 읽어오는 보조 함수이다.
def load_saved_image_as_tensor(path: str) -> np.ndarray:
    """저장된 전처리 이미지를 읽어 CNN 입력 텐서로 변환한다."""
    image_bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise FileNotFoundError(path)
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    return np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))


# 라바콘/노란색 표식처럼 주황색 계열이 많은지 빠르게 추정하는 보조 지표이다.
def orange_ratio_bgr(image_bgr: np.ndarray, roi_top_ratio: float = 0.45) -> float:
    """하단 ROI에서 주황색 라바콘 계열 픽셀 비율을 계산한다."""
    if image_bgr is None or image_bgr.size == 0:
        return 0.0
    h = image_bgr.shape[0]
    roi_top = int(np.clip(roi_top_ratio, 0.0, 0.9) * h)
    roi = image_bgr[roi_top:h, :]
    if roi.size == 0:
        return 0.0

    # HSV 공간은 조명 변화가 있어도 주황색 라바콘 픽셀을 BGR보다 안정적으로 분리할 수 있다.
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # Orange cone range. Adjust if simulator cone color is different.
    lower = np.array([3, 70, 70], dtype=np.uint8)
    upper = np.array([30, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    return float(np.count_nonzero(mask)) / float(mask.size)


# 라이다 전체 스캔 중 차량 전방 각도 범위만 잘라 장애물 판단에 사용한다.
def front_scan_vector(
    ranges,
    angle_min: float,
    angle_increment: float,
    front_degrees: float = 120.0,
    bins: int = 181,
    max_range: float = 8.0,
    min_range: float = 0.05,
    angle_offset: float = 0.0,
) -> np.ndarray:
    """라이다 전방 영역을 고정 길이 벡터로 변환한다.

    현재 제출 주행은 카메라 CNN을 중심으로 동작하지만, 라이다 기반 보조 입력을 확장할 때 사용할 수 있다.
    """
    if ranges is None or len(ranges) == 0:
        return np.ones((bins,), dtype=np.float32)

    half = math.radians(front_degrees * 0.5)
    target_angles = np.linspace(-half, half, bins).astype(np.float32)
    out = np.ones((bins,), dtype=np.float32) * float(max_range)

    # 라이다 각 빔을 전방 각도 구간의 고정 길이 배열에 누적해 모델/로직에서 다루기 쉽게 만든다.
    for i, r in enumerate(ranges):
        # inf/NaN 빔은 실제 장애물 거리가 아니므로 전방 거리 벡터에서 제외한다.
        if not math.isfinite(float(r)):
            continue
        angle = float(angle_min) + i * float(angle_increment) + float(angle_offset)
        if angle < -half or angle > half:
            continue
        idx = int(round((angle + half) / (2.0 * half) * (bins - 1)))
        if 0 <= idx < bins:
            out[idx] = min(out[idx], float(r))

    out = np.clip(out, min_range, max_range)
    out = out / float(max_range)
    return out.astype(np.float32)
