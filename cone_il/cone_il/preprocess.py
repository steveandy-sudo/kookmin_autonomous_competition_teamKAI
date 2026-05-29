import math
from typing import Optional

import cv2
import numpy as np


def preprocess_image_bgr(
    image_bgr: np.ndarray,
    roi_top_ratio: float = 0.45,
    width: int = 160,
    height: int = 90,
) -> np.ndarray:
    """Return RGB float32 image tensor in CHW format, range [0, 1].

    The same preprocessing must be used during data collection/training/inference.
    It crops away the upper part of the image and keeps the road/cone region.
    """
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError('empty image')

    h, w = image_bgr.shape[:2]
    roi_top = int(np.clip(roi_top_ratio, 0.0, 0.9) * h)
    roi = image_bgr[roi_top:h, :]
    if roi.size == 0:
        roi = image_bgr

    resized = cv2.resize(roi, (int(width), int(height)), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    chw = np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))
    return chw


def save_preprocessed_image_bgr(
    image_bgr: np.ndarray,
    path: str,
    roi_top_ratio: float = 0.45,
    width: int = 160,
    height: int = 90,
) -> None:
    """Save the exact cropped/resized image used by the model as JPG.

    This makes training/inference consistency easy because the training script can
    read the saved file directly without guessing the original camera resolution.
    """
    h = image_bgr.shape[0]
    roi_top = int(np.clip(roi_top_ratio, 0.0, 0.9) * h)
    roi = image_bgr[roi_top:h, :]
    if roi.size == 0:
        roi = image_bgr
    resized = cv2.resize(roi, (int(width), int(height)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(path, resized)


def load_saved_image_as_tensor(path: str) -> np.ndarray:
    """Read preprocessed JPG and return RGB CHW float32 tensor."""
    image_bgr = cv2.imread(path, cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise FileNotFoundError(path)
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    return np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))


def orange_ratio_bgr(image_bgr: np.ndarray, roi_top_ratio: float = 0.45) -> float:
    """Approximate orange/labacon pixel ratio in the lower image ROI."""
    if image_bgr is None or image_bgr.size == 0:
        return 0.0
    h = image_bgr.shape[0]
    roi_top = int(np.clip(roi_top_ratio, 0.0, 0.9) * h)
    roi = image_bgr[roi_top:h, :]
    if roi.size == 0:
        return 0.0

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # Orange cone range. Adjust if simulator cone color is different.
    lower = np.array([3, 70, 70], dtype=np.uint8)
    upper = np.array([30, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    return float(np.count_nonzero(mask)) / float(mask.size)


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
    """Convert LaserScan to fixed front-view vector normalized to [0, 1].

    This is saved for future multi-modal training. The basic CNN below uses only
    images, but keeping scan vectors is useful for later upgrades.
    """
    if ranges is None or len(ranges) == 0:
        return np.ones((bins,), dtype=np.float32)

    half = math.radians(front_degrees * 0.5)
    target_angles = np.linspace(-half, half, bins).astype(np.float32)
    out = np.ones((bins,), dtype=np.float32) * float(max_range)

    for i, r in enumerate(ranges):
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
