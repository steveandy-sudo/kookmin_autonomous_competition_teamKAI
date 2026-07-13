from __future__ import annotations

import cv2
import numpy as np


def _remove_small_components(mask: np.ndarray, min_area_px: int) -> np.ndarray:
    binary = (mask > 0).astype(np.uint8)
    if min_area_px <= 1:
        return binary * 255
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    cleaned = np.zeros_like(binary)
    for label in range(1, count):
        if int(stats[label, cv2.CC_STAT_AREA]) >= min_area_px:
            cleaned[labels == label] = 1
    return cleaned * 255


def _metric_crop(
    image: np.ndarray,
    *,
    lateral_m_per_px: float,
    forward_m_per_px: float,
    lateral_range_m: float,
    forward_range_m: float,
) -> np.ndarray:
    if lateral_m_per_px <= 0.0 or forward_m_per_px <= 0.0:
        raise ValueError("BEV metric scales must be positive")
    if lateral_range_m <= 0.0 or forward_range_m <= 0.0:
        raise ValueError("canonical metric ranges must be positive")

    source_height, source_width = image.shape[:2]
    crop_width = max(1, int(round(lateral_range_m / lateral_m_per_px)))
    crop_height = max(1, int(round(forward_range_m / forward_m_per_px)))
    center_x = source_width // 2
    left = center_x - crop_width // 2
    right = left + crop_width
    top = source_height - crop_height
    bottom = source_height

    source_left = max(0, left)
    source_right = min(source_width, right)
    source_top = max(0, top)
    source_bottom = min(source_height, bottom)
    cropped = image[source_top:source_bottom, source_left:source_right]

    pad_left = max(0, -left)
    pad_right = max(0, right - source_width)
    pad_top = max(0, -top)
    pad_bottom = max(0, bottom - source_height)
    if any((pad_left, pad_right, pad_top, pad_bottom)):
        border_value = 0 if image.ndim == 2 else [0] * image.shape[2]
        cropped = cv2.copyMakeBorder(
            cropped,
            pad_top,
            pad_bottom,
            pad_left,
            pad_right,
            cv2.BORDER_CONSTANT,
            value=border_value,
        )
    return cropped


def _fixed_width_mask(mask: np.ndarray, line_width_px: int) -> np.ndarray:
    work = (mask > 0).astype(np.uint8) * 255
    if not np.any(work):
        return work

    skeleton = np.zeros_like(work)
    cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    while np.any(work):
        eroded = cv2.erode(work, cross)
        opened = cv2.dilate(eroded, cross)
        skeleton = cv2.bitwise_or(skeleton, cv2.subtract(work, opened))
        work = eroded

    width = max(1, int(line_width_px))
    if width == 1:
        return skeleton
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (width, width))
    return cv2.dilate(skeleton, kernel)


def make_canonical_road_image(
    bev_bgr: np.ndarray,
    *,
    valid_mask: np.ndarray | None = None,
    lateral_m_per_px: float,
    forward_m_per_px: float,
    lateral_range_m: float = 1.4,
    forward_range_m: float = 1.2,
    output_width: int = 256,
    output_height: int = 144,
    background_gray: int = 36,
    line_width_px: int = 5,
    white_s_max: int = 120,
    white_v_min: int = 145,
    white_v_floor: int = 70,
    white_relative_delta: float = 9.0,
    yellow_h_min: int = 15,
    yellow_h_max: int = 42,
    yellow_s_min: int = 55,
    yellow_v_min: int = 90,
    min_component_area_px: int = 8,
    bottom_ignore_m: float = 0.08,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build a fixed metric, fixed-color road representation from a BEV image."""
    if bev_bgr is None or bev_bgr.size == 0:
        raise ValueError("BEV image is empty")
    if output_width <= 0 or output_height <= 0:
        raise ValueError("canonical output dimensions must be positive")

    metric_bev = _metric_crop(
        bev_bgr,
        lateral_m_per_px=lateral_m_per_px,
        forward_m_per_px=forward_m_per_px,
        lateral_range_m=lateral_range_m,
        forward_range_m=forward_range_m,
    )
    metric_valid = None
    if valid_mask is not None:
        if valid_mask.shape[:2] != bev_bgr.shape[:2]:
            raise ValueError("valid mask dimensions must match the BEV image")
        metric_valid = _metric_crop(
            valid_mask,
            lateral_m_per_px=lateral_m_per_px,
            forward_m_per_px=forward_m_per_px,
            lateral_range_m=lateral_range_m,
            forward_range_m=forward_range_m,
        )
    hsv = cv2.cvtColor(metric_bev, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(value)
    local_background = cv2.GaussianBlur(clahe, (0, 0), sigmaX=5.0, sigmaY=5.0)
    relative_brightness = clahe.astype(np.float32) - local_background.astype(np.float32)
    low_saturation = saturation <= int(white_s_max)
    absolute_white = value >= int(white_v_min)
    relative_white = (
        (value >= int(white_v_floor))
        & (relative_brightness >= float(white_relative_delta))
    )
    white_mask = (low_saturation & (absolute_white | relative_white)).astype(np.uint8) * 255

    yellow_mask = cv2.inRange(
        hsv,
        np.array([yellow_h_min, yellow_s_min, yellow_v_min], dtype=np.uint8),
        np.array([yellow_h_max, 255, 255], dtype=np.uint8),
    )
    if metric_valid is not None:
        valid_binary = (metric_valid > 0).astype(np.uint8) * 255
        white_mask = cv2.bitwise_and(white_mask, valid_binary)
        yellow_mask = cv2.bitwise_and(yellow_mask, valid_binary)
    ignore_rows = max(0, int(round(bottom_ignore_m / forward_m_per_px)))
    if ignore_rows > 0:
        white_mask[-ignore_rows:, :] = 0
        yellow_mask[-ignore_rows:, :] = 0

    close_kernel = np.ones((3, 3), dtype=np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, close_kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, close_kernel)
    white_mask = _remove_small_components(white_mask, min_component_area_px)
    yellow_mask = _remove_small_components(yellow_mask, min_component_area_px)

    output_size = (int(output_width), int(output_height))
    white_mask = cv2.resize(white_mask, output_size, interpolation=cv2.INTER_NEAREST)
    yellow_mask = cv2.resize(yellow_mask, output_size, interpolation=cv2.INTER_NEAREST)
    output_valid = None
    if metric_valid is not None:
        output_valid = cv2.resize(
            (metric_valid > 0).astype(np.uint8) * 255,
            output_size,
            interpolation=cv2.INTER_NEAREST,
        )
    white_mask = _fixed_width_mask(white_mask, line_width_px)
    yellow_mask = _fixed_width_mask(yellow_mask, line_width_px)
    if output_valid is not None:
        white_mask = cv2.bitwise_and(white_mask, output_valid)
        yellow_mask = cv2.bitwise_and(yellow_mask, output_valid)
    white_mask[yellow_mask > 0] = 0

    gray = int(np.clip(background_gray, 0, 255))
    canonical = np.full((output_height, output_width, 3), gray, dtype=np.uint8)
    canonical[white_mask > 0] = (255, 255, 255)
    canonical[yellow_mask > 0] = (0, 220, 255)
    return canonical, white_mask, yellow_mask
