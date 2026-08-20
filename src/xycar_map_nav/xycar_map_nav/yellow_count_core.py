"""Shared W1-BEV yellow-dash counting primitives."""

from __future__ import annotations

import cv2
import numpy as np

from lane_seg_control.canonical_adapter_node import (
    build_bev_geometry,
    warp_semantic_masks_only,
)


class W1BevYellowProjector:
    """Apply the exact mask-only BEV warp used before W1/W2 selection."""

    def __init__(self) -> None:
        self.geometry = None
        self.geometry_input_size: tuple[int, int] | None = None

    def project(self, raw_yellow: np.ndarray) -> np.ndarray:
        height, width = raw_yellow.shape
        size = (int(width), int(height))
        if self.geometry is None or self.geometry_input_size != size:
            self.geometry = build_bev_geometry(
                width,
                height,
                source_ratios=(
                    0.442578,
                    0.480781,
                    0.688281,
                    0.480781,
                    0.919141,
                    0.614189,
                    0.190625,
                    0.614189,
                ),
                destination_ratios=(
                    0.205714,
                    0.794286,
                    0.0,
                    0.666666667,
                ),
                bev_width=640,
                bev_height=660,
            )
            self.geometry_input_size = size
        empty_white = np.zeros_like(raw_yellow)
        _, bev_yellow, _ = warp_semantic_masks_only(
            empty_white,
            raw_yellow,
            self.geometry,
            valid_lateral_margin_px=0,
            valid_erode_px=0,
            clip_to_source_polygon=False,
        )
        return bev_yellow


def count_band_bounds(
    height: int,
    *,
    line_ratio: float,
    half_height_px: int,
) -> tuple[int, int, int]:
    ratio = min(0.95, max(0.05, float(line_ratio)))
    line_y = int(round((max(1, int(height)) - 1) * ratio))
    half_height = max(1, int(half_height_px))
    top = max(0, line_y - half_height)
    bottom = min(max(0, int(height) - 1), line_y + half_height)
    return top, line_y, bottom


def yellow_mask_occupies_count_band(
    yellow_mask: np.ndarray,
    *,
    line_ratio: float,
    half_height_px: int,
    component_minimum_area_px: int,
) -> bool:
    """Return whether a sufficiently large yellow component touches the band."""
    mask = (yellow_mask > 0).astype(np.uint8)
    band_top, _, band_bottom = count_band_bounds(
        mask.shape[0],
        line_ratio=line_ratio,
        half_height_px=half_height_px,
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    minimum_area = max(1, int(component_minimum_area_px))
    for index in range(1, count):
        _, y, _, component_height, area = stats[index]
        if int(area) < minimum_area:
            continue
        component_bottom = int(y + component_height - 1)
        if int(y) > band_bottom or component_bottom < band_top:
            continue
        if np.any(labels[band_top : band_bottom + 1] == index):
            return True
    return False


class YellowBandPassCounter:
    """Count debounced count-band enter/leave pulses."""

    def __init__(
        self,
        *,
        target: int = 1,
        visible_frames: int = 2,
        absent_frames: int = 1,
    ) -> None:
        self.target = max(1, int(target))
        self.visible_required = max(1, int(visible_frames))
        self.absent_required = max(1, int(absent_frames))
        self.reset()

    def reset(self) -> None:
        self.visible_frames = 0
        self.absent_frames = 0
        self.episode_active = False
        self.passed = 0
        self.triggered = False

    def update(self, occupied: bool) -> tuple[bool, bool, bool]:
        """Return (entered, passed, triggered_now) for one inference frame."""
        if self.triggered:
            return False, False, False
        entered = False
        passed = False
        if not self.episode_active:
            self.visible_frames = self.visible_frames + 1 if occupied else 0
            self.absent_frames = 0
            if self.visible_frames >= self.visible_required:
                self.episode_active = True
                entered = True
            return entered, False, False

        self.absent_frames = 0 if occupied else self.absent_frames + 1
        if self.absent_frames < self.absent_required:
            return False, False, False

        self.passed += 1
        self.episode_active = False
        self.visible_frames = 0
        self.absent_frames = 0
        passed = True
        triggered_now = self.passed >= self.target
        self.triggered = triggered_now
        return False, passed, triggered_now
