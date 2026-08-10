"""Pure helpers shared by object YOLO perception and mission gating."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class DetectionRecord:
    """One normalized 2-D object detection."""

    class_name: str
    class_id: int
    confidence: float
    xmin: int
    ymin: int
    xmax: int
    ymax: int

    @property
    def center_x(self) -> float:
        return 0.5 * (self.xmin + self.xmax)

    @property
    def center_y(self) -> float:
        return 0.5 * (self.ymin + self.ymax)

    @property
    def width(self) -> int:
        """Return the non-negative detection-box width in pixels."""
        return max(0, self.xmax - self.xmin)

    @property
    def height(self) -> int:
        """Return the non-negative detection-box height in pixels."""
        return max(0, self.ymax - self.ymin)

    @property
    def area(self) -> int:
        return self.width * self.height


@dataclass(frozen=True)
class HsvRoiEvidence:
    """Green-mask evidence measured only inside one detector-owned ROI."""

    valid: bool
    green_pixels: int
    pixel_ratio: float


def normalize_class_name(name: str) -> str:
    """Normalize model labels without changing their semantic meaning."""
    return "_".join(
        str(name).strip().lower().replace("-", " ").split()
    )


def parse_class_aliases(entries: Iterable[str]) -> dict[str, str]:
    """Parse ``source=target`` class aliases into normalized names."""
    aliases: dict[str, str] = {}
    for entry in entries:
        source, separator, target = str(entry).partition("=")
        source = normalize_class_name(source)
        target = normalize_class_name(target)
        if separator != "=" or not source or not target:
            raise ValueError(
                f"invalid class alias {entry!r}; expected source=target"
            )
        aliases[source] = target
    return aliases


def apply_class_aliases(
    records: Iterable[DetectionRecord],
    aliases: Mapping[str, str],
) -> list[DetectionRecord]:
    """Map model-specific labels to stable mission-level class names."""
    normalized_aliases = {
        normalize_class_name(source): normalize_class_name(target)
        for source, target in aliases.items()
    }
    mapped: list[DetectionRecord] = []
    for record in records:
        source = normalize_class_name(record.class_name)
        mapped.append(
            DetectionRecord(
                class_name=normalized_aliases.get(source, source),
                class_id=int(record.class_id),
                confidence=float(record.confidence),
                xmin=int(record.xmin),
                ymin=int(record.ymin),
                xmax=int(record.xmax),
                ymax=int(record.ymax),
            )
        )
    return mapped


def filter_detections(
    records: Iterable[DetectionRecord],
    thresholds: Mapping[str, float],
) -> list[DetectionRecord]:
    """Keep known classes whose confidence reaches their own threshold."""
    normalized_thresholds = {
        normalize_class_name(name): float(value)
        for name, value in thresholds.items()
    }
    accepted: list[DetectionRecord] = []
    for record in records:
        class_name = normalize_class_name(record.class_name)
        threshold = normalized_thresholds.get(class_name)
        if threshold is None or record.confidence < threshold:
            continue
        accepted.append(
            DetectionRecord(
                class_name=class_name,
                class_id=int(record.class_id),
                confidence=float(record.confidence),
                xmin=int(record.xmin),
                ymin=int(record.ymin),
                xmax=int(record.xmax),
                ymax=int(record.ymax),
            )
        )
    return accepted


def green_hsv_evidence_in_box(
    bgr_image: np.ndarray,
    box: Sequence[int],
    *,
    lower_hsv: Sequence[int] = (70, 100, 120),
    upper_hsv: Sequence[int] = (90, 255, 255),
) -> HsvRoiEvidence:
    """Measure green pixels strictly inside a YOLO traffic-light box.

    The detector box is expected to cover the complete traffic-light housing,
    so no geometric padding or image-wide HSV search is performed here.
    """
    if bgr_image.ndim != 3 or bgr_image.shape[2] != 3 or len(box) != 4:
        return HsvRoiEvidence(False, 0, 0.0)
    height, width = bgr_image.shape[:2]
    xmin, ymin, xmax, ymax = (int(value) for value in box)
    xmin = max(0, min(width, xmin))
    xmax = max(0, min(width, xmax))
    ymin = max(0, min(height, ymin))
    ymax = max(0, min(height, ymax))
    if xmax <= xmin or ymax <= ymin:
        return HsvRoiEvidence(False, 0, 0.0)

    lower = np.asarray(lower_hsv, dtype=np.uint8)
    upper = np.asarray(upper_hsv, dtype=np.uint8)
    if lower.shape != (3,) or upper.shape != (3,):
        return HsvRoiEvidence(False, 0, 0.0)

    roi = bgr_image[ymin:ymax, xmin:xmax]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    green_pixels = int(cv2.countNonZero(mask))
    return HsvRoiEvidence(
        True,
        green_pixels,
        green_pixels / float(max(1, roi.shape[0] * roi.shape[1])),
    )


def detection_side_counts(
    records: Iterable[DetectionRecord],
    *,
    image_width: int,
    class_name: str,
    center_deadband_ratio: float = 0.08,
) -> tuple[int, int, int]:
    """Return total, left and right counts using image-centre geometry."""
    if image_width <= 0:
        return 0, 0, 0
    target = normalize_class_name(class_name)
    center = 0.5 * float(image_width)
    deadband = max(0.0, float(center_deadband_ratio)) * image_width
    total = left = right = 0
    for record in records:
        if normalize_class_name(record.class_name) != target:
            continue
        total += 1
        if record.center_x < center - deadband:
            left += 1
        elif record.center_x > center + deadband:
            right += 1
    return total, left, right


def cone_modalities_match(
    *,
    yolo_total: int,
    yolo_left: int,
    yolo_right: int,
    lidar_total: int,
    lidar_left: int,
    lidar_right: int,
    minimum_yolo_count: int,
    minimum_lidar_count: int,
) -> bool:
    """Conservatively correlate image cones and LiDAR cone clusters.

    Exact camera–LiDAR projection is intentionally not used for mission entry.
    Both sensors must independently observe enough cones, and any clear
    left/right evidence from the image must exist on the same LiDAR side.
    """
    if yolo_total < minimum_yolo_count or lidar_total < minimum_lidar_count:
        return False
    if yolo_left > 0 and lidar_left <= 0:
        return False
    if yolo_right > 0 and lidar_right <= 0:
        return False
    return True


def restrict_yellow_mask_to_hints(
    mask: np.ndarray,
    hints: Iterable[DetectionRecord],
    *,
    image_width: int,
    class_name: str = "yellow_centerline",
    confidence: float = 0.45,
    horizontal_margin_px: int = 18,
    minimum_pixels: int = 3,
    minimum_retained_fraction: float = 0.25,
    minimum_vertical_span_fraction: float = 0.55,
) -> tuple[np.ndarray, bool]:
    """Use YOLO boxes only to reject implausible yellow mask components.

    The LR-ASPP mask remains authoritative. If a YOLO hint would remove too
    much evidence, the original mask is returned unchanged.
    """
    if mask.ndim != 2 or image_width <= 0:
        return mask, False
    target = normalize_class_name(class_name)
    selected_hints = [
        hint
        for hint in hints
        if normalize_class_name(hint.class_name) == target
        and hint.confidence >= confidence
    ]
    original_pixels = int(np.count_nonzero(mask))
    if not selected_hints or original_pixels < minimum_pixels:
        return mask, False

    component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        (mask > 0).astype(np.uint8),
        connectivity=8,
    )
    if component_count <= 2:
        return mask, False

    scale_x = mask.shape[1] / float(image_width)
    corridors: list[tuple[float, float]] = []
    for hint in selected_hints:
        xmin = hint.xmin * scale_x - horizontal_margin_px
        xmax = hint.xmax * scale_x + horizontal_margin_px
        corridors.append((xmin, xmax))

    kept = np.zeros_like(mask)
    for label in range(1, component_count):
        x = int(stats[label, cv2.CC_STAT_LEFT])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        centroid_x = float(centroids[label, 0])
        overlaps = any(
            centroid_x >= xmin
            and centroid_x <= xmax
            or x + width >= xmin
            and x <= xmax
            for xmin, xmax in corridors
        )
        if overlaps:
            kept[labels == label] = 255

    kept_pixels = int(np.count_nonzero(kept))
    if kept_pixels < minimum_pixels:
        return mask, False
    if kept_pixels / max(1, original_pixels) < minimum_retained_fraction:
        return mask, False

    original_rows = np.flatnonzero(np.any(mask > 0, axis=1))
    kept_rows = np.flatnonzero(np.any(kept > 0, axis=1))
    original_span = (
        int(original_rows[-1] - original_rows[0] + 1)
        if original_rows.size
        else 0
    )
    kept_span = (
        int(kept_rows[-1] - kept_rows[0] + 1)
        if kept_rows.size
        else 0
    )
    if (
        original_span > 0
        and kept_span / original_span < minimum_vertical_span_fraction
    ):
        return mask, False
    return kept, True
