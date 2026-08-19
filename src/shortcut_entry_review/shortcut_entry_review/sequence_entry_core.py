"""Sequence-aware W1/Y1 selection for the left_4 shortcut entry.

The hand annotations are supervision for *which branch to attend to*.  Their
absolute bag timestamps and pixel coordinates are deliberately not replayed at
runtime.  Candidate lines are described in the vehicle-fixed BEV frame and are
selected by topology, heading relative to the vehicle forward axis, and
frame-to-frame continuity.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math

import cv2
import numpy as np


class EntrySequencePhase(IntEnum):
    LEFT4_ARMED = 0
    W1_LOCKED = 1
    Y1_LOCKED = 2
    PAIR_TRACK = 3
    CRUISE_HANDOFF = 4


@dataclass(frozen=True)
class SequenceEntryConfig:
    minimum_vertical_span_ratio: float = 0.045
    minimum_segment_length_ratio: float = 0.035
    maximum_fit_rmse_ratio: float = 0.055
    maximum_abs_slope: float = 2.5
    hough_threshold: int = 10
    hough_max_gap_ratio: float = 0.035
    cluster_angle_tolerance_rad: float = math.radians(17.0)
    cluster_distance_ratio: float = 0.055
    cluster_y_gap_ratio: float = 0.18
    duplicate_distance_ratio: float = 0.045
    white_component_minimum_area_px: int = 20
    white_component_minimum_height_px: int = 10
    yellow_component_minimum_area_px: int = 50
    yellow_component_minimum_height_px: int = 15
    yellow_component_minimum_span_ratio: float = 0.020
    w2_acquisition_required_frames: int = 2
    w2_acquisition_minimum_span_ratio: float = 0.12
    w2_acquisition_min_mean_x_ratio: float = 0.05
    w2_acquisition_max_mean_x_ratio: float = 0.60
    w1_acquisition_required_frames: int = 2
    w1_acquisition_window_frames: int = 4
    w1_fast_lock_enabled: bool = True
    w1_fast_lock_minimum_slope: float = 0.35
    w1_fast_lock_minimum_span_ratio: float = 0.075
    w1_fast_lock_minimum_distance_ratio: float = 0.080
    w1_acquisition_minimum_max_y_ratio: float = 0.72
    w1_acquisition_minimum_distance_ratio: float = 0.065
    y1_acquisition_required_frames: int = 2
    acquisition_slope_minimum: float = 0.15
    w1_tracking_slope_minimum: float = 0.02
    w1_acquisition_max_mean_x_ratio: float = 0.40
    w1_branch_minimum_separation_ratio: float = 0.045
    w1_branch_expected_separation_ratio: float = 0.11
    w1_branch_maximum_separation_ratio: float = 0.24
    temporal_max_mean_x_jump_ratio: float = 0.12
    temporal_max_line_distance_ratio: float = 0.16
    tracking_white_max_mean_x_ratio: float = 0.42
    tracking_yellow_min_mean_x_ratio: float = 0.25
    tracking_yellow_max_mean_x_ratio: float = 0.70
    tracking_minimum_slope: float = -1.0
    tracking_maximum_slope: float = 2.0
    tracking_white_minimum_span_ratio: float = 0.030
    tracking_yellow_minimum_span_ratio: float = 0.020
    topology_continuity_weight: float = 0.30
    topology_slope_continuity_weight: float = 0.02
    minimum_pair_separation_ratio: float = 0.17
    expected_pair_separation_ratio: float = 0.31
    maximum_pair_separation_ratio: float = 0.46
    missing_hold_frames: int = 2
    actual_y1_blend_per_frame: float = 0.25
    w1_path_weight: float = 0.60
    alignment_max_abs_slope: float = 0.22
    alignment_required_frames: int = 3
    path_sample_count: int = 32
    maximum_fit_extrapolation_ratio: float = 0.16

    def __post_init__(self) -> None:
        if self.w2_acquisition_required_frames < 1:
            raise ValueError("w2 acquisition frames must be positive")
        if self.w1_acquisition_required_frames < 1:
            raise ValueError("w1 acquisition frames must be positive")
        if (
            self.w1_acquisition_window_frames
            < self.w1_acquisition_required_frames
        ):
            raise ValueError(
                "W1 acquisition window must contain the required hits"
            )
        if self.y1_acquisition_required_frames < 1:
            raise ValueError("y1 acquisition frames must be positive")
        if not 0.0 <= self.w1_acquisition_minimum_max_y_ratio <= 1.0:
            raise ValueError("W1 acquisition near-y ratio must be in [0, 1]")
        if not (
            self.duplicate_distance_ratio
            < self.w1_acquisition_minimum_distance_ratio
            <= self.w1_fast_lock_minimum_distance_ratio
        ):
            raise ValueError(
                "W1 acquisition distance must be above duplicate distance "
                "and no greater than the fast-lock distance"
            )
        if not (
            0.0
            <= self.w1_tracking_slope_minimum
            <= self.acquisition_slope_minimum
        ):
            raise ValueError(
                "W1 tracking slope minimum must stay left-positive and not "
                "exceed the acquisition minimum"
            )
        if self.path_sample_count < 3:
            raise ValueError("path_sample_count must be at least three")
        if self.yellow_component_minimum_area_px < 1:
            raise ValueError("yellow component minimum area must be positive")
        if self.white_component_minimum_area_px < 1:
            raise ValueError("white component minimum area must be positive")
        if self.white_component_minimum_height_px < 2:
            raise ValueError(
                "white component minimum height must be at least two"
            )
        if self.yellow_component_minimum_height_px < 2:
            raise ValueError(
                "yellow component minimum height must be at least two"
            )
        if not 0.0 < self.minimum_pair_separation_ratio:
            raise ValueError("minimum pair separation must be positive")
        if not (
            self.minimum_pair_separation_ratio
            < self.expected_pair_separation_ratio
            < self.maximum_pair_separation_ratio
        ):
            raise ValueError("expected pair separation must be inside limits")
        if not 0.5 <= self.w1_path_weight < 1.0:
            raise ValueError("W1 path weight must be in [0.5, 1.0)")
        if not (
            0.0
            < self.w1_branch_minimum_separation_ratio
            < self.w1_branch_expected_separation_ratio
            < self.w1_branch_maximum_separation_ratio
        ):
            raise ValueError("W1/W2 branch separation limits are invalid")


@dataclass(frozen=True)
class LineHypothesis:
    color: str
    coefficients: tuple[float, float]
    mean_x_ratio: float
    mean_y_ratio: float
    near_x_ratio: float
    far_x_ratio: float
    minimum_y_ratio: float
    maximum_y_ratio: float
    direction_dx_dy: float
    heading_from_vehicle_rad: float
    vertical_span_ratio: float
    fit_rmse_ratio: float
    support_length_ratio: float

    def x_ratio_at(self, y_ratio: float) -> float:
        slope, intercept = self.coefficients
        return float(slope * float(y_ratio) + intercept)


@dataclass(frozen=True)
class SequenceEntryResult:
    phase: EntrySequencePhase
    ready: bool
    path_valid: bool
    cruise_handoff: bool
    reason: str
    white_candidates: tuple[LineHypothesis, ...]
    yellow_candidates: tuple[LineHypothesis, ...]
    w1: LineHypothesis | None
    w2: LineHypothesis | None
    y1: LineHypothesis | None
    path_pixels: tuple[tuple[float, float], ...]
    used_synthetic_y1: bool
    pair_separation_ratio: float


@dataclass(frozen=True)
class SpatialSteeringGate:
    branch_distance_m: float
    trigger_distance_m: float
    ready: bool
    blend: float


@dataclass(frozen=True)
class BranchPointEstimate:
    """Selected W2 geometry used only by the spatial steering gate."""

    distance_m: float
    w2: LineHypothesis | None
    intersection_x_ratio: float = math.nan
    intersection_y_ratio: float = math.nan


@dataclass(frozen=True)
class _Segment:
    x1: float
    y1: float
    x2: float
    y2: float
    length: float
    slope: float

    @property
    def mean_x(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def mean_y(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def minimum_y(self) -> float:
        return min(self.y1, self.y2)

    @property
    def maximum_y(self) -> float:
        return max(self.y1, self.y2)

    def x_at(self, y: float) -> float:
        return self.mean_x + self.slope * (float(y) - self.mean_y)


def normalize_mask(mask: np.ndarray) -> np.ndarray:
    if mask is None or mask.ndim != 2 or mask.size == 0:
        raise ValueError("lane mask must be a non-empty mono image")
    return np.where(mask > 0, 255, 0).astype(np.uint8)


def _line_angle(slope: float) -> float:
    return math.atan(float(slope))


def _segment_y_gap(first: _Segment, second: _Segment) -> float:
    if first.maximum_y < second.minimum_y:
        return second.minimum_y - first.maximum_y
    if second.maximum_y < first.minimum_y:
        return first.minimum_y - second.maximum_y
    return 0.0


def _segments_compatible(
    first: _Segment,
    second: _Segment,
    *,
    width: int,
    height: int,
    config: SequenceEntryConfig,
) -> bool:
    if abs(_line_angle(first.slope) - _line_angle(second.slope)) > float(
        config.cluster_angle_tolerance_rad
    ):
        return False
    if _segment_y_gap(first, second) > height * float(
        config.cluster_y_gap_ratio
    ):
        return False
    overlap_low = max(first.minimum_y, second.minimum_y)
    overlap_high = min(first.maximum_y, second.maximum_y)
    if overlap_low <= overlap_high:
        reference_y = 0.5 * (overlap_low + overlap_high)
    else:
        reference_y = 0.5 * (first.mean_y + second.mean_y)
    return abs(first.x_at(reference_y) - second.x_at(reference_y)) <= (
        width * float(config.cluster_distance_ratio)
    )


def _raw_segments(
    mask: np.ndarray,
    config: SequenceEntryConfig,
) -> list[_Segment]:
    height, width = mask.shape
    # Canny follows both sides of a thick semantic stripe and remains available
    # on the stock OpenCV build used on the Xycar PC (no ximgproc dependency).
    edges = cv2.Canny(mask, 40, 120)
    minimum_length = max(
        12, int(round(height * config.minimum_segment_length_ratio))
    )
    lines = cv2.HoughLinesP(
        edges,
        1.0,
        np.pi / 360.0,
        threshold=max(5, int(config.hough_threshold)),
        minLineLength=minimum_length,
        maxLineGap=max(4, int(round(height * config.hough_max_gap_ratio))),
    )
    segments: list[_Segment] = []
    if lines is not None:
        for values in lines[:, 0, :]:
            x1, y1, x2, y2 = (float(value) for value in values)
            dy = y2 - y1
            length = float(math.hypot(x2 - x1, dy))
            if abs(dy) < height * config.minimum_vertical_span_ratio:
                continue
            slope = (x2 - x1) / dy
            if (
                not math.isfinite(slope)
                or abs(slope) > config.maximum_abs_slope
            ):
                continue
            segments.append(_Segment(x1, y1, x2, y2, length, slope))

    # Short isolated Y1 dashes can be too small for Hough.  Add a component
    # principal line as a second source; forked W1/W2 components are still
    # separated by the Hough segments above.
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (mask > 0).astype(np.uint8), connectivity=8
    )
    for index in range(1, count):
        x, y, component_width, component_height, area = (
            int(value) for value in stats[index]
        )
        if (
            area < 35
            or component_height < height * config.minimum_vertical_span_ratio
        ):
            continue
        rows, columns = np.nonzero(labels == index)
        if rows.size < 8 or float(np.ptp(rows)) < 2.0:
            continue
        slope, intercept = np.polyfit(
            rows.astype(float), columns.astype(float), 1
        )
        if not math.isfinite(slope) or abs(slope) > config.maximum_abs_slope:
            continue
        y1 = float(y)
        y2 = float(y + component_height - 1)
        x1 = float(slope * y1 + intercept)
        x2 = float(slope * y2 + intercept)
        length = float(math.hypot(x2 - x1, y2 - y1))
        segments.append(_Segment(x1, y1, x2, y2, length, float(slope)))
    return segments


def _fit_hypothesis(
    color: str,
    segments: list[_Segment],
    *,
    width: int,
    height: int,
    config: SequenceEntryConfig,
    minimum_vertical_span_ratio: float | None = None,
) -> LineHypothesis | None:
    points = np.asarray(
        [
            point
            for segment in segments
            for point in ((segment.x1, segment.y1), (segment.x2, segment.y2))
        ],
        dtype=np.float64,
    )
    if points.shape[0] < 2:
        return None
    ys = points[:, 1]
    xs = points[:, 0]
    for _ in range(2):
        slope, intercept = np.polyfit(ys, xs, 1)
        residuals = np.abs(xs - (slope * ys + intercept))
        keep = residuals <= max(5.0, width * config.maximum_fit_rmse_ratio)
        if int(np.count_nonzero(keep)) < 2 or bool(np.all(keep)):
            break
        xs = xs[keep]
        ys = ys[keep]
    slope, intercept = np.polyfit(ys, xs, 1)
    residuals = xs - (slope * ys + intercept)
    rmse_ratio = float(np.sqrt(np.mean(np.square(residuals))) / width)
    minimum_y = float(np.min(ys))
    maximum_y = float(np.max(ys))
    span_ratio = (maximum_y - minimum_y) / height
    required_span = float(
        config.minimum_vertical_span_ratio
        if minimum_vertical_span_ratio is None
        else minimum_vertical_span_ratio
    )
    if (
        span_ratio < required_span
        or rmse_ratio > config.maximum_fit_rmse_ratio
    ):
        return None
    mean_y = float(np.mean(ys))
    mean_x = float(slope * mean_y + intercept)
    near_x = float(slope * maximum_y + intercept)
    far_x = float(slope * minimum_y + intercept)
    support = sum(segment.length for segment in segments) / math.hypot(
        width, height
    )
    normalized_slope = float(slope * height / width)
    # Pixel x grows to vehicle-right while metric lateral grows left.  For a
    # near-to-far tangent, atan((lateral/forward scale) * dx/dy) is the signed
    # heading from the vehicle's +forward axis used by the annotation study.
    heading = math.atan((1.4 / 1.5) * normalized_slope)
    return LineHypothesis(
        color=color,
        coefficients=(normalized_slope, float(intercept / width)),
        mean_x_ratio=float(mean_x / width),
        mean_y_ratio=float(mean_y / height),
        near_x_ratio=float(near_x / width),
        far_x_ratio=float(far_x / width),
        minimum_y_ratio=float(minimum_y / height),
        maximum_y_ratio=float(maximum_y / height),
        direction_dx_dy=normalized_slope,
        heading_from_vehicle_rad=float(heading),
        vertical_span_ratio=float(span_ratio),
        fit_rmse_ratio=rmse_ratio,
        support_length_ratio=float(support),
    )


def _component_hypotheses(
    mask: np.ndarray,
    color: str,
    config: SequenceEntryConfig,
    *,
    minimum_area_px: int,
    minimum_height_px: int,
    minimum_span_ratio: float,
) -> list[LineHypothesis]:
    """Fit semantic component centrelines below the general Hough cutoff.

    This preserves both small white branch fragments and the 20--25 px Y1
    dashes seen during initial acquisition.  In particular, fitting the yellow
    component centre is safer than treating both edges of a thick Y2 stripe as
    independent line headings.
    """
    height, width = mask.shape
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (mask > 0).astype(np.uint8), connectivity=8
    )
    output: list[LineHypothesis] = []
    for index in range(1, count):
        _, _, _, component_height, area = (
            int(value) for value in stats[index]
        )
        if (
            area < minimum_area_px
            or component_height < minimum_height_px
        ):
            continue
        rows, columns = np.nonzero(labels == index)
        if rows.size < 8 or float(np.ptp(rows)) < 2.0:
            continue
        slope, intercept = np.polyfit(
            rows.astype(np.float64), columns.astype(np.float64), 1
        )
        normalized_slope = float(slope * height / width)
        if (
            not math.isfinite(normalized_slope)
            or abs(normalized_slope) > config.maximum_abs_slope
        ):
            continue
        minimum_y = float(np.min(rows))
        maximum_y = float(np.max(rows))
        segment = _Segment(
            float(slope * minimum_y + intercept),
            minimum_y,
            float(slope * maximum_y + intercept),
            maximum_y,
            float(
                math.hypot(
                    slope * (maximum_y - minimum_y),
                    maximum_y - minimum_y,
                )
            ),
            float(slope),
        )
        fitted = _fit_hypothesis(
            color,
            [segment],
            width=width,
            height=height,
            config=config,
            minimum_vertical_span_ratio=minimum_span_ratio,
        )
        if fitted is not None:
            output.append(fitted)
    return output


def _band_track_hypotheses(
    mask: np.ndarray,
    color: str,
    config: SequenceEntryConfig,
) -> list[LineHypothesis]:
    """Link stripe centres from many BEV heights, not just one histogram."""
    height, width = mask.shape
    band_height = max(10, int(round(height / 30.0)))
    bands: list[list[tuple[float, float, int]]] = []
    for bottom in range(height, 0, -band_height):
        top = max(0, bottom - band_height)
        band = mask[top:bottom] > 0
        histogram = np.count_nonzero(band, axis=0)
        columns = np.flatnonzero(histogram >= 2)
        observations: list[tuple[float, float, int]] = []
        if columns.size:
            breaks = np.flatnonzero(
                np.diff(columns) > max(3, int(round(width * 0.012)))
            ) + 1
            rows_y, rows_x = np.nonzero(band)
            for run in np.split(columns, breaks):
                selected = np.isin(rows_x, run)
                pixel_count = int(np.count_nonzero(selected))
                if pixel_count < 10:
                    continue
                observations.append(
                    (
                        float(np.mean(rows_x[selected])),
                        float(top + np.mean(rows_y[selected])),
                        pixel_count,
                    )
                )
        bands.append(observations)

    tracks: list[dict] = []
    for band_index, observations in enumerate(bands):
        matches = []
        for track_index, track in enumerate(tracks):
            if band_index - int(track["last_band"]) > 5:
                continue
            last_x, last_y, _ = track["points"][-1]
            slope = float(track.get("slope", 0.0))
            for observation_index, (x, y, _) in enumerate(observations):
                delta_y = y - last_y
                predicted_x = last_x + slope * delta_y
                distance = abs(x - predicted_x)
                allowance = width * 0.07 + 0.40 * abs(delta_y)
                if distance <= allowance:
                    matches.append(
                        (distance, track_index, observation_index)
                    )
        used_tracks: set[int] = set()
        used_observations: set[int] = set()
        for _, track_index, observation_index in sorted(matches):
            if (
                track_index in used_tracks
                or observation_index in used_observations
            ):
                continue
            track = tracks[track_index]
            last_x, last_y, _ = track["points"][-1]
            x, y, count = observations[observation_index]
            if abs(y - last_y) > 1.0:
                observed_slope = (x - last_x) / (y - last_y)
                previous_slope = float(
                    track.get("slope", observed_slope)
                )
                track["slope"] = (
                    0.65 * previous_slope + 0.35 * observed_slope
                )
            track["points"].append((x, y, count))
            track["last_band"] = band_index
            used_tracks.add(track_index)
            used_observations.add(observation_index)
        for observation_index, observation in enumerate(observations):
            if observation_index not in used_observations:
                tracks.append(
                    {"points": [observation], "last_band": band_index}
                )

    output: list[LineHypothesis] = []
    for track in tracks:
        points = track["points"]
        if len(points) < 2:
            continue
        pseudo_segments = []
        for first, second in zip(points, points[1:]):
            x1, y1, _ = first
            x2, y2, _ = second
            if abs(y2 - y1) < 1.0:
                continue
            slope = (x2 - x1) / (y2 - y1)
            if abs(slope) > config.maximum_abs_slope:
                continue
            pseudo_segments.append(
                _Segment(
                    x1,
                    y1,
                    x2,
                    y2,
                    float(math.hypot(x2 - x1, y2 - y1)),
                    float(slope),
                )
            )
        if not pseudo_segments:
            continue
        fitted = _fit_hypothesis(
            color,
            pseudo_segments,
            width=width,
            height=height,
            config=config,
        )
        if fitted is not None:
            output.append(fitted)
    return output


def _line_distance_ratio(
    first: LineHypothesis,
    second: LineHypothesis,
) -> float:
    low = max(first.minimum_y_ratio, second.minimum_y_ratio)
    high = min(first.maximum_y_ratio, second.maximum_y_ratio)
    if low <= high:
        rows = np.linspace(low, high, 5)
    else:
        rows = np.asarray(
            [0.5 * (first.mean_y_ratio + second.mean_y_ratio)]
        )
    return float(
        np.mean(
            [
                abs(first.x_ratio_at(row) - second.x_ratio_at(row))
                for row in rows
            ]
        )
    )


def extract_line_hypotheses(
    mask: np.ndarray,
    color: str,
    config: SequenceEntryConfig | None = None,
) -> tuple[LineHypothesis, ...]:
    """Extract multiple globally visible polylines, including far-only W1."""
    cfg = config or SequenceEntryConfig()
    binary = normalize_mask(mask)
    height, width = binary.shape
    segments = sorted(
        _raw_segments(binary, cfg), key=lambda item: -item.length
    )
    clusters: list[list[_Segment]] = []
    for segment in segments:
        compatible = [
            index
            for index, cluster in enumerate(clusters)
            if any(
                _segments_compatible(
                    segment,
                    member,
                    width=width,
                    height=height,
                    config=cfg,
                )
                for member in cluster
            )
        ]
        if compatible:
            clusters[compatible[0]].append(segment)
        else:
            clusters.append([segment])

    hypotheses = [
        fitted
        for cluster in clusters
        if (
            fitted := _fit_hypothesis(
                color,
                cluster,
                width=width,
                height=height,
                config=cfg,
            )
        )
        is not None
    ]
    hypotheses.extend(_band_track_hypotheses(binary, color, cfg))
    # Parallel edges of the same painted stripe make duplicate Hough fits.
    # Keep the stronger representative while preserving distinct fork arms.
    kept: list[LineHypothesis] = []
    for candidate in sorted(
        hypotheses, key=lambda item: -item.support_length_ratio
    ):
        duplicate = any(
            _line_distance_ratio(candidate, existing)
            <= cfg.duplicate_distance_ratio
            and abs(
                _line_angle(candidate.direction_dx_dy)
                - _line_angle(existing.direction_dx_dy)
            )
            <= cfg.cluster_angle_tolerance_rad
            for existing in kept
        )
        if not duplicate:
            kept.append(candidate)

    # Preserve fine branch hypotheses after de-duplicating the global tracks.
    # They intentionally do not pass through the broad duplicate suppression:
    # a short W1 segment can be close to W2 only at their fork, while still
    # carrying the correct heading.  The selector's topology score resolves
    # these alternatives without erasing either one here.
    if color == "white":
        kept.extend(
            fitted
            for segment in segments
            if (
                fitted := _fit_hypothesis(
                    color,
                    [segment],
                    width=width,
                    height=height,
                    config=cfg,
                )
            )
            is not None
        )
        kept.extend(
            _component_hypotheses(
                binary,
                color,
                cfg,
                minimum_area_px=cfg.white_component_minimum_area_px,
                minimum_height_px=cfg.white_component_minimum_height_px,
                minimum_span_ratio=cfg.tracking_white_minimum_span_ratio,
            )
        )
    elif color == "yellow":
        kept.extend(
            _component_hypotheses(
                binary,
                color,
                cfg,
                minimum_area_px=cfg.yellow_component_minimum_area_px,
                minimum_height_px=cfg.yellow_component_minimum_height_px,
                minimum_span_ratio=cfg.yellow_component_minimum_span_ratio,
            )
        )
    return tuple(sorted(kept, key=lambda item: item.mean_x_ratio))


def pixels_to_vehicle_path(
    path_pixels: tuple[tuple[float, float], ...],
    *,
    width: int,
    height: int,
    forward_range_m: float = 1.5,
    lateral_range_m: float = 1.4,
) -> np.ndarray:
    """Convert BEV pixels to base_footprint [forward, left-positive] metres."""
    if not path_pixels:
        return np.empty((0, 2), dtype=np.float64)
    denominator_y = float(max(1, height - 1))
    points = np.asarray(
        [
            (
                (height - 1 - float(y))
                * float(forward_range_m)
                / denominator_y,
                (width * 0.5 - float(x)) * float(lateral_range_m) / width,
            )
            for x, y in path_pixels
        ],
        dtype=np.float64,
    )
    return points[np.argsort(points[:, 0])]


def estimate_branch_point(
    w1: LineHypothesis | None,
    white_candidates: tuple[LineHypothesis, ...],
    *,
    forward_range_m: float,
    tracked_w2: LineHypothesis | None = None,
) -> BranchPointEstimate:
    """Estimate the fork from locked W1 and the temporally tracked W2."""
    if w1 is None:
        return BranchPointEstimate(distance_m=math.inf, w2=tracked_w2)
    best: tuple[float, float, LineHypothesis, float, float] | None = None
    w1_slope, w1_intercept = w1.coefficients
    candidates = (tracked_w2,) if tracked_w2 is not None else white_candidates
    for candidate in candidates:
        if candidate is w1 or candidate.color != "white":
            continue
        slope, intercept = candidate.coefficients
        slope_delta = w1_slope - slope
        if abs(slope_delta) < 0.08:
            continue
        # W2 is the branch on the image-right side before the fork.  This is
        # used only as a spatial trigger and never as steering geometry.
        sample_y = min(1.0, max(0.0, max(w1.maximum_y_ratio, candidate.maximum_y_ratio)))
        if candidate.x_ratio_at(sample_y) <= w1.x_ratio_at(sample_y):
            continue
        intersection_y = (intercept - w1_intercept) / slope_delta
        if not -0.10 <= intersection_y <= 1.25:
            continue
        intersection_x = w1.x_ratio_at(intersection_y)
        if not -0.15 <= intersection_x <= 1.15:
            continue
        distance = (1.0 - intersection_y) * float(forward_range_m)
        if distance < -0.10 or distance > 1.10 * float(forward_range_m):
            continue
        score = abs(intersection_x - 0.5) + 0.15 * abs(candidate.mean_x_ratio - w1.mean_x_ratio)
        if best is None or score < best[0]:
            best = (
                score,
                max(0.0, distance),
                candidate,
                intersection_x,
                intersection_y,
            )
    if best is None:
        return BranchPointEstimate(distance_m=math.inf, w2=tracked_w2)
    return BranchPointEstimate(
        distance_m=float(best[1]),
        w2=best[2],
        intersection_x_ratio=float(best[3]),
        intersection_y_ratio=float(best[4]),
    )


def branch_point_distance_m(
    w1: LineHypothesis | None,
    white_candidates: tuple[LineHypothesis, ...],
    *,
    forward_range_m: float,
) -> float:
    """Estimate remaining distance while preserving the float-only API."""
    return estimate_branch_point(
        w1,
        white_candidates,
        forward_range_m=forward_range_m,
    ).distance_m


def spatial_steering_gate(
    *,
    branch_distance_m: float,
    rule_speed_command: float,
    speed_command_to_mps: float,
    response_time_sec: float,
    minimum_trigger_distance_m: float,
    blend_distance_m: float,
) -> SpatialSteeringGate:
    speed_mps = max(0.0, float(rule_speed_command)) * max(
        0.0, float(speed_command_to_mps)
    )
    trigger = max(0.0, float(minimum_trigger_distance_m)) + (
        speed_mps * max(0.0, float(response_time_sec))
    )
    finite = math.isfinite(float(branch_distance_m))
    ready = bool(finite and float(branch_distance_m) <= trigger)
    if not ready:
        blend = 0.0
    else:
        blend = min(
            1.0,
            max(
                0.0,
                (trigger - float(branch_distance_m))
                / max(1.0e-6, float(blend_distance_m)),
            ),
        )
    return SpatialSteeringGate(
        branch_distance_m=float(branch_distance_m),
        trigger_distance_m=float(trigger),
        ready=ready,
        blend=float(blend),
    )


def w1_entry_trigger_ready(
    *,
    branch_distance_m: float,
    spatial_gate_ready: bool,
    start_on_intersection: bool,
) -> bool:
    """Start on the first valid W1/W2 intersection unless distance mode is selected."""
    if bool(start_on_intersection):
        return math.isfinite(float(branch_distance_m))
    return bool(spatial_gate_ready)


def w1_steering_delay_ready(
    *, observed_frames: int, required_frames: int
) -> bool:
    """Return whether accumulated valid W1 observations may start steering."""
    required = max(0, int(required_frames))
    return required == 0 or int(observed_frames) >= required


def update_w1_steering_delay_counts(
    *,
    observed_frames: int,
    missing_frames: int,
    w1_observed: bool,
    missing_tolerance_frames: int,
) -> tuple[int, int]:
    """Accumulate W1 frames while tolerating short semantic dropouts."""
    if w1_observed:
        return int(observed_frames) + 1, 0
    if int(observed_frames) <= 0:
        return 0, 0
    missing = int(missing_frames) + 1
    if missing > max(0, int(missing_tolerance_frames)):
        return 0, 0
    return int(observed_frames), missing


def select_entry_handoff_condition(
    *,
    geometric_handoff: bool,
    steering_started: bool,
    progress_m: float,
    minimum_progress_m: float,
    pair_track_confirmed: bool,
    y1_confirmed: bool,
    w1_visible: bool,
    w1_loss_handoff_enabled: bool = True,
    steering_active_sec: float = 0.0,
    maximum_steering_sec: float = math.inf,
) -> str | None:
    """Choose the first satisfied semantic-entry completion condition."""
    if not steering_started:
        return None
    if float(progress_m) >= float(minimum_progress_m):
        if geometric_handoff:
            return "forward_alignment_and_progress"
        if pair_track_confirmed:
            return "pair_track_and_progress"
        if w1_loss_handoff_enabled and y1_confirmed and not w1_visible:
            return "y1_confirmed_w1_lost_and_progress"
    if 0.0 < float(maximum_steering_sec) <= float(steering_active_sec):
        return "maximum_w1_steering_time_elapsed"
    return None


class SequenceAwareEntrySelector:
    """Track the continuing W2 first, then lock its new left branch as W1."""

    def __init__(self, config: SequenceEntryConfig | None = None) -> None:
        self.config = config or SequenceEntryConfig()
        self.reset()

    def reset(self) -> None:
        self.phase = EntrySequencePhase.LEFT4_ARMED
        self.w1: LineHypothesis | None = None
        self.w2: LineHypothesis | None = None
        self.y1: LineHypothesis | None = None
        self.pending_w1: LineHypothesis | None = None
        self.pending_w2: LineHypothesis | None = None
        self.pending_y1: LineHypothesis | None = None
        self.w1_confirmation_history: list[LineHypothesis | None] = []
        self.w1_confirmations = 0
        self.w1_fast_locked = False
        self.w2_confirmations = 0
        self.y1_confirmations = 0
        self.w1_missing_frames = 0
        self.w2_missing_frames = 0
        self.y1_missing_frames = 0
        self.y1_blend = 0.0
        self.alignment_frames = 0

    def _w1_topology_score(
        self,
        candidate: LineHypothesis,
        previous: LineHypothesis | None = None,
    ) -> float:
        score = (
            candidate.mean_x_ratio
            + 0.20 * abs(candidate.direction_dx_dy - 0.50)
            - 0.50 * candidate.vertical_span_ratio
            - 0.20 * candidate.mean_y_ratio
            - 0.05 * candidate.support_length_ratio
        )
        if previous is not None:
            anchor_jump = abs(
                candidate.x_ratio_at(0.65) - previous.x_ratio_at(0.65)
            )
            slope_jump = abs(
                candidate.direction_dx_dy - previous.direction_dx_dy
            )
            score += self.config.topology_continuity_weight * min(
                anchor_jump, 0.20
            )
            score += self.config.topology_slope_continuity_weight * min(
                slope_jump, 1.0
            )
        return float(score)

    def _y1_topology_score(
        self,
        candidate: LineHypothesis,
        w1: LineHypothesis,
        previous: LineHypothesis | None = None,
    ) -> float:
        separation = candidate.mean_x_ratio - w1.mean_x_ratio
        score = (
            abs(separation - self.config.expected_pair_separation_ratio)
            + 0.50
            * abs(candidate.direction_dx_dy - w1.direction_dx_dy)
            + 0.20 * candidate.mean_x_ratio
            - 0.50 * candidate.vertical_span_ratio
        )
        if previous is not None:
            anchor_jump = abs(
                candidate.x_ratio_at(0.70) - previous.x_ratio_at(0.70)
            )
            slope_jump = abs(
                candidate.direction_dx_dy - previous.direction_dx_dy
            )
            score += self.config.topology_continuity_weight * min(
                anchor_jump, 0.20
            )
            score += self.config.topology_slope_continuity_weight * min(
                slope_jump, 1.0
            )
        return float(score)

    def _matches_previous(
        self,
        candidate: LineHypothesis,
        previous: LineHypothesis,
    ) -> bool:
        return bool(
            abs(candidate.mean_x_ratio - previous.mean_x_ratio)
            <= self.config.temporal_max_mean_x_jump_ratio
            and _line_distance_ratio(candidate, previous)
            <= self.config.temporal_max_line_distance_ratio
        )

    def _w2_topology_score(
        self,
        candidate: LineHypothesis,
        previous: LineHypothesis | None = None,
    ) -> float:
        score = (
            -1.35 * candidate.vertical_span_ratio
            - 0.18 * candidate.support_length_ratio
            + 0.08 * abs(candidate.mean_x_ratio - 0.32)
        )
        if previous is not None:
            score += 2.0 * _line_distance_ratio(candidate, previous)
            score += 0.35 * abs(
                candidate.direction_dx_dy - previous.direction_dx_dy
            )
        return float(score)

    def _acquire_w2(
        self, candidates: tuple[LineHypothesis, ...]
    ) -> LineHypothesis | None:
        eligible = [
            item
            for item in candidates
            if (
                self.config.w2_acquisition_min_mean_x_ratio
                <= item.mean_x_ratio
                <= self.config.w2_acquisition_max_mean_x_ratio
                and item.vertical_span_ratio
                >= self.config.w2_acquisition_minimum_span_ratio
                and self.config.tracking_minimum_slope
                <= item.direction_dx_dy
                <= self.config.tracking_maximum_slope
            )
        ]
        if not eligible:
            return None
        return min(eligible, key=self._w2_topology_score)

    def _track_w2(
        self,
        candidates: tuple[LineHypothesis, ...],
        previous: LineHypothesis | None,
    ) -> LineHypothesis | None:
        if previous is None:
            return None
        eligible = [
            item
            for item in candidates
            if (
                self.config.tracking_minimum_slope
                <= item.direction_dx_dy
                <= self.config.tracking_maximum_slope
                and item.vertical_span_ratio
                >= self.config.tracking_white_minimum_span_ratio
                and self._matches_previous(item, previous)
            )
        ]
        if not eligible:
            return None
        return min(
            eligible,
            key=lambda item: self._w2_topology_score(item, previous),
        )

    def _w1_branch_metrics(
        self,
        candidate: LineHypothesis,
        w2: LineHypothesis,
    ) -> tuple[float, float]:
        distance = _line_distance_ratio(candidate, w2)
        overlap_min_y = max(candidate.minimum_y_ratio, w2.minimum_y_ratio)
        overlap_max_y = min(candidate.maximum_y_ratio, w2.maximum_y_ratio)
        if overlap_max_y <= overlap_min_y:
            return float("-inf"), float(distance)
        sample_y = np.linspace(overlap_min_y, overlap_max_y, 3)
        separations = [
            w2.x_ratio_at(y_ratio) - candidate.x_ratio_at(y_ratio)
            for y_ratio in sample_y
        ]
        return float(np.median(separations)), float(distance)

    def _is_w1_branch(
        self,
        candidate: LineHypothesis,
        w2: LineHypothesis,
        *,
        require_acquisition_heading: bool = True,
    ) -> bool:
        separation, distance = self._w1_branch_metrics(candidate, w2)
        minimum_slope = (
            self.config.acquisition_slope_minimum
            if require_acquisition_heading
            else self.config.w1_tracking_slope_minimum
        )
        return bool(
            candidate is not w2
            and candidate.mean_x_ratio
            <= self.config.w1_acquisition_max_mean_x_ratio
            and candidate.direction_dx_dy >= minimum_slope
            and candidate.vertical_span_ratio
            >= self.config.tracking_white_minimum_span_ratio
            and self.config.w1_branch_minimum_separation_ratio
            <= separation
            <= self.config.w1_branch_maximum_separation_ratio
            and distance > self.config.duplicate_distance_ratio
        )

    def _is_w1_acquisition_candidate(
        self,
        candidate: LineHypothesis,
        w2: LineHypothesis,
    ) -> bool:
        _, distance = self._w1_branch_metrics(candidate, w2)
        return bool(
            self._is_w1_branch(candidate, w2)
            and candidate.maximum_y_ratio
            >= self.config.w1_acquisition_minimum_max_y_ratio
            and distance >= self.config.w1_acquisition_minimum_distance_ratio
        )

    def _is_strong_w1_branch(
        self,
        candidate: LineHypothesis,
        w2: LineHypothesis,
    ) -> bool:
        if not self.config.w1_fast_lock_enabled:
            return False
        _, distance = self._w1_branch_metrics(candidate, w2)
        return bool(
            self._is_w1_acquisition_candidate(candidate, w2)
            and candidate.direction_dx_dy
            >= self.config.w1_fast_lock_minimum_slope
            and candidate.vertical_span_ratio
            >= self.config.w1_fast_lock_minimum_span_ratio
            and distance >= self.config.w1_fast_lock_minimum_distance_ratio
        )

    def _w1_branch_score(
        self,
        candidate: LineHypothesis,
        w2: LineHypothesis,
        previous: LineHypothesis | None = None,
    ) -> float:
        separation, _ = self._w1_branch_metrics(candidate, w2)
        score = (
            abs(separation - self.config.w1_branch_expected_separation_ratio)
            + 0.18
            * abs(
                candidate.direction_dx_dy
                - max(self.config.acquisition_slope_minimum, 0.35)
            )
            - 0.35 * candidate.vertical_span_ratio
            - 0.05 * candidate.support_length_ratio
        )
        if previous is not None:
            score += 1.6 * _line_distance_ratio(candidate, previous)
            score += 0.15 * abs(
                candidate.direction_dx_dy - previous.direction_dx_dy
            )
        return float(score)

    def _acquire_w1(
        self,
        candidates: tuple[LineHypothesis, ...],
        w2: LineHypothesis | None,
    ) -> LineHypothesis | None:
        if w2 is None:
            return None
        eligible = [
            item
            for item in candidates
            if self._is_w1_acquisition_candidate(item, w2)
        ]
        if not eligible:
            return None
        return min(eligible, key=lambda item: self._w1_branch_score(item, w2))

    def _track_w1(
        self,
        candidates: tuple[LineHypothesis, ...],
        previous: LineHypothesis | None,
        w2: LineHypothesis | None,
    ) -> LineHypothesis | None:
        if previous is None or w2 is None:
            return None
        eligible = [
            item
            for item in candidates
            if (
                self._is_w1_branch(
                    item,
                    w2,
                    require_acquisition_heading=False,
                )
                and item.mean_x_ratio
                <= self.config.tracking_white_max_mean_x_ratio
                and self.config.tracking_minimum_slope
                <= item.direction_dx_dy
                <= self.config.tracking_maximum_slope
                and item.vertical_span_ratio
                >= self.config.tracking_white_minimum_span_ratio
                and self._matches_previous(item, previous)
            )
        ]
        if not eligible:
            return None
        return min(
            eligible,
            key=lambda item: self._w1_branch_score(item, w2, previous),
        )

    def _track_y1(
        self,
        candidates: tuple[LineHypothesis, ...],
        previous: LineHypothesis | None,
        w1: LineHypothesis | None,
    ) -> LineHypothesis | None:
        if previous is None or w1 is None:
            return None
        minimum_separation = 0.70 * self.config.minimum_pair_separation_ratio
        maximum_separation = 1.20 * self.config.maximum_pair_separation_ratio
        eligible = [
            item
            for item in candidates
            if (
                self.config.tracking_yellow_min_mean_x_ratio
                <= item.mean_x_ratio
                <= self.config.tracking_yellow_max_mean_x_ratio
                and item.vertical_span_ratio
                >= self.config.tracking_yellow_minimum_span_ratio
                and minimum_separation
                <= item.mean_x_ratio - w1.mean_x_ratio
                <= maximum_separation
            )
        ]
        if not eligible:
            return None
        return min(
            eligible,
            key=lambda item: self._y1_topology_score(item, w1, previous),
        )

    def _track(
        self,
        candidates: tuple[LineHypothesis, ...],
        previous: LineHypothesis | None,
    ) -> LineHypothesis | None:
        if previous is None:
            return None
        eligible = [
            item
            for item in candidates
            if self._matches_previous(item, previous)
        ]
        if not eligible:
            return None
        # After Y1 has established the fork topology, prefer a white track
        # supported across several BEV bands.  This prevents a short edge of a
        # thick W1 blob from replacing the full W1 centreline as the vehicle
        # becomes parallel to it.  The rule is intentionally not applied while
        # acquiring the far-only W1 or to short dashed Y1 observations.
        if (
            previous.color == "white"
            and self.phase >= EntrySequencePhase.PAIR_TRACK
        ):
            long_tracks = [
                item for item in eligible if item.vertical_span_ratio >= 0.15
            ]
            if long_tracks:
                eligible = long_tracks
        return min(
            eligible,
            key=lambda item: (
                1.8 * _line_distance_ratio(item, previous)
                + abs(item.mean_x_ratio - previous.mean_x_ratio)
                + 0.08
                * abs(item.direction_dx_dy - previous.direction_dx_dy)
                - 0.08 * item.vertical_span_ratio
                - 0.03 * item.support_length_ratio
            ),
        )

    def _acquire_y1(
        self,
        candidates: tuple[LineHypothesis, ...],
        w1: LineHypothesis,
    ) -> LineHypothesis | None:
        eligible = []
        for item in candidates:
            separation = item.mean_x_ratio - w1.mean_x_ratio
            if (
                item.direction_dx_dy
                >= self.config.acquisition_slope_minimum
                and self.config.minimum_pair_separation_ratio
                <= separation
                <= self.config.maximum_pair_separation_ratio
            ):
                eligible.append(item)
        if not eligible:
            # In the annotation order the lone early yellow is Y2.  It has a
            # non-positive vehicle heading and must not be used as Y1 fallback.
            return None
        return min(
            eligible,
            key=lambda item: self._y1_topology_score(item, w1),
        )

    def _confirm_w2(self, candidate: LineHypothesis | None) -> None:
        if candidate is None:
            self.pending_w2 = None
            self.w2_confirmations = 0
            return
        if self.pending_w2 is not None and self._matches_previous(
            candidate, self.pending_w2
        ):
            self.w2_confirmations += 1
        else:
            self.pending_w2 = candidate
            self.w2_confirmations = 1
        if (
            self.w2_confirmations
            >= self.config.w2_acquisition_required_frames
        ):
            self.w2 = candidate
            self.w2_missing_frames = 0

    def _confirm_w1(
        self,
        candidate: LineHypothesis | None,
        w2: LineHypothesis | None,
    ) -> None:
        if candidate is not None and self.pending_w1 is not None:
            if not self._matches_previous(candidate, self.pending_w1):
                self.w1_confirmation_history = []
        self.w1_confirmation_history.append(candidate)
        window = self.config.w1_acquisition_window_frames
        self.w1_confirmation_history = self.w1_confirmation_history[-window:]
        valid_history = [
            item for item in self.w1_confirmation_history if item is not None
        ]
        self.pending_w1 = valid_history[-1] if valid_history else None
        self.w1_confirmations = len(valid_history)
        fast_lock = bool(
            candidate is not None
            and w2 is not None
            and self._is_strong_w1_branch(candidate, w2)
        )
        if (
            self.w1_confirmations >= self.config.w1_acquisition_required_frames
            or fast_lock
        ) and candidate is not None:
            self.w1 = candidate
            self.phase = EntrySequencePhase.W1_LOCKED
            self.w1_missing_frames = 0
            self.w1_fast_locked = fast_lock

    def _confirm_y1(self, candidate: LineHypothesis | None) -> None:
        if candidate is None:
            self.pending_y1 = None
            self.y1_confirmations = 0
            return
        if self.pending_y1 is not None and self._matches_previous(
            candidate, self.pending_y1
        ):
            self.y1_confirmations += 1
        else:
            self.pending_y1 = candidate
            self.y1_confirmations = 1
        if (
            self.y1_confirmations
            >= self.config.y1_acquisition_required_frames
        ):
            self.y1 = candidate
            self.phase = EntrySequencePhase.Y1_LOCKED
            self.y1_missing_frames = 0
            self.y1_blend = min(
                1.0, self.config.actual_y1_blend_per_frame
            )

    def _path(
        self,
        *,
        width: int,
        height: int,
        use_y1: bool,
    ) -> tuple[tuple[tuple[float, float], ...], bool, float]:
        source = self.y1 if use_y1 and self.y1 is not None else self.w1
        if source is None:
            return (), False, 0.0
        w1 = self.w1
        sampling_reference = w1 if w1 is not None else source
        top = sampling_reference.minimum_y_ratio
        bottom = sampling_reference.maximum_y_ratio
        rows_ratio = np.linspace(
            bottom, top, int(self.config.path_sample_count)
        )
        points = []
        separations = []
        for row_ratio in rows_ratio:
            source_x = source.x_ratio_at(float(row_ratio))
            points.append(
                (
                    float(source_x * width),
                    float(row_ratio * height),
                )
            )
            if w1 is not None and source is not w1:
                separations.append(
                    source_x - w1.x_ratio_at(float(row_ratio))
                )
        if len(points) < 3:
            return (), False, 0.0
        separation = float(np.median(separations)) if separations else 0.0
        return tuple(points), False, separation

    def process(
        self,
        white_mask: np.ndarray,
        yellow_mask: np.ndarray,
    ) -> SequenceEntryResult:
        white = normalize_mask(white_mask)
        yellow = normalize_mask(yellow_mask)
        if white.shape != yellow.shape:
            raise ValueError("white and yellow masks must have the same shape")
        height, width = white.shape
        whites = extract_line_hypotheses(white, "white", self.config)
        yellows = extract_line_hypotheses(yellow, "yellow", self.config)

        if self.w2 is None:
            self._confirm_w2(self._acquire_w2(whites))
        else:
            tracked_w2 = self._track_w2(whites, self.w2)
            if tracked_w2 is not None:
                self.w2 = tracked_w2
                self.w2_missing_frames = 0
            else:
                self.w2_missing_frames += 1

        w2_usable = bool(
            self.w2 is not None
            and self.w2_missing_frames <= self.config.missing_hold_frames
        )
        if self.phase == EntrySequencePhase.LEFT4_ARMED:
            self._confirm_w1(
                self._acquire_w1(whites, self.w2)
                if w2_usable and self.w2_missing_frames == 0
                else None,
                self.w2 if w2_usable else None,
            )
        else:
            tracked_w1 = self._track_w1(whites, self.w1, self.w2)
            if tracked_w1 is not None:
                self.w1 = tracked_w1
                self.w1_missing_frames = 0
            else:
                self.w1_missing_frames += 1

        if self.w1 is not None and self.y1 is None:
            self._confirm_y1(self._acquire_y1(yellows, self.w1))
        elif self.y1 is not None:
            tracked_y1 = self._track_y1(yellows, self.y1, self.w1)
            if tracked_y1 is not None:
                self.y1 = tracked_y1
                self.y1_missing_frames = 0
                self.y1_blend = min(
                    1.0,
                    self.y1_blend + self.config.actual_y1_blend_per_frame,
                )
                if self.phase == EntrySequencePhase.Y1_LOCKED:
                    self.phase = EntrySequencePhase.PAIR_TRACK
            else:
                self.y1_missing_frames += 1

        w1_usable = bool(
            self.w1 is not None
            and self.w1_missing_frames <= self.config.missing_hold_frames
        )
        y1_usable = bool(
            self.y1 is not None
            and self.y1_missing_frames <= self.config.missing_hold_frames
        )
        selected_source_usable = bool(y1_usable or w1_usable)
        if not selected_source_usable:
            path = ()
            synthetic = False
            separation = 0.0
            reason = "W1/Y1 unavailable; safe stop"
        else:
            path, synthetic, separation = self._path(
                width=width,
                height=height,
                use_y1=bool(y1_usable and self.y1 is not None),
            )
            if not path:
                reason = "selected steering geometry invalid; safe stop"
            elif not y1_usable or self.y1 is None:
                reason = "PATH=W1 direct; waiting confirmed Y1; W2/Y2 ignored"
            else:
                reason = "PATH=Y1 direct; confirmed yellow centerline; W2/Y2 ignored"

        if (
            self.phase in (
                EntrySequencePhase.Y1_LOCKED,
                EntrySequencePhase.PAIR_TRACK,
            )
            and w1_usable
            and y1_usable
            and self.w1 is not None
            and self.y1 is not None
            and abs(self.w1.direction_dx_dy)
            <= self.config.alignment_max_abs_slope
            and abs(self.y1.direction_dx_dy)
            <= self.config.alignment_max_abs_slope
        ):
            self.alignment_frames += 1
        else:
            self.alignment_frames = 0
        if self.alignment_frames >= self.config.alignment_required_frames:
            self.phase = EntrySequencePhase.CRUISE_HANDOFF
            reason = "W1/Y1 aligned with vehicle forward axis; cruise handoff"

        ready = self.phase != EntrySequencePhase.LEFT4_ARMED
        path_valid = bool(path and ready and selected_source_usable)
        if self.phase == EntrySequencePhase.LEFT4_ARMED:
            if w2_usable:
                phase_reason = "W2 tracked; waiting for left-diverging W1"
            else:
                phase_reason = "left_4 armed; acquiring persistent W2"
        else:
            phase_reason = reason
        return SequenceEntryResult(
            phase=self.phase,
            ready=ready,
            path_valid=path_valid,
            cruise_handoff=self.phase == EntrySequencePhase.CRUISE_HANDOFF,
            reason=phase_reason,
            white_candidates=whites,
            yellow_candidates=yellows,
            w1=self.w1 if w1_usable else None,
            w2=self.w2 if w2_usable else None,
            y1=self.y1 if y1_usable else None,
            path_pixels=path,
            used_synthetic_y1=bool(synthetic),
            pair_separation_ratio=float(separation),
        )


def render_sequence_debug(
    white_mask: np.ndarray,
    yellow_mask: np.ndarray,
    result: SequenceEntryResult,
    branch_estimate: BranchPointEstimate | None = None,
    *,
    branch_latched: bool = False,
) -> np.ndarray:
    white = normalize_mask(white_mask)
    yellow = normalize_mask(yellow_mask)
    height, width = white.shape
    output = np.full((height, width, 3), 22, dtype=np.uint8)
    output[white > 0] = (235, 235, 235)
    output[yellow > 0] = (0, 190, 225)

    def draw_line(
        item: LineHypothesis,
        color: tuple[int, int, int],
        thickness: int,
    ):
        y1 = int(round(item.minimum_y_ratio * height))
        y2 = int(round(item.maximum_y_ratio * height))
        x1 = int(round(item.x_ratio_at(item.minimum_y_ratio) * width))
        x2 = int(round(item.x_ratio_at(item.maximum_y_ratio) * width))
        cv2.line(output, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)

    for item in result.white_candidates:
        draw_line(item, (120, 90, 90), 1)
    for item in result.yellow_candidates:
        draw_line(item, (40, 100, 120), 1)
    if branch_estimate is not None and branch_estimate.w2 is not None:
        w2 = branch_estimate.w2
        draw_line(w2, (255, 255, 0), 3)
        label_y = int(
            round(0.5 * (w2.minimum_y_ratio + w2.maximum_y_ratio) * height)
        )
        label_x = int(round(w2.x_ratio_at(label_y / height) * width))
        cv2.putText(
            output,
            "W2 (GATE LATCHED)" if branch_latched else "W2 (GATE CANDIDATE)",
            (max(4, min(width - 150, label_x + 8)), max(92, label_y)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
    if result.w1 is not None:
        draw_line(result.w1, (255, 0, 255), 4)
    if result.y1 is not None:
        draw_line(result.y1, (0, 255, 80), 4)
    if len(result.path_pixels) >= 2:
        pixels = np.asarray(result.path_pixels, dtype=np.int32)
        cv2.polylines(output, [pixels], False, (255, 100, 0), 4, cv2.LINE_AA)
    if (
        branch_estimate is not None
        and branch_estimate.w2 is not None
        and math.isfinite(branch_estimate.intersection_x_ratio)
        and math.isfinite(branch_estimate.intersection_y_ratio)
    ):
        raw_intersection = (
            int(round(branch_estimate.intersection_x_ratio * width)),
            int(round(branch_estimate.intersection_y_ratio * height)),
        )
        marker_margin = 12
        intersection = (
            max(marker_margin, min(width - marker_margin - 1, raw_intersection[0])),
            max(marker_margin, min(height - marker_margin - 1, raw_intersection[1])),
        )
        cv2.circle(output, intersection, 10, (0, 0, 255), 4, cv2.LINE_AA)
        text_x = max(4, min(width - 185, intersection[0] + 12))
        text_y = max(104, min(height - 10, intersection[1] - 12))
        label = (
            f"W1/W2 GATE X  {branch_estimate.distance_m:.3f}m"
            if branch_latched
            else f"W1/W2 X  {branch_estimate.distance_m:.3f}m"
        )
        cv2.putText(
            output,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    cv2.line(
        output,
        (width // 2, height - 1),
        (width // 2, max(0, height // 4)),
        (255, 80, 80),
        1,
        cv2.LINE_AA,
    )
    status_color = (40, 220, 70) if result.path_valid else (40, 40, 255)
    cv2.rectangle(output, (0, 0), (width, 78), (12, 12, 12), -1)
    cv2.putText(
        output,
        (
            f"{result.phase.name}  "
            f"{'VALID' if result.path_valid else 'WAIT/STOP'}"
        ),
        (10, 26),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        status_color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        result.reason,
        (10, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.43,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        (
            f"W candidates={len(result.white_candidates)}  "
            f"Y candidates={len(result.yellow_candidates)}  "
            f"path_source={'Y1' if result.y1 is not None else 'W1'}  "
            f"synthetic_Y1={int(result.used_synthetic_y1)}"
        ),
        (10, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )
    return output
