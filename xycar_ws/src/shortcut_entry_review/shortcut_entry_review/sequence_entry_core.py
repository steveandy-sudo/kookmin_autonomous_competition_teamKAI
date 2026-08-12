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
    white_branch_hough_threshold: int = 5
    white_branch_minimum_length_px: int = 6
    white_branch_minimum_vertical_span_ratio: float = 0.035
    white_branch_maximum_gap_ratio: float = 0.045
    white_branch_minimum_slope: float = 0.03
    fine_duplicate_distance_ratio: float = 0.012
    fine_duplicate_angle_tolerance_rad: float = math.radians(5.0)
    yellow_component_minimum_area_px: int = 50
    yellow_component_minimum_height_px: int = 15
    yellow_component_minimum_span_ratio: float = 0.020
    w1_acquisition_required_frames: int = 1
    y1_acquisition_required_frames: int = 2
    acquisition_slope_minimum: float = 0.15
    w1_acquisition_max_mean_x_ratio: float = 0.40
    branch_w1_minimum_slope: float = 0.30
    branch_w1_minimum_span_ratio: float = 0.03
    branch_w2_maximum_slope: float = -0.03
    branch_w2_minimum_span_ratio: float = 0.06
    branch_w2_tracking_maximum_slope: float = -0.03
    branch_w2_tracking_minimum_span_ratio: float = 0.06
    branch_expected_mean_separation_ratio: float = 0.08
    branch_maximum_join_distance_ratio: float = 0.14
    branch_join_endpoint_tolerance_ratio: float = 0.02
    branch_yellow_minimum_span_ratio: float = 0.08
    branch_yellow_curve_fallback_slope: float = 0.30
    branch_tracking_maximum_cost: float = 0.42
    branch_w2_tracking_maximum_cost: float = 0.78
    temporal_max_mean_x_jump_ratio: float = 0.12
    temporal_max_line_distance_ratio: float = 0.19
    tracking_white_max_mean_x_ratio: float = 0.42
    tracking_yellow_min_mean_x_ratio: float = 0.25
    tracking_yellow_max_mean_x_ratio: float = 0.70
    # The 55 hand-labelled W1 frames bottom out at dx/dy=-0.174.  Keep a
    # small margin, but reject abrupt vehicle-right fits beyond this bound.
    tracking_minimum_slope: float = -0.20
    tracking_maximum_slope: float = 2.0
    # Labelled W1 spans start at 0.085; 0.05 retains margin while excluding
    # short edge fragments created by a rejected steep-right stripe.
    tracking_white_minimum_span_ratio: float = 0.050
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
        if self.w1_acquisition_required_frames < 1:
            raise ValueError("w1 acquisition frames must be positive")
        if self.y1_acquisition_required_frames < 1:
            raise ValueError("y1 acquisition frames must be positive")
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
        if self.branch_expected_mean_separation_ratio <= 0.0:
            raise ValueError("expected W1/W2 separation must be positive")
        if self.branch_w1_minimum_span_ratio <= 0.0:
            raise ValueError("W1 branch minimum span must be positive")
        if self.branch_w2_minimum_span_ratio <= 0.0:
            raise ValueError("W2 branch minimum span must be positive")
        if self.branch_w2_tracking_minimum_span_ratio <= 0.0:
            raise ValueError("W2 tracking minimum span must be positive")
        if self.branch_maximum_join_distance_ratio <= 0.0:
            raise ValueError("W1/W2 maximum join distance must be positive")
        if self.branch_join_endpoint_tolerance_ratio < 0.0:
            raise ValueError("W1/W2 endpoint tolerance cannot be negative")


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


def _filled_white_branch_hypotheses(
    mask: np.ndarray,
    config: SequenceEntryConfig,
) -> list[LineHypothesis]:
    """Extract short W1 arms through the centre of the semantic mask.

    The ordinary extractor intentionally starts from Canny edges.  That is a
    good source for the long continuing W2 boundary, but it can miss a short
    fork arm whose *centre* is visible while its two jagged edges point in
    different directions.  Running a small Hough transform on the filled mask
    supplies only positive-heading W1 alternatives; negative micro-segments
    are deliberately excluded because they can look like a false W2 inside
    an isolated white blob.
    """
    height, width = mask.shape
    minimum_span_px = max(
        int(config.white_branch_minimum_length_px),
        int(round(height * config.white_branch_minimum_vertical_span_ratio)),
    )
    lines = cv2.HoughLinesP(
        mask,
        1.0,
        np.pi / 720.0,
        threshold=int(config.white_branch_hough_threshold),
        minLineLength=minimum_span_px,
        maxLineGap=max(
            4,
            int(round(height * config.white_branch_maximum_gap_ratio)),
        ),
    )
    if lines is None:
        return []

    output: list[LineHypothesis] = []
    for x1, y1, x2, y2 in lines[:, 0, :]:
        dy = float(y2 - y1)
        if abs(dy) < minimum_span_px:
            continue
        pixel_slope = float(x2 - x1) / dy
        normalized_slope = pixel_slope * height / width
        if not (
            config.white_branch_minimum_slope
            <= normalized_slope
            <= config.maximum_abs_slope
        ):
            continue
        segment = _Segment(
            float(x1),
            float(y1),
            float(x2),
            float(y2),
            float(math.hypot(x2 - x1, dy)),
            float(pixel_slope),
        )
        fitted = _fit_hypothesis(
            "white",
            [segment],
            width=width,
            height=height,
            config=config,
            minimum_vertical_span_ratio=(
                config.white_branch_minimum_vertical_span_ratio
            ),
        )
        if fitted is not None:
            output.append(fitted)
    return output


def _fine_deduplicate_hypotheses(
    candidates: list[LineHypothesis],
    config: SequenceEntryConfig,
) -> list[LineHypothesis]:
    """Remove nearly identical fits without merging the two fork arms."""
    kept: list[LineHypothesis] = []
    for candidate in sorted(
        candidates,
        key=lambda item: (
            -item.vertical_span_ratio,
            -item.support_length_ratio,
            item.fit_rmse_ratio,
            item.mean_x_ratio,
        ),
    ):
        duplicate = any(
            _line_distance_ratio(candidate, existing)
            <= config.fine_duplicate_distance_ratio
            and abs(
                _line_angle(candidate.direction_dx_dy)
                - _line_angle(existing.direction_dx_dy)
            )
            <= config.fine_duplicate_angle_tolerance_rad
            for existing in kept
        )
        if not duplicate:
            kept.append(candidate)
    return kept


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


def _branch_join_distance_ratio(
    first: LineHypothesis,
    second: LineHypothesis,
) -> float:
    """Minimum branch distance inside their observed overlap or y-gap.

    W1 and W2 are two arms of the same fork, so their fitted lines approach
    one another where the observed segments overlap or meet.  Sampling only
    that supported envelope avoids accepting unrelated lines merely because
    their infinite extrapolations cross somewhere outside the BEV evidence.
    """
    overlap_low = max(first.minimum_y_ratio, second.minimum_y_ratio)
    overlap_high = min(first.maximum_y_ratio, second.maximum_y_ratio)
    if overlap_low <= overlap_high:
        low, high = overlap_low, overlap_high
    elif first.maximum_y_ratio < second.minimum_y_ratio:
        low, high = first.maximum_y_ratio, second.minimum_y_ratio
    else:
        low, high = second.maximum_y_ratio, first.minimum_y_ratio
    rows = np.linspace(float(low), float(high), 21)
    return float(
        min(
            abs(first.x_ratio_at(row) - second.x_ratio_at(row))
            for row in rows
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
        kept.extend(_filled_white_branch_hypotheses(binary, cfg))
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
    kept = _fine_deduplicate_hypotheses(kept, cfg)
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


class SequenceAwareEntrySelector:
    """Lock W1 first, acquire Y1 later, and never substitute W2/Y2."""

    def __init__(self, config: SequenceEntryConfig | None = None) -> None:
        self.config = config or SequenceEntryConfig()
        self.reset()

    def reset(self) -> None:
        self.phase = EntrySequencePhase.LEFT4_ARMED
        self.w1: LineHypothesis | None = None
        self.w2: LineHypothesis | None = None
        self.y1: LineHypothesis | None = None
        self.pending_w1: LineHypothesis | None = None
        self.pending_y1: LineHypothesis | None = None
        self.w1_confirmations = 0
        self.y1_confirmations = 0
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

    def _w1_tracking_score(
        self,
        candidate: LineHypothesis,
        previous: LineHypothesis,
    ) -> float:
        """Rank an already locked W1 without an acquisition-heading prior.

        Once identity is locked, W1 may rotate through a wide range of valid
        headings.  Prefer the left branch and complete support, then use only
        bounded polyline continuity.  This keeps the tracker on W1 instead of
        a similarly angled edge cut from W2 while still allowing a fresh fit.
        """
        anchor_jump = abs(
            candidate.x_ratio_at(0.65) - previous.x_ratio_at(0.65)
        )
        slope_jump = abs(
            candidate.direction_dx_dy - previous.direction_dx_dy
        )
        return float(
            candidate.mean_x_ratio
            - 0.10 * candidate.vertical_span_ratio
            - 0.05 * candidate.support_length_ratio
            + self.config.topology_continuity_weight
            * min(anchor_jump, 0.20)
            + self.config.topology_slope_continuity_weight
            * min(slope_jump, 1.0)
        )

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

    def _outer_yellow_reference(
        self,
        candidates: tuple[LineHypothesis, ...],
    ) -> LineHypothesis | None:
        """Return the deterministic Y2-like outer reference for the fork."""
        eligible = [
            item
            for item in candidates
            if (
                item.vertical_span_ratio
                >= self.config.branch_yellow_minimum_span_ratio
            )
        ]
        if not eligible:
            return None
        return max(
            eligible,
            key=lambda item: (
                item.mean_x_ratio,
                item.vertical_span_ratio,
                item.support_length_ratio,
            ),
        )

    @staticmethod
    def _right_of_reference_fraction(
        candidate: LineHypothesis,
        reference: LineHypothesis,
    ) -> float:
        overlap_low = max(
            candidate.minimum_y_ratio, reference.minimum_y_ratio
        )
        overlap_high = min(
            candidate.maximum_y_ratio, reference.maximum_y_ratio
        )
        if overlap_low > overlap_high:
            overlap_low = candidate.minimum_y_ratio
            overlap_high = candidate.maximum_y_ratio
        rows = np.linspace(float(overlap_low), float(overlap_high), 21)
        return float(
            np.mean(
                [
                    candidate.x_ratio_at(row)
                    >= reference.x_ratio_at(row)
                    for row in rows
                ]
            )
        )

    def _w2_is_inside_yellow(
        self,
        candidate: LineHypothesis,
        yellow: LineHypothesis,
    ) -> bool:
        right_fraction = self._right_of_reference_fraction(
            candidate, yellow
        )
        if right_fraction < 0.50:
            return True
        # A single straight fit is a poor left/right reference where the
        # outer yellow itself bends sharply.  Only that observable geometry
        # enables the fallback; no bag time or sequence index is involved.
        return bool(
            yellow.direction_dx_dy
            >= self.config.branch_yellow_curve_fallback_slope
        )

    @staticmethod
    def _branch_intersection_y_ratio(
        first: LineHypothesis,
        second: LineHypothesis,
    ) -> float | None:
        denominator = first.direction_dx_dy - second.direction_dx_dy
        if abs(denominator) < 1e-8:
            return None
        return float(
            (second.coefficients[1] - first.coefficients[1])
            / denominator
        )

    @staticmethod
    def _branch_tracking_cost(
        candidate: LineHypothesis,
        previous: LineHypothesis,
    ) -> float:
        return float(
            2.5 * _line_distance_ratio(candidate, previous)
            + 0.65
            * abs(
                candidate.direction_dx_dy - previous.direction_dx_dy
            )
            + 0.50
            * abs(candidate.mean_x_ratio - previous.mean_x_ratio)
        )

    def _w2_companion(
        self,
        white_candidates: tuple[LineHypothesis, ...],
        yellow_candidates: tuple[LineHypothesis, ...],
        w1: LineHypothesis,
    ) -> LineHypothesis | None:
        """Find W2 only when it forms a fork left of the outer yellow line."""
        yellow = self._outer_yellow_reference(yellow_candidates)
        if yellow is None:
            return None
        eligible: list[tuple[float, LineHypothesis]] = []
        for item in white_candidates:
            separation = item.mean_x_ratio - w1.mean_x_ratio
            if not (
                w1.mean_x_ratio <= item.mean_x_ratio + 0.035
                and item.direction_dx_dy
                <= self.config.branch_w2_maximum_slope
                and item.vertical_span_ratio
                >= self.config.branch_w2_minimum_span_ratio
                and self._w2_is_inside_yellow(item, yellow)
            ):
                continue
            join_distance = _branch_join_distance_ratio(w1, item)
            if (
                join_distance
                > self.config.branch_maximum_join_distance_ratio
            ):
                continue
            crossing_y = self._branch_intersection_y_ratio(w1, item)
            if crossing_y is None or not -0.20 <= crossing_y <= 1.35:
                continue
            # A real branch arm approaches W2 at (or just beyond) its near
            # observed endpoint.  A positive edge fragment cut out of the
            # thick W2 stripe crosses W2 inside its own observed span instead;
            # accepting it is the original W2-as-W1 failure mode.
            if (
                crossing_y
                < w1.maximum_y_ratio
                - self.config.branch_join_endpoint_tolerance_ratio
            ):
                continue
            score = (
                0.45
                * abs(
                    separation
                    - self.config.branch_expected_mean_separation_ratio
                )
                + 2.50 * join_distance
                + 0.12 * abs(crossing_y - 0.70)
                - 0.35
                * (w1.vertical_span_ratio + item.vertical_span_ratio)
                - 0.08
                * (w1.support_length_ratio + item.support_length_ratio)
            )
            eligible.append((float(score), item))
        if not eligible:
            return None
        return min(
            eligible,
            key=lambda scored: (
                scored[0],
                scored[1].mean_x_ratio,
                scored[1].direction_dx_dy,
            ),
        )[1]

    def _acquire_w1_w2_pair(
        self,
        white_candidates: tuple[LineHypothesis, ...],
        yellow_candidates: tuple[LineHypothesis, ...],
    ) -> tuple[LineHypothesis | None, LineHypothesis | None]:
        """Acquire the entry only from the observed W1-W2-Y2 topology.

        This is deliberately independent of timestamps and recorded mission
        state.  The same masks always produce the same pair: a sufficiently
        supported positive-heading W1 must join an opposite-heading W2, and
        both must lie on the left side of the outer yellow boundary.
        """
        outer_yellow = self._outer_yellow_reference(yellow_candidates)
        if outer_yellow is None:
            return None, None

        pairs: list[tuple[float, LineHypothesis, LineHypothesis]] = []
        for w1 in white_candidates:
            if not (
                w1.mean_x_ratio
                <= self.config.w1_acquisition_max_mean_x_ratio
                and w1.direction_dx_dy
                >= self.config.branch_w1_minimum_slope
                and w1.vertical_span_ratio
                >= self.config.branch_w1_minimum_span_ratio
            ):
                continue
            # Initial identity acquisition is deliberately stricter than
            # later tracking: both observed white branches must be on the
            # image-left side of the outer yellow boundary.  This prevents an
            # otherwise plausible fork outside the drivable corridor from
            # being assigned the persistent W1 identity.
            if self._right_of_reference_fraction(w1, outer_yellow) >= 0.50:
                continue
            w2 = self._w2_companion(
                white_candidates, yellow_candidates, w1
            )
            if w2 is None:
                continue
            if self._right_of_reference_fraction(w2, outer_yellow) >= 0.50:
                continue
            join_distance = _branch_join_distance_ratio(w1, w2)
            score = (
                2.50 * join_distance
                + 0.45
                * abs(
                    (w2.mean_x_ratio - w1.mean_x_ratio)
                    - self.config.branch_expected_mean_separation_ratio
                )
                + 0.18 * abs(w1.direction_dx_dy - 0.45)
                - 0.35
                * (w1.vertical_span_ratio + w2.vertical_span_ratio)
                - 0.08
                * (w1.support_length_ratio + w2.support_length_ratio)
            )
            pairs.append((float(score), w1, w2))
        if not pairs:
            return None, None
        _, w1, w2 = min(
            pairs,
            key=lambda scored: (
                scored[0],
                scored[1].mean_x_ratio,
                scored[2].mean_x_ratio,
            ),
        )
        return w1, w2

    def _track_w2(
        self,
        white_candidates: tuple[LineHypothesis, ...],
        yellow_candidates: tuple[LineHypothesis, ...],
        w1: LineHypothesis,
        previous: LineHypothesis | None,
    ) -> LineHypothesis | None:
        """Continue an acquired W2 without reapplying acquisition geometry."""
        if previous is None:
            return None
        yellow = self._outer_yellow_reference(yellow_candidates)
        if yellow is None:
            return None
        eligible = []
        for item in white_candidates:
            if not (
                item.direction_dx_dy
                <= self.config.branch_w2_tracking_maximum_slope
                and item.vertical_span_ratio
                >= self.config.branch_w2_tracking_minimum_span_ratio
                and self._w2_is_inside_yellow(item, yellow)
            ):
                continue
            if (
                self._branch_tracking_cost(item, previous)
                <= self.config.branch_w2_tracking_maximum_cost
            ):
                eligible.append(item)
        if not eligible:
            return None
        return min(
            eligible,
            key=lambda item: (
                self._branch_tracking_cost(item, previous)
                - 0.65 * item.vertical_span_ratio
                - 0.08 * item.support_length_ratio,
                item.mean_x_ratio,
            ),
        )

    def _track_entry_w1(
        self,
        candidates: tuple[LineHypothesis, ...],
        previous: LineHypothesis | None,
    ) -> LineHypothesis | None:
        """Track the positive W1 arm before the W1/Y1 pair rotates."""
        if previous is None:
            return None
        eligible = []
        for item in candidates:
            if not (
                item.mean_x_ratio <= 0.52
                and item.direction_dx_dy
                >= max(
                    self.config.white_branch_minimum_slope,
                    self.config.tracking_minimum_slope,
                )
                and item.vertical_span_ratio
                >= max(
                    self.config.branch_w1_minimum_span_ratio,
                    self.config.tracking_white_minimum_span_ratio,
                )
            ):
                continue
            if self._matches_previous(item, previous):
                eligible.append(item)
        if not eligible:
            return None
        # Replaying the same semantic mask must reproduce the same locked W1.
        # Returning the identical fresh hypothesis also prevents Y1 appearing
        # in another mask from perturbing an unchanged white observation.
        reproduced = next(
            (item for item in eligible if item == previous), None
        )
        if reproduced is not None:
            return reproduced
        return min(
            eligible,
            key=lambda item: (
                self._w1_tracking_score(item, previous),
                -item.vertical_span_ratio,
                -item.support_length_ratio,
                item.mean_x_ratio,
            ),
        )

    def _track_w1(
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
                item.mean_x_ratio
                <= self.config.tracking_white_max_mean_x_ratio
                and self.config.tracking_minimum_slope
                <= item.direction_dx_dy
                <= self.config.tracking_maximum_slope
                and item.vertical_span_ratio
                >= self.config.tracking_white_minimum_span_ratio
            )
        ]
        if not eligible:
            return None
        long_tracks = [
            item for item in eligible if item.vertical_span_ratio >= 0.15
        ]
        if long_tracks:
            eligible = long_tracks
        return min(
            eligible,
            key=lambda item: (
                self._branch_tracking_cost(item, previous)
                - 0.12 * item.vertical_span_ratio,
                item.mean_x_ratio,
            ),
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

    def _confirm_w1(self, candidate: LineHypothesis | None) -> None:
        if candidate is None:
            self.pending_w1 = None
            self.w1_confirmations = 0
            return
        if self.pending_w1 is not None and self._matches_previous(
            candidate, self.pending_w1
        ):
            self.w1_confirmations += 1
        else:
            self.pending_w1 = candidate
            self.w1_confirmations = 1
        if (
            self.w1_confirmations
            >= self.config.w1_acquisition_required_frames
        ):
            self.w1 = candidate
            self.phase = EntrySequencePhase.W1_LOCKED

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
    ) -> tuple[tuple[tuple[float, float], ...], bool, float]:
        if self.w1 is None:
            return (), False, 0.0
        w1 = self.w1
        # Steering geometry is derived exclusively from the locked/tracked W1.
        # The expected lane width contributes only a fixed lateral offset so
        # the controller follows the lane interior instead of the painted W1
        # itself.  Y1 remains a semantic phase/handoff observation and cannot
        # rotate or translate the entry steering path.
        synthetic_separation = self.config.expected_pair_separation_ratio
        top = w1.minimum_y_ratio
        bottom = w1.maximum_y_ratio
        rows_ratio = np.linspace(
            bottom, top, int(self.config.path_sample_count)
        )
        points = []
        for row_ratio in rows_ratio:
            white_x = w1.x_ratio_at(float(row_ratio))
            synthetic_yellow_x = white_x + synthetic_separation
            points.append(
                (
                    float(
                        (
                            self.config.w1_path_weight * white_x
                            + (1.0 - self.config.w1_path_weight)
                            * synthetic_yellow_x
                        )
                        * width
                    ),
                    float(row_ratio * height),
                )
            )
        if len(points) < 3:
            return (), True, float(synthetic_separation)
        return tuple(points), True, float(synthetic_separation)

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

        if self.phase == EntrySequencePhase.LEFT4_ARMED:
            acquired_w1, acquired_w2 = self._acquire_w1_w2_pair(
                whites, yellows
            )
            self._confirm_w1(acquired_w1)
            if self.w1 is not None:
                self.w2 = acquired_w2
        else:
            # Until Y1 establishes the fork pair, position continuity alone is
            # ambiguous: as the vehicle turns, W1 moves sharply vehicle-left
            # while the old W2 can remain closer to W1's previous x.  The 55
            # hand-labelled frames show that W1 keeps its positive acquisition
            # heading throughout this interval whereas W2 keeps the opposite
            # heading.  Re-apply that *relative branch identity* (not an
            # absolute pixel) until Y1 is locked.  Afterwards W1/Y1 may both
            # rotate through zero, so tracking deliberately returns to temporal
            # polyline continuity and does not reapply the positive-heading
            # gate.
            if self.y1 is None:
                tracked_w1 = self._track_entry_w1(whites, self.w1)
            else:
                tracked_w1 = self._track_w1(whites, self.w1)
            if tracked_w1 is not None:
                self.w1 = tracked_w1

        if self.w1 is not None:
            previous_w2 = self.w2
            tracked_w2 = self._track_w2(
                whites, yellows, self.w1, previous_w2
            )
            self.w2 = (
                tracked_w2
                if tracked_w2 is not None
                else (
                    self._w2_companion(whites, yellows, self.w1)
                    if previous_w2 is not None
                    else None
                )
            )
        else:
            self.w2 = None

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

        # Once acquired, W1 is the persistent steering reference.  If a later
        # semantic frame has no matching white candidate, keep using the last
        # fitted W1 indefinitely instead of invalidating the path by a dropout
        # frame count.  A newly matched candidate still updates the fit above.
        w1_usable = self.w1 is not None
        y1_usable = bool(
            self.y1 is not None
            and self.y1_missing_frames <= self.config.missing_hold_frames
        )
        if not w1_usable:
            path = ()
            synthetic = False
            separation = 0.0
            reason = "W1 unavailable; safe stop"
        else:
            # A short Y1 dropout keeps the last locked geometry.  Before Y1 is
            # ever acquired, the path is offset from W1 by the annotated lane
            # width distribution; the visible Y2 is never substituted.
            path, synthetic, separation = self._path(
                width=width, height=height
            )
            if not path:
                reason = "W1/Y1 geometry invalid; safe stop"
            else:
                reason = (
                    "W1-only steering; fixed lane-width offset; "
                    "W2/Y2 ignored; Y1 excluded from control path"
                )

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
        path_valid = bool(path and ready and w1_usable)
        return SequenceEntryResult(
            phase=self.phase,
            ready=ready,
            path_valid=path_valid,
            cruise_handoff=self.phase == EntrySequencePhase.CRUISE_HANDOFF,
            reason=(
                reason
                if ready
                else "left_4 armed; searching for W1/W2 fork left of yellow"
            ),
            white_candidates=whites,
            yellow_candidates=yellows,
            w1=self.w1 if w1_usable else None,
            w2=self.w2 if w1_usable else None,
            y1=self.y1 if y1_usable else None,
            path_pixels=path,
            used_synthetic_y1=bool(synthetic),
            pair_separation_ratio=float(separation),
        )


def render_sequence_debug(
    white_mask: np.ndarray,
    yellow_mask: np.ndarray,
    result: SequenceEntryResult,
    *,
    show_candidates: bool = True,
    show_status: bool = True,
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
        label: str | None = None,
        label_position: str = "middle",
    ):
        y1 = int(round(item.minimum_y_ratio * height))
        y2 = int(round(item.maximum_y_ratio * height))
        x1 = int(round(item.x_ratio_at(item.minimum_y_ratio) * width))
        x2 = int(round(item.x_ratio_at(item.maximum_y_ratio) * width))
        cv2.line(output, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)
        if label is not None:
            if label_position == "far":
                label_x, label_y = x1 + 4, y1 - 4
            elif label_position == "near":
                label_x, label_y = x2 + 4, y2 + 12
            else:
                label_x = (x1 + x2) // 2 + 4
                label_y = (y1 + y2) // 2
            anchor = (
                max(2, min(width - 30, label_x)),
                max(12, min(height - 3, label_y)),
            )
            # A dark outline keeps labels readable without a large box that
            # would hide the nearby fork pixels.
            cv2.putText(
                output,
                label,
                anchor,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (12, 12, 12),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                output,
                label,
                anchor,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                color,
                1,
                cv2.LINE_AA,
            )

    if show_candidates:
        for item in result.white_candidates:
            draw_line(item, (120, 90, 90), 1)
        for item in result.yellow_candidates:
            draw_line(item, (40, 100, 120), 1)
    if result.w1 is not None:
        draw_line(result.w1, (255, 0, 255), 3, "W1", "far")
    if result.w2 is not None:
        draw_line(result.w2, (255, 150, 40), 3, "W2", "near")
    if result.y1 is not None:
        draw_line(result.y1, (0, 255, 80), 3, "Y1", "middle")
    if len(result.path_pixels) >= 2:
        pixels = np.asarray(result.path_pixels, dtype=np.int32)
        cv2.polylines(output, [pixels], False, (255, 100, 0), 3, cv2.LINE_AA)

    cv2.line(
        output,
        (width // 2, height - 1),
        (width // 2, max(0, height // 4)),
        (78, 78, 78),
        1,
        cv2.LINE_AA,
    )
    if show_status:
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
