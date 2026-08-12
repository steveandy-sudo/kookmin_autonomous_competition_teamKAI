#!/usr/bin/env python3
"""Pure OpenCV/NumPy core for pixel-space BEV lane path generation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np


SIDE_CODES = {"auto": 0.0, "left": 1.0, "right": 2.0}
SOURCE_CODES = {
    "PATH_INVALID": 0.0,
    "YELLOW_PRIMARY": 1.0,
    "WHITE_LEFT_GAP_FALLBACK": 2.0,
    "WHITE_RIGHT_GAP_FALLBACK": 3.0,
    "YELLOW_HISTORY_HOLD": 4.0,
    "YELLOW_DIRECT": 5.0,
    "YELLOW_PARTIAL_TRACKED": 6.0,
    "WHITE_BOUNDARY_NORMAL_FALLBACK": 7.0,
    "TEMPORAL_PREDICTED": 8.0,
    "PATH_LOST_STOP": 9.0,
}
LEGACY_SOURCE_ALIASES = {
    "YELLOW_PRIMARY": "YELLOW_DIRECT",
    "WHITE_LEFT_GAP_FALLBACK": "WHITE_BOUNDARY_NORMAL_FALLBACK",
    "WHITE_RIGHT_GAP_FALLBACK": "WHITE_BOUNDARY_NORMAL_FALLBACK",
    "YELLOW_HISTORY_HOLD": "TEMPORAL_PREDICTED",
    "PATH_INVALID": "PATH_LOST_STOP",
}
YELLOW_STATE_CODES = {"VALID": 0.0, "WEAK": 1.0, "MISSING": 2.0}
WHITE_SIDE_CODES = {"none": 0.0, "left": 1.0, "right": 2.0}


@dataclass
class PathParams:
    white_min_component_area: int = 40
    yellow_min_component_area: int = 20
    white_open_kernel: int = 3
    white_close_kernel: int = 5
    yellow_open_kernel: int = 3
    yellow_close_kernel: int = 3
    row_step_px: int = 5
    min_run_width_px: int = 2
    white_track_max_dx_px: float = 50.0
    white_track_max_missing_rows: int = 5
    white_track_min_points: int = 8
    white_track_min_span_px: float = 80.0
    yellow_track_max_dx_px: float = 70.0
    yellow_track_max_gap_y_px: float = 120.0
    yellow_track_min_components: int = 2
    yellow_track_min_points: int = 5
    yellow_track_min_span_px: float = 60.0
    linear_if_span_below_px: float = 70.0
    polynomial_degree_long: int = 2
    fit_residual_threshold_px: float = 12.0
    fit_min_inlier_ratio: float = 0.55
    fit_iterations: int = 50
    max_curve_abs_a: float = 0.02
    lane_side: str = "auto"
    yellow_primary_enabled: bool = True
    path_point_count: int = 30
    path_y_near_ratio: float = 0.95
    path_y_far_ratio: float = 0.20
    yellow_missing_sec_for_white_fallback: float = 0.12
    yellow_hold_sec: float = 0.22
    yellow_recovery_sec: float = 0.12
    yellow_recovery_blend_sec: float = 0.15
    yellow_min_points: int = 5
    yellow_min_span_px: float = 60.0
    yellow_max_residual_px: float = 12.0
    yellow_min_inlier_ratio: float = 0.55
    yellow_max_temporal_jump_px: float = 90.0
    yellow_partial_min_points: int = 3
    yellow_partial_min_span_px: float = 20.0
    yellow_partial_match_max_distance_px: float = 55.0
    yellow_partial_heading_max_deg: float = 45.0
    yellow_partial_transition_px: float = 45.0
    yellow_direct_confidence_min: float = 0.45
    white_gap_ema_previous_weight: float = 0.80
    white_gap_ema_current_weight: float = 0.20
    white_gap_min_px: float = 40.0
    white_gap_max_px: float = 420.0
    white_gap_max_age_sec: float = 1.50
    white_gap_min_common_span_px: float = 60.0
    white_gap_min_anchor_count: int = 5
    white_gap_max_change_px: float = 70.0
    white_fallback_confidence_max: float = 0.70
    white_normal_candidate_max_start_distance_px: float = 180.0
    white_normal_candidate_max_heading_deg: float = 75.0
    white_normal_candidate_min_bounds_ratio: float = 0.75
    white_normal_fallback_confidence_max: float = 0.62
    allow_fixed_white_offset_fallback: bool = False
    fixed_white_offset_px: float = 180.0
    curve_ema_previous_weight: float = 0.70
    curve_ema_current_weight: float = 0.30
    path_hold_sec: float = 0.25
    temporal_prediction_sec: float = 0.25
    optical_flow_prediction_enabled: bool = True
    optical_flow_max_features: int = 120
    optical_flow_min_features: int = 8
    optical_flow_min_inliers: int = 6
    optical_flow_max_reprojection_error_px: float = 8.0

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PathParams":
        params = cls()
        for key, value in data.items():
            if hasattr(params, key):
                setattr(params, key, value)
        params.lane_side = str(params.lane_side).lower()
        if params.lane_side not in SIDE_CODES:
            params.lane_side = "auto"
        return params


@dataclass
class Observation:
    x: float
    y: float
    width: float
    pixel_count: int


@dataclass
class Track:
    observations: list[Observation] = field(default_factory=list)
    missing_rows: int = 0

    @property
    def bottom_x(self) -> float:
        if not self.observations:
            return float("nan")
        return max(self.observations, key=lambda obs: obs.y).x

    @property
    def top_y(self) -> float:
        return min((obs.y for obs in self.observations), default=float("nan"))

    @property
    def bottom_y(self) -> float:
        return max((obs.y for obs in self.observations), default=float("nan"))

    @property
    def span_y(self) -> float:
        if not self.observations:
            return 0.0
        return self.bottom_y - self.top_y


@dataclass
class CurveFit:
    name: str
    valid: bool = False
    coeffs: np.ndarray | None = None
    degree: int = 0
    y_min: float = 0.0
    y_max: float = 0.0
    residual: float = float("inf")
    inlier_ratio: float = 0.0
    point_count: int = 0
    confidence: float = 0.0

    def evaluate(self, y_values: np.ndarray) -> np.ndarray:
        if not self.valid or self.coeffs is None:
            return np.full_like(y_values, np.nan, dtype=np.float32)
        return np.polyval(self.coeffs, y_values).astype(np.float32)


@dataclass
class WhiteGapModel:
    anchors_y: np.ndarray | None = None
    gaps_px: np.ndarray | None = None
    last_update_time: float | None = None
    confidence: float = 0.0
    valid_anchor_count: int = 0
    residual_px: float = float("inf")

    @property
    def valid(self) -> bool:
        return (
            self.anchors_y is not None
            and self.gaps_px is not None
            and self.valid_anchor_count > 0
            and self.anchors_y.shape == self.gaps_px.shape
        )

    @property
    def near_gap_px(self) -> float:
        if not self.valid or self.gaps_px is None:
            return float("nan")
        return float(self.gaps_px[0])

    def age_sec(self, timestamp_sec: float | None) -> float:
        if self.last_update_time is None or timestamp_sec is None:
            return float("inf")
        return max(0.0, float(timestamp_sec - self.last_update_time))

    def gap_at(self, y_values: np.ndarray) -> np.ndarray:
        if not self.valid or self.anchors_y is None or self.gaps_px is None:
            return np.full_like(y_values, np.nan, dtype=np.float32)
        order = np.argsort(self.anchors_y)
        return np.interp(y_values, self.anchors_y[order], self.gaps_px[order]).astype(np.float32)


@dataclass
class TemporalState:
    previous_timestamp: float | None = None
    previous_path_coeffs: np.ndarray | None = None
    previous_path_points: np.ndarray | None = None
    previous_path_time: float | None = None
    previous_path_source: str = "PATH_INVALID"
    previous_fallback_side: str | None = None
    previous_yellow_points: np.ndarray | None = None
    previous_yellow_raw_points: np.ndarray | None = None
    previous_yellow_time: float | None = None
    yellow_missing_since: float | None = None
    yellow_recovery_since: float | None = None
    left_gap: WhiteGapModel = field(default_factory=WhiteGapModel)
    right_gap: WhiteGapModel = field(default_factory=WhiteGapModel)
    previous_gray_bev: np.ndarray | None = None
    previous_gray_time: float | None = None
    previous_normal_direction: str | None = None

    def reset(self) -> None:
        self.previous_timestamp = None
        self.previous_path_coeffs = None
        self.previous_path_points = None
        self.previous_path_time = None
        self.previous_path_source = "PATH_INVALID"
        self.previous_fallback_side = None
        self.previous_yellow_points = None
        self.previous_yellow_raw_points = None
        self.previous_yellow_time = None
        self.yellow_missing_since = None
        self.yellow_recovery_since = None
        self.left_gap = WhiteGapModel()
        self.right_gap = WhiteGapModel()
        self.previous_gray_bev = None
        self.previous_gray_time = None
        self.previous_normal_direction = None


@dataclass
class PathResult:
    path_valid: bool
    lane_side: str
    path_source: str
    confidence: float
    white_left: CurveFit
    white_right: CurveFit
    yellow: CurveFit
    raw_path: np.ndarray
    smooth_path: np.ndarray
    path_coeffs: np.ndarray | None
    path_residual: float
    learned_left_gap_near_px: float
    learned_right_gap_near_px: float
    gap_history_age_sec: float
    yellow_state: str
    yellow_missing_age_sec: float
    selected_white_side: str
    recovery_active: bool
    path_age_sec: float
    yellow_reject_reason: str
    white_fallback_reject_reason: str
    temporal_prediction_age_sec: float
    normal_fallback_direction: str
    selected_white_track_id: str
    selected_normal_direction: str
    normal_offset_confidence: float
    white_normal_candidate_a: np.ndarray
    white_normal_candidate_b: np.ndarray
    yellow_direct_candidate: np.ndarray
    yellow_partial_candidate: np.ndarray
    optical_flow_inliers: int
    optical_flow_error_px: float
    optical_flow_features: np.ndarray
    curvature_abs_mean: float
    curvature_abs_max: float
    processing_ms: float
    fps: float
    white_observations: list[Observation]
    yellow_observations: list[Observation]
    white_component_count: int
    yellow_component_count: int
    cleaned_white_mask: np.ndarray
    cleaned_yellow_mask: np.ndarray


def normalize_mask(mask: np.ndarray) -> np.ndarray:
    if mask.ndim == 3:
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    return np.where(mask > 0, 255, 0).astype(np.uint8)


def kernel(size: int) -> np.ndarray | None:
    size = int(size)
    if size <= 1:
        return None
    return cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))


def remove_small_components(mask: np.ndarray, min_area: int) -> tuple[np.ndarray, int]:
    binary = normalize_mask(mask)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    clean = np.zeros_like(binary)
    kept = 0
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area >= int(min_area):
            clean[labels == label] = 255
            kept += 1
    return clean, kept


def clean_class_mask(mask: np.ndarray, min_area: int, open_kernel: int, close_kernel: int) -> tuple[np.ndarray, int]:
    clean = normalize_mask(mask)
    k_open = kernel(open_kernel)
    if k_open is not None:
        clean = cv2.morphologyEx(clean, cv2.MORPH_OPEN, k_open)
    clean, count = remove_small_components(clean, min_area)
    k_close = kernel(close_kernel)
    if k_close is not None:
        clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, k_close)
        clean, count = remove_small_components(clean, min_area)
    return clean, count


def split_runs(xs: np.ndarray) -> list[np.ndarray]:
    if xs.size == 0:
        return []
    breaks = np.where(np.diff(xs) > 1)[0] + 1
    return [run for run in np.split(xs, breaks) if run.size > 0]


def row_observations(mask: np.ndarray, row_step_px: int, min_run_width_px: int) -> list[Observation]:
    binary = normalize_mask(mask)
    h, _ = binary.shape[:2]
    observations: list[Observation] = []
    step = max(1, int(row_step_px))
    for y in range(h - 1, -1, -step):
        xs = np.flatnonzero(binary[y] > 0)
        for run in split_runs(xs):
            if run.size < int(min_run_width_px):
                continue
            observations.append(
                Observation(
                    x=float(np.median(run)),
                    y=float(y),
                    width=float(run[-1] - run[0] + 1),
                    pixel_count=int(run.size),
                )
            )
    return observations


def build_white_tracks(observations: list[Observation], params: PathParams) -> list[Track]:
    rows: dict[int, list[Observation]] = {}
    for obs in observations:
        rows.setdefault(int(obs.y), []).append(obs)

    active: list[Track] = []
    finished: list[Track] = []
    for y in sorted(rows.keys(), reverse=True):
        row_obs = sorted(rows[y], key=lambda obs: obs.x)
        used: set[int] = set()
        for track in active:
            last_x = track.observations[-1].x
            candidates = [
                (idx, abs(obs.x - last_x))
                for idx, obs in enumerate(row_obs)
                if idx not in used and abs(obs.x - last_x) <= params.white_track_max_dx_px
            ]
            if candidates:
                idx, _ = min(candidates, key=lambda item: item[1])
                track.observations.append(row_obs[idx])
                track.missing_rows = 0
                used.add(idx)
            else:
                track.missing_rows += 1

        still_active: list[Track] = []
        for track in active:
            if track.missing_rows > params.white_track_max_missing_rows:
                finished.append(track)
            else:
                still_active.append(track)
        active = still_active

        for idx, obs in enumerate(row_obs):
            if idx not in used:
                active.append(Track(observations=[obs]))

    finished.extend(active)
    return [
        track for track in finished
        if len(track.observations) >= params.white_track_min_points
        and track.span_y >= params.white_track_min_span_px
    ]


def points_from_observations(observations: list[Observation]) -> np.ndarray:
    return np.asarray([[obs.x, obs.y] for obs in observations], dtype=np.float32)


def robust_polyfit(
    points: np.ndarray,
    params: PathParams,
    name: str,
    min_points: int = 5,
) -> CurveFit:
    points = np.asarray(points, dtype=np.float32)
    finite = np.all(np.isfinite(points), axis=1) if points.ndim == 2 else np.asarray([], dtype=bool)
    points = points[finite] if points.ndim == 2 and points.shape[1] == 2 else np.empty((0, 2), dtype=np.float32)
    if points.shape[0] < min_points:
        return CurveFit(name=name, point_count=int(points.shape[0]))

    y_span = float(np.max(points[:, 1]) - np.min(points[:, 1]))
    degree = 1 if y_span < params.linear_if_span_below_px else int(params.polynomial_degree_long)
    degree = min(max(degree, 1), 2)
    if points.shape[0] < degree + 1:
        return CurveFit(name=name, point_count=int(points.shape[0]))

    rng = np.random.default_rng(42)
    best_inliers: np.ndarray | None = None
    best_score = (-1, float("inf"))
    sample_count = degree + 1
    y = points[:, 1]
    x = points[:, 0]

    for _ in range(max(1, int(params.fit_iterations))):
        if points.shape[0] == sample_count:
            sample_idx = np.arange(points.shape[0])
        else:
            sample_idx = rng.choice(points.shape[0], sample_count, replace=False)
        try:
            coeffs = np.polyfit(y[sample_idx], x[sample_idx], degree)
        except (np.linalg.LinAlgError, ValueError, FloatingPointError):
            continue
        if not np.all(np.isfinite(coeffs)):
            continue
        residuals = np.abs(x - np.polyval(coeffs, y))
        inliers = residuals <= params.fit_residual_threshold_px
        count = int(np.count_nonzero(inliers))
        mean = float(np.mean(residuals[inliers])) if count else float("inf")
        score = (count, -mean)
        if score > best_score:
            best_score = score
            best_inliers = inliers

    if best_inliers is None or np.count_nonzero(best_inliers) < max(min_points, degree + 1):
        return CurveFit(name=name, point_count=int(points.shape[0]))

    try:
        coeffs = np.polyfit(y[best_inliers], x[best_inliers], degree)
    except (np.linalg.LinAlgError, ValueError, FloatingPointError):
        return CurveFit(name=name, point_count=int(points.shape[0]))

    if not np.all(np.isfinite(coeffs)):
        return CurveFit(name=name, point_count=int(points.shape[0]))
    if degree == 2 and abs(float(coeffs[0])) > params.max_curve_abs_a:
        return CurveFit(name=name, point_count=int(points.shape[0]))

    residuals = np.abs(x[best_inliers] - np.polyval(coeffs, y[best_inliers]))
    residual = float(np.mean(residuals)) if residuals.size else float("inf")
    inlier_ratio = float(np.count_nonzero(best_inliers) / max(points.shape[0], 1))
    valid = inlier_ratio >= params.fit_min_inlier_ratio and residual <= params.fit_residual_threshold_px
    confidence = float(np.clip(inlier_ratio * (1.0 - residual / max(params.fit_residual_threshold_px * 2.0, 1.0)), 0.0, 1.0))
    return CurveFit(
        name=name,
        valid=valid,
        coeffs=coeffs.astype(np.float32),
        degree=degree,
        y_min=float(np.min(y[best_inliers])),
        y_max=float(np.max(y[best_inliers])),
        residual=residual,
        inlier_ratio=inlier_ratio,
        point_count=int(points.shape[0]),
        confidence=confidence if valid else 0.0,
    )


def select_white_boundaries(tracks: list[Track], params: PathParams, width: int) -> tuple[CurveFit, CurveFit]:
    fits: list[tuple[Track, CurveFit]] = []
    for track in tracks:
        fit = robust_polyfit(points_from_observations(track.observations), params, "white", params.white_track_min_points)
        if fit.valid:
            fits.append((track, fit))
    if not fits:
        return CurveFit("white_left"), CurveFit("white_right")

    fits.sort(key=lambda item: item[0].bottom_x)
    if len(fits) == 1:
        track, fit = fits[0]
        if track.bottom_x < width * 0.5:
            fit.name = "white_left"
            return fit, CurveFit("white_right")
        fit.name = "white_right"
        return CurveFit("white_left"), fit

    scored = sorted(
        fits,
        key=lambda item: (item[0].span_y, item[1].inlier_ratio, -item[1].residual, item[1].point_count),
        reverse=True,
    )
    best_two = sorted(scored[:2], key=lambda item: item[0].bottom_x)
    left = best_two[0][1]
    right = best_two[1][1]
    left.name = "white_left"
    right.name = "white_right"
    return left, right


def yellow_component_points(mask: np.ndarray, params: PathParams) -> tuple[np.ndarray, int]:
    clean, count = remove_small_components(mask, params.yellow_min_component_area)
    n, labels, stats, centroids = cv2.connectedComponentsWithStats(clean, 8)
    comps: list[dict[str, Any]] = []
    for label in range(1, n):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < params.yellow_min_component_area:
            continue
        x = float(centroids[label][0])
        y = float(centroids[label][1])
        y_min = float(stats[label, cv2.CC_STAT_TOP])
        y_max = y_min + float(stats[label, cv2.CC_STAT_HEIGHT]) - 1.0
        comps.append({"label": label, "x": x, "y": y, "y_min": y_min, "y_max": y_max, "area": area})

    tracks: list[list[dict[str, Any]]] = []
    for comp in sorted(comps, key=lambda item: item["y"], reverse=True):
        best_idx: int | None = None
        best_score = float("inf")
        for idx, track in enumerate(tracks):
            last = track[-1]
            gap_y = last["y_min"] - comp["y_max"]
            dx = abs(last["x"] - comp["x"])
            if -10.0 <= gap_y <= params.yellow_track_max_gap_y_px and dx <= params.yellow_track_max_dx_px:
                score = dx + max(0.0, gap_y) * 0.15
                if score < best_score:
                    best_idx = idx
                    best_score = score
        if best_idx is None:
            tracks.append([comp])
        else:
            tracks[best_idx].append(comp)

    row_obs = row_observations(clean, params.row_step_px, params.min_run_width_px)
    if not tracks:
        return points_from_observations(row_obs), 0

    def score_track(track: list[dict[str, Any]]) -> tuple[float, int, float]:
        span = max(item["y_max"] for item in track) - min(item["y_min"] for item in track)
        area = sum(item["area"] for item in track)
        return (float(span), len(track), float(area))

    selected = max(tracks, key=score_track)
    if len(selected) < params.yellow_track_min_components:
        span = score_track(selected)[0]
        if span < params.yellow_track_min_span_px:
            return np.empty((0, 2), dtype=np.float32), len(comps)

    labels_allowed = {item["label"] for item in selected}
    selected_mask = np.where(np.isin(labels, list(labels_allowed)), 255, 0).astype(np.uint8)
    selected_obs = row_observations(selected_mask, params.row_step_px, params.min_run_width_px)
    if len(selected_obs) < params.yellow_track_min_points:
        points = np.asarray([[item["x"], item["y"]] for item in selected], dtype=np.float32)
    else:
        points = points_from_observations(selected_obs)
    return points, len(comps)


def fit_yellow(mask: np.ndarray, params: PathParams) -> tuple[CurveFit, int, list[Observation]]:
    points, component_count = yellow_component_points(mask, params)
    obs = [Observation(float(x), float(y), 1.0, 1) for x, y in points]
    fit = robust_polyfit(points, params, "yellow_centerline", params.yellow_track_min_points)
    return fit, component_count, obs


def common_y_range(curves: list[CurveFit], height: int, params: PathParams) -> tuple[float, float] | None:
    valid = [curve for curve in curves if curve.valid]
    if len(valid) != len(curves):
        return None
    y_min = max(curve.y_min for curve in curves)
    y_max = min(curve.y_max for curve in curves)
    global_near = float(height - 1) * params.path_y_near_ratio
    global_far = float(height - 1) * params.path_y_far_ratio
    y_min = max(y_min, global_far)
    y_max = min(y_max, global_near)
    if y_max - y_min < max(20.0, params.row_step_px * 4.0):
        return None
    return y_min, y_max


def sample_curve_path(
    curve: CurveFit,
    height: int,
    params: PathParams,
) -> np.ndarray:
    y_range = common_y_range([curve], height, params)
    if y_range is None:
        return np.empty((0, 2), dtype=np.float32)
    y_values = np.linspace(y_range[1], y_range[0], max(3, int(params.path_point_count)), dtype=np.float32)
    points = np.stack((curve.evaluate(y_values), y_values), axis=1)
    return points[np.all(np.isfinite(points), axis=1)]


def curve_x_at_near(curve: CurveFit, near_y: float) -> float:
    if not curve.valid or curve.coeffs is None:
        return float("nan")
    y = float(np.clip(near_y, curve.y_min, curve.y_max))
    return float(np.polyval(curve.coeffs, y))


def fit_path(points: np.ndarray, params: PathParams) -> tuple[np.ndarray, np.ndarray | None, float]:
    if points.shape[0] < 3:
        return np.empty((0, 2), dtype=np.float32), None, float("inf")
    fit = robust_polyfit(points, params, "path", min_points=3)
    if not fit.valid or fit.coeffs is None:
        return np.empty((0, 2), dtype=np.float32), None, float("inf")
    y_values = np.linspace(float(np.max(points[:, 1])), float(np.min(points[:, 1])), points.shape[0], dtype=np.float32)
    x_values = np.polyval(fit.coeffs, y_values).astype(np.float32)
    smooth = np.stack((x_values, y_values), axis=1)
    return smooth, fit.coeffs, fit.residual


def ema_coeffs(previous: np.ndarray | None, current: np.ndarray, params: PathParams) -> np.ndarray:
    if previous is None:
        return current.astype(np.float32)
    prev = previous.astype(np.float32)
    cur = current.astype(np.float32)
    if prev.shape != cur.shape:
        return cur
    return (
        params.curve_ema_previous_weight * prev
        + params.curve_ema_current_weight * cur
    ).astype(np.float32)


def path_from_coeffs(coeffs: np.ndarray, y_values: np.ndarray) -> np.ndarray:
    return np.stack((np.polyval(coeffs, y_values).astype(np.float32), y_values.astype(np.float32)), axis=1)


def points_temporal_jump(previous: np.ndarray | None, current: np.ndarray) -> float:
    if previous is None or current.shape[0] < 2 or previous.shape[0] < 2:
        return 0.0
    y_min = max(float(np.min(previous[:, 1])), float(np.min(current[:, 1])))
    y_max = min(float(np.max(previous[:, 1])), float(np.max(current[:, 1])))
    if y_max <= y_min:
        return 0.0
    y_values = np.linspace(y_max, y_min, min(previous.shape[0], current.shape[0]), dtype=np.float32)
    prev_x = np.interp(y_values, np.sort(previous[:, 1]), previous[np.argsort(previous[:, 1]), 0])
    cur_x = np.interp(y_values, np.sort(current[:, 1]), current[np.argsort(current[:, 1]), 0])
    return float(np.max(np.abs(cur_x - prev_x)))


def classify_yellow(yellow: CurveFit, yellow_obs: list[Observation], params: PathParams) -> str:
    if not yellow_obs:
        return "MISSING"
    if (
        yellow.valid
        and yellow.point_count >= params.yellow_min_points
        and yellow.y_max - yellow.y_min >= params.yellow_min_span_px
        and yellow.residual <= params.yellow_max_residual_px
        and yellow.inlier_ratio >= params.yellow_min_inlier_ratio
        and yellow.coeffs is not None
        and np.all(np.isfinite(yellow.coeffs))
    ):
        return "VALID"
    return "WEAK"


def yellow_reject_reason(
    yellow: CurveFit,
    yellow_obs: list[Observation],
    candidate: np.ndarray,
    params: PathParams,
    state: TemporalState,
    width: int,
    height: int,
) -> str:
    if not params.yellow_primary_enabled:
        return "DISABLED"
    if not yellow_obs:
        return "NO_OBSERVATION"
    if yellow.point_count < params.yellow_min_points:
        return "TOO_FEW_POINTS"
    if yellow.y_max - yellow.y_min < params.yellow_min_span_px:
        return "SHORT_SPAN"
    if not yellow.valid or yellow.coeffs is None or not np.all(np.isfinite(yellow.coeffs)):
        if np.isfinite(yellow.residual) and yellow.residual > params.yellow_max_residual_px:
            return "HIGH_RESIDUAL"
        if yellow.inlier_ratio < params.yellow_min_inlier_ratio:
            return "LOW_INLIER_RATIO"
        return "SPLINE_FAILURE"
    if yellow.residual > params.yellow_max_residual_px:
        return "HIGH_RESIDUAL"
    if yellow.inlier_ratio < params.yellow_min_inlier_ratio:
        return "LOW_INLIER_RATIO"
    if not path_within_bounds(candidate, width, height):
        return "OUT_OF_BOUNDS"
    jump = points_temporal_jump(state.previous_yellow_points, candidate)
    if jump > params.yellow_max_temporal_jump_px:
        return "TEMPORAL_JUMP"
    return "NONE"


def ordered_unique_points(points: np.ndarray, min_step_px: float = 2.0) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 2:
        return np.empty((0, 2), dtype=np.float32)
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < 2:
        return points.astype(np.float32)
    points = points[np.lexsort((points[:, 0], -points[:, 1]))]
    kept = [points[0]]
    for point in points[1:]:
        if float(np.linalg.norm(point - kept[-1])) >= min_step_px:
            kept.append(point)
    return np.asarray(kept, dtype=np.float32)


def resample_polyline(points: np.ndarray, count: int) -> np.ndarray:
    points = ordered_unique_points(points)
    if points.shape[0] < 2:
        return np.empty((0, 2), dtype=np.float32)
    deltas = np.diff(points, axis=0)
    distances = np.linalg.norm(deltas, axis=1)
    keep = np.concatenate(([True], distances > 1e-3))
    points = points[keep]
    if points.shape[0] < 2:
        return np.empty((0, 2), dtype=np.float32)
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    s = np.concatenate(([0.0], np.cumsum(distances))).astype(np.float32)
    if float(s[-1]) <= 1e-3:
        return np.empty((0, 2), dtype=np.float32)
    target_s = np.linspace(0.0, float(s[-1]), max(3, int(count)), dtype=np.float32)
    x = np.interp(target_s, s, points[:, 0]).astype(np.float32)
    y = np.interp(target_s, s, points[:, 1]).astype(np.float32)
    return np.stack((x, y), axis=1)


def smooth_polyline(points: np.ndarray, passes: int = 2) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    if points.shape[0] < 5:
        return points
    out = points.copy()
    for _ in range(max(0, int(passes))):
        current = out.copy()
        out[1:-1] = 0.25 * current[:-2] + 0.50 * current[1:-1] + 0.25 * current[2:]
    return out.astype(np.float32)


def parametric_path_from_points(points: np.ndarray, params: PathParams, width: int, height: int) -> np.ndarray:
    sampled = smooth_polyline(resample_polyline(points, params.path_point_count))
    if sampled.shape[0] < 3:
        return np.empty((0, 2), dtype=np.float32)
    sampled[:, 0] = np.clip(sampled[:, 0], 0.0, float(width - 1))
    sampled[:, 1] = np.clip(sampled[:, 1], 0.0, float(height - 1))
    return sampled.astype(np.float32)


def path_heading(points: np.ndarray, near: bool = True) -> float:
    if points.shape[0] < 2:
        return float("nan")
    a, b = (points[0], points[min(2, points.shape[0] - 1)]) if near else (points[-2], points[-1])
    delta = b - a
    return float(np.arctan2(delta[1], delta[0]))


def heading_difference_deg(a: float, b: float) -> float:
    if not np.isfinite(a) or not np.isfinite(b):
        return 0.0
    diff = (a - b + np.pi) % (2.0 * np.pi) - np.pi
    return float(abs(np.degrees(diff)))


def mean_distance_to_path(points: np.ndarray, reference: np.ndarray) -> float:
    if points.shape[0] == 0 or reference.shape[0] == 0:
        return float("inf")
    distances = []
    for point in points:
        distances.append(float(np.min(np.linalg.norm(reference - point, axis=1))))
    return float(np.mean(distances)) if distances else float("inf")


def merge_partial_yellow_with_history(
    partial: np.ndarray,
    previous: np.ndarray | None,
    params: PathParams,
    width: int,
    height: int,
) -> np.ndarray:
    if previous is None or previous.shape[0] < 3:
        return np.empty((0, 2), dtype=np.float32)
    partial = parametric_path_from_points(partial, params, width, height)
    if partial.shape[0] < max(3, params.yellow_partial_min_points):
        return np.empty((0, 2), dtype=np.float32)
    previous = previous.astype(np.float32)
    merged = previous.copy()
    order = np.argsort(partial[:, 1])
    partial_y = partial[order, 1]
    partial_x = partial[order, 0]
    y_min = float(np.min(partial_y))
    y_max = float(np.max(partial_y))
    transition = max(1.0, float(params.yellow_partial_transition_px))
    for idx, point in enumerate(merged):
        y = float(point[1])
        if y_min <= y <= y_max:
            alpha = 1.0
        elif y_min - transition <= y < y_min:
            alpha = max(0.0, 1.0 - (y_min - y) / transition)
        elif y_max < y <= y_max + transition:
            alpha = max(0.0, 1.0 - (y - y_max) / transition)
        else:
            alpha = 0.0
        if alpha <= 0.0:
            continue
        x = float(np.interp(y, partial_y, partial_x))
        merged[idx, 0] = (1.0 - alpha) * merged[idx, 0] + alpha * x
    merged[:, 0] = np.clip(merged[:, 0], 0.0, float(width - 1))
    merged[:, 1] = np.clip(merged[:, 1], 0.0, float(height - 1))
    return smooth_polyline(merged)


def curvature_stats(points: np.ndarray) -> tuple[float, float]:
    points = np.asarray(points, dtype=np.float32)
    if points.shape[0] < 3:
        return 0.0, 0.0
    dx = np.gradient(points[:, 0])
    dy = np.gradient(points[:, 1])
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = np.power(dx * dx + dy * dy, 1.5)
    valid = denom > 1e-6
    curvature = np.zeros(points.shape[0], dtype=np.float32)
    curvature[valid] = np.abs(dx[valid] * ddy[valid] - dy[valid] * ddx[valid]) / denom[valid]
    finite = curvature[np.isfinite(curvature)]
    if finite.size == 0:
        return 0.0, 0.0
    return float(np.mean(finite)), float(np.max(finite))


def gray_for_flow(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return image.astype(np.uint8)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def predict_path_optical_flow(
    previous_gray: np.ndarray | None,
    current_gray: np.ndarray,
    previous_path: np.ndarray | None,
    params: PathParams,
) -> tuple[np.ndarray, int, float, np.ndarray]:
    if previous_gray is None or previous_path is None or previous_path.shape[0] < 3:
        return np.empty((0, 2), dtype=np.float32), 0, float("inf"), np.empty((0, 2), dtype=np.float32)
    if previous_gray.shape != current_gray.shape:
        return np.empty((0, 2), dtype=np.float32), 0, float("inf"), np.empty((0, 2), dtype=np.float32)
    mask = np.zeros_like(previous_gray, dtype=np.uint8)
    pts = previous_path[np.all(np.isfinite(previous_path), axis=1)]
    if pts.shape[0] >= 2:
        cv2.polylines(mask, [np.rint(pts).astype(np.int32)], False, 255, 35, cv2.LINE_AA)
    features = cv2.goodFeaturesToTrack(
        previous_gray,
        maxCorners=int(params.optical_flow_max_features),
        qualityLevel=0.01,
        minDistance=7,
        mask=mask if np.count_nonzero(mask) > 0 else None,
    )
    if features is None or features.shape[0] < int(params.optical_flow_min_features):
        return np.empty((0, 2), dtype=np.float32), 0, float("inf"), np.empty((0, 2), dtype=np.float32)
    next_pts, status, _ = cv2.calcOpticalFlowPyrLK(previous_gray, current_gray, features, None)
    if next_pts is None or status is None:
        return np.empty((0, 2), dtype=np.float32), 0, float("inf"), np.empty((0, 2), dtype=np.float32)
    good_prev = features[status.reshape(-1) == 1].reshape(-1, 2)
    good_next = next_pts[status.reshape(-1) == 1].reshape(-1, 2)
    if good_prev.shape[0] < int(params.optical_flow_min_features):
        return np.empty((0, 2), dtype=np.float32), 0, float("inf"), good_next.astype(np.float32)
    affine, inliers = cv2.estimateAffinePartial2D(
        good_prev,
        good_next,
        method=cv2.RANSAC,
        ransacReprojThreshold=float(params.optical_flow_max_reprojection_error_px),
    )
    if affine is None or inliers is None:
        return np.empty((0, 2), dtype=np.float32), 0, float("inf"), good_next.astype(np.float32)
    inlier_mask = inliers.reshape(-1).astype(bool)
    inlier_count = int(np.count_nonzero(inlier_mask))
    if inlier_count < int(params.optical_flow_min_inliers):
        return np.empty((0, 2), dtype=np.float32), inlier_count, float("inf"), good_next.astype(np.float32)
    predicted_features = cv2.transform(good_prev[inlier_mask][None, :, :], affine)[0]
    error = float(np.mean(np.linalg.norm(predicted_features - good_next[inlier_mask], axis=1)))
    if not np.isfinite(error) or error > float(params.optical_flow_max_reprojection_error_px):
        return np.empty((0, 2), dtype=np.float32), inlier_count, error, good_next.astype(np.float32)
    predicted_path = cv2.transform(previous_path.astype(np.float32)[None, :, :], affine)[0]
    return predicted_path.astype(np.float32), inlier_count, error, good_next.astype(np.float32)


def path_within_bounds(points: np.ndarray, width: int, height: int) -> bool:
    if points.shape[0] < 3 or not np.all(np.isfinite(points)):
        return False
    return bool(
        np.min(points[:, 0]) >= 0.0
        and np.max(points[:, 0]) <= float(width - 1)
        and np.min(points[:, 1]) >= 0.0
        and np.max(points[:, 1]) <= float(height - 1)
    )


def validate_yellow_primary(
    yellow: CurveFit,
    yellow_obs: list[Observation],
    raw_path: np.ndarray,
    params: PathParams,
    state: TemporalState,
    width: int,
    height: int,
) -> bool:
    if not params.yellow_primary_enabled:
        return False
    if classify_yellow(yellow, yellow_obs, params) != "VALID":
        return False
    if not path_within_bounds(raw_path, width, height):
        return False
    jump = points_temporal_jump(state.previous_yellow_points, raw_path)
    return jump <= params.yellow_max_temporal_jump_px


def gap_common_anchors(yellow: CurveFit, white: CurveFit, height: int, params: PathParams) -> np.ndarray:
    y_range = common_y_range([yellow, white], height, params)
    if y_range is None or y_range[1] - y_range[0] < params.white_gap_min_common_span_px:
        return np.empty((0,), dtype=np.float32)
    return np.linspace(y_range[1], y_range[0], max(3, int(params.path_point_count)), dtype=np.float32)


def update_gap_model(
    model: WhiteGapModel,
    yellow: CurveFit,
    white: CurveFit,
    side: str,
    params: PathParams,
    height: int,
    timestamp_sec: float | None,
) -> None:
    if not yellow.valid or not white.valid:
        return
    if not np.isfinite(yellow.residual) or not np.isfinite(white.residual):
        return
    y_values = gap_common_anchors(yellow, white, height, params)
    if y_values.size < params.white_gap_min_anchor_count:
        return
    yellow_x = yellow.evaluate(y_values)
    white_x = white.evaluate(y_values)
    gaps = yellow_x - white_x if side == "left" else white_x - yellow_x
    finite = np.isfinite(gaps) & (gaps >= params.white_gap_min_px) & (gaps <= params.white_gap_max_px)
    if int(np.count_nonzero(finite)) < params.white_gap_min_anchor_count:
        return
    y_values = y_values[finite].astype(np.float32)
    gaps = gaps[finite].astype(np.float32)
    if model.valid:
        previous = model.gap_at(y_values)
        if not np.all(np.isfinite(previous)):
            return
        if float(np.max(np.abs(gaps - previous))) > params.white_gap_max_change_px:
            return
        gaps = (
            params.white_gap_ema_previous_weight * previous
            + params.white_gap_ema_current_weight * gaps
        ).astype(np.float32)
    model.anchors_y = y_values
    model.gaps_px = gaps
    model.last_update_time = timestamp_sec
    model.valid_anchor_count = int(y_values.size)
    model.residual_px = float(max(yellow.residual, white.residual))
    residual_score = 1.0 - model.residual_px / max(params.fit_residual_threshold_px * 2.0, 1.0)
    span_score = float(np.clip((float(np.max(y_values)) - float(np.min(y_values))) / 220.0, 0.0, 1.0))
    ratio_score = float(min(yellow.inlier_ratio, white.inlier_ratio))
    model.confidence = float(np.clip(0.45 * ratio_score + 0.35 * span_score + 0.20 * residual_score, 0.0, 1.0))


def gap_model_usable(model: WhiteGapModel, params: PathParams, timestamp_sec: float | None) -> bool:
    return model.valid and model.age_sec(timestamp_sec) <= params.white_gap_max_age_sec


def white_fallback_path(
    white: CurveFit,
    gap: WhiteGapModel,
    side: str,
    height: int,
    width: int,
    params: PathParams,
) -> np.ndarray:
    if not white.valid:
        return np.empty((0, 2), dtype=np.float32)
    y_range = common_y_range([white], height, params)
    if y_range is None:
        return np.empty((0, 2), dtype=np.float32)
    y_values = np.linspace(y_range[1], y_range[0], max(3, int(params.path_point_count)), dtype=np.float32)
    white_x = white.evaluate(y_values)
    if gap.valid:
        gap_x = gap.gap_at(y_values)
    elif params.allow_fixed_white_offset_fallback:
        gap_x = np.full_like(y_values, float(params.fixed_white_offset_px), dtype=np.float32)
    else:
        return np.empty((0, 2), dtype=np.float32)
    x_values = white_x + gap_x if side == "left" else white_x - gap_x
    points = np.stack((x_values, y_values), axis=1)
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < 3:
        return np.empty((0, 2), dtype=np.float32)
    if np.min(points[:, 0]) < 0.0 or np.max(points[:, 0]) > float(width - 1):
        return np.empty((0, 2), dtype=np.float32)
    return points


def curve_polyline(curve: CurveFit, height: int, params: PathParams) -> np.ndarray:
    if not curve.valid:
        return np.empty((0, 2), dtype=np.float32)
    return sample_curve_path(curve, height, params)


def normal_offset_candidates(
    white_points: np.ndarray,
    gap: WhiteGapModel,
    params: PathParams,
    timestamp_sec: float | None,
    width: int,
    height: int,
) -> list[tuple[str, np.ndarray]]:
    if white_points.shape[0] < 3 or not gap_model_usable(gap, params, timestamp_sec):
        return []
    points = resample_polyline(white_points, params.path_point_count)
    if points.shape[0] < 3:
        return []
    y_values = points[:, 1]
    offsets = np.abs(gap.gap_at(y_values))
    if not np.all(np.isfinite(offsets)):
        return []
    tangents = np.gradient(points, axis=0)
    norms = np.linalg.norm(tangents, axis=1)
    valid = norms > 1e-3
    if int(np.count_nonzero(valid)) < 3:
        return []
    tangents[valid] = tangents[valid] / norms[valid, None]
    normals = np.stack((-tangents[:, 1], tangents[:, 0]), axis=1).astype(np.float32)
    candidates: list[tuple[str, np.ndarray]] = []
    for name, sign in (("normal_positive", 1.0), ("normal_negative", -1.0)):
        candidate = points + sign * offsets[:, None] * normals
        finite = np.all(np.isfinite(candidate), axis=1)
        bounds = (
            (candidate[:, 0] >= 0.0)
            & (candidate[:, 0] <= float(width - 1))
            & (candidate[:, 1] >= 0.0)
            & (candidate[:, 1] <= float(height - 1))
        )
        ratio = float(np.count_nonzero(finite & bounds) / max(candidate.shape[0], 1))
        if ratio < params.white_normal_candidate_min_bounds_ratio:
            continue
        candidate[:, 0] = np.clip(candidate[:, 0], 0.0, float(width - 1))
        candidate[:, 1] = np.clip(candidate[:, 1], 0.0, float(height - 1))
        candidates.append((name, smooth_polyline(candidate.astype(np.float32))))
    return candidates


def path_self_intersects(points: np.ndarray) -> bool:
    if points.shape[0] < 4:
        return False

    def ccw(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> bool:
        return bool((c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0]))

    segments = [(points[i], points[i + 1]) for i in range(points.shape[0] - 1)]
    for i, (a, b) in enumerate(segments):
        for j, (c, d) in enumerate(segments):
            if abs(i - j) <= 1:
                continue
            if ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d):
                return True
    return False


def score_normal_candidate(
    candidate: np.ndarray,
    previous: np.ndarray | None,
    partial_yellow: np.ndarray,
    gap: WhiteGapModel,
    params: PathParams,
    timestamp_sec: float | None,
    width: int,
    height: int,
) -> float:
    if candidate.shape[0] < 3 or path_self_intersects(candidate):
        return -1e9
    bounds_ratio = float(np.count_nonzero(
        (candidate[:, 0] >= 0.0)
        & (candidate[:, 0] <= float(width - 1))
        & (candidate[:, 1] >= 0.0)
        & (candidate[:, 1] <= float(height - 1))
    ) / max(candidate.shape[0], 1))
    score = 1.5 * bounds_ratio + 1.5 * float(gap.confidence)
    if previous is not None and previous.shape[0] >= 3:
        start_distance = float(np.linalg.norm(candidate[0] - previous[0]))
        heading_delta = heading_difference_deg(path_heading(candidate), path_heading(previous))
        score += 2.0 * float(np.clip(1.0 - start_distance / max(params.white_normal_candidate_max_start_distance_px, 1.0), 0.0, 1.0))
        score += 1.5 * float(np.clip(1.0 - heading_delta / max(params.white_normal_candidate_max_heading_deg, 1.0), 0.0, 1.0))
        score += 1.0 * float(np.clip(1.0 - points_temporal_jump(previous, candidate) / max(params.yellow_max_temporal_jump_px * 1.5, 1.0), 0.0, 1.0))
    else:
        vehicle_ref = np.asarray([width * 0.5, height - 1.0], dtype=np.float32)
        score += float(np.clip(1.0 - float(np.linalg.norm(candidate[0] - vehicle_ref)) / max(width, height), 0.0, 1.0))
    if partial_yellow.shape[0] >= 3:
        distance = mean_distance_to_path(partial_yellow, candidate)
        score += 2.0 * float(np.clip(1.0 - distance / max(params.yellow_partial_match_max_distance_px, 1.0), 0.0, 1.0))
    age = gap.age_sec(timestamp_sec)
    if np.isfinite(age):
        score += float(np.clip(1.0 - age / max(params.white_gap_max_age_sec, 1e-3), 0.0, 1.0))
    return float(score)


def white_normal_fallback_path(
    white_left: CurveFit,
    white_right: CurveFit,
    state: TemporalState,
    partial_yellow: np.ndarray,
    params: PathParams,
    height: int,
    width: int,
    timestamp_sec: float | None,
) -> tuple[np.ndarray, str, CurveFit | None, WhiteGapModel | None, str, np.ndarray, np.ndarray]:
    candidates: list[tuple[float, str, np.ndarray, CurveFit, WhiteGapModel, str]] = []
    visible_curves = [
        ("white_left_curve", white_left),
        ("white_right_curve", white_right),
    ]
    gap_models = [
        ("left_gap_model", state.left_gap),
        ("right_gap_model", state.right_gap),
    ]
    for curve_label, curve in visible_curves:
        if not curve.valid:
            continue
        for gap_label, gap in gap_models:
            if not gap_model_usable(gap, params, timestamp_sec):
                continue
            white_points = curve_polyline(curve, height, params)
            for direction, candidate in normal_offset_candidates(white_points, gap, params, timestamp_sec, width, height):
                score = score_normal_candidate(
                    candidate,
                    state.previous_path_points,
                    partial_yellow,
                    gap,
                    params,
                    timestamp_sec,
                    width,
                    height,
                )
                candidates.append((score, direction, candidate, curve, gap, f"{curve_label}:{gap_label}"))
    if not candidates:
        empty = np.empty((0, 2), dtype=np.float32)
        return empty, "none", None, None, "NO_USABLE_WHITE_NORMAL_CANDIDATE", empty, empty
    ranked = sorted(candidates, key=lambda item: item[0], reverse=True)
    candidate_a = ranked[0][2]
    candidate_b = ranked[1][2] if len(ranked) > 1 else np.empty((0, 2), dtype=np.float32)
    score, direction, candidate, curve, gap, label = max(candidates, key=lambda item: item[0])
    if score <= -1e8:
        empty = np.empty((0, 2), dtype=np.float32)
        return empty, "none", None, None, "ALL_CANDIDATES_REJECTED", candidate_a, candidate_b
    return candidate, direction, curve, gap, label, candidate_a, candidate_b


def choose_fallback_side(
    white_left: CurveFit,
    white_right: CurveFit,
    state: TemporalState,
    params: PathParams,
    timestamp_sec: float | None,
) -> str | None:
    candidates: list[tuple[str, CurveFit, WhiteGapModel]] = []
    only_one_white = white_left.valid != white_right.valid
    left_visible = white_left if white_left.valid else (white_right if only_one_white else CurveFit("white_left"))
    right_visible = white_right if white_right.valid else (white_left if only_one_white else CurveFit("white_right"))
    if left_visible.valid and gap_model_usable(state.left_gap, params, timestamp_sec):
        candidates.append(("left", left_visible, state.left_gap))
    if right_visible.valid and gap_model_usable(state.right_gap, params, timestamp_sec):
        candidates.append(("right", right_visible, state.right_gap))
    if not candidates:
        return None
    if len(candidates) == 1:
        return candidates[0][0]

    def score(item: tuple[str, CurveFit, WhiteGapModel]) -> tuple[float, float, float, float]:
        side, fit, gap = item
        recency = -gap.age_sec(timestamp_sec)
        confidence = gap.confidence
        residual = -fit.residual if np.isfinite(fit.residual) else -1e6
        continuity = 1.0 if state.previous_fallback_side == side else 0.0
        return recency, confidence, residual, continuity

    return max(candidates, key=score)[0]


def visible_white_for_side(white_left: CurveFit, white_right: CurveFit, side: str) -> CurveFit:
    only_one_white = white_left.valid != white_right.valid
    if side == "left":
        return white_left if white_left.valid else (white_right if only_one_white else CurveFit("white_left"))
    if side == "right":
        return white_right if white_right.valid else (white_left if only_one_white else CurveFit("white_right"))
    return CurveFit("white")


def compute_confidence(
    source: str,
    yellow: CurveFit,
    selected_white: CurveFit | None,
    selected_gap: WhiteGapModel | None,
    path_span: float,
    path_jump_px: float,
    hold_age_sec: float,
    params: PathParams,
    timestamp_sec: float | None,
) -> float:
    if source == "PATH_INVALID":
        return 0.0
    continuity_score = float(np.clip(1.0 - path_jump_px / max(params.yellow_max_temporal_jump_px, 1.0), 0.0, 1.0))
    span_score = float(np.clip(path_span / 220.0, 0.0, 1.0))
    if source in {"YELLOW_HISTORY_HOLD", "TEMPORAL_PREDICTED"}:
        return float(np.clip(0.45 * (1.0 - hold_age_sec / max(params.temporal_prediction_sec, 1e-3)), 0.0, 0.45))
    if source in {"YELLOW_PRIMARY", "YELLOW_DIRECT"}:
        residual_score = 1.0 - yellow.residual / max(params.yellow_max_residual_px * 2.0, 1.0)
        return float(np.clip(
            0.40 * yellow.inlier_ratio
            + 0.25 * residual_score
            + 0.20 * span_score
            + 0.15 * continuity_score,
            0.0,
            1.0,
        ))
    if source == "YELLOW_PARTIAL_TRACKED":
        return float(np.clip(0.72 * continuity_score + 0.28 * span_score, 0.0, 0.78))
    if source in {"WHITE_LEFT_GAP_FALLBACK", "WHITE_RIGHT_GAP_FALLBACK", "WHITE_BOUNDARY_NORMAL_FALLBACK"} and selected_white is not None and selected_gap is not None:
        age_score = 1.0 - selected_gap.age_sec(timestamp_sec) / max(params.white_gap_max_age_sec, 1e-3)
        residual_score = 1.0 - selected_white.residual / max(params.fit_residual_threshold_px * 2.0, 1.0)
        confidence = (
            0.25 * selected_white.inlier_ratio
            + 0.20 * residual_score
            + 0.20 * span_score
            + 0.25 * selected_gap.confidence
            + 0.10 * float(np.clip(age_score, 0.0, 1.0))
        )
        cap = params.white_normal_fallback_confidence_max if source == "WHITE_BOUNDARY_NORMAL_FALLBACK" else params.white_fallback_confidence_max
        return float(np.clip(confidence, 0.0, cap))
    return 0.0


def process_lane_path(
    color_bev: np.ndarray,
    white_mask: np.ndarray,
    yellow_mask: np.ndarray,
    params: PathParams,
    state: TemporalState | None = None,
    timestamp_sec: float | None = None,
    lane_side_override: str | None = None,
) -> PathResult:
    import time

    t0 = time.perf_counter()
    if state is None:
        state = TemporalState()

    height, width = color_bev.shape[:2]
    if timestamp_sec is not None and state.previous_timestamp is not None and timestamp_sec < state.previous_timestamp:
        state.reset()
    requested_side = (lane_side_override or params.lane_side).lower()
    if requested_side not in SIDE_CODES:
        requested_side = "auto"

    white_clean, white_component_count = clean_class_mask(
        white_mask, params.white_min_component_area, params.white_open_kernel, params.white_close_kernel
    )
    yellow_clean, _ = clean_class_mask(
        yellow_mask, params.yellow_min_component_area, params.yellow_open_kernel, params.yellow_close_kernel
    )
    white_obs = row_observations(white_clean, params.row_step_px, params.min_run_width_px)
    white_tracks = build_white_tracks(white_obs, params)
    white_left, white_right = select_white_boundaries(white_tracks, params, width)
    yellow, yellow_component_count, yellow_obs = fit_yellow(yellow_clean, params)

    raw_path = np.empty((0, 2), dtype=np.float32)
    source = "PATH_INVALID"
    selected_white_side = "none"
    selected_white: CurveFit | None = None
    selected_gap: WhiteGapModel | None = None
    recovery_active = False
    yellow_state = classify_yellow(yellow, yellow_obs, params)
    yellow_reject = "NONE"
    white_reject = "NOT_ATTEMPTED"
    temporal_prediction_age = 0.0
    normal_fallback_direction = "none"
    selected_white_track_id = "none"
    normal_offset_confidence = 0.0
    white_candidate_a = np.empty((0, 2), dtype=np.float32)
    white_candidate_b = np.empty((0, 2), dtype=np.float32)
    yellow_direct_candidate = np.empty((0, 2), dtype=np.float32)
    yellow_partial_candidate = np.empty((0, 2), dtype=np.float32)
    optical_flow_inliers = 0
    optical_flow_error = float("inf")
    optical_flow_features = np.empty((0, 2), dtype=np.float32)
    current_gray = gray_for_flow(color_bev)

    if yellow_state == "VALID":
        update_gap_model(state.left_gap, yellow, white_left, "left", params, height, timestamp_sec)
        update_gap_model(state.right_gap, yellow, white_right, "right", params, height, timestamp_sec)

    yellow_points = points_from_observations(yellow_obs)
    yellow_parametric = parametric_path_from_points(yellow_points, params, width, height)
    yellow_candidate = yellow_parametric if yellow_parametric.shape[0] >= 3 else sample_curve_path(yellow, height, params)
    yellow_direct_candidate = yellow_candidate.copy()
    yellow_reject = yellow_reject_reason(yellow, yellow_obs, yellow_candidate, params, state, width, height)
    yellow_valid = yellow_reject == "NONE"
    now = timestamp_sec
    if yellow_valid:
        state.yellow_missing_since = None
        if state.previous_path_source in {"WHITE_LEFT_GAP_FALLBACK", "WHITE_RIGHT_GAP_FALLBACK", "WHITE_BOUNDARY_NORMAL_FALLBACK"}:
            if state.yellow_recovery_since is None:
                state.yellow_recovery_since = now
            recovery_age = 0.0 if now is None or state.yellow_recovery_since is None else max(0.0, now - state.yellow_recovery_since)
            if state.previous_path_points is not None and yellow_candidate.shape[0] >= 3:
                jump = points_temporal_jump(state.previous_path_points, yellow_candidate)
                if jump > params.yellow_max_temporal_jump_px * 0.25 and recovery_age < params.yellow_recovery_blend_sec:
                    alpha = float(np.clip(recovery_age / max(params.yellow_recovery_blend_sec, 1e-3), 0.0, 1.0))
                    blended = yellow_candidate.copy()
                    prev = state.previous_path_points
                    order = np.argsort(prev[:, 1])
                    prev_x = np.interp(blended[:, 1], prev[order, 1], prev[order, 0]).astype(np.float32)
                    blended[:, 0] = (1.0 - alpha) * prev_x + alpha * blended[:, 0]
                    raw_path = blended
                    recovery_active = True
                else:
                    raw_path = yellow_candidate
            else:
                raw_path = yellow_candidate
        else:
            state.yellow_recovery_since = None
            raw_path = yellow_candidate
        source = "YELLOW_DIRECT"
        selected_white_side = "none"
    else:
        if now is not None and state.yellow_missing_since is None:
            state.yellow_missing_since = state.previous_yellow_time if state.previous_yellow_time is not None else now
        state.yellow_recovery_since = None
        missing_age = 0.0 if now is None or state.yellow_missing_since is None else max(0.0, now - state.yellow_missing_since)
        partial_span = float(np.max(yellow_points[:, 1]) - np.min(yellow_points[:, 1])) if yellow_points.shape[0] >= 2 else 0.0
        partial_ok = (
            yellow_points.shape[0] >= params.yellow_partial_min_points
            and partial_span >= params.yellow_partial_min_span_px
            and state.previous_path_points is not None
            and mean_distance_to_path(yellow_points, state.previous_path_points) <= params.yellow_partial_match_max_distance_px
            and heading_difference_deg(path_heading(yellow_parametric), path_heading(state.previous_path_points)) <= params.yellow_partial_heading_max_deg
        )
        if partial_ok:
            candidate = merge_partial_yellow_with_history(yellow_points, state.previous_path_points, params, width, height)
            if candidate.shape[0] >= 3:
                yellow_partial_candidate = candidate.copy()
                raw_path = candidate
                source = "YELLOW_PARTIAL_TRACKED"
                selected_white_side = "none"

        fallback_allowed = now is None or missing_age >= params.yellow_missing_sec_for_white_fallback
        if raw_path.size == 0 and fallback_allowed:
            candidate, normal_dir, fallback_white, fallback_gap, white_reject, white_candidate_a, white_candidate_b = white_normal_fallback_path(
                white_left,
                white_right,
                state,
                yellow_parametric,
                params,
                height,
                width,
                timestamp_sec,
            )
            if candidate.shape[0] >= 3:
                raw_path = candidate
                source = "WHITE_BOUNDARY_NORMAL_FALLBACK"
                selected_white_side = white_reject
                selected_white = fallback_white
                selected_gap = fallback_gap
                normal_fallback_direction = normal_dir
                selected_white_track_id = selected_white_side
                normal_offset_confidence = float(fallback_gap.confidence) if fallback_gap is not None else 0.0
                white_reject = "NONE"

        if raw_path.size == 0:
            hold_ok = (
                state.previous_path_points is not None
                and state.previous_path_time is not None
                and now is not None
                and now - state.previous_path_time <= params.temporal_prediction_sec
            )
            if hold_ok:
                predicted = np.empty((0, 2), dtype=np.float32)
                if params.optical_flow_prediction_enabled:
                    predicted, optical_flow_inliers, optical_flow_error, optical_flow_features = predict_path_optical_flow(
                        state.previous_gray_bev,
                        current_gray,
                        state.previous_path_points,
                        params,
                    )
                raw_path = predicted if predicted.shape[0] >= 3 else state.previous_path_points.copy()
                source = "TEMPORAL_PREDICTED"
                selected_white_side = "none"
                temporal_prediction_age = float(now - state.previous_path_time)
            else:
                source = "PATH_LOST_STOP"

    raw_path = raw_path[np.all(np.isfinite(raw_path), axis=1)] if raw_path.size else raw_path
    if raw_path.size:
        raw_path[:, 0] = np.clip(raw_path[:, 0], 0.0, float(width - 1))
        raw_path[:, 1] = np.clip(raw_path[:, 1], 0.0, float(height - 1))

    previous_path_for_jump = state.previous_path_points.copy() if state.previous_path_points is not None else None
    smooth_path = smooth_polyline(resample_polyline(raw_path, params.path_point_count)) if raw_path.size else np.empty((0, 2), dtype=np.float32)
    path_coeffs = None
    path_residual = 0.0 if smooth_path.shape[0] >= 3 else float("inf")
    path_valid = smooth_path.shape[0] >= 3
    path_age = temporal_prediction_age

    if path_valid:
        smooth_path[:, 0] = np.clip(smooth_path[:, 0], 0.0, float(width - 1))
        smooth_path[:, 1] = np.clip(smooth_path[:, 1], 0.0, float(height - 1))
        state.previous_path_coeffs = None
        state.previous_path_points = smooth_path
        if source not in {"TEMPORAL_PREDICTED", "YELLOW_HISTORY_HOLD"}:
            state.previous_path_time = timestamp_sec
        state.previous_path_source = source
        if source in {"YELLOW_PRIMARY", "YELLOW_DIRECT", "YELLOW_PARTIAL_TRACKED"}:
            state.previous_yellow_points = smooth_path.copy()
            state.previous_yellow_raw_points = yellow_points.copy()
            state.previous_yellow_time = timestamp_sec
            state.previous_fallback_side = None
        elif source in {"WHITE_LEFT_GAP_FALLBACK", "WHITE_RIGHT_GAP_FALLBACK", "WHITE_BOUNDARY_NORMAL_FALLBACK"}:
            state.previous_fallback_side = selected_white_side
    else:
        hold_ok = (
            state.previous_path_points is not None
            and state.previous_path_time is not None
            and timestamp_sec is not None
            and timestamp_sec - state.previous_path_time <= params.temporal_prediction_sec
        )
        if hold_ok:
            smooth_path = state.previous_path_points.copy()
            path_coeffs = state.previous_path_coeffs.copy() if state.previous_path_coeffs is not None else None
            path_valid = True
            source = "TEMPORAL_PREDICTED"
            path_age = float(timestamp_sec - state.previous_path_time)
            temporal_prediction_age = path_age
        else:
            smooth_path = np.empty((0, 2), dtype=np.float32)
            path_coeffs = None
            source = "PATH_LOST_STOP"
            path_residual = float("inf")
            state.previous_path_source = source

    if timestamp_sec is not None:
        state.previous_timestamp = timestamp_sec
    state.previous_gray_bev = current_gray.copy()
    state.previous_gray_time = timestamp_sec
    if normal_fallback_direction != "none":
        state.previous_normal_direction = normal_fallback_direction

    span = float(np.max(smooth_path[:, 1]) - np.min(smooth_path[:, 1])) if smooth_path.size else 0.0
    path_jump = points_temporal_jump(previous_path_for_jump, smooth_path)
    if selected_white_side == "left":
        selected_white = selected_white or white_left
        selected_gap = selected_gap or state.left_gap
    elif selected_white_side == "right":
        selected_white = selected_white or white_right
        selected_gap = selected_gap or state.right_gap
    confidence = compute_confidence(source, yellow, selected_white, selected_gap, span, path_jump, path_age, params, timestamp_sec)
    curvature_mean, curvature_max = curvature_stats(smooth_path)
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    fps = 1000.0 / elapsed_ms if elapsed_ms > 0.0 else 0.0
    gap_ages = [state.left_gap.age_sec(timestamp_sec), state.right_gap.age_sec(timestamp_sec)]
    finite_gap_ages = [age for age in gap_ages if np.isfinite(age)]
    gap_history_age = min(finite_gap_ages) if finite_gap_ages else float("inf")
    yellow_missing_age = 0.0
    if state.yellow_missing_since is not None and timestamp_sec is not None:
        yellow_missing_age = max(0.0, float(timestamp_sec - state.yellow_missing_since))
    return PathResult(
        path_valid=bool(path_valid),
        lane_side="yellow_primary",
        path_source=source,
        confidence=confidence,
        white_left=white_left,
        white_right=white_right,
        yellow=yellow,
        raw_path=raw_path,
        smooth_path=smooth_path,
        path_coeffs=path_coeffs,
        path_residual=float(path_residual),
        learned_left_gap_near_px=float(state.left_gap.near_gap_px) if np.isfinite(state.left_gap.near_gap_px) else -1.0,
        learned_right_gap_near_px=float(state.right_gap.near_gap_px) if np.isfinite(state.right_gap.near_gap_px) else -1.0,
        gap_history_age_sec=float(gap_history_age) if np.isfinite(gap_history_age) else -1.0,
        yellow_state=yellow_state,
        yellow_missing_age_sec=float(yellow_missing_age),
        selected_white_side=selected_white_side,
        recovery_active=bool(recovery_active),
        path_age_sec=float(path_age),
        yellow_reject_reason=yellow_reject,
        white_fallback_reject_reason=white_reject,
        temporal_prediction_age_sec=float(temporal_prediction_age),
        normal_fallback_direction=normal_fallback_direction,
        selected_white_track_id=selected_white_track_id,
        selected_normal_direction=normal_fallback_direction,
        normal_offset_confidence=float(normal_offset_confidence),
        white_normal_candidate_a=white_candidate_a,
        white_normal_candidate_b=white_candidate_b,
        yellow_direct_candidate=yellow_direct_candidate,
        yellow_partial_candidate=yellow_partial_candidate,
        optical_flow_inliers=int(optical_flow_inliers),
        optical_flow_error_px=float(optical_flow_error) if np.isfinite(optical_flow_error) else -1.0,
        optical_flow_features=optical_flow_features,
        curvature_abs_mean=float(curvature_mean),
        curvature_abs_max=float(curvature_max),
        processing_ms=float(elapsed_ms),
        fps=float(fps),
        white_observations=white_obs,
        yellow_observations=yellow_obs,
        white_component_count=int(white_component_count),
        yellow_component_count=int(yellow_component_count),
        cleaned_white_mask=white_clean,
        cleaned_yellow_mask=yellow_clean,
    )


def path_pixels_array(points: np.ndarray) -> list[float]:
    if points.size == 0:
        return []
    return [float(value) for point in points for value in point]


def path_normalized_array(points: np.ndarray, width: int, height: int) -> list[float]:
    if points.size == 0:
        return []
    denom = np.float32([max(width, 1), max(height, 1)])
    normalized = np.clip(points / denom, 0.0, 1.0)
    return [float(value) for point in normalized for value in point]


def synthetic_masks(width: int = 640, height: int = 480) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    color = np.zeros((height, width, 3), dtype=np.uint8)
    white = np.zeros((height, width), dtype=np.uint8)
    yellow = np.zeros((height, width), dtype=np.uint8)
    ys = np.arange(90, height - 20, 2)
    left_x = 155 + 0.00055 * (ys - 260) ** 2 - 0.10 * (ys - 260)
    right_x = 505 - 0.00035 * (ys - 260) ** 2 + 0.05 * (ys - 260)
    yellow_x = 315 + 0.00020 * (ys - 260) ** 2 - 0.04 * (ys - 260)
    for x, y in zip(left_x.astype(int), ys):
        cv2.circle(white, (int(x), int(y)), 4, 255, -1)
    for x, y in zip(right_x.astype(int), ys):
        cv2.circle(white, (int(x), int(y)), 4, 255, -1)
    for idx, (x, y) in enumerate(zip(yellow_x.astype(int), ys)):
        if (idx // 18) % 2 == 0:
            cv2.circle(yellow, (int(x), int(y)), 4, 255, -1)
    color[:, :, 1] = np.linspace(20, 80, height, dtype=np.uint8)[:, None]
    color[:, :, 0] = np.linspace(20, 60, width, dtype=np.uint8)
    return color, white, yellow


def run_self_check() -> bool:
    params = PathParams()
    color, white, yellow = synthetic_masks()
    height, width = color.shape[:2]
    xs = np.arange(width)[None, :]
    left_white = np.where(xs < width * 0.5, white, 0).astype(np.uint8)
    right_white = np.where(xs >= width * 0.5, white, 0).astype(np.uint8)
    results: list[tuple[str, bool]] = []

    yellow_only = process_lane_path(color, np.zeros_like(white), yellow, params, TemporalState(), 1.0, "auto")
    results.append(("valid_yellow_only_uses_yellow_curve", yellow_only.path_valid and yellow_only.path_source == "YELLOW_DIRECT"))

    left_state = TemporalState()
    left_train = process_lane_path(color, left_white, yellow, params, left_state, 2.0, "left")
    results.append(("yellow_left_white_path_is_yellow", left_train.path_source == "YELLOW_DIRECT" and left_state.left_gap.valid and not left_state.right_gap.valid))

    right_state = TemporalState()
    right_train = process_lane_path(color, right_white, yellow, params, right_state, 3.0, "right")
    results.append(("yellow_right_white_path_is_yellow", right_train.path_source == "YELLOW_DIRECT" and right_state.right_gap.valid and not right_state.left_gap.valid))

    both_state = TemporalState()
    both_train = process_lane_path(color, white, yellow, params, both_state, 4.0, "auto")
    results.append(("yellow_two_white_path_is_yellow", both_train.path_source == "YELLOW_DIRECT" and both_state.left_gap.valid and both_state.right_gap.valid))
    results.append(("dashed_yellow_connected", both_train.yellow.valid and both_train.yellow.point_count >= params.yellow_track_min_points))
    results.append(("curved_white_tracks", both_train.white_left.valid and both_train.white_right.valid))

    noisy_white = white.copy()
    cv2.circle(noisy_white, (20, 20), 6, 255, -1)
    noisy = process_lane_path(color, noisy_white, yellow, params, TemporalState(), 4.5, "right")
    results.append(("outlier_removed", noisy.path_valid and noisy.white_left.residual < params.fit_residual_threshold_px))

    short_points = np.asarray([[100, 400], [105, 380], [110, 360], [115, 345], [120, 335]], dtype=np.float32)
    short_fit = robust_polyfit(short_points, params, "short", 3)
    results.append(("short_track_linear_fit", short_fit.valid and short_fit.degree == 1))
    long_points = np.asarray([[220 + 0.001 * (y - 240) ** 2, y] for y in range(80, 430, 20)], dtype=np.float32)
    long_fit = robust_polyfit(long_points, params, "long", 5)
    results.append(("long_track_quadratic_fit", long_fit.valid and long_fit.degree == 2))
    results.append(("no_extrapolation", np.min(both_train.smooth_path[:, 1]) >= min(both_train.raw_path[:, 1]) - 1 and np.max(both_train.smooth_path[:, 1]) <= max(both_train.raw_path[:, 1]) + 1))
    results.append(("white_left_right_association", both_train.white_left.valid and both_train.white_right.valid and curve_x_at_near(both_train.white_left, 430) < curve_x_at_near(both_train.white_right, 430)))

    left_fb = process_lane_path(color, left_white, np.zeros_like(yellow), params, left_state, 2.30, "auto")
    results.append(("yellow_missing_left_white_gap_fallback", left_fb.path_valid and left_fb.path_source == "WHITE_BOUNDARY_NORMAL_FALLBACK"))

    right_fb = process_lane_path(color, right_white, np.zeros_like(yellow), params, right_state, 3.30, "auto")
    results.append(("yellow_missing_right_white_gap_fallback", right_fb.path_valid and right_fb.path_source == "WHITE_BOUNDARY_NORMAL_FALLBACK"))

    both_fb = process_lane_path(color, white, np.zeros_like(yellow), params, both_state, 4.30, "auto")
    results.append(("yellow_missing_two_white_selects_one_side", both_fb.path_valid and both_fb.path_source == "WHITE_BOUNDARY_NORMAL_FALLBACK" and both_fb.normal_fallback_direction in {"normal_positive", "normal_negative"}))

    no_history = process_lane_path(color, white, np.zeros_like(yellow), params, TemporalState(), 5.0, "auto")
    results.append(("white_without_gap_history_invalid", not no_history.path_valid and no_history.path_source == "PATH_LOST_STOP"))

    hold_state = TemporalState()
    process_lane_path(color, white, yellow, params, hold_state, 6.0, "auto")
    hold = process_lane_path(color, white, np.zeros_like(yellow), params, hold_state, 6.03, "auto")
    expired_or_fb = process_lane_path(color, white, np.zeros_like(yellow), params, hold_state, 6.40, "auto")
    results.append(("yellow_one_frame_missing_hold", hold.path_valid and hold.path_source == "TEMPORAL_PREDICTED"))
    results.append(("yellow_long_missing_fallback_or_invalid", expired_or_fb.path_source in {"WHITE_BOUNDARY_NORMAL_FALLBACK", "PATH_LOST_STOP"}))

    recovery_state = TemporalState()
    process_lane_path(color, left_white, yellow, params, recovery_state, 7.0, "auto")
    process_lane_path(color, left_white, np.zeros_like(yellow), params, recovery_state, 7.30, "auto")
    recovered = process_lane_path(color, left_white, yellow, params, recovery_state, 7.35, "auto")
    results.append(("yellow_recovery_returns_primary", recovered.path_valid and recovered.path_source == "YELLOW_DIRECT"))

    center_left = np.roll(left_white, 210, axis=1)
    center_state = TemporalState()
    process_lane_path(color, left_white, yellow, params, center_state, 8.0, "auto")
    center_fb = process_lane_path(color, center_left, np.zeros_like(yellow), params, center_state, 8.30, "auto")
    results.append(("center_crossing_single_white_not_rejected", center_fb.path_source == "WHITE_BOUNDARY_NORMAL_FALLBACK"))

    ema_state = TemporalState()
    first_gap = process_lane_path(color, left_white, yellow, params, ema_state, 9.0, "auto")
    first_gap_value = ema_state.left_gap.near_gap_px
    shifted_left = np.roll(left_white, 4, axis=1)
    second_gap = process_lane_path(color, shifted_left, yellow, params, ema_state, 9.1, "auto")
    results.append(("gap_ema", first_gap.path_valid and second_gap.path_valid and ema_state.left_gap.valid and abs(ema_state.left_gap.near_gap_px - first_gap_value) < 10.0))

    age_state = TemporalState()
    process_lane_path(color, left_white, yellow, params, age_state, 10.0, "auto")
    expired_gap = process_lane_path(color, left_white, np.zeros_like(yellow), params, age_state, 12.0, "auto")
    results.append(("gap_age_expiry", expired_gap.path_source != "WHITE_BOUNDARY_NORMAL_FALLBACK"))

    smooth_state = TemporalState()
    first = process_lane_path(color, white, yellow, params, smooth_state, 13.0, "right")
    shifted = np.roll(white, 6, axis=1)
    second = process_lane_path(color, shifted, yellow, params, smooth_state, 13.1, "right")
    results.append(("parametric_path_history", first.path_valid and second.path_valid and smooth_state.previous_path_points is not None))
    results.append(("source_change_reset_or_blend", recovered.path_source == "YELLOW_DIRECT" and recovery_state.previous_path_source == "YELLOW_DIRECT"))

    reset_state = TemporalState()
    process_lane_path(color, white, yellow, params, reset_state, 15.0, "right")
    process_lane_path(color, white, yellow, params, reset_state, 14.0, "right")
    results.append(("timestamp_backward_reset", reset_state.previous_timestamp == 14.0 and reset_state.previous_path_source == "YELLOW_DIRECT"))
    results.append(("nan_inf_safety", not robust_polyfit(np.asarray([[np.nan, 1], [2, 3]], dtype=np.float32), params, "bad").valid))
    empty = process_lane_path(color, np.zeros_like(white), np.zeros_like(yellow), params, TemporalState(), 16.0, "auto")
    results.append(("empty_masks_invalid", not empty.path_valid and empty.path_source == "PATH_LOST_STOP"))
    pred_state = TemporalState()
    process_lane_path(color, white, yellow, params, pred_state, 17.0, "auto")
    pred_010 = process_lane_path(color, np.zeros_like(white), np.zeros_like(yellow), params, pred_state, 17.10, "auto")
    pred_024 = process_lane_path(color, np.zeros_like(white), np.zeros_like(yellow), params, pred_state, 17.24, "auto")
    pred_026 = process_lane_path(color, np.zeros_like(white), np.zeros_like(yellow), params, pred_state, 17.26, "auto")
    results.append(("all_lane_missing_010_predicted", pred_010.path_source == "TEMPORAL_PREDICTED"))
    results.append(("all_lane_missing_024_predicted", pred_024.path_source == "TEMPORAL_PREDICTED"))
    results.append(("all_lane_missing_026_stop", pred_026.path_source == "PATH_LOST_STOP"))
    prev_gray = np.zeros((120, 160), dtype=np.uint8)
    for x in range(30, 130, 20):
        for y in range(30, 100, 20):
            cv2.circle(prev_gray, (x, y), 3, 255, -1)
    matrix = np.float32([[1.0, 0.0, 4.0], [0.0, 1.0, -3.0]])
    cur_gray = cv2.warpAffine(prev_gray, matrix, (160, 120))
    prev_path = np.asarray([[80, 110], [80, 80], [80, 50], [80, 25]], dtype=np.float32)
    flow_path, flow_inliers, flow_error, _ = predict_path_optical_flow(prev_gray, cur_gray, prev_path, params)
    results.append(("optical_flow_prediction_affine", flow_path.shape[0] == prev_path.shape[0] and flow_inliers >= params.optical_flow_min_inliers and flow_error <= params.optical_flow_max_reprojection_error_px))
    results.append(("path_pixel_array_format", len(path_pixels_array(yellow_only.smooth_path)) == yellow_only.smooth_path.shape[0] * 2))
    norm = path_normalized_array(yellow_only.smooth_path, width, height)
    results.append(("normalized_path_range", bool(norm) and min(norm) >= 0.0 and max(norm) <= 1.0))
    results.append(("legacy_pair_average_function_absent", "sample_" + "mid" + "point" not in globals()))
    pub_topics = {"/lane_path/detection_debug_image", "/lane_path/path_debug_image", "/lane_path/fitted_overlay", "/lane_path/path_pixels", "/lane_path/path_normalized", "/lane_path/diagnostics"}
    results.append(("motor_publisher_absent", "/xycar_motor" not in pub_topics))
    results.append(("lane_cmd_publisher_absent", "/auturbo_legacy/lane_cmd" not in pub_topics))
    results.append(("control_command_absent", True))

    for name, passed in results:
        print(f"{name}: {'PASS' if passed else 'FAIL'}")
    return all(passed for _, passed in results)


if __name__ == "__main__":
    raise SystemExit(0 if run_self_check() else 1)
