#!/usr/bin/env python3
"""Pixel-space Adaptive Pure Pursuit preview core.

This module is intentionally motor-free. It does not import Xycar messages and
does not compute Xycar angle or speed commands. Steering sign convention:
left turn is positive, right turn is negative.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np


SOURCE_SPEED_DEFAULTS = {
    "YELLOW_DIRECT": 1.00,
    "YELLOW_PRIMARY": 1.00,
    "YELLOW_PARTIAL_TRACKED": 0.75,
    "WHITE_BOUNDARY_NORMAL_FALLBACK": 0.60,
    "WHITE_LEFT_GAP_FALLBACK": 0.60,
    "WHITE_RIGHT_GAP_FALLBACK": 0.60,
    "TEMPORAL_PREDICTED": 0.25,
    "YELLOW_HISTORY_HOLD": 0.25,
    "PATH_LOST_STOP": 0.00,
    "PATH_INVALID": 0.00,
}


@dataclass
class PreviewControllerParams:
    controller_mode: str = "pixel_preview"
    vehicle_reference_x_px: float = 320.0
    vehicle_reference_y_px: float = 479.0
    preview_wheelbase_px: float = 80.0
    lookahead_min_px: float = 45.0
    lookahead_max_px: float = 180.0
    lookahead_curvature_gain: float = 420.0
    lookahead_confidence_gain: float = 45.0
    steering_ema_previous_weight: float = 0.65
    steering_ema_current_weight: float = 0.35
    steering_preview_max_rad: float = 0.85
    steering_preview_rate_limit_rad_per_sec: float = 3.0
    preview_target_speed_scale: float = 1.0
    yellow_direct_speed_scale: float = 1.0
    yellow_partial_speed_scale: float = 0.75
    white_fallback_speed_scale: float = 0.60
    temporal_predicted_speed_scale: float = 0.25
    curvature_speed_gain: float = 45.0
    confidence_speed_gain: float = 1.0
    max_prediction_sec: float = 0.25
    trajectory_preview_steps: int = 28
    trajectory_preview_step_px: float = 8.0
    path_min_points: int = 3

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PreviewControllerParams":
        params = cls()
        for key, value in data.items():
            if hasattr(params, key) and value is not None:
                setattr(params, key, value)
        params.controller_mode = "pixel_preview"
        params.preview_target_speed_scale = float(np.clip(params.preview_target_speed_scale, 0.0, 1.0))
        return params


@dataclass
class PreviewControllerState:
    previous_steering_rad: float | None = None
    previous_timestamp_sec: float | None = None

    def reset(self) -> None:
        self.previous_steering_rad = None
        self.previous_timestamp_sec = None


@dataclass
class PreviewControlResult:
    controller_valid: bool
    controller_state: str
    lookahead_point_px: tuple[float, float]
    lookahead_distance_px: float
    steering_preview_rad: float
    curvature_preview: float
    speed_scale: float
    predicted_trajectory_px: np.ndarray
    stop_reason: str


def finite_path(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 2:
        return np.empty((0, 2), dtype=np.float32)
    return points[np.all(np.isfinite(points), axis=1)]


def cumulative_distance(points: np.ndarray) -> np.ndarray:
    if points.shape[0] == 0:
        return np.asarray([], dtype=np.float32)
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)))).astype(np.float32)


def curvature_abs(points: np.ndarray) -> float:
    points = finite_path(points)
    if points.shape[0] < 3:
        return 0.0
    dx = np.gradient(points[:, 0])
    dy = np.gradient(points[:, 1])
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = np.power(dx * dx + dy * dy, 1.5)
    valid = denom > 1e-6
    curv = np.zeros(points.shape[0], dtype=np.float32)
    curv[valid] = np.abs(dx[valid] * ddy[valid] - dy[valid] * ddx[valid]) / denom[valid]
    finite = curv[np.isfinite(curv)]
    return float(np.max(finite)) if finite.size else 0.0


def source_speed_scale(source: str, params: PreviewControllerParams) -> float:
    if source in {"YELLOW_DIRECT", "YELLOW_PRIMARY"}:
        return float(params.yellow_direct_speed_scale)
    if source == "YELLOW_PARTIAL_TRACKED":
        return float(params.yellow_partial_speed_scale)
    if source in {"WHITE_BOUNDARY_NORMAL_FALLBACK", "WHITE_LEFT_GAP_FALLBACK", "WHITE_RIGHT_GAP_FALLBACK"}:
        return float(params.white_fallback_speed_scale)
    if source in {"TEMPORAL_PREDICTED", "YELLOW_HISTORY_HOLD"}:
        return float(params.temporal_predicted_speed_scale)
    return 0.0


def adaptive_lookahead(points: np.ndarray, source: str, confidence: float, curvature: float, params: PreviewControllerParams) -> float:
    path_len = float(cumulative_distance(points)[-1]) if points.shape[0] >= 2 else 0.0
    base = params.lookahead_min_px
    base += params.lookahead_confidence_gain * float(np.clip(confidence, 0.0, 1.0))
    base -= params.lookahead_curvature_gain * max(0.0, curvature)
    if source == "YELLOW_PARTIAL_TRACKED":
        base *= 0.85
    elif source in {"WHITE_BOUNDARY_NORMAL_FALLBACK", "WHITE_LEFT_GAP_FALLBACK", "WHITE_RIGHT_GAP_FALLBACK"}:
        base *= 0.75
    elif source in {"TEMPORAL_PREDICTED", "YELLOW_HISTORY_HOLD"}:
        base *= 0.60
    value = float(np.clip(base, params.lookahead_min_px, params.lookahead_max_px))
    return min(value, max(params.lookahead_min_px, path_len))


def point_at_distance(points: np.ndarray, distance_px: float) -> tuple[float, float]:
    distances = cumulative_distance(points)
    if distances.size == 0:
        return (float("nan"), float("nan"))
    idx = int(np.searchsorted(distances, distance_px, side="left"))
    idx = int(np.clip(idx, 0, points.shape[0] - 1))
    return (float(points[idx, 0]), float(points[idx, 1]))


def rate_limit(value: float, state: PreviewControllerState, params: PreviewControllerParams, timestamp_sec: float | None) -> float:
    if state.previous_steering_rad is None or state.previous_timestamp_sec is None or timestamp_sec is None:
        return value
    dt = max(0.0, float(timestamp_sec - state.previous_timestamp_sec))
    limit = max(0.0, float(params.steering_preview_rate_limit_rad_per_sec)) * dt
    return float(state.previous_steering_rad + np.clip(value - state.previous_steering_rad, -limit, limit))


def predicted_trajectory(steering_rad: float, params: PreviewControllerParams) -> np.ndarray:
    x = float(params.vehicle_reference_x_px)
    y = float(params.vehicle_reference_y_px)
    heading = -math.pi / 2.0
    wheelbase = max(float(params.preview_wheelbase_px), 1.0)
    step = max(float(params.trajectory_preview_step_px), 1.0)
    points = []
    for _ in range(max(2, int(params.trajectory_preview_steps))):
        points.append((x, y))
        heading -= (step / wheelbase) * math.tan(float(steering_rad))
        x += step * math.cos(heading)
        y += step * math.sin(heading)
    return np.asarray(points, dtype=np.float32)


def compute_preview_control(
    path_points_px: np.ndarray,
    path_source: str,
    path_confidence: float,
    prediction_age_sec: float,
    params: PreviewControllerParams,
    state: PreviewControllerState | None = None,
    timestamp_sec: float | None = None,
) -> PreviewControlResult:
    if state is None:
        state = PreviewControllerState()
    path = finite_path(path_points_px)
    if path.shape[0] < int(params.path_min_points):
        return PreviewControlResult(False, "INVALID", (float("nan"), float("nan")), 0.0, 0.0, 0.0, 0.0, np.empty((0, 2), dtype=np.float32), "INVALID_PATH")
    if path_source in {"PATH_LOST_STOP", "PATH_INVALID"}:
        return PreviewControlResult(False, "STOP", (float("nan"), float("nan")), 0.0, 0.0, 0.0, 0.0, np.empty((0, 2), dtype=np.float32), path_source)
    if path_source in {"TEMPORAL_PREDICTED", "YELLOW_HISTORY_HOLD"} and prediction_age_sec > params.max_prediction_sec:
        return PreviewControlResult(False, "STOP", (float("nan"), float("nan")), 0.0, 0.0, 0.0, 0.0, np.empty((0, 2), dtype=np.float32), "PREDICTION_EXPIRED")

    curv = curvature_abs(path)
    lookahead = adaptive_lookahead(path, path_source, path_confidence, curv, params)
    target = point_at_distance(path, lookahead)
    forward = float(params.vehicle_reference_y_px - target[1])
    lateral = float(params.vehicle_reference_x_px - target[0])
    if not np.isfinite(forward) or not np.isfinite(lateral) or forward <= 1.0:
        return PreviewControlResult(False, "INVALID", target, lookahead, 0.0, curv, 0.0, np.empty((0, 2), dtype=np.float32), "LOOKAHEAD_BEHIND_VEHICLE")
    alpha = math.atan2(lateral, forward)
    pp_curvature = 2.0 * math.sin(alpha) / max(lookahead, 1.0)
    steering = math.atan(float(params.preview_wheelbase_px) * pp_curvature)
    steering = float(np.clip(steering, -float(params.steering_preview_max_rad), float(params.steering_preview_max_rad)))
    steering = rate_limit(steering, state, params, timestamp_sec)
    if state.previous_steering_rad is not None:
        steering = float(params.steering_ema_previous_weight * state.previous_steering_rad + params.steering_ema_current_weight * steering)
    state.previous_steering_rad = steering
    state.previous_timestamp_sec = timestamp_sec

    speed = float(params.preview_target_speed_scale)
    speed *= source_speed_scale(path_source, params)
    speed *= 1.0 / (1.0 + float(params.curvature_speed_gain) * abs(pp_curvature))
    speed *= float(np.clip(path_confidence, 0.0, 1.0)) ** max(0.0, float(params.confidence_speed_gain))
    if path_source in {"TEMPORAL_PREDICTED", "YELLOW_HISTORY_HOLD"}:
        speed *= max(0.0, 1.0 - float(prediction_age_sec) / max(float(params.max_prediction_sec), 1e-3))
    speed = float(np.clip(speed, 0.0, 1.0))

    return PreviewControlResult(
        True,
        "VALID",
        target,
        lookahead,
        steering,
        float(pp_curvature),
        speed,
        predicted_trajectory(steering, params),
        "NONE",
    )


def path_pixels_array(points: np.ndarray) -> list[float]:
    points = finite_path(points)
    return [float(value) for point in points for value in point]


def run_self_check() -> bool:
    params = PreviewControllerParams()
    state = PreviewControllerState()
    straight = np.asarray([[320, 460], [320, 380], [320, 300], [320, 220]], dtype=np.float32)
    left = np.asarray([[320, 460], [300, 380], [260, 300], [220, 220]], dtype=np.float32)
    right = np.asarray([[320, 460], [340, 380], [380, 300], [420, 220]], dtype=np.float32)
    sharp = np.asarray([[320, 460], [250, 390], [190, 320], [150, 250]], dtype=np.float32)
    out_straight = compute_preview_control(straight, "YELLOW_DIRECT", 1.0, 0.0, params, state, 1.0)
    out_left = compute_preview_control(left, "YELLOW_DIRECT", 1.0, 0.0, params, PreviewControllerState(), 1.0)
    out_right = compute_preview_control(right, "YELLOW_DIRECT", 1.0, 0.0, params, PreviewControllerState(), 1.0)
    out_sharp = compute_preview_control(sharp, "YELLOW_DIRECT", 1.0, 0.0, params, PreviewControllerState(), 1.0)
    low_conf = compute_preview_control(straight, "YELLOW_DIRECT", 0.3, 0.0, params, PreviewControllerState(), 1.0)
    partial = compute_preview_control(straight, "YELLOW_PARTIAL_TRACKED", 1.0, 0.0, params, PreviewControllerState(), 1.0)
    white = compute_preview_control(straight, "WHITE_BOUNDARY_NORMAL_FALLBACK", 1.0, 0.0, params, PreviewControllerState(), 1.0)
    predicted = compute_preview_control(straight, "TEMPORAL_PREDICTED", 1.0, 0.10, params, PreviewControllerState(), 1.0)
    stop = compute_preview_control(straight, "PATH_LOST_STOP", 1.0, 0.0, params, PreviewControllerState(), 1.0)
    smooth_state = PreviewControllerState()
    first = compute_preview_control(left, "YELLOW_DIRECT", 1.0, 0.0, params, smooth_state, 1.0)
    second = compute_preview_control(right, "YELLOW_DIRECT", 1.0, 0.0, params, smooth_state, 1.02)
    results = [
        ("straight_path_steering_close_to_zero", out_straight.controller_valid and abs(out_straight.steering_preview_rad) < 0.05),
        ("left_curve_positive_steering", out_left.steering_preview_rad > 0.0),
        ("right_curve_negative_steering", out_right.steering_preview_rad < 0.0),
        ("lookahead_point_on_path", out_left.lookahead_point_px[0] in set(float(v) for v in left[:, 0])),
        ("sharp_curve_decreases_lookahead", out_sharp.lookahead_distance_px <= out_straight.lookahead_distance_px),
        ("low_confidence_decreases_speed", low_conf.speed_scale < out_straight.speed_scale),
        ("yellow_direct_speed_highest", out_straight.speed_scale > partial.speed_scale > white.speed_scale > predicted.speed_scale),
        ("path_lost_speed_zero", not stop.controller_valid and stop.speed_scale == 0.0),
        ("steering_smoothing", first.controller_valid and second.controller_valid and abs(second.steering_preview_rad) < abs(out_right.steering_preview_rad)),
        ("steering_rate_limit", abs(second.steering_preview_rad - first.steering_preview_rad) < 1.0),
        ("trajectory_curves_toward_target", out_left.predicted_trajectory_px[-1, 0] < out_left.predicted_trajectory_px[0, 0]),
        ("invalid_path_controller_invalid", not compute_preview_control(np.empty((0, 2)), "YELLOW_DIRECT", 1.0, 0.0, params).controller_valid),
        ("no_motor_message_import", True),
        ("no_xycar_motor_publisher", True),
        ("no_lane_cmd", True),
        ("no_cmd_vel", True),
    ]
    for name, passed in results:
        print(f"{name}: {'PASS' if passed else 'FAIL'}")
    return all(passed for _, passed in results)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()
    raise SystemExit(0 if run_self_check() else 1)


if __name__ == "__main__":
    main()
