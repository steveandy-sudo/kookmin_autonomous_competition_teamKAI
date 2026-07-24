#!/usr/bin/env python3
"""Calibrate encoder-less Xycar odometry from LiDAR, IMU, and motor commands.

The LiDAR range unit is the metric reference.  Track drawings and the length of
an accumulated SLAM trajectory are deliberately not used as ground truth.

Run this inside a sourced ROS 2 environment:

  python3 scripts/calibrate_command_odom_from_bag.py \
    /path/to/track_full_sensor_20260724_163034 \
    --output-dir data/odom_calibration/2026-07-24
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import subprocess
from typing import Any, Iterable

import numpy as np


STEERING_COMMANDS = np.array(
    [-42.0, -40.0, -35.0, -30.0, -20.0, -10.0, 0.0,
     10.0, 20.0, 30.0, 35.0, 40.0, 42.0],
    dtype=np.float64,
)
STEERING_CURVATURES = np.array(
    [1.502435, 1.383494, 1.174860, 0.922781, 0.552809, 0.194230, 0.0,
     -0.556883, -0.959829, -1.369323, -1.601706, -1.853397, -1.939236],
    dtype=np.float64,
)


@dataclass
class ScanRecord:
    time_sec: float
    points: np.ndarray


@dataclass
class CalibrationWindow:
    start_sec: float
    end_sec: float
    dt_sec: float
    angle_command: float
    speed_command: float
    lidar_dx_m: float
    lidar_dy_m: float
    lidar_yaw_rad: float
    lidar_arc_m: float
    gyro_integral_raw_rad: float
    icp_rmse_m: float
    icp_inliers: int
    icp_inlier_ratio: float
    icp_seed_gain_span: float
    accepted: bool
    rejection_reason: str


def _open_bag_reader(bag_path: Path):
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def _message_time_sec(message: Any, bag_time_ns: int) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns > 0:
            return stamp_ns * 1e-9
    return bag_time_ns * 1e-9


def read_bag(
    bag_path: Path,
    scan_topic: str,
    imu_topic: str,
    motor_topic: str,
    range_min_m: float,
    range_max_m: float,
    ray_step: int,
) -> tuple[list[ScanRecord], np.ndarray, np.ndarray, np.ndarray]:
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _open_bag_reader(bag_path)
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    wanted = {scan_topic, imu_topic, motor_topic}
    missing = sorted(topic for topic in wanted if topic not in topic_types)
    if missing:
        raise RuntimeError(f"bag is missing required topics: {missing}")
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in wanted
    }

    scans: list[ScanRecord] = []
    imu_rows: list[tuple[float, float]] = []
    motor_rows: list[tuple[float, float, float]] = []
    while reader.has_next():
        topic, serialized, bag_time_ns = reader.read_next()
        if topic not in wanted:
            continue
        message = deserialize_message(serialized, message_types[topic])
        time_sec = _message_time_sec(message, bag_time_ns)
        if topic == scan_topic:
            ranges = np.asarray(message.ranges, dtype=np.float64)
            indices = np.arange(ranges.size, dtype=np.float64)
            angles = float(message.angle_min) + indices * float(message.angle_increment)
            valid = (
                np.isfinite(ranges)
                & (ranges >= max(range_min_m, float(message.range_min)))
                & (ranges <= min(range_max_m, float(message.range_max)))
            )
            selected = np.flatnonzero(valid)[::max(1, ray_step)]
            points = np.column_stack(
                (
                    ranges[selected] * np.cos(angles[selected]),
                    ranges[selected] * np.sin(angles[selected]),
                )
            )
            scans.append(ScanRecord(time_sec, points))
        elif topic == imu_topic:
            imu_rows.append((time_sec, float(message.angular_velocity.z)))
        else:
            if len(message.data) >= 2:
                motor_rows.append(
                    (time_sec, float(message.data[0]), float(message.data[1]))
                )

    if len(scans) < 2 or len(imu_rows) < 2 or len(motor_rows) < 2:
        raise RuntimeError(
            "not enough usable scan, IMU, or motor messages for calibration"
        )
    return (
        scans,
        np.asarray(imu_rows, dtype=np.float64),
        np.asarray(motor_rows, dtype=np.float64),
        np.asarray(
            [
                len(record.points) for record in scans
            ],
            dtype=np.int64,
        ),
    )


def delayed_command(
    motor: np.ndarray,
    query_time_sec: float,
    delay_sec: float = 0.0,
    timeout_sec: float = 0.30,
) -> tuple[float, float, bool]:
    effective_time = query_time_sec - delay_sec
    index = int(np.searchsorted(motor[:, 0], effective_time, side="right") - 1)
    if index < 0:
        return 0.0, 0.0, False
    fresh = effective_time - float(motor[index, 0]) <= timeout_sec
    if not fresh:
        return 0.0, 0.0, False
    return float(motor[index, 1]), float(motor[index, 2]), True


def integrate_gyro(imu: np.ndarray, start_sec: float, end_sec: float) -> float:
    if end_sec <= start_sec:
        return 0.0
    left = int(np.searchsorted(imu[:, 0], start_sec, side="right"))
    right = int(np.searchsorted(imu[:, 0], end_sec, side="left"))
    times = np.concatenate(
        (
            np.array([start_sec]),
            imu[left:right, 0],
            np.array([end_sec]),
        )
    )
    values = np.concatenate(
        (
            np.array([np.interp(start_sec, imu[:, 0], imu[:, 1])]),
            imu[left:right, 1],
            np.array([np.interp(end_sec, imu[:, 0], imu[:, 1])]),
        )
    )
    return float(np.trapz(values, times))


def rotation_matrix(yaw_rad: float) -> np.ndarray:
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return np.array([[cosine, -sine], [sine, cosine]], dtype=np.float64)


def arc_translation(distance_m: float, yaw_rad: float) -> np.ndarray:
    if abs(yaw_rad) < 1e-8:
        return np.array([distance_m, 0.0], dtype=np.float64)
    radius = distance_m / yaw_rad
    return np.array(
        [radius * math.sin(yaw_rad), radius * (1.0 - math.cos(yaw_rad))],
        dtype=np.float64,
    )


def base_to_lidar_relative(
    base_translation: np.ndarray,
    yaw_rad: float,
    lidar_x_m: float,
) -> np.ndarray:
    lever = np.array([lidar_x_m, 0.0], dtype=np.float64)
    return base_translation + rotation_matrix(yaw_rad) @ lever - lever


def lidar_to_base_relative(
    lidar_translation: np.ndarray,
    yaw_rad: float,
    lidar_x_m: float,
) -> np.ndarray:
    lever = np.array([lidar_x_m, 0.0], dtype=np.float64)
    return lever + lidar_translation - rotation_matrix(yaw_rad) @ lever


def best_fit_rigid(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    u_matrix, _, vt_matrix = np.linalg.svd(source_zero.T @ target_zero)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0.0:
        vt_matrix[-1, :] *= -1.0
        rotation = vt_matrix.T @ u_matrix.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def icp_2d(
    source: np.ndarray,
    target: np.ndarray,
    initial_yaw_rad: float,
    initial_translation: np.ndarray,
    maximum_correspondence_m: float,
    trim_quantile: float = 0.82,
    maximum_iterations: int = 20,
) -> tuple[float, np.ndarray, float, int, float]:
    from scipy.spatial import cKDTree

    rotation = rotation_matrix(initial_yaw_rad)
    translation = np.asarray(initial_translation, dtype=np.float64).copy()
    tree = cKDTree(target)
    previous_rmse = math.inf
    for _ in range(maximum_iterations):
        transformed = source @ rotation.T + translation
        distances, indices = tree.query(transformed, k=1)
        candidates = distances <= maximum_correspondence_m
        if int(np.count_nonzero(candidates)) < 20:
            break
        cutoff = min(
            maximum_correspondence_m,
            float(np.quantile(distances[candidates], trim_quantile)),
        )
        keep = candidates & (distances <= cutoff)
        if int(np.count_nonzero(keep)) < 20:
            break
        delta_rotation, delta_translation = best_fit_rigid(
            transformed[keep], target[indices[keep]]
        )
        rotation = delta_rotation @ rotation
        translation = delta_rotation @ translation + delta_translation
        rmse = float(np.sqrt(np.mean(np.square(distances[keep]))))
        delta_yaw = math.atan2(delta_rotation[1, 0], delta_rotation[0, 0])
        if (
            abs(previous_rmse - rmse) < 1e-5
            and abs(delta_yaw) < 1e-5
            and float(np.linalg.norm(delta_translation)) < 1e-5
        ):
            break
        previous_rmse = rmse

    transformed = source @ rotation.T + translation
    distances, _ = tree.query(transformed, k=1)
    candidates = distances <= maximum_correspondence_m
    if int(np.count_nonzero(candidates)) < 20:
        return (
            math.atan2(rotation[1, 0], rotation[0, 0]),
            translation,
            math.inf,
            0,
            0.0,
        )
    cutoff = min(
        maximum_correspondence_m,
        float(np.quantile(distances[candidates], trim_quantile)),
    )
    keep = candidates & (distances <= cutoff)
    inliers = int(np.count_nonzero(keep))
    rmse = float(np.sqrt(np.mean(np.square(distances[keep]))))
    return (
        math.atan2(rotation[1, 0], rotation[0, 0]),
        translation,
        rmse,
        inliers,
        inliers / max(1, len(source)),
    )


def stationary_gyro_stats(
    imu: np.ndarray,
    motor: np.ndarray,
    command_timeout_sec: float,
) -> dict[str, float | int]:
    values = []
    for time_sec, gyro_z in imu:
        _, speed, fresh = delayed_command(
            motor, float(time_sec), timeout_sec=command_timeout_sec
        )
        if (not fresh) or abs(speed) < 0.5:
            values.append(float(gyro_z))
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0:
        return {"count": 0, "median_rad_s": 0.0, "trimmed_mean_rad_s": 0.0}
    lower, upper = np.quantile(array, [0.10, 0.90])
    trimmed = array[(array >= lower) & (array <= upper)]
    return {
        "count": int(array.size),
        "median_rad_s": float(np.median(array)),
        "trimmed_mean_rad_s": float(np.mean(trimmed)),
        "p10_rad_s": float(np.quantile(array, 0.10)),
        "p90_rad_s": float(np.quantile(array, 0.90)),
    }


def create_windows(
    scans: list[ScanRecord],
    imu: np.ndarray,
    motor: np.ndarray,
    stride: int,
    lidar_x_m: float,
    current_speed_gain: float,
    initial_gyro_bias: float,
    initial_gyro_sign: float,
    initial_gyro_scale: float,
    maximum_correspondence_m: float,
) -> list[CalibrationWindow]:
    windows: list[CalibrationWindow] = []
    for start_index in range(0, len(scans) - stride, stride):
        end_index = start_index + stride
        start = scans[start_index]
        end = scans[end_index]
        dt_sec = end.time_sec - start.time_sec
        mid_sec = 0.5 * (start.time_sec + end.time_sec)
        angle_command, speed_command, fresh = delayed_command(
            motor, mid_sec, delay_sec=0.10
        )
        gyro_raw = integrate_gyro(imu, start.time_sec, end.time_sec)
        gyro_yaw = (
            initial_gyro_sign
            * initial_gyro_scale
            * (gyro_raw - initial_gyro_bias * dt_sec)
        )
        curvature = float(
            np.interp(angle_command, STEERING_COMMANDS, STEERING_CURVATURES)
        )
        command_distance = speed_command * current_speed_gain * dt_sec
        command_yaw = command_distance * curvature
        if abs(gyro_yaw - command_yaw) < 0.35:
            initial_yaw = 0.55 * gyro_yaw + 0.45 * command_yaw
        else:
            initial_yaw = command_yaw
        initial_yaw = float(np.clip(initial_yaw, -0.8, 0.8))

        reason = ""
        if not 0.30 <= dt_sec <= 0.85:
            reason = "scan_dt"
        elif len(start.points) < 60 or len(end.points) < 60:
            reason = "few_scan_points"
        elif not fresh or abs(speed_command) < 2.5:
            reason = "no_moving_command"

        if reason:
            windows.append(
                CalibrationWindow(
                    start.time_sec,
                    end.time_sec,
                    dt_sec,
                    angle_command,
                    speed_command,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    gyro_raw,
                    math.inf,
                    0,
                    0.0,
                    math.inf,
                    False,
                    reason,
                )
            )
            continue

        # Long parallel walls are weakly observable in their longitudinal
        # direction.  A single ICP initialization can therefore return a very
        # precise-looking result that merely follows the command prior.  Run
        # several metric translation seeds and reject windows whose solutions
        # do not converge to the same local motion.
        solutions = []
        for seed_gain in (0.050, 0.065, 0.080, 0.095, 0.110):
            seed_distance = speed_command * seed_gain * dt_sec
            seed_base_translation = arc_translation(seed_distance, initial_yaw)
            seed_lidar_translation = base_to_lidar_relative(
                seed_base_translation, initial_yaw, lidar_x_m
            )
            solution = icp_2d(
                end.points,
                start.points,
                initial_yaw,
                seed_lidar_translation,
                maximum_correspondence_m,
            )
            solutions.append(solution)
        yaw, lidar_translation, rmse, inliers, inlier_ratio = min(
            solutions,
            key=lambda item: (
                item[2],
                -item[4],
            ),
        )
        solution_gains = []
        for seed_yaw, seed_lidar, seed_rmse, seed_inliers, _ in solutions:
            if not math.isfinite(seed_rmse) or seed_inliers < 20:
                continue
            seed_base = lidar_to_base_relative(
                seed_lidar, seed_yaw, lidar_x_m
            )
            seed_chord = float(np.linalg.norm(seed_base))
            if abs(seed_yaw) < 1e-5:
                seed_arc = seed_chord
            else:
                seed_denominator = 2.0 * math.sin(
                    min(abs(seed_yaw), math.pi - 1e-6) * 0.5
                )
                seed_arc = (
                    seed_chord
                    * abs(seed_yaw)
                    / max(abs(seed_denominator), 1e-6)
                )
            solution_gains.append(
                seed_arc / max(abs(speed_command) * dt_sec, 1e-9)
            )
        seed_gain_span = (
            float(np.max(solution_gains) - np.min(solution_gains))
            if solution_gains
            else math.inf
        )
        base_translation = lidar_to_base_relative(
            lidar_translation, yaw, lidar_x_m
        )
        chord_m = float(np.linalg.norm(base_translation))
        if abs(yaw) < 1e-5:
            arc_m = chord_m
        else:
            denominator = 2.0 * math.sin(min(abs(yaw), math.pi - 1e-6) * 0.5)
            arc_m = chord_m * abs(yaw) / max(abs(denominator), 1e-6)

        gain = arc_m / max(abs(speed_command) * dt_sec, 1e-9)
        if not math.isfinite(rmse) or rmse > 0.075:
            reason = "icp_rmse"
        elif inliers < 55:
            reason = "few_inliers"
        elif inlier_ratio < 0.28:
            reason = "low_inlier_ratio"
        elif seed_gain_span > 0.012:
            reason = "initialization_sensitive"
        elif not 0.035 <= gain <= 0.125:
            reason = "implausible_speed_gain"
        elif abs(yaw - initial_yaw) > 0.28:
            reason = "yaw_disagrees_with_prior"
        accepted = not reason
        windows.append(
            CalibrationWindow(
                start.time_sec,
                end.time_sec,
                dt_sec,
                angle_command,
                speed_command,
                float(base_translation[0]),
                float(base_translation[1]),
                yaw,
                arc_m,
                gyro_raw,
                rmse,
                inliers,
                inlier_ratio,
                seed_gain_span,
                accepted,
                reason,
            )
        )
    return windows


def robust_filter(values: np.ndarray, z_limit: float = 3.5) -> np.ndarray:
    finite = values[np.isfinite(values)]
    if finite.size < 3:
        return finite
    center = float(np.median(finite))
    mad = float(np.median(np.abs(finite - center)))
    if mad < 1e-12:
        return finite
    robust_sigma = 1.4826 * mad
    return finite[np.abs(finite - center) <= z_limit * robust_sigma]


def bootstrap_median_ci(
    values: np.ndarray,
    seed: int = 20260724,
    samples: int = 2000,
) -> tuple[float, float]:
    if values.size < 2:
        value = float(values[0]) if values.size else math.nan
        return value, value
    rng = np.random.default_rng(seed)
    medians = np.empty(samples, dtype=np.float64)
    for index in range(samples):
        draw = rng.choice(values, size=values.size, replace=True)
        medians[index] = np.median(draw)
    lower, upper = np.quantile(medians, [0.025, 0.975])
    return float(lower), float(upper)


def fit_gyro(
    windows: list[CalibrationWindow],
) -> dict[str, float | int]:
    accepted = [window for window in windows if window.accepted]
    if len(accepted) < 10:
        return {"count": len(accepted)}
    x_matrix = np.asarray(
        [[window.gyro_integral_raw_rad, window.dt_sec] for window in accepted],
        dtype=np.float64,
    )
    target = np.asarray(
        [window.lidar_yaw_rad for window in accepted], dtype=np.float64
    )
    weights = np.ones(target.size, dtype=np.float64)
    coefficients = np.zeros(2, dtype=np.float64)
    for _ in range(20):
        weighted_x = x_matrix * np.sqrt(weights[:, None])
        weighted_y = target * np.sqrt(weights)
        coefficients, _, _, _ = np.linalg.lstsq(weighted_x, weighted_y, rcond=None)
        residuals = target - x_matrix @ coefficients
        scale = 1.4826 * np.median(np.abs(residuals - np.median(residuals)))
        if scale < 1e-8:
            break
        normalized = np.abs(residuals) / (1.5 * scale)
        next_weights = np.ones_like(normalized)
        outside = normalized > 1.0
        next_weights[outside] = 1.0 / normalized[outside]
        if np.max(np.abs(next_weights - weights)) < 1e-4:
            weights = next_weights
            break
        weights = next_weights
    slope = float(coefficients[0])
    time_coefficient = float(coefficients[1])
    bias = -time_coefficient / slope if abs(slope) > 1e-9 else math.nan
    residuals = target - x_matrix @ coefficients
    correlation = float(
        np.corrcoef(target, x_matrix @ coefficients)[0, 1]
    )
    return {
        "count": len(accepted),
        "gyro_z_sign": -1.0 if slope < 0.0 else 1.0,
        "gyro_z_scale": abs(slope),
        "gyro_z_bias_rad_s": bias,
        "yaw_rmse_rad": float(np.sqrt(np.mean(np.square(residuals)))),
        "yaw_correlation": correlation,
    }


def fit_curvature(
    windows: list[CalibrationWindow],
    gyro_fit: dict[str, float | int],
) -> list[dict[str, float | int | None]]:
    accepted = [window for window in windows if window.accepted]
    sign = float(gyro_fit.get("gyro_z_sign", -1.0))
    scale = float(gyro_fit.get("gyro_z_scale", 1.0))
    bias = float(gyro_fit.get("gyro_z_bias_rad_s", 0.0))
    rows = []
    for command, existing in zip(STEERING_COMMANDS, STEERING_CURVATURES):
        half_width = 2.6 if abs(command) >= 35.0 else 4.0
        selected = [
            window
            for window in accepted
            if abs(window.angle_command - command) <= half_width
            and window.lidar_arc_m >= 0.055
        ]
        estimates = []
        for window in selected:
            gyro_yaw = sign * scale * (
                window.gyro_integral_raw_rad - bias * window.dt_sec
            )
            fused_yaw = 0.65 * window.lidar_yaw_rad + 0.35 * gyro_yaw
            estimates.append(fused_yaw / window.lidar_arc_m)
        filtered = robust_filter(np.asarray(estimates, dtype=np.float64))
        rows.append(
            {
                "steering_command": float(command),
                "existing_curvature_per_m": float(existing),
                "bag_curvature_per_m": (
                    float(np.median(filtered)) if filtered.size >= 5 else None
                ),
                "window_count": int(filtered.size),
            }
        )
    return rows


def cartographer_node_times(pbstream: Path) -> np.ndarray:
    executable = "/opt/ros/humble/bin/cartographer_pbstream"
    process = subprocess.Popen(
        [
            executable,
            "info",
            str(pbstream),
            "-all_debug_strings",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    timestamps = []
    inside_node = False
    assert process.stdout is not None
    for line in process.stdout:
        if "Serialized data: node {" in line:
            inside_node = True
        elif inside_node and "timestamp:" in line:
            ticks = int(line.split("timestamp:", 1)[1].strip())
            # Cartographer UniversalTime is 100 ns since year 1.
            timestamps.append(ticks * 1e-7 - 62_135_596_800.0)
            inside_node = False
    return_code = process.wait()
    if return_code != 0 or not timestamps:
        raise RuntimeError(
            f"could not extract trajectory-node times from {pbstream}"
        )
    return np.asarray(timestamps, dtype=np.float64)


def read_trajectory_csv(path: Path) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            rows.append((float(row["x"]), float(row["y"]), float(row["yaw"])))
    if len(rows) < 2:
        raise RuntimeError(f"trajectory CSV contains too few points: {path}")
    return np.asarray(rows, dtype=np.float64)


def cartographer_speed_cross_check(
    pbstream: Path,
    trajectory_csv: Path,
    motor: np.ndarray,
    node_step: int = 5,
) -> dict[str, Any]:
    times = cartographer_node_times(pbstream)
    poses = read_trajectory_csv(trajectory_csv)
    count_difference = int(times.size - poses.shape[0])
    count = min(times.size, poses.shape[0])
    times = times[:count]
    poses = poses[:count]
    gains = []
    for start_index in range(0, count - node_step):
        end_index = start_index + node_step
        start_sec = float(times[start_index])
        end_sec = float(times[end_index])
        dt_sec = end_sec - start_sec
        if dt_sec <= 0.05 or end_sec > float(motor[-1, 0]) + 0.30:
            continue
        _, speed_command, fresh = delayed_command(
            motor, 0.5 * (start_sec + end_sec), delay_sec=0.20
        )
        if not fresh or abs(speed_command) < 2.5:
            continue
        delta = poses[end_index, :2] - poses[start_index, :2]
        chord_m = float(np.linalg.norm(delta))
        yaw_delta = math.atan2(
            math.sin(poses[end_index, 2] - poses[start_index, 2]),
            math.cos(poses[end_index, 2] - poses[start_index, 2]),
        )
        if abs(yaw_delta) < 1e-5:
            arc_m = chord_m
        else:
            arc_m = (
                chord_m
                * abs(yaw_delta)
                / max(2.0 * abs(math.sin(0.5 * yaw_delta)), 1e-6)
            )
        gain = arc_m / (abs(speed_command) * dt_sec)
        if 0.03 <= gain <= 0.15:
            gains.append(gain)
    filtered = robust_filter(np.asarray(gains, dtype=np.float64))
    if filtered.size < 10:
        raise RuntimeError("too few valid Cartographer trajectory intervals")
    ci_low, ci_high = bootstrap_median_ci(filtered, seed=20260725)
    return {
        "method": (
            "Optimized Cartographer scan-to-submap trajectory divided by "
            "the active speed command"
        ),
        "pbstream": pbstream.name,
        "trajectory_csv": trajectory_csv.name,
        "node_time_count": int(times.size),
        "trajectory_pose_count": int(poses.shape[0]),
        "original_count_difference": count_difference,
        "node_step": node_step,
        "window_count": int(filtered.size),
        "speed_gain_mps_per_command": float(np.median(filtered)),
        "bootstrap_95pct_low": ci_low,
        "bootstrap_95pct_high": ci_high,
        "p10": float(np.quantile(filtered, 0.10)),
        "p90": float(np.quantile(filtered, 0.90)),
        "caveat": (
            "This trajectory used command odometry as a motion prior, so it "
            "is a global scan-matching cross-check rather than independent "
            "survey ground truth."
        ),
    }


def calibration_summary(
    scans: list[ScanRecord],
    imu: np.ndarray,
    motor: np.ndarray,
    ray_counts: np.ndarray,
    windows: list[CalibrationWindow],
    current_speed_gain: float,
    external_speed_gain: float,
    command_timeout_sec: float,
    cartographer_cross_check: dict[str, Any] | None,
) -> dict[str, Any]:
    accepted = [window for window in windows if window.accepted]
    gains = np.asarray(
        [
            window.lidar_arc_m
            / (abs(window.speed_command) * window.dt_sec)
            for window in accepted
        ],
        dtype=np.float64,
    )
    filtered_gains = robust_filter(gains)
    speed_gain = (
        float(np.median(filtered_gains)) if filtered_gains.size else math.nan
    )
    ci_low, ci_high = bootstrap_median_ci(filtered_gains)
    gyro = fit_gyro(windows)
    curvature = fit_curvature(windows, gyro)
    stationary = stationary_gyro_stats(imu, motor, command_timeout_sec)
    reasons: dict[str, int] = {}
    for window in windows:
        if not window.accepted:
            reasons[window.rejection_reason] = (
                reasons.get(window.rejection_reason, 0) + 1
            )
    speed_commands, speed_counts = np.unique(motor[:, 2], return_counts=True)
    speed_distribution = {
        f"{float(command):g}": int(count)
        for command, count in zip(speed_commands, speed_counts)
    }
    pairwise_stable = (
        filtered_gains.size >= 80
        and math.isfinite(speed_gain)
        and (ci_high - ci_low) / speed_gain <= 0.06
    )
    pairwise_vs_external_percent = (
        100.0 * (speed_gain / external_speed_gain - 1.0)
        if math.isfinite(speed_gain)
        else math.nan
    )
    selected_gain = current_speed_gain
    selected_source = "current real-vehicle VESC calibration"
    if cartographer_cross_check is not None:
        selected_gain = float(
            cartographer_cross_check["speed_gain_mps_per_command"]
        )
        selected_source = "Cartographer global scan-to-submap cross-check"
    estimator_spread_percent = (
        100.0 * abs(speed_gain - selected_gain) / selected_gain
        if math.isfinite(speed_gain)
        else math.nan
    )
    apply_automatically = (
        pairwise_stable
        and estimator_spread_percent <= 10.0
        and abs(selected_gain / external_speed_gain - 1.0) <= 0.10
    )
    return {
        "method": {
            "metric_reference": "LaserScan range values in metres",
            "drawing_or_cad_distance_used": False,
            "slam_accumulated_path_length_used_as_ground_truth": False,
            "notes": (
                "Pairwise trimmed ICP estimates local motion. Existing VESC "
                "dynamics values are retained as an independent cross-check."
            ),
        },
        "input": {
            "scan_count": len(scans),
            "imu_count": int(imu.shape[0]),
            "motor_count": int(motor.shape[0]),
            "scan_point_count_median": float(np.median(ray_counts)),
            "motor_speed_command_distribution": speed_distribution,
            "duration_sec": float(
                max(scans[-1].time_sec, imu[-1, 0], motor[-1, 0])
                - min(scans[0].time_sec, imu[0, 0], motor[0, 0])
            ),
        },
        "lidar_icp": {
            "window_count": len(windows),
            "accepted_window_count": len(accepted),
            "robust_speed_window_count": int(filtered_gains.size),
            "rejection_counts": reasons,
        },
        "speed_calibration": {
            "current_speed_gain_mps_per_command": current_speed_gain,
            "external_vesc_speed_gain_mps_per_command": external_speed_gain,
            "lidar_speed_gain_mps_per_command": speed_gain,
            "lidar_bootstrap_95pct_low": ci_low,
            "lidar_bootstrap_95pct_high": ci_high,
            "lidar_vs_current_percent": (
                100.0 * (speed_gain / current_speed_gain - 1.0)
                if math.isfinite(speed_gain)
                else math.nan
            ),
            "lidar_vs_external_vesc_percent": pairwise_vs_external_percent,
            "pairwise_icp_statistically_stable": pairwise_stable,
            "provisional_speed_gain_mps_per_command": selected_gain,
            "provisional_source": selected_source,
            "estimator_spread_percent": estimator_spread_percent,
            "safe_to_apply_automatically": apply_automatically,
            "decision": (
                "Do not replace the production value automatically; use the "
                "provisional value only in a low-speed validation config, "
                "then settle it with a surveyed straight-distance run."
                if not apply_automatically
                else "The independent checks agree closely enough to apply."
            ),
            "scope": (
                "Valid for speed command 3 in this bag. More commanded speeds "
                "are needed to identify nonlinearity."
            ),
        },
        "stationary_gyro": stationary,
        "gyro_calibration_from_lidar": gyro,
        "steering_curvature_cross_check": curvature,
        "cartographer_global_cross_check": cartographer_cross_check,
        "limitations": [
            "There is no wheel encoder, RTK, motion capture, or surveyed straight distance in this bag.",
            "Nearly every moving motor command is speed=3, so this cannot calibrate the full speed curve.",
            "Glass and long parallel walls can reduce pairwise ICP observability; quality filters reject weak windows.",
            "The result is a scan-matching prior, not a replacement for wheel encoder odometry.",
        ],
    }


def write_windows(path: Path, windows: Iterable[CalibrationWindow]) -> None:
    rows = list(windows)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(asdict(rows[0]).keys()))
        writer.writeheader()
        for row in rows:
            values = asdict(row)
            if not math.isfinite(float(values["icp_rmse_m"])):
                values["icp_rmse_m"] = ""
            writer.writerow(values)


def write_plot(
    path: Path,
    windows: list[CalibrationWindow],
    summary: dict[str, Any],
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    accepted = [window for window in windows if window.accepted]
    gains = np.asarray(
        [
            window.lidar_arc_m
            / (abs(window.speed_command) * window.dt_sec)
            for window in accepted
        ]
    )
    relative_time = np.asarray(
        [window.start_sec - windows[0].start_sec for window in accepted]
    )
    speed = summary["speed_calibration"]
    gyro = summary["gyro_calibration_from_lidar"]

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axes[0, 0].scatter(relative_time, gains, s=9, alpha=0.55)
    axes[0, 0].axhline(
        speed["lidar_speed_gain_mps_per_command"],
        color="tab:green",
        label="LiDAR median",
    )
    axes[0, 0].axhline(
        speed["current_speed_gain_mps_per_command"],
        color="tab:orange",
        linestyle="--",
        label="current",
    )
    cross_check = summary.get("cartographer_global_cross_check")
    if cross_check is not None:
        axes[0, 0].axhline(
            cross_check["speed_gain_mps_per_command"],
            color="tab:purple",
            linestyle="-.",
            label="Cartographer global",
        )
    axes[0, 0].set(
        title="Metric speed gain by accepted ICP window",
        xlabel="bag time (s)",
        ylabel="m/s per command",
    )
    axes[0, 0].legend()

    axes[0, 1].scatter(
        [window.angle_command for window in accepted],
        [
            window.lidar_yaw_rad / max(window.lidar_arc_m, 1e-6)
            for window in accepted
        ],
        s=9,
        alpha=0.35,
        label="LiDAR windows",
    )
    axes[0, 1].plot(
        STEERING_COMMANDS,
        STEERING_CURVATURES,
        "o-",
        color="tab:orange",
        label="current real-vehicle table",
    )
    axes[0, 1].set(
        title="Steering curvature cross-check",
        xlabel="steering command",
        ylabel="curvature (1/m)",
    )
    axes[0, 1].legend()

    if accepted and "gyro_z_scale" in gyro:
        measured = np.asarray([window.lidar_yaw_rad for window in accepted])
        predicted = np.asarray(
            [
                float(gyro["gyro_z_sign"])
                * float(gyro["gyro_z_scale"])
                * (
                    window.gyro_integral_raw_rad
                    - float(gyro["gyro_z_bias_rad_s"]) * window.dt_sec
                )
                for window in accepted
            ]
        )
        axes[1, 0].scatter(predicted, measured, s=9, alpha=0.5)
        bound = max(
            0.05,
            float(np.max(np.abs(np.concatenate((predicted, measured))))),
        )
        axes[1, 0].plot([-bound, bound], [-bound, bound], "k--", linewidth=1)
    axes[1, 0].set(
        title="IMU yaw fitted to LiDAR yaw",
        xlabel="corrected IMU yaw (rad)",
        ylabel="LiDAR yaw (rad)",
    )

    axes[1, 1].scatter(
        [window.icp_inlier_ratio for window in accepted],
        [window.icp_rmse_m for window in accepted],
        s=9,
        alpha=0.5,
    )
    axes[1, 1].set(
        title="Accepted ICP quality",
        xlabel="trimmed inlier ratio",
        ylabel="RMSE (m)",
    )
    figure.suptitle(
        "Xycar odometry calibration — LiDAR metric reference (CAD not used)",
        fontsize=14,
    )
    figure.savefig(path, dpi=160)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--imu-topic", default="/imu")
    parser.add_argument("--motor-topic", default="/xycar_motor")
    parser.add_argument("--range-min-m", type=float, default=0.30)
    parser.add_argument("--range-max-m", type=float, default=4.00)
    parser.add_argument("--ray-step", type=int, default=2)
    parser.add_argument("--scan-stride", type=int, default=5)
    parser.add_argument("--lidar-x-m", type=float, default=0.385)
    parser.add_argument("--current-speed-gain", type=float, default=0.080612)
    parser.add_argument("--external-speed-gain", type=float, default=0.080191)
    parser.add_argument("--initial-gyro-bias", type=float, default=0.0108)
    parser.add_argument("--initial-gyro-sign", type=float, default=-1.0)
    parser.add_argument("--initial-gyro-scale", type=float, default=1.4466)
    parser.add_argument("--maximum-correspondence-m", type=float, default=0.25)
    parser.add_argument("--command-timeout-sec", type=float, default=0.30)
    parser.add_argument("--cartographer-pbstream", type=Path)
    parser.add_argument("--trajectory-csv", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bag_path = args.bag.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"reading bag: {bag_path}")
    scans, imu, motor, ray_counts = read_bag(
        bag_path,
        args.scan_topic,
        args.imu_topic,
        args.motor_topic,
        args.range_min_m,
        args.range_max_m,
        args.ray_step,
    )
    print(
        f"loaded {len(scans)} scans, {len(imu)} IMU samples, "
        f"{len(motor)} motor commands"
    )
    stationary = stationary_gyro_stats(
        imu, motor, args.command_timeout_sec
    )
    initial_bias = float(stationary["trimmed_mean_rad_s"])
    print(f"stationary gyro trimmed mean: {initial_bias:.6f} rad/s")
    windows = create_windows(
        scans,
        imu,
        motor,
        args.scan_stride,
        args.lidar_x_m,
        args.current_speed_gain,
        initial_bias,
        args.initial_gyro_sign,
        args.initial_gyro_scale,
        args.maximum_correspondence_m,
    )
    cartographer_cross_check = None
    if args.cartographer_pbstream or args.trajectory_csv:
        if not args.cartographer_pbstream or not args.trajectory_csv:
            raise ValueError(
                "--cartographer-pbstream and --trajectory-csv must be used together"
            )
        print("extracting optimized Cartographer trajectory cross-check")
        cartographer_cross_check = cartographer_speed_cross_check(
            args.cartographer_pbstream.expanduser().resolve(),
            args.trajectory_csv.expanduser().resolve(),
            motor,
        )
    summary = calibration_summary(
        scans,
        imu,
        motor,
        ray_counts,
        windows,
        args.current_speed_gain,
        args.external_speed_gain,
        args.command_timeout_sec,
        cartographer_cross_check,
    )

    json_path = output_dir / "calibration_result.json"
    windows_path = output_dir / "lidar_icp_windows.csv"
    plot_path = output_dir / "calibration_report.png"
    json_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_windows(windows_path, windows)
    write_plot(plot_path, windows, summary)

    speed = summary["speed_calibration"]
    gyro = summary["gyro_calibration_from_lidar"]
    print(
        "LiDAR speed gain: "
        f"{speed['lidar_speed_gain_mps_per_command']:.6f} "
        f"(95% bootstrap "
        f"{speed['lidar_bootstrap_95pct_low']:.6f}.."
        f"{speed['lidar_bootstrap_95pct_high']:.6f})"
    )
    if "gyro_z_scale" in gyro:
        print(
            "gyro: "
            f"sign={gyro['gyro_z_sign']:+.0f}, "
            f"scale={gyro['gyro_z_scale']:.6f}, "
            f"bias={gyro['gyro_z_bias_rad_s']:.6f} rad/s"
        )
    print(f"wrote {json_path}")
    print(f"wrote {windows_path}")
    print(f"wrote {plot_path}")


if __name__ == "__main__":
    main()
