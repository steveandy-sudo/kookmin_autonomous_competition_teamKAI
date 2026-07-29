import math
from pathlib import Path

import numpy as np

from xycar_map_nav.control_core import (
    alignment_limited_speed_command,
    filtered_steering_command,
    nearest_path_index,
    rate_limited_speed_command,
    stanley_path_command,
    steering_command_for_curvature,
)
from xycar_map_nav.grid_planner import (
    load_map_grid,
    load_path_csv,
    smooth_path_points,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
STEERING_COMMANDS = [
    -42.0, -40.0, -35.0, -30.0, -20.0, -10.0, 0.0,
    10.0, 20.0, 30.0, 35.0, 40.0, 42.0,
]
STEERING_CURVATURES = [
    1.502435, 1.383494, 1.174860, 0.922781, 0.552809,
    0.194230, 0.0, -0.556883, -0.959829, -1.369323,
    -1.601706, -1.853397, -1.939236,
]


def _route():
    path_csv = (
        SOURCE_ROOT
        / "xycar_gazebo_bridge/maps/slam_glass_balanced/"
        "one_lap_path.csv"
    )
    points = load_path_csv(path_csv, closed=True, spacing_m=0.10)
    map_grid = load_map_grid(
        SOURCE_ROOT
        / "xycar_gazebo_bridge/maps/slam_glass_balanced/"
        "slam_glass_balanced.yaml",
        inflation_radius_m=0.23,
    )
    return tuple(
        smooth_path_points(
            map_grid,
            points,
            closed=True,
            data_weight=0.10,
            smooth_weight=0.40,
            iterations=100,
        )
    )


def _actual_curvature(command: float) -> float:
    return float(
        np.interp(command, STEERING_COMMANDS, STEERING_CURVATURES)
    )


def _simulate_lap(
    speed_command: float,
    *,
    stanley_gain: float = 1.20,
    feedforward_gain: float = 0.50,
    curvature_preview_m: float = 0.10,
    heading_preview_m: float = 0.0,
) -> tuple[bool, float, int]:
    points = _route()
    x, y = points[0]
    yaw = math.atan2(
        points[1][1] - points[0][1],
        points[1][0] - points[0][0],
    )
    dt = 0.05
    speed_mps = 0.0
    yaw_rate_radps = 0.0
    steering_command = 0.0
    speed_command_output = 0.0
    delayed_commands = [0.0, 0.0]
    nearest = 0
    maximum_error = 0.0
    strong_reversals = 0
    previous_strong_sign = 0
    wrapped = False

    for _ in range(6000):
        nearest = nearest_path_index(
            points,
            x,
            y,
            previous_index=nearest,
            closed=True,
        )
        command_kwargs = dict(
            points=points,
            nearest_index=nearest,
            vehicle_x=x,
            vehicle_y=y,
            vehicle_yaw=yaw,
            speed_mps=speed_mps,
            lateral_offset_m=0.0,
            closed=True,
            wheelbase_m=0.32,
            front_axle_offset_m=0.16,
            steering_delay_sec=0.10,
            curvature_feedforward_gain=feedforward_gain,
            heading_window_m=0.40,
            heading_preview_m=heading_preview_m,
            curvature_window_m=0.55,
            curvature_preview_m=curvature_preview_m,
            maximum_steering_angle_rad=0.62,
        )
        command = stanley_path_command(
            stanley_gain=stanley_gain,
            stanley_softening_mps=0.45,
            heading_gain=1.0,
            **command_kwargs,
        )
        straight = abs(command.path_curvature_per_m) <= 0.16
        if straight:
            command = stanley_path_command(
                stanley_gain=0.45,
                stanley_softening_mps=0.80,
                heading_gain=0.55,
                **command_kwargs,
            )
        damping = 0.45 if straight else 0.12
        damped_steering = command.steering_angle_rad - damping * (
            yaw_rate_radps - speed_mps * command.path_curvature_per_m
        )
        damped_steering = max(-0.62, min(0.62, damped_steering))
        target_steering = steering_command_for_curvature(
            math.tan(damped_steering) / 0.32,
            STEERING_COMMANDS,
            STEERING_CURVATURES,
        )
        steering_command = filtered_steering_command(
            target_steering,
            steering_command,
            dt_sec=dt,
            rate_limit_command_per_sec=90.0 if straight else 300.0,
            time_constant_sec=0.16 if straight else 0.04,
        )
        delayed_commands.append(steering_command)
        applied_curvature = _actual_curvature(delayed_commands.pop(0))

        path_curvature = abs(command.path_curvature_per_m)
        curve_ratio = min(1.0, path_curvature / 0.80)
        target_speed_command = (
            float(speed_command)
            + (3.0 - float(speed_command)) * curve_ratio
        )
        if path_curvature > 1.0e-4:
            target_speed_command = min(
                target_speed_command,
                math.sqrt(0.90 / path_curvature) / 0.080612,
            )
        target_speed_command = alignment_limited_speed_command(
            target_speed_command,
            3.0,
            cross_track_error_m=command.cross_track_error_m,
            heading_error_rad=command.heading_error_rad,
            cross_track_soft_m=0.05,
            cross_track_hard_m=0.20,
            heading_soft_rad=0.08,
            heading_hard_rad=0.35,
        )
        speed_command_output = rate_limited_speed_command(
            target_speed_command,
            speed_command_output,
            dt_sec=dt,
            acceleration_rate_command_per_sec=5.0,
            deceleration_rate_command_per_sec=30.0,
        )
        target_speed = speed_command_output * 0.080612
        speed_mps += (target_speed - speed_mps) * dt / (0.19 + dt)
        x += speed_mps * math.cos(yaw) * dt
        y += speed_mps * math.sin(yaw) * dt
        yaw_rate_radps = speed_mps * applied_curvature
        yaw += yaw_rate_radps * dt
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        maximum_error = max(
            maximum_error,
            math.hypot(x - points[nearest][0], y - points[nearest][1]),
        )
        strong_sign = (
            1 if steering_command >= 12.0
            else -1 if steering_command <= -12.0
            else 0
        )
        if (
            strong_sign
            and previous_strong_sign
            and strong_sign != previous_strong_sign
            and abs(command.path_curvature_per_m) <= 0.16
        ):
            strong_reversals += 1
        if strong_sign:
            previous_strong_sign = strong_sign

        if nearest > int(len(points) * 0.80):
            wrapped = True
        if wrapped and nearest < int(len(points) * 0.10):
            return True, maximum_error, strong_reversals
        if maximum_error > 0.45:
            break
    return False, maximum_error, strong_reversals


def test_measured_dynamics_completes_without_straight_oscillation():
    for speed_command in (5.0, 7.0, 10.0):
        completed, maximum_error, strong_reversals = _simulate_lap(
            speed_command
        )
        assert completed, f"speed={speed_command}, cte={maximum_error:.3f}"
        assert maximum_error < 0.21
        assert strong_reversals <= 2
