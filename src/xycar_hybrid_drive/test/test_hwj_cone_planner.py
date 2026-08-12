import math

import numpy as np

from xycar_hybrid_drive.hwj_cone_planner import (
    ConePlannerConfig,
    HwjConePlanner,
    wheel_angle_to_xycar_command,
)


def test_pairs_each_cone_boundary_once():
    planner = HwjConePlanner(ConePlannerConfig())
    left = [(0.50, 0.39), (0.85, 0.39), (1.20, 0.39)]
    right = [(0.52, -0.39), (0.87, -0.39), (1.18, -0.39)]

    midpoints = planner.calculate_midpoints(left, right)

    assert planner.midpoint_source == "paired"
    assert np.allclose(
        midpoints,
        [(0.51, 0.0), (0.86, 0.0), (1.19, 0.0)],
    )


def test_single_boundary_offsets_toward_corridor_center():
    planner = HwjConePlanner(ConePlannerConfig())
    left = [(0.50, 0.39), (0.85, 0.45), (1.20, 0.55)]

    midpoints = planner.calculate_midpoints(left, [])

    assert planner.midpoint_source == "left_offset"
    assert len(midpoints) == 3
    assert all(
        abs(midpoint[1]) < abs(boundary[1])
        for midpoint, boundary in zip(midpoints, left)
    )


def test_hwj_physical_angle_is_converted_exactly_once():
    assert wheel_angle_to_xycar_command(0.0) == 0.0
    assert wheel_angle_to_xycar_command(4.0) == 10.0
    assert wheel_angle_to_xycar_command(10.0) == 20.0
    assert wheel_angle_to_xycar_command(-16.0) == -30.0
    assert wheel_angle_to_xycar_command(26.0) == 42.0
    assert math.isclose(wheel_angle_to_xycar_command(7.0), 15.0)
