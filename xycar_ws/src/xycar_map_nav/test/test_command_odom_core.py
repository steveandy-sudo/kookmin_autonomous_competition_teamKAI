import math

from xycar_map_nav.command_odom_core import (
    OdomState,
    curvature_for_steering_command,
    integrate_ackermann,
)


def test_straight_command_odometry():
    state = integrate_ackermann(
        OdomState(),
        speed_mps=0.25,
        curvature_per_m=0.0,
        dt_sec=2.0,
    )
    assert math.isclose(state.x, 0.5)
    assert math.isclose(state.y, 0.0)
    assert math.isclose(state.yaw, 0.0)


def test_turning_command_odometry_changes_heading_and_lateral_position():
    state = integrate_ackermann(
        OdomState(),
        speed_mps=0.5,
        curvature_per_m=1.0,
        dt_sec=1.0,
    )
    assert state.x > 0.0
    assert state.y > 0.0
    assert math.isclose(state.yaw, 0.5)


def test_real_steering_map_interpolation_and_sign():
    commands = [-42.0, 0.0, 42.0]
    curvatures = [1.5, 0.0, -1.9]
    assert curvature_for_steering_command(
        -21.0, commands, curvatures
    ) > 0.0
    assert curvature_for_steering_command(
        21.0, commands, curvatures
    ) < 0.0
