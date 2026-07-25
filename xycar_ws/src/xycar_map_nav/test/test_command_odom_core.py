import math

from xycar_map_nav.command_odom_core import (
    OdomState,
    curvature_for_steering_command,
    first_order_response,
    integrate_ackermann,
    integrate_planar_velocity,
    integrate_with_heading,
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


def test_first_order_response_is_bounded_and_uses_time_constant():
    halfway = first_order_response(
        0.0,
        1.0,
        dt_sec=math.log(2.0),
        time_constant_sec=1.0,
    )
    assert math.isclose(halfway, 0.5)
    assert 0.0 < first_order_response(
        0.0, 1.0, dt_sec=0.1, time_constant_sec=1.0
    ) < 1.0


def test_gyro_yaw_can_be_integrated_independently_from_command_curvature():
    state = integrate_planar_velocity(
        OdomState(),
        speed_mps=0.25,
        yaw_rate_rad_s=-0.5,
        dt_sec=1.0,
    )
    assert state.x > 0.0
    assert state.y < 0.0
    assert math.isclose(state.yaw, -0.5)


def test_external_heading_controls_translation_and_wraparound():
    state = integrate_with_heading(
        OdomState(yaw=math.radians(179.0)),
        speed_mps=1.0,
        heading_rad=math.radians(-179.0),
        dt_sec=1.0,
    )
    assert state.x < -0.99
    assert abs(state.y) < 0.05
    assert math.isclose(state.yaw, math.radians(-179.0))
