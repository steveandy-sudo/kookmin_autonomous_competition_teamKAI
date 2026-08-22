import math

import pytest

from xycar_parking_nav.command_core import (
    DirectionChangeGuard,
    Footprint,
    MinimumCommandPulseController,
    MotorCalibration,
    motor_command_to_twist,
    predicted_stop_distance,
    scan_points_in_base,
    swept_footprint_collision,
    twist_to_motor_command,
)


@pytest.fixture
def calibration():
    return MotorCalibration(
        speed_gain_mps_per_command=0.08,
        minimum_moving_command=4.0,
        maximum_forward_command=4.0,
        maximum_reverse_command=4.0,
        steering_commands=(-42.0, 0.0, 42.0),
        steering_curvatures=(1.5, 0.0, -1.94),
    )


def test_twist_conversion_uses_measured_inverse_map(calibration):
    straight = twist_to_motor_command(0.24, 0.0, calibration)
    assert straight.steering_command == pytest.approx(0.0)
    assert straight.speed_command == pytest.approx(4.0)
    assert straight.requested_speed_command == pytest.approx(3.0)

    left = twist_to_motor_command(0.24, 0.24 * 1.5, calibration)
    assert left.steering_command == pytest.approx(-42.0)
    assert left.speed_command == pytest.approx(4.0)
    assert left.requested_speed_command == pytest.approx(3.0)


def test_reverse_preserves_path_curvature(calibration):
    command = twist_to_motor_command(-0.24, 0.24, calibration)
    assert command.curvature == pytest.approx(-1.0)
    assert command.steering_command > 0.0
    assert command.speed_command == pytest.approx(-4.0)
    assert command.requested_speed_command == pytest.approx(-3.0)


def test_unreachable_curvature_reports_saturated_physical_value(calibration):
    command = twist_to_motor_command(0.24, 0.24 * 3.0, calibration)

    assert command.steering_command == pytest.approx(-42.0)
    assert command.curvature == pytest.approx(1.5)


def test_motor_shadow_round_trip_preserves_executable_pulse_speed(calibration):
    requested_linear = -0.32
    requested_angular = 0.32
    command = twist_to_motor_command(
        requested_linear,
        requested_angular,
        calibration,
    )
    linear, angular = motor_command_to_twist(
        command.steering_command,
        command.speed_command,
        calibration,
    )

    assert linear == pytest.approx(requested_linear)
    assert angular == pytest.approx(requested_angular)


def test_sub_threshold_command_is_realized_as_four_zero_pulses():
    controller = MinimumCommandPulseController(4.0, 0.05)
    period_sec = 0.05
    outputs = [
        controller.filter(2.0, index * period_sec)[0]
        for index in range(201)
    ]

    assert set(outputs) == {0.0, 4.0}
    average = sum(outputs[:-1]) / len(outputs[:-1])
    assert average == pytest.approx(2.0, abs=0.08)
    longest_on_run = max(
        len(run)
        for run in "".join("1" if output else "0" for output in outputs).split("0")
    )
    assert longest_on_run * period_sec <= 0.10 + 1.0e-9


def test_pulse_controller_preserves_reverse_and_resets_safely():
    controller = MinimumCommandPulseController(4.0, 0.10)

    assert controller.filter(-1.0, 0.0) == (-4.0, "pulse_on")
    assert controller.filter(0.0, 0.05) == (0.0, "stopped")
    assert controller.filter(1.0, 1.0) == (4.0, "pulse_on")


def test_pulse_controller_passes_command_four_continuously():
    controller = MinimumCommandPulseController(4.0, 0.10)

    assert controller.filter(4.0, 0.0) == (4.0, "continuous")
    assert controller.filter(-4.0, 0.1) == (-4.0, "continuous")


def test_minimum_command_cannot_exceed_command_limits(calibration):
    with pytest.raises(ValueError, match="must not exceed"):
        MotorCalibration(
            speed_gain_mps_per_command=calibration.speed_gain_mps_per_command,
            minimum_moving_command=4.1,
            maximum_forward_command=4.0,
            maximum_reverse_command=4.0,
            steering_commands=calibration.steering_commands,
            steering_curvatures=calibration.steering_curvatures,
        )


def test_invalid_motor_shadow_fails_stopped(calibration):
    assert motor_command_to_twist(math.nan, 3.0, calibration) == (0.0, 0.0)


def test_rotate_in_place_is_rejected(calibration):
    command = twist_to_motor_command(0.0, 1.0, calibration)
    assert command.speed_command == 0.0
    assert command.reason == "rotate_in_place_rejected"


def test_direction_change_guard_inserts_dwell():
    guard = DirectionChangeGuard(0.4)
    assert guard.filter(3.0, 1.0) == (3.0, "ok")
    assert guard.filter(-3.0, 1.1) == (0.0, "direction_change_dwell")
    assert guard.filter(-3.0, 1.49) == (0.0, "direction_change_dwell")
    assert guard.filter(-3.0, 1.51) == (-3.0, "ok")


def test_direction_guard_reset_clears_direction_history():
    guard = DirectionChangeGuard(0.4)
    assert guard.filter(3.0, 1.0) == (3.0, "ok")
    guard.reset()
    assert guard.filter(-3.0, 2.0) == (-3.0, "ok")


def test_stop_distance_includes_reaction_and_braking():
    distance = predicted_stop_distance(
        0.4,
        reaction_time_sec=0.25,
        braking_deceleration_mps2=0.8,
        minimum_projection_m=0.05,
        maximum_projection_m=1.0,
    )
    assert distance == pytest.approx(0.2)


def test_scan_transform_includes_laser_offset():
    points = scan_points_in_base(
        [1.0],
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.1,
        range_max=5.0,
        laser_x=0.065,
        laser_y=0.0,
        laser_yaw=0.0,
    )
    assert points == pytest.approx([(1.065, 0.0)])


def test_swept_body_detects_forward_obstacle():
    result = swept_footprint_collision(
        [(0.32, 0.0)],
        speed_mps=0.24,
        curvature=0.0,
        footprint=Footprint(-0.45, 0.08, 0.15),
        margin_m=0.05,
        reaction_time_sec=0.25,
        braking_deceleration_mps2=0.8,
        minimum_projection_m=0.25,
        maximum_projection_m=0.8,
        sample_step_m=0.025,
    )
    assert result.collision


def test_swept_body_ignores_obstacle_behind_when_moving_forward():
    result = swept_footprint_collision(
        [(-0.8, 0.0)],
        speed_mps=0.24,
        curvature=0.0,
        footprint=Footprint(-0.45, 0.08, 0.15),
        margin_m=0.04,
        reaction_time_sec=0.2,
        braking_deceleration_mps2=0.8,
        minimum_projection_m=0.2,
        maximum_projection_m=0.8,
        sample_step_m=0.025,
    )
    assert not result.collision


def test_swept_body_checks_curved_path():
    # A left arc reaches positive Y even though the point is outside a straight corridor.
    result = swept_footprint_collision(
        [(0.42, 0.25)],
        speed_mps=0.30,
        curvature=1.5,
        footprint=Footprint(-0.20, 0.20, 0.12),
        margin_m=0.04,
        reaction_time_sec=0.4,
        braking_deceleration_mps2=0.6,
        minimum_projection_m=0.45,
        maximum_projection_m=0.8,
        sample_step_m=0.02,
    )
    assert result.collision
