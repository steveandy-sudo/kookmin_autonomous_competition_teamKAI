from xycar_hybrid_drive.three_level_curve_rule import (
    ContinuousCurveController,
    ThreeLevelCurveController,
    cad_curve_rule_command,
    cad_pure_pursuit_curve_command,
    command_for_measured_curvature,
    feedback_curve_candidate,
    quantize_three_level_curve_command,
)


def test_continuous_curve_controller_limits_sign_reversal_without_quantizing():
    controller = ContinuousCurveController(alpha=1.0, rate_limit_command=8.0)
    assert controller.update(-20.0) == -8.0
    assert controller.update(-20.0) == -16.0
    assert controller.update(20.0) == -8.0
    assert controller.update(20.0) == 0.0
    assert controller.update(20.0) == 8.0


def test_continuous_curve_controller_preserves_fractional_command():
    controller = ContinuousCurveController(alpha=1.0, rate_limit_command=42.0)
    assert controller.update(13.25) == 13.25


def test_measured_curvature_inverse_preserves_calibration_points():
    assert command_for_measured_curvature(0.0) == 0.0
    assert command_for_measured_curvature(0.552809) == -20.0
    assert command_for_measured_curvature(-0.959829) == 20.0


def test_cad_curve_rule_combines_feedforward_and_pose_feedback():
    centered = cad_curve_rule_command(
        0.552809,
        cross_track_error_m=0.0,
        heading_error_rad=0.0,
    )
    outside = cad_curve_rule_command(
        0.552809,
        cross_track_error_m=-0.10,
        heading_error_rad=-0.10,
    )
    assert centered == -20.0
    assert outside < centered


def test_cad_pure_pursuit_command_turns_toward_lookahead_point():
    left = cad_pure_pursuit_curve_command(
        vehicle_x=0.0,
        vehicle_y=0.0,
        vehicle_yaw_rad=0.0,
        target_x=1.0,
        target_y=0.25,
    )
    right = cad_pure_pursuit_curve_command(
        vehicle_x=0.0,
        vehicle_y=0.0,
        vehicle_yaw_rad=0.0,
        target_x=1.0,
        target_y=-0.25,
    )
    assert left < 0.0
    assert right > 0.0


def test_curve_rule_uses_only_three_requested_commands():
    outputs = {
        quantize_three_level_curve_command(value)
        for value in (-100.0, -8.0, -2.0, 0.0, 2.0, 8.0, 100.0)
    }
    assert outputs == {-42.0, 0.0, 42.0}


def test_curve_rule_zero_band_prevents_small_sign_chatter():
    assert quantize_three_level_curve_command(-2.0) == 0.0
    assert quantize_three_level_curve_command(2.0) == 0.0
    assert quantize_three_level_curve_command(-2.01) == -42.0
    assert quantize_three_level_curve_command(2.01) == 42.0


def test_curve_rule_rejects_nonpositive_full_lock():
    try:
        quantize_three_level_curve_command(5.0, full_lock_command=0.0)
    except ValueError as exc:
        assert "positive" in str(exc)
    else:
        raise AssertionError("non-positive full lock must be rejected")


def test_pose_feedback_flips_rule_only_after_error_deadband():
    assert feedback_curve_candidate(
        -18.0,
        route_command_sign=-1,
        cross_track_error_m=0.10,
        heading_error_rad=0.05,
    ) == 18.0
    assert feedback_curve_candidate(
        7.0,
        route_command_sign=-1,
        cross_track_error_m=0.0,
        heading_error_rad=0.0,
    ) == -12.0


def test_pulse_controller_uses_only_requested_commands():
    controller = ThreeLevelCurveController()
    outputs = [
        controller.update(value)
        for value in (-18.0,) * 8 + (0.0,) * 2 + (18.0,) * 8
    ]
    assert set(outputs) <= {-42.0, 0.0, 42.0}
    assert -42.0 in outputs
    assert 42.0 in outputs


def test_pulse_controller_preserves_average_continuous_steering():
    controller = ThreeLevelCurveController()
    outputs = [controller.update(10.0) for _ in range(420)]
    assert abs(sum(outputs) / len(outputs) - 10.0) <= 0.1


def test_pulse_controller_reacts_to_s_curve_sign_reversal():
    controller = ThreeLevelCurveController()
    for _ in range(12):
        controller.update(18.0)
    outputs = [controller.update(-18.0) for _ in range(3)]
    assert -42.0 in outputs
    assert 42.0 not in outputs


def test_stable_weak_s_curve_signal_gets_one_assist_pulse():
    controller = ThreeLevelCurveController(
        duty_scale=0.85,
        minimum_zero_frames=1,
        maximum_zero_frames=2,
        forced_pulse_min_command=8.0,
        stable_pulse_min_command=5.0,
        stable_pulse_frames=2,
    )
    assert controller.update(-7.6) == 0.0
    assert controller.update(0.0) == 0.0
    assert controller.update(7.2) == 0.0
    assert controller.update(7.2) == 42.0


def test_pulse_controller_duty_scale_limits_full_lock_time():
    controller = ThreeLevelCurveController(
        duty_scale=0.45,
        minimum_zero_frames=0,
    )
    outputs = [controller.update(42.0) for _ in range(200)]
    assert set(outputs) <= {0.0, 42.0}
    assert abs(sum(outputs) / len(outputs) - 42.0 * 0.45) <= 0.21


def test_pulse_controller_inserts_two_zero_frames_between_full_locks():
    controller = ThreeLevelCurveController(
        duty_scale=1.0,
        minimum_zero_frames=2,
    )
    outputs = [controller.update(-42.0) for _ in range(20)]
    assert -42.0 in outputs
    pulse_indices = [
        index for index, value in enumerate(outputs) if value == -42.0
    ]
    assert all(
        right - left >= 3
        for left, right in zip(pulse_indices, pulse_indices[1:])
    )


def test_deployment_pulse_spacing_never_holds_adjacent_full_lock():
    controller = ThreeLevelCurveController(
        duty_scale=0.75,
        minimum_zero_frames=1,
        maximum_zero_frames=1,
        forced_pulse_min_command=10.0,
    )
    outputs = [controller.update(-23.0) for _ in range(20)]
    pulse_indices = [
        index for index, value in enumerate(outputs) if value == -42.0
    ]
    assert all(
        right - left >= 2
        for left, right in zip(pulse_indices, pulse_indices[1:])
    )
    assert all(
        right - left == 2
        for left, right in zip(pulse_indices, pulse_indices[1:])
    )


def test_strong_curve_pulses_are_evenly_distributed():
    controller = ThreeLevelCurveController(
        duty_scale=0.75,
        minimum_zero_frames=2,
        maximum_zero_frames=2,
        forced_pulse_min_command=10.0,
    )
    outputs = [controller.update(-18.0) for _ in range(10)]
    assert outputs == [
        -42.0,
        0.0,
        0.0,
        -42.0,
        0.0,
        0.0,
        -42.0,
        0.0,
        0.0,
        -42.0,
    ]
