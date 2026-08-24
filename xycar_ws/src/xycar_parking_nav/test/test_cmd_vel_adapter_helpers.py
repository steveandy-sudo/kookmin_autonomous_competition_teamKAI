import pytest

from xycar_parking_nav.command_core import should_hold_for_steering_settle, slew


def test_slew_limits_rate():
    assert slew(0.0, 10.0, 4.0, 0.5) == pytest.approx(2.0)
    assert slew(0.0, -10.0, 4.0, 0.5) == pytest.approx(-2.0)


def test_slew_reaches_near_target_without_overshoot():
    assert slew(1.0, 1.1, 4.0, 0.5) == pytest.approx(1.1)


def test_steering_is_pre_aligned_before_traction_starts():
    assert should_hold_for_steering_settle(
        applied_speed_command=0.0,
        desired_steering_command=30.0,
        next_steering_command=8.0,
        tolerance_command=3.0,
    )


@pytest.mark.parametrize("speed", [4.0, -4.0])
def test_steering_slew_does_not_pulse_traction_after_motion_starts(speed):
    assert not should_hold_for_steering_settle(
        applied_speed_command=speed,
        desired_steering_command=-30.0,
        next_steering_command=8.0,
        tolerance_command=3.0,
    )
