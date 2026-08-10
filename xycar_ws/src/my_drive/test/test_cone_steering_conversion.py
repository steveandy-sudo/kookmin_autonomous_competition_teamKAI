import pytest

from my_drive.sequential_hybrid_driver import interpolate_command


ACTUAL_ANGLES_DEG = [0.0, 4.0, 10.0, 16.0, 26.0]
STEERING_COMMANDS = [0.0, 10.0, 20.0, 30.0, 42.0]


@pytest.mark.parametrize(
    ("target", "expected"),
    [
        (-26.0, -42.0),
        (-16.0, -30.0),
        (-10.0, -20.0),
        (-4.0, -10.0),
        (0.0, 0.0),
        (4.0, 10.0),
        (10.0, 20.0),
        (16.0, 30.0),
        (26.0, 42.0),
    ],
)
def test_main_vehicle_cone_steering_calibration(target, expected):
    assert interpolate_command(
        target,
        ACTUAL_ANGLES_DEG,
        STEERING_COMMANDS,
    ) == pytest.approx(expected)


def test_cone_steering_calibration_interpolates_between_measurements():
    assert interpolate_command(
        13.0,
        ACTUAL_ANGLES_DEG,
        STEERING_COMMANDS,
    ) == pytest.approx(25.0)


def test_cone_steering_calibration_saturates_outside_measured_range():
    assert interpolate_command(
        50.0,
        ACTUAL_ANGLES_DEG,
        STEERING_COMMANDS,
    ) == pytest.approx(42.0)
