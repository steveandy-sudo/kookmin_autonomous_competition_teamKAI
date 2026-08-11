from xycar_rule_drive.keyboard_teleop import (
    hold_speed_command,
    hold_steering_command,
    ramped_steering_command,
)


def test_hold_left_commands_full_left_lock() -> None:
    assert hold_steering_command(True, False, 42.0) == -42.0


def test_hold_right_commands_full_right_lock() -> None:
    assert hold_steering_command(False, True, 42.0) == 42.0


def test_releasing_keys_centers_steering() -> None:
    assert hold_steering_command(False, False, 42.0) == 0.0


def test_holding_both_keys_centers_steering() -> None:
    assert hold_steering_command(True, True, 42.0) == 0.0


def test_steering_ramps_to_left_lock_in_ten_unit_steps() -> None:
    angle = 0.0
    observed = []
    for _ in range(4):
        angle = ramped_steering_command(angle, True, False, 30.0, 10.0)
        observed.append(angle)
    assert observed == [-10.0, -20.0, -30.0, -30.0]


def test_released_steering_ramps_back_to_center() -> None:
    angle = 30.0
    observed = []
    for _ in range(4):
        angle = ramped_steering_command(angle, False, False, 30.0, 10.0)
        observed.append(angle)
    assert observed == [20.0, 10.0, 0.0, 0.0]


def test_opposite_key_moves_from_current_angle_without_jumping() -> None:
    assert ramped_steering_command(-42.0, False, True, 42.0, 10.0) == -32.0


def test_holding_drive_key_commands_fixed_speed() -> None:
    assert hold_speed_command(True, 17.0) == 17.0


def test_releasing_drive_key_stops() -> None:
    assert hold_speed_command(False, 17.0) == 0.0
