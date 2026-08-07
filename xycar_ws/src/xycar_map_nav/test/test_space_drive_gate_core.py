import pytest

from xycar_map_nav.space_drive_gate_core import SpaceDriveGateController
from xycar_map_nav.space_drive_gate_core import steering_speed_limit


def test_space_toggles_continuous_fixed_speed_and_stop():
    controller = SpaceDriveGateController(speed_command=5.0)
    stopped = controller.command(
        candidate_fresh=True,
        candidate_angle_command=8.0,
        candidate_speed_command=20.0,
    )
    assert (stopped.angle_command, stopped.speed_command) == (0.0, 0.0)

    assert controller.toggle() is True
    running = controller.command(
        candidate_fresh=True,
        candidate_angle_command=8.0,
        candidate_speed_command=20.0,
    )
    assert (running.angle_command, running.speed_command) == (8.0, 5.0)

    assert controller.toggle() is False
    stopped_again = controller.command(
        candidate_fresh=True,
        candidate_angle_command=8.0,
        candidate_speed_command=20.0,
    )
    assert (stopped_again.angle_command, stopped_again.speed_command) == (
        0.0,
        0.0,
    )


def test_gate_does_not_override_selector_stop_or_stale_command():
    controller = SpaceDriveGateController(speed_command=4.0)
    controller.toggle()
    stale = controller.command(
        candidate_fresh=False,
        candidate_angle_command=15.0,
        candidate_speed_command=6.0,
    )
    selector_stop = controller.command(
        candidate_fresh=True,
        candidate_angle_command=15.0,
        candidate_speed_command=0.0,
    )
    assert stale.speed_command == 0.0
    assert stale.reason == "CANDIDATE_STALE"
    assert selector_stop.speed_command == 0.0
    assert selector_stop.reason == "SELECTOR_STOP"


def test_steering_only_keeps_candidate_angle_and_forces_zero_speed():
    controller = SpaceDriveGateController(
        speed_command=0.0,
        steering_only=True,
    )
    controller.toggle()
    output = controller.command(
        candidate_fresh=True,
        candidate_angle_command=-31.0,
        candidate_speed_command=0.0,
    )
    assert output.angle_command == -31.0
    assert output.speed_command == 0.0
    assert output.reason == "SPACE_RUN"


def test_speed_and_angle_are_clamped():
    controller = SpaceDriveGateController(
        speed_command=20.0,
        maximum_speed_command=10.0,
        maximum_abs_angle_command=42.0,
        adaptive_steering_speed_enabled=False,
    )
    controller.toggle()
    output = controller.command(
        candidate_fresh=True,
        candidate_angle_command=80.0,
        candidate_speed_command=20.0,
    )
    assert output.angle_command == 42.0
    assert output.speed_command == 10.0


def test_selector_can_apply_a_lower_avoidance_speed_cap():
    controller = SpaceDriveGateController(speed_command=8.0)
    controller.toggle()
    output = controller.command(
        candidate_fresh=True,
        candidate_angle_command=-12.0,
        candidate_speed_command=4.0,
    )
    assert output.angle_command == -12.0
    assert output.speed_command == 4.0


def test_default_hard_limit_allows_command_30():
    controller = SpaceDriveGateController(speed_command=30.0)
    controller.toggle()
    output = controller.command(
        candidate_fresh=True,
        candidate_angle_command=0.0,
        candidate_speed_command=30.0,
    )
    assert output.speed_command == 30.0


def test_steering_speed_limit_holds_cap_then_slows_to_8():
    common = dict(
        speed_cap_command=10.0,
        turn_speed_command=8.0,
        slowdown_start_angle_command=20.0,
        full_slowdown_angle_command=42.0,
    )
    assert steering_speed_limit(0.0, **common) == 10.0
    assert steering_speed_limit(20.0, **common) == 10.0
    assert steering_speed_limit(31.0, **common) == 9.0
    assert steering_speed_limit(-42.0, **common) == 8.0


def test_integrated_gate_applies_steering_speed_limit():
    controller = SpaceDriveGateController(speed_command=10.0)
    controller.toggle()
    output = controller.command(
        candidate_fresh=True,
        candidate_angle_command=31.0,
        candidate_speed_command=10.0,
    )
    assert output.speed_command == pytest.approx(9.0)


def test_lower_cone_or_avoidance_speed_still_wins():
    controller = SpaceDriveGateController(speed_command=10.0)
    controller.toggle()
    output = controller.command(
        candidate_fresh=True,
        candidate_angle_command=31.0,
        candidate_speed_command=6.0,
    )
    assert output.speed_command == 6.0


def test_adaptive_steering_speed_can_be_disabled():
    controller = SpaceDriveGateController(
        speed_command=10.0,
        adaptive_steering_speed_enabled=False,
    )
    controller.toggle()
    output = controller.command(
        candidate_fresh=True,
        candidate_angle_command=42.0,
        candidate_speed_command=10.0,
    )
    assert output.speed_command == 10.0
