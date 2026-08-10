from my_drive.traffic_shortcut import SignalGateConfig
from my_drive.traffic_shortcut import SignalGateController
from my_drive.traffic_shortcut import SignalState


def test_initial_green_releases_drive_and_arms_only_after_signal_clears():
    gate = SignalGateController(SignalGateConfig(start_clear_sec=1.0))
    gate.observe_detections(
        now_sec=0.0, red=True, green=False, yellow=False, left=False
    )
    assert gate.command(
        now_sec=0.1,
        candidate_age_sec=0.0,
        candidate_angle=3.0,
        candidate_speed=8.0,
    )[1] == 0.0
    gate.observe_start_green(0.2)
    assert gate.command(
        now_sec=0.3,
        candidate_age_sec=0.0,
        candidate_angle=3.0,
        candidate_speed=8.0,
    )[:2] == (3.0, 8.0)
    gate.update(1.1)
    assert not gate.reencounter_armed
    gate.update(1.2)
    assert gate.reencounter_armed


def test_red_reencounter_holds_until_confirmed_green():
    gate = SignalGateController(SignalGateConfig(signal_required_frames=2))
    gate.observe_start_green(0.0)
    gate.update(2.0)
    for now in (3.0, 3.1):
        gate.observe_detections(
            now_sec=now, red=True, green=False, yellow=False, left=False
        )
    assert gate.state == SignalState.RED_HOLD
    for now in (3.2, 3.3):
        gate.observe_detections(
            now_sec=now, red=False, green=True, yellow=False, left=False
        )
    assert gate.state == SignalState.RUNNING


def test_green_left_turn_is_used_once_and_returns_to_lane_command():
    gate = SignalGateController(
        SignalGateConfig(
            start_clear_sec=0.5,
            shortcut_duration_sec=1.0,
            shortcut_left_command=-18.0,
            shortcut_lane_command_weight=0.5,
        )
    )
    gate.observe_start_green(0.0)
    gate.update(1.0)
    gate.observe_detections(
        now_sec=2.0, red=False, green=True, yellow=False, left=True
    )
    angle, speed, _ = gate.command(
        now_sec=2.1,
        candidate_age_sec=0.0,
        candidate_angle=4.0,
        candidate_speed=12.0,
    )
    assert (angle, speed) == (-16.0, 8.0)
    assert gate.shortcut_taken
    assert gate.command(
        now_sec=3.1,
        candidate_age_sec=0.0,
        candidate_angle=4.0,
        candidate_speed=12.0,
    )[:2] == (4.0, 12.0)
