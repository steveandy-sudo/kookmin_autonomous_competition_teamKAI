from my_drive.sequential_hybrid_core import CandidateSource
from my_drive.sequential_hybrid_core import HybridState
from my_drive.sequential_hybrid_core import SequentialHybridConfig
from my_drive.sequential_hybrid_core import SequentialHybridController
from my_drive.sequential_hybrid_core import SequentialHybridInput


def make_controller(source=CandidateSource.RULE):
    return SequentialHybridController(
        SequentialHybridConfig(
            initial_source=source,
            start_delay_sec=0.0,
            candidate_fresh_sec=0.2,
            candidate_hold_sec=0.5,
            minimum_speed_command=3.0,
            maximum_speed_command=10.0,
            maximum_abs_angle_command=42.0,
        )
    )


def make_input(*, scan_fresh=True, rl_age=0.0, rule_age=0.0):
    return SequentialHybridInput(
        scan_fresh=scan_fresh,
        rl_command_age_sec=rl_age,
        rl_angle_command=5.0,
        rl_speed_command=4.0,
        rule_command_age_sec=rule_age,
        rule_angle_command=-20.0,
        rule_speed_command=6.0,
    )


def test_rule_source_stays_fixed_without_waypoint_switching():
    controller = make_controller()
    for _ in range(20):
        output = controller.step(make_input(), dt_sec=0.1)
    assert output.state == HybridState.RUNNING
    assert output.source == CandidateSource.RULE
    assert output.angle_command == -20.0
    assert output.speed_command == 6.0


def test_rl_source_can_be_selected_statically():
    controller = make_controller(CandidateSource.RL)
    output = controller.step(make_input(), dt_sec=0.1)
    assert output.source == CandidateSource.RL
    assert output.angle_command == 5.0
    assert output.speed_command == 4.0


def test_candidate_dropout_holds_then_stops_and_recovers():
    controller = make_controller()
    moving = controller.step(make_input(), dt_sec=0.1)
    assert moving.speed_command == 6.0
    held = controller.step(make_input(rule_age=0.3), dt_sec=0.1)
    assert held.speed_command == 6.0
    stopped = controller.step(make_input(rule_age=0.6), dt_sec=0.1)
    assert stopped.state == HybridState.SENSOR_STOP
    assert stopped.speed_command == 0.0
    recovered = controller.step(make_input(), dt_sec=0.1)
    assert recovered.state == HybridState.RUNNING


def test_stale_scan_stops_and_recovers():
    controller = make_controller()
    stopped = controller.step(make_input(scan_fresh=False), dt_sec=0.1)
    assert stopped.state == HybridState.SENSOR_STOP
    assert stopped.speed_command == 0.0
    recovered = controller.step(make_input(), dt_sec=0.1)
    assert recovered.state == HybridState.RUNNING


def test_command_limits_apply_without_changing_source():
    controller = make_controller()
    inputs = SequentialHybridInput(
        **{
            **make_input().__dict__,
            "rule_angle_command": -80.0,
            "rule_speed_command": 20.0,
        }
    )
    output = controller.step(inputs, dt_sec=0.1)
    assert output.source == CandidateSource.RULE
    assert output.angle_command == -42.0
    assert output.speed_command == 10.0


def test_invalid_speed_range_is_rejected():
    try:
        SequentialHybridConfig(
            minimum_speed_command=10.0,
            maximum_speed_command=3.0,
        )
    except ValueError:
        return
    raise AssertionError("reversed speed range must fail")
