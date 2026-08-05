import math

from xycar_map_nav.sequential_hybrid_core import CandidateSource
from xycar_map_nav.sequential_hybrid_core import GateDefinition
from xycar_map_nav.sequential_hybrid_core import HybridState
from xycar_map_nav.sequential_hybrid_core import SequentialHybridConfig
from xycar_map_nav.sequential_hybrid_core import SequentialHybridController
from xycar_map_nav.sequential_hybrid_core import SequentialHybridInput


def _config():
    return SequentialHybridConfig(
        gates=(
            GateDefinition(
                "RULE_START",
                CandidateSource.RULE,
                primary_sector_index=0,
                primary_min_m=0.0,
                primary_max_m=2.0,
            ),
            GateDefinition(
                "IL_RETURN",
                CandidateSource.RL,
                primary_sector_index=1,
                primary_min_m=5.0,
                primary_max_m=20.0,
                secondary_sector_index=0,
                secondary_min_m=0.0,
                secondary_max_m=1.5,
            ),
        ),
        start_delay_sec=0.0,
        minimum_stage_sec=0.0,
        required_gate_frames=2,
        candidate_fresh_sec=0.2,
        candidate_hold_sec=0.5,
        transition_blend_sec=0.0,
    )


def _input(
    distances=(3.0, 3.0, 3.0),
    *,
    new=True,
    scan_fresh=True,
    rl_age=0.0,
    rule_age=0.0,
):
    return SequentialHybridInput(
        scan_fresh=scan_fresh,
        scan_sample_is_new=new,
        sector_distances_m=distances,
        rl_command_age_sec=rl_age,
        rl_angle_command=5.0,
        rl_speed_command=4.0,
        rule_command_age_sec=rule_age,
        rule_angle_command=-20.0,
        rule_speed_command=3.0,
    )


def test_gate_requires_consecutive_new_scans_and_selects_rule():
    controller = SequentialHybridController(_config())
    first = controller.step(_input((1.8, 3.0, 3.0)), dt_sec=0.1)
    assert first.source == CandidateSource.RL
    repeated = controller.step(
        _input((1.8, 3.0, 3.0), new=False), dt_sec=0.1
    )
    assert repeated.source == CandidateSource.RL
    changed = controller.step(_input((1.8, 3.0, 3.0)), dt_sec=0.1)
    assert changed.source == CandidateSource.RULE
    assert changed.angle_command == -20.0
    assert changed.next_gate_name == "IL_RETURN"


def test_gate_does_not_advance_until_space_gate_is_armed():
    controller = SequentialHybridController(_config())
    paused = SequentialHybridInput(
        **{
            **_input((1.8, 3.0, 3.0)).__dict__,
            "gate_advancement_enabled": False,
        }
    )
    for _ in range(5):
        output = controller.step(paused, dt_sec=0.1)
    assert output.source == CandidateSource.RL
    assert output.next_gate_number == 1
    assert output.gate_streak == 0

    armed = SequentialHybridInput(
        **{**paused.__dict__, "gate_advancement_enabled": True}
    )
    controller.step(armed, dt_sec=0.1)
    changed = controller.step(armed, dt_sec=0.1)
    assert changed.source == CandidateSource.RULE
    assert changed.next_gate_number == 2


def test_secondary_condition_is_required_for_return_to_il():
    controller = SequentialHybridController(_config())
    controller.step(_input((1.8, 3.0, 3.0)), dt_sec=0.1)
    controller.step(_input((1.8, 3.0, 3.0)), dt_sec=0.1)
    controller.step(_input((2.0, 6.0, 3.0)), dt_sec=0.1)
    no_change = controller.step(_input((2.0, 6.0, 3.0)), dt_sec=0.1)
    assert no_change.source == CandidateSource.RULE
    controller.step(_input((1.0, 6.0, 3.0)), dt_sec=0.1)
    changed = controller.step(_input((1.0, 6.0, 3.0)), dt_sec=0.1)
    assert changed.source == CandidateSource.RL
    assert changed.lap_count == 1


def test_candidate_dropout_holds_then_stops_and_recovers():
    controller = SequentialHybridController(_config())
    moving = controller.step(_input(), dt_sec=0.1)
    assert moving.speed_command == 4.0
    held = controller.step(_input(rl_age=0.3), dt_sec=0.1)
    assert held.speed_command == 4.0
    stopped = controller.step(_input(rl_age=0.6), dt_sec=0.1)
    assert stopped.state == HybridState.SENSOR_STOP
    assert stopped.speed_command == 0.0
    recovered = controller.step(_input(), dt_sec=0.1)
    assert recovered.state == HybridState.RUNNING


def test_close_front_sample_does_not_latch_the_selector():
    controller = SequentialHybridController(_config())
    output = controller.step(_input((3.0, 3.0, 0.1)), dt_sec=0.1)
    assert output.state == HybridState.RUNNING
    assert output.speed_command == 4.0


def test_invalid_gate_ranges_are_rejected():
    try:
        GateDefinition(
            "BAD",
            CandidateSource.RL,
            primary_sector_index=0,
            primary_min_m=2.0,
            primary_max_m=1.0,
        )
    except ValueError:
        return
    raise AssertionError("reversed gate range must fail")


def test_missing_sector_does_not_match():
    controller = SequentialHybridController(_config())
    result = controller.step(
        _input((math.inf, 3.0, 3.0)), dt_sec=0.1
    )
    assert result.gate_streak == 0


def test_gate_can_require_a_near_straight_rule_command():
    config = SequentialHybridConfig(
        gates=(
            GateDefinition(
                "STRAIGHT_RETURN",
                CandidateSource.RL,
                primary_sector_index=0,
                primary_min_m=5.0,
                primary_max_m=20.0,
                rule_abs_angle_max=8.0,
            ),
        ),
        initial_source=CandidateSource.RULE,
        start_delay_sec=0.0,
        minimum_stage_sec=0.0,
        required_gate_frames=1,
        transition_blend_sec=0.0,
    )
    controller = SequentialHybridController(config)
    curved = _input((6.0, 3.0, 3.0))
    curved = SequentialHybridInput(
        **{**curved.__dict__, "rule_angle_command": 20.0}
    )
    assert controller.step(curved, dt_sec=0.1).source == CandidateSource.RULE
    straight = SequentialHybridInput(
        **{**curved.__dict__, "rule_angle_command": 4.0}
    )
    assert controller.step(straight, dt_sec=0.1).source == CandidateSource.RL


def test_user_selected_start_waypoint_infers_incoming_controller():
    controller = SequentialHybridController(_config())
    controller.select_start_waypoint(2)
    assert controller.gate_index == 1
    assert controller.source == CandidateSource.RULE
    assert controller.lap_count == 0

    controller.select_start_waypoint(1)
    assert controller.gate_index == 0
    assert controller.source == CandidateSource.RL


def test_start_waypoint_rejects_out_of_range_number():
    controller = SequentialHybridController(_config())
    try:
        controller.select_start_waypoint(3)
    except ValueError:
        return
    raise AssertionError("out-of-range start waypoint must fail")
