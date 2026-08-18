from shortcut_entry_review.shortcut_candidate_mux_node import (
    entry_speed_cap,
    enforce_directional_hold,
    held_w1_candidate,
    rule_handoff_candidate,
    rule_search_candidate,
    semantic_entry_candidate,
    shortcut_core_candidate,
)


def test_semantic_entry_blends_only_steering_and_preserves_rule_speed():
    command = semantic_entry_candidate(
        rule_command=(10.0, 18.0),
        entry_command=(-30.0, 4.0),
        steering_blend=0.25,
        phase=2.0,
    )

    assert command[0] == 0.0
    assert command[1] == 18.0


def test_shortcut_core_preserves_rule_speed_through_handoff():
    command = shortcut_core_candidate(
        rule_command=(10.0, 22.0),
        legacy_command=(-20.0, 1.0, 0.0, 3.0),
    )

    assert command == (-20.0, 22.0, 0.0, 3.0)


def test_rule_handoff_preserves_rule_command_and_marks_entry_done():
    command = rule_handoff_candidate(rule_command=(12.5, 18.0))

    assert command == (12.5, 18.0, 1.0, 4.0)


def test_w1_search_retains_rule_instead_of_publishing_safe_stop():
    command = rule_search_candidate(
        rule_command=(-7.0, 20.0), phase=0.0
    )

    assert command == (-7.0, 20.0, 0.0, 10.0)


def test_temporary_loss_holds_last_w1_angle_with_current_rule_speed():
    command = held_w1_candidate(
        rule_command=(3.0, 12.0), held_angle=-36.0, phase=3.0
    )

    assert command == (-36.0, 12.0, 0.0, 13.0)


def test_entry_hold_prevents_w1_from_flipping_out_of_left_turn():
    assert enforce_directional_hold(
        candidate_angle=1.33, hold_command=-30.0
    ) == -30.0
    assert enforce_directional_hold(
        candidate_angle=-42.0, hold_command=-30.0
    ) == -42.0


def test_shortcut_entry_speed_caps_rule_without_accelerating_degraded_path():
    assert entry_speed_cap(rule_speed=12.0, maximum_entry_speed=9.0) == 9.0
    assert entry_speed_cap(rule_speed=4.0, maximum_entry_speed=9.0) == 4.0
    assert entry_speed_cap(rule_speed=0.0, maximum_entry_speed=9.0) == 0.0
