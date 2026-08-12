import math

from shortcut_entry_review.spatial_entry_control import (
    blend_rule_and_w1_command,
    distance_blend_ratio,
    dynamic_blend_start_distance_m,
)


def test_distance_blend_is_spatial_and_bounded():
    assert distance_blend_ratio(
        remaining_distance_m=0.70,
        start_distance_m=0.55,
        full_control_distance_m=0.15,
    ) == 0.0
    middle = distance_blend_ratio(
        remaining_distance_m=0.35,
        start_distance_m=0.55,
        full_control_distance_m=0.15,
    )
    assert math.isclose(middle, 0.5)
    assert distance_blend_ratio(
        remaining_distance_m=0.10,
        start_distance_m=0.55,
        full_control_distance_m=0.15,
    ) == 1.0


def test_higher_speed_moves_activation_farther_out():
    slow = dynamic_blend_start_distance_m(
        speed_mps=0.2,
        full_control_distance_m=0.15,
        minimum_start_distance_m=0.55,
        maximum_start_distance_m=1.20,
        control_latency_sec=0.25,
        distance_margin_m=0.08,
    )
    fast = dynamic_blend_start_distance_m(
        speed_mps=2.0,
        full_control_distance_m=0.15,
        minimum_start_distance_m=0.55,
        maximum_start_distance_m=1.20,
        control_latency_sec=0.25,
        distance_margin_m=0.08,
    )
    assert slow == 0.55
    assert fast > slow


def test_rule_speed_is_never_blended_or_capped():
    angle, speed = blend_rule_and_w1_command(
        rule_angle=3.0,
        rule_speed=19.0,
        w1_angle=-25.0,
        blend_ratio=0.60,
    )
    assert math.isclose(angle, -13.8)
    assert speed == 19.0
