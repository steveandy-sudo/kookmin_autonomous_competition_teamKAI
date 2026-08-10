import numpy as np

from my_control.rule_command_adapter_core import (
    adaptive_speed_for_steering,
    avoidance_offset_for_obstacle,
    fit_yellow_reference,
    ObstacleOffsetLatch,
    ObstacleSide,
    obstacle_side_from_reference,
)


def test_speed_holds_cap_until_20_degrees_then_slows_to_8_at_42():
    common = dict(
        straight_speed_command=10.0,
        turn_speed_command=8.0,
        slowdown_start_angle_command=20.0,
        full_slowdown_angle_command=42.0,
    )
    assert adaptive_speed_for_steering(0.0, **common) == 10.0
    assert adaptive_speed_for_steering(20.0, **common) == 10.0
    assert adaptive_speed_for_steering(31.0, **common) == 9.0
    assert adaptive_speed_for_steering(-42.0, **common) == 8.0
    assert adaptive_speed_for_steering(60.0, **common) == 8.0


def test_turn_speed_never_exceeds_input_speed_cap():
    assert adaptive_speed_for_steering(
        42.0,
        straight_speed_command=5.0,
        turn_speed_command=8.0,
        slowdown_start_angle_command=20.0,
        full_slowdown_angle_command=42.0,
    ) == 5.0


def test_obstacle_offset_always_moves_to_the_opposite_side():
    assert avoidance_offset_for_obstacle(ObstacleSide.LEFT, 0.20) == -0.20
    assert avoidance_offset_for_obstacle(ObstacleSide.RIGHT, 0.20) == 0.20


def test_sparse_yellow_fragments_are_extended_for_side_classification():
    mask = np.zeros((144, 256), dtype=np.uint8)
    for row in (20, 21, 60, 61, 100, 101):
        column = 100 + row // 4
        mask[row, column - 1 : column + 2] = 255
    coefficients = fit_yellow_reference(mask)
    left, basis = obstacle_side_from_reference(
        object_center_x=420.0,
        object_bottom_y=700.0,
        image_width=1280,
        image_height=1024,
        yellow_coefficients=coefficients,
        yellow_width=256,
        yellow_height=144,
    )
    right, _ = obstacle_side_from_reference(
        object_center_x=900.0,
        object_bottom_y=700.0,
        image_width=1280,
        image_height=1024,
        yellow_coefficients=coefficients,
        yellow_width=256,
        yellow_height=144,
    )
    assert basis == "yellow"
    assert left == ObstacleSide.LEFT
    assert right == ObstacleSide.RIGHT


def test_image_centre_is_a_fallback_when_yellow_is_missing():
    side, basis = obstacle_side_from_reference(
        object_center_x=100.0,
        object_bottom_y=200.0,
        image_width=640,
        image_height=480,
        yellow_coefficients=None,
        yellow_width=0,
        yellow_height=0,
    )
    assert side == ObstacleSide.LEFT
    assert basis == "image_center"


def test_one_frame_latches_side_and_two_seconds_clear_it():
    latch = ObstacleOffsetLatch(shift_m=0.20, release_delay_sec=2.0)
    assert latch.observe(ObstacleSide.RIGHT, 10.0)
    assert latch.offset_m == 0.20
    assert not latch.update(11.99)
    assert latch.update(12.0)
    assert latch.offset_m == 0.0


def test_latched_side_does_not_flip_during_the_same_avoidance_episode():
    latch = ObstacleOffsetLatch(shift_m=0.20, release_delay_sec=2.0)
    latch.observe(ObstacleSide.LEFT, 0.0)
    assert not latch.observe(ObstacleSide.RIGHT, 0.1)
    assert latch.side == ObstacleSide.LEFT
    assert latch.offset_m == -0.20
