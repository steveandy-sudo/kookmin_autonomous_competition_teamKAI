import cv2
import numpy as np

from shortcut_entry_review.sequence_entry_core import (
    EntrySequencePhase,
    SequenceEntryConfig,
    SequenceAwareEntrySelector,
    pixels_to_vehicle_path,
)


WIDTH = 640
HEIGHT = 660


def make_phase_masks(*, include_y1: bool, aligned: bool = False):
    white = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    yellow = np.zeros_like(white)
    if aligned:
        cv2.line(white, (170, 600), (158, 250), 12, cv2.LINE_AA)
        cv2.line(yellow, (360, 600), (350, 250), 12, cv2.LINE_AA)
    else:
        # W1 and Y1 turn toward vehicle-left when read near-to-far.  W2 and
        # Y2 are deliberately thicker and more central but have the opposite
        # tangent, matching the annotated A/B topology.
        cv2.line(white, (170, 540), (78, 330), 12, cv2.LINE_AA)
        cv2.line(white, (250, 610), (325, 245), 22, cv2.LINE_AA)
        cv2.line(yellow, (450, 610), (500, 280), 22, cv2.LINE_AA)
        if include_y1:
            cv2.line(yellow, (355, 540), (258, 330), 12, cv2.LINE_AA)
    return white, yellow


def test_w1_first_frame_disables_rule_but_lone_y2_is_never_substituted():
    selector = SequenceAwareEntrySelector()
    white, yellow = make_phase_masks(include_y1=False)

    result = selector.process(white, yellow)

    assert result.phase == EntrySequencePhase.W1_LOCKED
    assert result.ready
    assert result.path_valid
    assert result.w1 is not None
    assert result.w1.direction_dx_dy > 0.15
    assert result.y1 is None
    assert result.used_synthetic_y1
    assert "W2/Y2 ignored" in result.reason


def test_y1_requires_sequence_confirmation_then_tracks_left_yellow_branch():
    selector = SequenceAwareEntrySelector()
    selector.process(*make_phase_masks(include_y1=False))

    first = selector.process(*make_phase_masks(include_y1=True))
    second = selector.process(*make_phase_masks(include_y1=True))

    assert first.phase == EntrySequencePhase.W1_LOCKED
    assert first.y1 is None
    assert second.phase == EntrySequencePhase.Y1_LOCKED
    assert second.y1 is not None
    assert second.w1 is not None
    separation = second.y1.mean_x_ratio - second.w1.mean_x_ratio
    assert 0.17 <= separation <= 0.46
    assert second.y1.direction_dx_dy > 0.15
    assert not second.used_synthetic_y1


def test_entry_path_uses_sixty_percent_w1_and_forty_percent_y1():
    config = SequenceEntryConfig(w1_path_weight=0.60)
    selector = SequenceAwareEntrySelector(config)
    result = selector.process(*make_phase_masks(include_y1=False))

    assert result.w1 is not None
    assert result.used_synthetic_y1
    for x_px, y_px in result.path_pixels:
        row_ratio = y_px / HEIGHT
        white_x = result.w1.x_ratio_at(row_ratio)
        synthetic_yellow_x = (
            white_x + config.expected_pair_separation_ratio
        )
        expected_x = 0.60 * white_x + 0.40 * synthetic_yellow_x
        midpoint_x = 0.50 * (white_x + synthetic_yellow_x)
        assert np.isclose(x_px / WIDTH, expected_x)
        assert x_px / WIDTH < midpoint_x


def test_w1_identity_does_not_jump_to_nearer_w2_before_y1_exists():
    selector = SequenceAwareEntrySelector()

    first_white = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    first_yellow = np.zeros_like(first_white)
    cv2.line(first_white, (165, 525), (90, 340), 12, cv2.LINE_AA)
    cv2.line(first_white, (225, 610), (275, 250), 22, cv2.LINE_AA)
    cv2.line(first_yellow, (410, 610), (450, 280), 22, cv2.LINE_AA)
    first = selector.process(first_white, first_yellow)
    assert first.w1 is not None

    # W1 moves left by more than W2 moves.  A nearest-x-only tracker would
    # incorrectly switch to W2 here, exactly as it did on saved frame 2.
    second_white = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    second_yellow = np.zeros_like(second_white)
    cv2.line(second_white, (125, 525), (35, 340), 12, cv2.LINE_AA)
    cv2.line(second_white, (205, 610), (265, 250), 22, cv2.LINE_AA)
    cv2.line(second_yellow, (415, 610), (455, 280), 22, cv2.LINE_AA)
    second = selector.process(second_white, second_yellow)

    assert second.w1 is not None
    assert second.w1.direction_dx_dy > 0.15
    assert second.w1.mean_x_ratio < 0.22
    assert second.y1 is None
    assert second.used_synthetic_y1


def test_short_y1_component_below_general_hough_span_can_lock():
    selector = SequenceAwareEntrySelector()
    white, yellow = make_phase_masks(include_y1=False)
    selector.process(white, yellow)

    with_short_y1 = yellow.copy()
    # A 22 px-high semantic dash is deliberately shorter than the general
    # 0.045 * 660 ~= 30 px Hough/component cutoff.
    cv2.line(with_short_y1, (264, 398), (254, 387), 4, cv2.LINE_AA)
    first = selector.process(white, with_short_y1)
    second = selector.process(white, with_short_y1)

    assert first.y1 is None
    assert second.y1 is not None
    assert second.phase == EntrySequencePhase.Y1_LOCKED
    assert second.y1.vertical_span_ratio < 0.045
    assert second.y1.direction_dx_dy > 0.15
    assert second.y1.mean_x_ratio < 0.55


def test_locked_pair_tracks_when_both_boundaries_rotate_past_zero_slope():
    selector = SequenceAwareEntrySelector()
    selector.process(*make_phase_masks(include_y1=False))
    selector.process(*make_phase_masks(include_y1=True))
    locked = selector.process(*make_phase_masks(include_y1=True))
    assert locked.y1 is not None

    white = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    yellow = np.zeros_like(white)
    cv2.line(white, (165, 600), (205, 300), 12, cv2.LINE_AA)
    cv2.line(yellow, (355, 600), (395, 300), 12, cv2.LINE_AA)

    result = selector.process(white, yellow)

    assert result.w1 is not None
    assert result.y1 is not None
    assert result.w1.direction_dx_dy < 0.0
    assert result.y1.direction_dx_dy < 0.0
    assert result.path_valid


def test_forward_alignment_hands_entry_to_existing_shortcut_cruise_by_order():
    selector = SequenceAwareEntrySelector()
    selector.process(*make_phase_masks(include_y1=False))
    selector.process(*make_phase_masks(include_y1=True))
    selector.process(*make_phase_masks(include_y1=True))

    results = [
        selector.process(*make_phase_masks(include_y1=True, aligned=True))
        for _ in range(3)
    ]

    assert results[-1].phase == EntrySequencePhase.CRUISE_HANDOFF
    assert results[-1].cruise_handoff
    assert all(result.path_valid for result in results)


def test_bev_path_uses_vehicle_forward_and_left_positive_coordinates():
    path = pixels_to_vehicle_path(
        ((320.0, 659.0), (160.0, 329.5), (320.0, 0.0)),
        width=WIDTH,
        height=HEIGHT,
    )

    assert np.all(np.diff(path[:, 0]) >= 0.0)
    assert np.isclose(path[0, 0], 0.0)
    assert np.isclose(path[-1, 0], 1.5)
    assert np.max(path[:, 1]) > 0.0
