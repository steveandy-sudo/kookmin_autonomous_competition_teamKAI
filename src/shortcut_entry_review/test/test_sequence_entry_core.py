import math

import cv2
import numpy as np

from shortcut_entry_review.sequence_entry_core import (
    EntrySequencePhase,
    LineHypothesis,
    SequenceEntryConfig,
    SequenceAwareEntrySelector,
    branch_point_distance_m,
    estimate_branch_point,
    pixels_to_vehicle_path,
    select_entry_handoff_condition,
    spatial_steering_gate,
    update_w1_steering_delay_counts,
    w1_entry_trigger_ready,
    w1_steering_delay_ready,
)


WIDTH = 640
HEIGHT = 660


def test_red_intersection_can_trigger_before_legacy_distance_gate():
    assert w1_entry_trigger_ready(
        branch_distance_m=1.03,
        spatial_gate_ready=False,
        start_on_intersection=True,
    )
    assert not w1_entry_trigger_ready(
        branch_distance_m=1.03,
        spatial_gate_ready=False,
        start_on_intersection=False,
    )
    assert not w1_entry_trigger_ready(
        branch_distance_m=math.inf,
        spatial_gate_ready=True,
        start_on_intersection=True,
    )


def make_phase_masks(
    *,
    include_y1: bool,
    aligned: bool = False,
    include_w1: bool = True,
):
    white = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    yellow = np.zeros_like(white)
    if aligned:
        cv2.line(white, (170, 600), (158, 250), 12, cv2.LINE_AA)
        cv2.line(yellow, (360, 600), (350, 250), 12, cv2.LINE_AA)
    else:
        # W1 and Y1 turn toward vehicle-left when read near-to-far.  W2 and
        # Y2 are deliberately thicker and more central but have the opposite
        # tangent, matching the annotated A/B topology.
        if include_w1:
            cv2.line(white, (170, 540), (78, 330), 12, cv2.LINE_AA)
        cv2.line(white, (250, 610), (325, 245), 22, cv2.LINE_AA)
        cv2.line(yellow, (450, 610), (500, 280), 22, cv2.LINE_AA)
        if include_y1:
            cv2.line(yellow, (355, 540), (258, 330), 12, cv2.LINE_AA)
    return white, yellow


def lock_w1(selector: SequenceAwareEntrySelector):
    selector.process(
        *make_phase_masks(include_y1=False, include_w1=False)
    )
    w2_locked = selector.process(
        *make_phase_masks(include_y1=False, include_w1=False)
    )
    first_branch = selector.process(*make_phase_masks(include_y1=False))
    w1_locked = selector.process(*make_phase_masks(include_y1=False))
    return w2_locked, first_branch, w1_locked


def lock_y1(selector: SequenceAwareEntrySelector):
    lock_w1(selector)
    first = selector.process(*make_phase_masks(include_y1=True))
    second = selector.process(*make_phase_masks(include_y1=True))
    return first, second


def test_w2_is_tracked_before_left_diverging_w1_can_lock():
    selector = SequenceAwareEntrySelector()
    w2_locked, first_branch, result = lock_w1(selector)

    assert w2_locked.phase == EntrySequencePhase.LEFT4_ARMED
    assert w2_locked.w2 is not None
    assert w2_locked.w1 is None
    assert not w2_locked.ready
    # The long, clearly separated, left-positive branch satisfies the strict
    # one-frame fast path.  W2 still had to be locked first.
    assert first_branch.phase == EntrySequencePhase.W1_LOCKED
    assert selector.w1_fast_locked
    assert result.phase == EntrySequencePhase.W1_LOCKED
    assert result.ready
    assert result.path_valid
    assert result.w1 is not None
    assert result.w2 is not None
    assert result.w1.direction_dx_dy > 0.15
    assert result.w1.mean_x_ratio < result.w2.mean_x_ratio
    assert result.y1 is None
    assert not result.used_synthetic_y1
    assert "W2/Y2 ignored" in result.reason


def test_w1_confirmation_keeps_matching_hit_across_one_missing_frame():
    selector = SequenceAwareEntrySelector(
        SequenceEntryConfig(w1_fast_lock_enabled=False)
    )
    no_w1 = make_phase_masks(include_y1=False, include_w1=False)
    with_w1 = make_phase_masks(include_y1=False, include_w1=True)

    selector.process(*no_w1)
    selector.process(*no_w1)
    first = selector.process(*with_w1)
    missing = selector.process(*no_w1)
    locked = selector.process(*with_w1)

    assert first.phase == EntrySequencePhase.LEFT4_ARMED
    assert missing.phase == EntrySequencePhase.LEFT4_ARMED
    assert locked.phase == EntrySequencePhase.W1_LOCKED
    assert selector.w1_confirmations == 2
    assert not selector.w1_fast_locked


def test_w1_separation_is_measured_only_in_shared_observed_y_range():
    def line(
        slope: float,
        intercept: float,
        minimum_y: float,
        maximum_y: float,
    ) -> LineHypothesis:
        midpoint_y = 0.5 * (minimum_y + maximum_y)
        return LineHypothesis(
            color="white",
            coefficients=(slope, intercept),
            mean_x_ratio=slope * midpoint_y + intercept,
            mean_y_ratio=midpoint_y,
            near_x_ratio=slope * maximum_y + intercept,
            far_x_ratio=slope * minimum_y + intercept,
            minimum_y_ratio=minimum_y,
            maximum_y_ratio=maximum_y,
            direction_dx_dy=slope,
            heading_from_vehicle_rad=0.0,
            vertical_span_ratio=maximum_y - minimum_y,
            fit_rmse_ratio=0.0,
            support_length_ratio=maximum_y - minimum_y,
        )

    selector = SequenceAwareEntrySelector()
    # W1 is visibly left of W2 across y=0.20..0.40, but extending W1 to W2's
    # old y=0.90 anchor would put it to the right and reject it.
    w1 = line(0.50, 0.08, 0.20, 0.40)
    w2 = line(-0.10, 0.38, 0.18, 0.90)

    separation, _ = selector._w1_branch_metrics(w1, w2)

    assert np.isclose(separation, 0.12)
    assert selector._is_w1_branch(w1, w2)
    assert w2.x_ratio_at(0.90) - w1.x_ratio_at(0.90) < 0.0


def test_w1_acquisition_rejects_far_fragment_until_branch_reaches_lower_bev():
    def line(
        slope: float,
        intercept: float,
        minimum_y: float,
        maximum_y: float,
    ) -> LineHypothesis:
        midpoint_y = 0.5 * (minimum_y + maximum_y)
        return LineHypothesis(
            color="white",
            coefficients=(slope, intercept),
            mean_x_ratio=slope * midpoint_y + intercept,
            mean_y_ratio=midpoint_y,
            near_x_ratio=slope * maximum_y + intercept,
            far_x_ratio=slope * minimum_y + intercept,
            minimum_y_ratio=minimum_y,
            maximum_y_ratio=maximum_y,
            direction_dx_dy=slope,
            heading_from_vehicle_rad=math.atan(slope),
            vertical_span_ratio=maximum_y - minimum_y,
            fit_rmse_ratio=0.01,
            support_length_ratio=maximum_y - minimum_y,
        )

    selector = SequenceAwareEntrySelector()
    w2 = line(-0.20, 0.47, 0.25, 0.90)
    # Both hypotheses describe the same left-positive line.  Only the second
    # observation reaches the near/lower portion where the labelled W1 first
    # becomes real in the review bag.
    far_fragment = line(0.55, -0.10, 0.43, 0.69)
    lower_branch = line(0.55, -0.10, 0.52, 0.78)
    lower_w2_edge = line(0.25, 0.1175, 0.52, 0.78)

    assert selector._is_w1_branch(far_fragment, w2)
    assert not selector._is_w1_acquisition_candidate(far_fragment, w2)
    assert selector._acquire_w1((far_fragment, w2), w2) is None

    assert selector._is_w1_acquisition_candidate(lower_branch, w2)
    assert selector._acquire_w1((lower_branch, w2), w2) is lower_branch

    assert selector._is_w1_branch(lower_w2_edge, w2)
    assert not selector._is_w1_acquisition_candidate(lower_w2_edge, w2)


def test_continuing_w2_alone_never_creates_w1_or_red_intersection():
    selector = SequenceAwareEntrySelector()
    results = [
        selector.process(
            *make_phase_masks(include_y1=False, include_w1=False)
        )
        for _ in range(6)
    ]

    assert results[-1].w2 is not None
    assert all(result.w1 is None for result in results)
    assert all(not result.ready for result in results)
    estimate = estimate_branch_point(
        results[-1].w1,
        results[-1].white_candidates,
        forward_range_m=1.5,
        tracked_w2=results[-1].w2,
    )
    assert not np.isfinite(estimate.distance_m)


def test_y1_confirmation_switches_path_from_w1_to_yellow_centerline():
    selector = SequenceAwareEntrySelector()
    first, second = lock_y1(selector)

    assert first.phase == EntrySequencePhase.W1_LOCKED
    assert first.y1 is None
    assert second.phase == EntrySequencePhase.Y1_LOCKED
    assert second.y1 is not None
    assert second.w1 is not None
    separation = second.y1.mean_x_ratio - second.w1.mean_x_ratio
    assert 0.17 <= separation <= 0.46
    assert second.y1.direction_dx_dy > 0.15
    assert not second.used_synthetic_y1
    assert "PATH=Y1 direct" in second.reason
    for x_px, y_px in second.path_pixels:
        row_ratio = y_px / HEIGHT
        assert np.isclose(
            x_px / WIDTH,
            second.y1.x_ratio_at(row_ratio),
        )


def test_entry_path_follows_w1_directly_before_y1_confirmation():
    config = SequenceEntryConfig(w1_path_weight=0.60)
    selector = SequenceAwareEntrySelector(config)
    _, _, result = lock_w1(selector)

    assert result.w1 is not None
    assert result.y1 is None
    assert not result.used_synthetic_y1
    assert "PATH=W1 direct" in result.reason
    for x_px, y_px in result.path_pixels:
        row_ratio = y_px / HEIGHT
        assert np.isclose(
            x_px / WIDTH,
            result.w1.x_ratio_at(row_ratio),
        )


def test_w1_identity_does_not_jump_to_nearer_w2_before_y1_exists():
    selector = SequenceAwareEntrySelector()

    first_white = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    first_yellow = np.zeros_like(first_white)
    cv2.line(first_white, (165, 525), (90, 340), 12, cv2.LINE_AA)
    cv2.line(first_white, (225, 610), (275, 250), 22, cv2.LINE_AA)
    cv2.line(first_yellow, (410, 610), (450, 280), 22, cv2.LINE_AA)
    selector.process(first_white, first_yellow)
    selector.process(first_white, first_yellow)
    selector.process(first_white, first_yellow)
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
    assert not second.used_synthetic_y1


def test_short_y1_component_below_general_hough_span_can_lock():
    selector = SequenceAwareEntrySelector()
    white, yellow = make_phase_masks(include_y1=False)
    lock_w1(selector)

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


def test_right_leaning_white_line_cannot_replace_locked_left_w1():
    selector = SequenceAwareEntrySelector()
    _, locked = lock_y1(selector)
    assert locked.y1 is not None
    assert locked.w1 is not None
    locked_w1_slope = locked.w1.direction_dx_dy
    assert locked_w1_slope >= selector.config.acquisition_slope_minimum

    white = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    _, yellow = make_phase_masks(include_y1=True)
    # Negative dx/dy leans toward vehicle-right in canonical BEV.  These are
    # the false W1 segments captured in the review screenshots.
    cv2.line(white, (165, 600), (205, 300), 12, cv2.LINE_AA)

    held = [selector.process(white, yellow) for _ in range(2)]
    unavailable = selector.process(white, yellow)

    assert all(result.w1 is not None for result in held)
    assert all(result.w1.direction_dx_dy > 0.0 for result in held)
    assert all(
        np.isclose(result.w1.direction_dx_dy, locked_w1_slope)
        for result in held
    )
    assert unavailable.w1 is None
    assert unavailable.y1 is not None
    assert unavailable.path_valid
    assert "PATH=Y1 direct" in unavailable.reason
    for x_px, y_px in unavailable.path_pixels:
        row_ratio = y_px / HEIGHT
        assert np.isclose(
            x_px / WIDTH,
            unavailable.y1.x_ratio_at(row_ratio),
        )


def test_forward_alignment_hands_entry_to_existing_shortcut_cruise_by_order():
    selector = SequenceAwareEntrySelector()
    lock_y1(selector)

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


def test_spatial_gate_uses_branch_distance_and_rule_speed():
    slow = spatial_steering_gate(
        branch_distance_m=0.40,
        rule_speed_command=5.0,
        speed_command_to_mps=0.04,
        response_time_sec=0.50,
        minimum_trigger_distance_m=0.20,
        blend_distance_m=0.20,
    )
    fast = spatial_steering_gate(
        branch_distance_m=0.40,
        rule_speed_command=15.0,
        speed_command_to_mps=0.04,
        response_time_sec=0.50,
        minimum_trigger_distance_m=0.20,
        blend_distance_m=0.20,
    )

    assert not slow.ready
    assert fast.ready
    assert fast.trigger_distance_m > slow.trigger_distance_m


def test_w1_steering_delay_requires_configured_valid_frames():
    assert not w1_steering_delay_ready(observed_frames=3, required_frames=4)
    assert w1_steering_delay_ready(observed_frames=4, required_frames=4)
    assert w1_steering_delay_ready(observed_frames=0, required_frames=0)


def test_w1_steering_delay_tolerates_short_dropouts():
    observed, missing = update_w1_steering_delay_counts(
        observed_frames=2,
        missing_frames=0,
        w1_observed=False,
        missing_tolerance_frames=2,
    )
    assert (observed, missing) == (2, 1)

    observed, missing = update_w1_steering_delay_counts(
        observed_frames=observed,
        missing_frames=missing,
        w1_observed=True,
        missing_tolerance_frames=2,
    )
    assert (observed, missing) == (3, 0)


def test_w1_steering_delay_resets_after_dropout_tolerance():
    observed, missing = update_w1_steering_delay_counts(
        observed_frames=2,
        missing_frames=2,
        w1_observed=False,
        missing_tolerance_frames=2,
    )
    assert (observed, missing) == (0, 0)


def test_entry_handoff_prefers_geometric_alignment_after_progress():
    condition = select_entry_handoff_condition(
        geometric_handoff=True,
        steering_started=True,
        progress_m=0.50,
        minimum_progress_m=0.50,
        pair_track_confirmed=True,
        y1_confirmed=True,
        w1_visible=True,
    )

    assert condition == "forward_alignment_and_progress"


def test_entry_handoff_accepts_confirmed_pair_after_progress():
    condition = select_entry_handoff_condition(
        geometric_handoff=False,
        steering_started=True,
        progress_m=0.55,
        minimum_progress_m=0.50,
        pair_track_confirmed=True,
        y1_confirmed=True,
        w1_visible=True,
    )

    assert condition == "pair_track_and_progress"


def test_entry_handoff_accepts_confirmed_y1_after_w1_loss():
    condition = select_entry_handoff_condition(
        geometric_handoff=False,
        steering_started=True,
        progress_m=0.55,
        minimum_progress_m=0.50,
        pair_track_confirmed=False,
        y1_confirmed=True,
        w1_visible=False,
    )

    assert condition == "y1_confirmed_w1_lost_and_progress"


def test_entry_handoff_requires_minimum_progress_for_all_conditions():
    condition = select_entry_handoff_condition(
        geometric_handoff=True,
        steering_started=True,
        progress_m=0.49,
        minimum_progress_m=0.50,
        pair_track_confirmed=True,
        y1_confirmed=True,
        w1_visible=False,
    )

    assert condition is None


def test_entry_handoff_uses_active_steering_time_as_final_fallback():
    condition = select_entry_handoff_condition(
        geometric_handoff=False,
        steering_started=True,
        progress_m=0.20,
        minimum_progress_m=0.50,
        pair_track_confirmed=False,
        y1_confirmed=False,
        w1_visible=True,
        steering_active_sec=1.50,
        maximum_steering_sec=1.50,
    )

    assert condition == "maximum_w1_steering_time_elapsed"


def test_branch_distance_uses_second_white_only_as_spatial_gate():
    selector = SequenceAwareEntrySelector()
    _, _, result = lock_w1(selector)

    distance = branch_point_distance_m(
        result.w1,
        result.white_candidates,
        forward_range_m=1.5,
    )

    assert np.isfinite(distance)
    assert 0.0 <= distance <= 1.5

    estimate = estimate_branch_point(
        result.w1,
        result.white_candidates,
        forward_range_m=1.5,
        tracked_w2=result.w2,
    )
    assert estimate.w2 is not None
    assert estimate.w2 is not result.w1
    assert np.isclose(estimate.distance_m, distance)
    assert np.isfinite(estimate.intersection_x_ratio)
    assert np.isfinite(estimate.intersection_y_ratio)
