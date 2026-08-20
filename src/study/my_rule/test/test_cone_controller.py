import math
from collections import deque

import numpy as np
import pytest

from my_rule.cone_node import ConeNode


class _Parameter:
    def __init__(self, value):
        self.value = value


class _PurePursuitHarness:
    pure_pursuit = ConeNode.pure_pursuit
    pure_pursuit_at_distance = ConeNode.pure_pursuit_at_distance
    dynamic_lookahead = ConeNode.dynamic_lookahead
    point_at_path_distance = staticmethod(ConeNode.point_at_path_distance)
    path_heading_demand_deg = staticmethod(ConeNode.path_heading_demand_deg)

    parameters = {
        "wheelbase_m": 0.33,
        "max_steer_cmd": 42.0,
        "lookahead_min_m": 0.7,
        "lookahead_max_m": 1.45,
        "lookahead_scale": 0.12,
        "far_preview_distance_m": 0.75,
        "far_preview_weight": 0.65,
        "steering_gain": 1.05,
        "sharp_curve_preview_enabled": True,
        "sharp_curve_heading_threshold_deg": 22.0,
        "sharp_curve_steering_gain": 1.18,
    }

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _GeometryHarness:
    calculate_midpoints = ConeNode.calculate_midpoints
    guard_single_boundary_reacquisition = (
        ConeNode.guard_single_boundary_reacquisition
    )
    infer_midpoints_from_single_boundary = ConeNode.infer_midpoints_from_single_boundary
    infer_midpoints_from_richer_boundary = ConeNode.infer_midpoints_from_richer_boundary
    select_inferred_boundary = ConeNode.select_inferred_boundary
    continuous_boundary_segment = ConeNode.continuous_boundary_segment
    boundary_is_sufficient = ConeNode.boundary_is_sufficient
    nearest_gate_midpoint = ConeNode.nearest_gate_midpoint
    offset_boundary_to_center = ConeNode.offset_boundary_to_center
    point_to_path_distance = staticmethod(ConeNode.point_to_path_distance)
    lateral_path_distance_at_x = staticmethod(
        ConeNode.lateral_path_distance_at_x
    )
    recover_corridor_partners = ConeNode.recover_corridor_partners
    effective_corridor_width = ConeNode.effective_corridor_width
    update_corridor_width = ConeNode.update_corridor_width
    select_planning_clusters = ConeNode.select_planning_clusters
    bridge_boundary_groups = ConeNode.bridge_boundary_groups
    bridge_midpoint_gaps = ConeNode.bridge_midpoint_gaps
    order_connected_points = ConeNode.order_connected_points
    stabilize_boundary_identity = ConeNode.stabilize_boundary_identity
    boundary_match_score = staticmethod(ConeNode.boundary_match_score)
    final_left_phase_active = ConeNode.final_left_phase_active
    point_is_yolo_confirmed = ConeNode.point_is_yolo_confirmed

    parameters = {
        "pair_max_forward_delta_m": 0.30,
        "min_corridor_width_m": 0.68,
        "max_corridor_width_m": 0.98,
        "expected_corridor_width_m": 0.85,
        "corridor_width_learning_enabled": True,
        "corridor_width_learning_alpha": 0.20,
        "corridor_width_max_update_m": 0.04,
        "corridor_width_learning_min_pairs": 2,
        "lidar_geometry_planning_enabled": True,
        "lidar_geometry_path_band_margin_m": 0.16,
        "lidar_geometry_reference_timeout_sec": 1.00,
        "allow_single_boundary_fallback": True,
        "single_boundary_min_cones": 2,
        "single_boundary_min_span_m": 0.20,
        "allow_nearest_gate_fallback": True,
        "fallback_pair_min_lateral_separation_m": 0.4,
        "fallback_pair_max_center_offset_m": 0.65,
        "group_grow_distance_m": 0.5,
        "boundary_expected_spacing_m": 0.30,
        "boundary_direct_gap_max_m": 0.50,
        "boundary_missing_gap_min_m": 0.50,
        "boundary_gap_max_m": 0.75,
        "boundary_gap_max_turn_deg": 55.0,
        "boundary_extended_gap_max_m": 1.05,
        "boundary_extended_gap_max_turn_deg": 30.0,
        "boundary_extended_path_tube_m": 0.15,
        "boundary_extended_required_frames": 2,
        "boundary_extended_match_distance_m": 0.25,
        "boundary_extended_min_track_cones": 2,
        "boundary_identity_memory_frames": 6,
        "boundary_identity_match_distance_m": 0.35,
        "boundary_identity_swap_margin_m": 0.08,
        "path_max_heading_range_deg": 120.0,
        "single_boundary_switch_frames": 3,
        "paired_boundary_reanchor_frames": 3,
        "single_boundary_reacquire_min_fused_clusters": 3,
        "single_boundary_reacquire_require_opposite_support": True,
        "min_path_midpoints": 2,
        "cone_yolo_recover_corridor_partner": True,
        "cone_yolo_recovered_centerline_max_deviation_m": 0.25,
        "lidar_to_rear_axle_m": 0.42,
        "path_gap_fill_start_m": 0.35,
        "path_gap_fill_max_m": 0.95,
        "path_gap_sample_spacing_m": 0.12,
        "final_left_pair_max_width_m": 1.10,
        "final_left_pair_max_forward_delta_m": 0.45,
        "final_left_wide_pair_path_deviation_m": 0.20,
        "final_left_wide_pair_required_frames": 2,
        "sparse_cluster_match_distance_m": 0.18,
    }

    def __init__(self):
        self.midpoints_inferred = False
        self.midpoint_source = "none"
        self.active_inferred_boundary = None
        self.pending_inferred_boundary = None
        self.pending_inferred_frames = 0
        self.paired_boundary_frames = 0
        self.wide_pair_candidate_frames = 0
        self.last_wide_pair_width_m = 0.0
        self.last_observed_pair_width_m = 0.0
        self.wide_pair_active = False
        self.extended_boundary_candidates = {0: None, 1: None}
        self.extended_boundary_candidate_frames = {0: 0, 1: 0}
        self.current_yolo_confirmed_clusters = []
        self.cone_turn_phase = "approach"
        self.prev_path = None
        self.had_valid_path = False
        self.learned_corridor_width_m = 0.85
        self.cone_yolo_association_enabled = True
        self.geometry_planning_unlocked = False
        self.current_scan_time = 1.0
        self.geometry_reference_path = None
        self.geometry_reference_time = None
        self.previous_left_boundary = []
        self.previous_right_boundary = []
        self.left_boundary_memory_age = 0
        self.right_boundary_memory_age = 0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _PathHarness:
    pure_pursuit_at_distance = ConeNode.pure_pursuit_at_distance
    preview_steering_demand = ConeNode.preview_steering_demand
    straight_boost_ratio = ConeNode.straight_boost_ratio
    compute_speed = ConeNode.compute_speed
    held_path_progress = ConeNode.held_path_progress
    point_at_path_distance = staticmethod(ConeNode.point_at_path_distance)

    parameters = {
        "lookahead_min_m": 0.7,
        "lookahead_max_m": 1.45,
        "far_preview_distance_m": 0.75,
        "far_preview_weight": 0.65,
        "wheelbase_m": 0.33,
        "max_steer_cmd": 42.0,
        "cone_speed_full_steer_deg": 26.0,
        "cone_speed": 17.0,
        "cone_min_drive_speed": 9.0,
        "cone_speed_steer_exponent": 1.0,
        "cone_speed_confidence_floor_ratio": 0.35,
        "cone_preview_speed_control": True,
        "cone_straight_boost_speed": 21.0,
        "cone_straight_boost_min_confidence": 0.75,
        "cone_straight_boost_min_path_distance_m": 1.35,
        "cone_straight_boost_full_angle_deg": 1.0,
        "cone_straight_boost_max_angle_deg": 3.0,
        "cone_speed_preview_near_m": 0.80,
        "cone_speed_preview_far_m": 1.45,
        "cone_speed_preview_samples": 4,
        "min_confidence": 0.3,
        "path_hold_frames": 5,
        "path_hold_sec": 0.40,
        "single_boundary_max_speed": 9.5,
    }

    def __init__(self):
        self.path_is_held = False
        self.path_miss_count = 0
        self.midpoint_source = "paired"

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _PathAcceptanceHarness:
    accept_new_path = ConeNode.accept_new_path
    hold_previous_path = ConeNode.hold_previous_path
    path_target_lateral = ConeNode.path_target_lateral
    point_at_path_distance = staticmethod(ConeNode.point_at_path_distance)
    path_max_heading_step_deg = staticmethod(ConeNode.path_max_heading_step_deg)
    path_arc_length = staticmethod(ConeNode.path_arc_length)
    path_nearest_index = staticmethod(ConeNode.path_nearest_index)
    path_start_is_consistent = ConeNode.path_start_is_consistent
    path_heading_at_distance_deg = staticmethod(
        ConeNode.path_heading_at_distance_deg
    )
    heading_difference_deg = staticmethod(ConeNode.heading_difference_deg)
    normalize_control_path = ConeNode.normalize_control_path
    final_left_phase_active = ConeNode.final_left_phase_active

    parameters = {
        "lookahead_min_m": 0.7,
        "path_hold_frames": 5,
        "path_hold_sec": 0.40,
        "max_path_target_jump_m": 0.30,
        "inferred_max_path_target_jump_m": 0.08,
        "inferred_path_target_rate_mps": 0.80,
        "paired_reacquire_frames": 3,
        "paired_reacquire_min_target_jump_m": 0.08,
        "paired_reacquire_path_target_rate_mps": 0.65,
        "path_target_rate_max_dt_sec": 0.10,
        "path_max_heading_range_deg": 120.0,
        "path_min_control_arc_m": 0.20,
        "single_boundary_min_control_arc_m": 0.50,
        "cone_turn_sequence_guard_enabled": True,
        "final_left_single_path_max_start_m": 0.85,
        "final_left_reversal_target_margin_m": 0.01,
        "path_orientation_start_margin_m": 0.04,
        "path_start_nearest_max_fraction": 0.20,
        "path_start_nearest_distance_margin_m": 0.05,
        "path_heading_jump_threshold_deg": 55.0,
        "path_heading_jump_required_frames": 2,
        "path_heading_confirmation_tolerance_deg": 20.0,
        "path_heading_probe_distance_m": 0.35,
        "path_reversal_min_lateral_m": 0.12,
        "path_reversal_required_frames": 2,
    }

    def __init__(self):
        self.prev_path = None
        self.last_path_target_lateral = None
        self.path_miss_count = 0
        self.path_is_held = False
        self.midpoint_source = "paired"
        self.current_scan_time = 1.0
        self.last_path_accept_time = None
        self.last_path_update_time = None
        self.last_raw_path_target_lateral = 0.0
        self.last_output_path_target_lateral = 0.0
        self.path_target_limit_applied = False
        self.pending_path_reversal_sign = 0
        self.pending_path_reversal_frames = 0
        self.pending_path_heading_deg = None
        self.pending_path_heading_frames = 0
        self.last_path_heading_deg = None
        self.last_path_nearest_index = 0
        self.last_path_remaining_arc_m = 0.0
        self.last_path_orientation_reversed = False
        self.last_path_rejection_reason = "none"
        self.invalid_inferred_recovery_active = False
        self.accepted_path_source = "none"
        self.paired_reacquire_frames = 0
        self.cone_turn_phase = "approach"
        self.geometry_reference_path = None
        self.geometry_reference_time = None

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _InterpolationHarness(_PathAcceptanceHarness):
    interpolate_path = ConeNode.interpolate_path
    order_connected_points = ConeNode.order_connected_points
    cubic_interpolate = ConeNode.cubic_interpolate
    natural_cubic_interpolate = staticmethod(
        ConeNode.natural_cubic_interpolate
    )

    parameters = {
        **_PathAcceptanceHarness.parameters,
        "boundary_expected_spacing_m": 0.30,
        "path_max_heading_range_deg": 120.0,
        "path_gap_fill_max_m": 0.95,
        "min_path_midpoints": 2,
        "lidar_to_rear_axle_m": 0.42,
        "min_path_span_m": 0.15,
        "path_sample_count": 100,
        "path_interpolation_method": "linear",
    }


class _SteeringHarness:
    stabilize_steering = ConeNode.stabilize_steering
    update_cone_turn_phase = ConeNode.update_cone_turn_phase
    apply_cone_turn_phase_guard = ConeNode.apply_cone_turn_phase_guard
    final_left_fallback_angle = ConeNode.final_left_fallback_angle
    final_left_release_recovery_angle = (
        ConeNode.final_left_release_recovery_angle
    )

    parameters = {
        "inferred_steering_max_rate_deg_per_sec": 100.0,
        "steering_max_rate_deg_per_sec": 180.0,
        "steering_rate_max_dt_sec": 0.12,
        "inferred_steering_max_delta_deg": 4.0,
        "steering_max_delta_deg": 14.0,
        "cone_turn_sequence_guard_enabled": True,
        "cone_first_left_enter_steer_deg": 5.0,
        "cone_right_enter_steer_deg": 8.0,
        "cone_final_left_pending_steer_deg": 3.0,
        "cone_final_left_enter_steer_deg": 5.0,
        "cone_final_left_pending_frames": 2,
        "cone_final_left_pending_hold_steer_deg": 5.0,
        "cone_final_left_min_hold_steer_deg": 12.0,
        "cone_final_left_fallback_target_deg": 16.5,
        "cone_final_left_fallback_max_steer_deg": 22.0,
        "cone_final_left_fallback_ramp_deg_per_sec": 10.0,
        "cone_final_left_release_min_peak_deg": 15.0,
        "cone_final_left_release_guarded_min_peak_deg": 12.0,
        "cone_final_left_release_guarded_output_deg": 21.0,
        "cone_final_left_release_guarded_hold_sec": 0.60,
        "cone_final_left_release_margin_deg": 3.0,
        "cone_final_left_release_required_frames": 2,
        "cone_final_left_release_recovery_rate_deg_per_sec": 3.0,
        "cone_final_left_exit_steer_deg": 3.0,
        "cone_final_left_exit_frames": 3,
    }

    def __init__(self):
        self.midpoint_source = "right_offset"
        self.steering_history = deque(maxlen=1)
        self.stabilized_steering = 0.0
        self.last_steering_time = 1.0
        self.current_scan_time = 1.0
        self.path_is_held = False
        self.cone_turn_phase = "approach"
        self.final_left_pending_frames = 0
        self.final_left_pending_started_at = None
        self.final_left_committed_at = None
        self.final_left_fallback_started_at = None
        self.final_left_strong_hold_started_at = None
        self.final_left_peak_steering = 0.0
        self.final_left_measured_peak_steering = 0.0
        self.final_left_release_frames = 0
        self.final_left_release_reference_steering = 0.0
        self.final_left_release_reference_time = None
        self.final_left_exit_frames = 0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _RecoveryHarness:
    publish_blind_recovery = ConeNode.publish_blind_recovery
    final_left_phase_active = ConeNode.final_left_phase_active
    final_left_fallback_angle = ConeNode.final_left_fallback_angle
    final_left_release_recovery_angle = (
        ConeNode.final_left_release_recovery_angle
    )
    parameters = {
        "blind_recovery_frames": 5,
        "blind_recovery_max_sec": 0.35,
        "invalid_inferred_recovery_frames": 10,
        "invalid_inferred_recovery_max_sec": 1.00,
        "blind_recovery_min_clusters": 2,
        "blind_recovery_steer_decay": 0.92,
        "blind_recovery_speed": 4.0,
        "invalid_inferred_recovery_speed": 3.0,
        "invalid_inferred_recovery_steer_decay": 1.0,
        "cone_final_left_recovery_speed": 4.0,
        "cone_final_left_recovery_frames": 20,
        "cone_final_left_recovery_max_sec": 2.00,
        "cone_final_left_min_hold_steer_deg": 12.0,
        "cone_final_left_fallback_target_deg": 16.5,
        "cone_final_left_fallback_max_steer_deg": 22.0,
        "cone_final_left_fallback_ramp_deg_per_sec": 10.0,
        "cone_final_left_release_guarded_output_deg": 21.0,
        "cone_final_left_release_recovery_rate_deg_per_sec": 3.0,
        "cone_min_drive_speed": 9.0,
        "min_confidence": 0.3,
    }

    def __init__(self):
        self.had_valid_path = True
        self.blind_recovery_count = 0
        self.last_valid_steering = 10.0
        self.commands = []
        self.current_scan_time = 1.0
        self.blind_recovery_started_at = None
        self.invalid_inferred_recovery_active = False
        self.cone_turn_phase = "approach"
        self.final_left_fallback_started_at = None
        self.final_left_strong_hold_started_at = None
        self.final_left_peak_steering = 0.0
        self.final_left_release_reference_steering = 0.0
        self.final_left_release_reference_time = None
        self.last_recovery_angle = 0.0
        self.last_recovery_speed = 0.0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])

    def publish_cmd(self, angle, speed, confidence):
        self.commands.append((angle, speed, confidence))


def test_pure_pursuit_keeps_physical_angle_contract():
    angle = _PurePursuitHarness().pure_pursuit(
        [(0.8, 0.08), (1.0, 0.10)]
    )
    assert math.isfinite(angle)
    assert 0.0 < abs(angle) < 10.0
    assert abs(angle) <= 26.0


def test_bilateral_gate_uses_the_competition_corridor_width():
    harness = _GeometryHarness()
    valid = harness.nearest_gate_midpoint([(0.20, 0.43), (0.22, -0.42)])
    too_wide = harness.nearest_gate_midpoint([(0.20, 0.52), (0.22, -0.52)])
    too_diagonal = harness.nearest_gate_midpoint(
        [(0.20, 0.42), (0.52, -0.37)]
    )
    assert np.allclose(valid, (0.21, 0.005))
    assert too_wide is None
    assert too_diagonal is None


def test_single_boundary_builds_a_virtual_opposite_boundary():
    harness = _GeometryHarness()
    left = [(0.50, 0.39), (0.85, 0.45), (1.20, 0.55)]
    midpoints = harness.calculate_midpoints(left, [])
    assert harness.midpoints_inferred
    assert harness.midpoint_source == "left_offset"
    assert len(midpoints) == 3
    assert all(
        abs(midpoint[1]) < abs(boundary[1])
        for midpoint, boundary in zip(midpoints, left)
    )


def test_single_boundary_offset_is_perpendicular_to_local_tangent():
    harness = _GeometryHarness()
    boundary = [
        (0.40, 0.30),
        (0.65, 0.36),
        (0.90, 0.50),
        (1.15, 0.72),
        (1.40, 1.00),
    ]
    centerline = harness.offset_boundary_to_center(
        boundary,
        is_left_boundary=True,
    )
    half_width = 0.5 * harness.parameters["expected_corridor_width_m"]
    for index, (boundary_point, center_point) in enumerate(
        zip(boundary, centerline)
    ):
        before = boundary[max(0, index - 2)]
        after = boundary[min(len(boundary) - 1, index + 2)]
        tangent = np.asarray(after) - np.asarray(before)
        tangent /= np.linalg.norm(tangent)
        offset = np.asarray(center_point) - np.asarray(boundary_point)
        assert np.linalg.norm(offset) == pytest.approx(half_width)
        assert float(np.dot(offset, tangent)) == pytest.approx(0.0, abs=1e-6)


def test_single_boundary_normal_uses_previous_path_when_bearing_side_is_wrong():
    harness = _GeometryHarness()
    harness.prev_path = [(0.80, 0.0), (1.20, 0.0), (1.60, 0.0)]
    boundary = [(0.40, 0.42), (0.80, 0.42), (1.20, 0.42)]

    centerline = harness.offset_boundary_to_center(
        boundary,
        is_left_boundary=False,
    )

    assert all(abs(y) < 0.01 for _, y in centerline)


def test_single_boundary_previous_path_score_does_not_penalize_x_shift():
    harness = _GeometryHarness()
    # This reproduces the bend where the previous implementation selected the
    # outward normal because its Euclidean score was dominated by the x gap.
    harness.prev_path = [(1.64, 0.21), (1.85, -0.10)]
    right_boundary = [(0.622, -0.270), (0.843, 0.058)]

    centerline = harness.offset_boundary_to_center(
        right_boundary,
        is_left_boundary=False,
    )

    # The correct inward normal lies to the rear/left in x on this diagonal
    # boundary.  Longitudinal displacement must not make the outward normal win.
    assert centerline[0] == pytest.approx((0.269, -0.032), abs=0.002)
    assert centerline[1] == pytest.approx((0.490, 0.296), abs=0.002)


def test_expired_path_rejects_pure_two_point_single_side_reacquisition():
    harness = _GeometryHarness()
    harness.had_valid_path = True
    harness.midpoints_inferred = True
    harness.midpoint_source = "right_offset"
    right = [(0.867, -0.027), (1.074, -0.334)]
    inferred = [(1.219, 0.211), (1.426, -0.096)]

    guarded = harness.guard_single_boundary_reacquisition(
        inferred,
        fused_clusters=right,
        left_cones=[],
        right_cones=right,
    )

    assert guarded == []
    assert harness.midpoint_source == "reacquire_pending"


def test_expired_path_allows_single_side_reacquisition_with_support():
    harness = _GeometryHarness()
    harness.had_valid_path = True
    harness.midpoint_source = "right_offset"
    right = [(0.622, -0.270), (0.843, 0.058)]
    left = [(1.061, 0.410)]
    inferred = [(0.269, -0.032), (0.490, 0.296)]

    guarded = harness.guard_single_boundary_reacquisition(
        inferred,
        fused_clusters=[*right, *left],
        left_cones=left,
        right_cones=right,
    )

    assert guarded == inferred
    assert harness.midpoint_source == "right_offset"


def test_initial_single_side_path_keeps_existing_behavior():
    harness = _GeometryHarness()
    harness.midpoint_source = "right_offset"
    right = [(0.867, -0.027), (1.074, -0.334)]
    inferred = [(1.219, 0.211), (1.426, -0.096)]

    guarded = harness.guard_single_boundary_reacquisition(
        inferred,
        fused_clusters=right,
        left_cones=[],
        right_cones=right,
    )

    assert guarded == inferred


def test_yolo_anchor_recovers_only_corridor_partner_near_previous_path():
    harness = _GeometryHarness()
    harness.prev_path = [(0.70, 0.0), (1.10, 0.0), (1.50, 0.0)]
    anchor = (0.60, 0.42)
    true_partner = (0.62, -0.43)
    chair_leg = (0.60, 1.27)

    recovered = harness.recover_corridor_partners(
        [anchor, true_partner, chair_leg],
        [anchor],
    )

    assert recovered == [anchor, true_partner]


def test_yolo_unlock_keeps_only_lidar_geometry_near_previous_corridor():
    harness = _GeometryHarness()
    harness.prev_path = [(0.70, 0.0), (1.10, 0.0), (1.50, 0.0)]
    confirmed = (0.60, 0.42)
    opposite_boundary = (0.75, -0.43)
    centre_clutter = (0.90, 0.02)
    far_clutter = (0.90, 1.30)

    assert harness.select_planning_clusters(
        [confirmed, opposite_boundary, centre_clutter, far_clutter],
        [],
    ) == []

    selected = harness.select_planning_clusters(
        [confirmed, opposite_boundary, centre_clutter, far_clutter],
        [confirmed],
    )

    assert confirmed in selected
    assert opposite_boundary in selected
    assert centre_clutter not in selected
    assert far_clutter not in selected


def test_recent_geometry_reference_allows_lidar_only_reacquisition():
    harness = _GeometryHarness()
    harness.geometry_planning_unlocked = True
    harness.geometry_reference_path = [
        (0.70, 0.0),
        (1.10, 0.0),
        (1.50, 0.0),
    ]
    harness.geometry_reference_time = 1.0
    boundary = (0.75, -0.43)

    harness.current_scan_time = 1.90
    assert harness.select_planning_clusters([boundary], []) == [boundary]

    harness.current_scan_time = 2.01
    assert harness.select_planning_clusters([boundary], []) == []


def test_corridor_width_learns_slowly_from_two_bilateral_pairs():
    harness = _GeometryHarness()
    harness.update_corridor_width([0.75, 0.77])
    assert harness.effective_corridor_width() == pytest.approx(0.832)

    # A single pair is insufficient and must not move the learned RC width.
    harness.update_corridor_width([0.68])
    assert harness.effective_corridor_width() == pytest.approx(0.832)


def test_boundary_bridge_requires_a_tangent_consistent_gap():
    harness = _GeometryHarness()
    left = [(0.30, 0.42), (0.70, 0.44)]
    continuation = (1.25, 0.48)
    sharp_outlier = (1.00, 1.00)

    bridged_left, _ = harness.bridge_boundary_groups(
        left,
        [],
        [*left, continuation, sharp_outlier],
    )

    assert continuation in bridged_left
    assert sharp_outlier not in bridged_left


def test_final_left_extended_boundary_gap_requires_two_consistent_scans():
    harness = _GeometryHarness()
    harness.cone_turn_phase = "final_left_committed"
    left = [(0.30, 0.42), (0.60, 0.42)]
    continuation = (1.50, 0.43)

    first, _ = harness.bridge_boundary_groups(
        left, [], [*left, continuation]
    )
    second, _ = harness.bridge_boundary_groups(
        left, [], [*left, continuation]
    )

    assert continuation not in first
    assert continuation in second


def test_extended_boundary_gap_stays_disabled_before_final_left():
    harness = _GeometryHarness()
    left = [(0.30, 0.42), (0.60, 0.42)]
    continuation = (1.50, 0.43)

    for _ in range(3):
        bridged, _ = harness.bridge_boundary_groups(
            left, [], [*left, continuation]
        )

    assert continuation not in bridged


def test_temporary_wide_final_left_pair_needs_confirmation_and_is_not_learned():
    harness = _GeometryHarness()
    harness.cone_turn_phase = "final_left_committed"
    harness.prev_path = [(0.92, 0.0), (1.32, 0.0)]
    left = [(0.50, 0.52), (0.90, 0.52)]
    right = [(0.50, -0.52), (0.90, -0.52)]

    first = harness.calculate_midpoints(left, right)
    second = harness.calculate_midpoints(left, right)

    assert first
    assert harness.midpoint_source == "paired"
    assert second == pytest.approx([(0.50, 0.0), (0.90, 0.0)])
    assert harness.wide_pair_active
    assert harness.effective_corridor_width() == pytest.approx(0.85)


def test_midpoint_gap_is_densified_but_large_void_is_not_crossed():
    harness = _GeometryHarness()
    filled = harness.bridge_midpoint_gaps(
        [(0.30, 0.00), (0.80, 0.05), (1.20, 0.12)]
    )
    assert len(filled) > 3
    assert filled[0] == (0.30, 0.00)
    assert filled[-1] == (1.20, 0.12)

    split = harness.bridge_midpoint_gaps(
        [(0.20, 0.00), (0.50, 0.02), (1.80, 0.70)]
    )
    assert all(point[0] <= 0.50 for point in split)


def test_arc_length_path_preserves_a_near_right_angle_turn():
    harness = _InterpolationHarness()
    path = harness.interpolate_path(
        [
            (0.30, 0.00),
            (0.58, 0.03),
            (0.64, 0.31),
            (0.61, 0.60),
        ]
    )

    assert len(path) == 100
    assert max(y for _, y in path) > 0.55
    # An x=f(s), y=f(s) path is allowed to turn back slightly; old y=f(x)
    # interpolation collapsed these points because x was not monotonic.
    assert any(second[0] < first[0] for first, second in zip(path, path[1:]))


def test_path_lookup_keeps_the_accepted_forward_order_on_a_hairpin():
    # The final sample is closest to the origin, but it is still the end of the
    # accepted forward curve.  A global nearest-point search used to jump there.
    path = [(0.40, 0.00), (0.70, 0.10), (0.55, 0.32), (0.18, 0.20)]
    target = ConeNode.point_at_path_distance(path, 0.50)

    assert target[0] > 0.40
    assert target[1] < 0.20


def test_short_replacement_path_holds_the_previous_curve():
    harness = _PathAcceptanceHarness()
    harness.prev_path = [(0.45, 0.0), (0.80, 0.0), (1.15, 0.0)]
    harness.last_path_accept_time = 1.0
    harness.last_path_heading_deg = 0.0

    accepted = harness.accept_new_path([(0.40, 0.0), (0.52, 0.0)])

    assert accepted == harness.prev_path
    assert harness.path_is_held


def test_single_frame_heading_jump_holds_until_geometry_repeats():
    harness = _PathAcceptanceHarness()
    old_path = [(0.45, 0.0), (0.80, 0.0), (1.15, 0.0)]
    harness.prev_path = list(old_path)
    harness.last_path_accept_time = 1.0
    harness.last_path_heading_deg = 0.0
    candidate = [(0.42, 0.0), (0.42, 0.35), (0.42, 0.70)]

    first = harness.accept_new_path(candidate)
    assert first == old_path
    assert harness.path_is_held

    harness.current_scan_time = 1.10
    second = harness.accept_new_path(candidate)
    assert second == candidate
    assert not harness.path_is_held


def test_boundary_identity_memory_corrects_a_seed_side_swap():
    harness = _GeometryHarness()
    harness.previous_left_boundary = [(0.40, 0.42), (0.70, 0.45)]
    harness.previous_right_boundary = [(0.42, -0.42), (0.72, -0.44)]

    left, right = harness.stabilize_boundary_identity(
        [(0.43, -0.41), (0.73, -0.43)],
        [(0.41, 0.43), (0.71, 0.46)],
    )

    assert all(y > 0.0 for _, y in left)
    assert all(y < 0.0 for _, y in right)


def test_corridor_partner_bootstraps_without_previous_path_when_geometry_is_safe():
    harness = _GeometryHarness()
    anchor = (0.60, 0.42)
    partner = (0.62, -0.43)

    assert harness.recover_corridor_partners(
        [anchor, partner],
        [anchor],
    ) == [anchor, partner]


def test_single_boundary_stays_selected_until_bilateral_path_returns():
    harness = _GeometryHarness()
    left_candidates = {"left": (2, -0.1, [(0.5, 0.0), (0.9, 0.0)])}
    both_candidates = {
        **left_candidates,
        "right": (5, -0.05, [(0.5, 0.0), (0.9, 0.0)]),
    }
    assert harness.select_inferred_boundary("left", left_candidates) == "left"
    for _ in range(5):
        assert harness.select_inferred_boundary("right", both_candidates) == "left"


def test_paired_frame_preserves_single_boundary_identity():
    harness = _GeometryHarness()
    harness.active_inferred_boundary = "left"
    paired = harness.calculate_midpoints(
        [(0.50, 0.42), (0.90, 0.44)],
        [(0.52, -0.43), (0.92, -0.41)],
    )
    assert len(paired) == 2
    assert harness.midpoint_source == "paired"
    assert harness.active_inferred_boundary == "left"

    right_candidates = {
        "right": (3, -0.1, [(0.5, 0.0), (0.9, 0.0)])
    }
    assert harness.select_inferred_boundary("right", right_candidates) is None
    assert harness.select_inferred_boundary("right", right_candidates) is None
    assert harness.select_inferred_boundary("right", right_candidates) == "right"


def test_stable_paired_frames_reanchor_single_boundary_identity():
    harness = _GeometryHarness()
    harness.active_inferred_boundary = "left"
    left = [(0.50, 0.42), (0.90, 0.44)]
    right = [(0.52, -0.43), (0.92, -0.41)]

    for _ in range(3):
        assert len(harness.calculate_midpoints(left, right)) == 2

    assert harness.active_inferred_boundary is None
    right_candidates = {
        "right": (3, -0.1, [(0.5, 0.0), (0.9, 0.0)])
    }
    assert harness.select_inferred_boundary("right", right_candidates) == "right"


def test_inferred_path_is_laterally_slew_limited_not_rejected():
    harness = _PathAcceptanceHarness()
    harness.prev_path = [(0.70, 0.0), (1.00, 0.0)]
    harness.last_path_target_lateral = 0.0
    harness.midpoint_source = "right_offset"

    accepted = harness.accept_new_path(
        [(0.70, 0.40), (1.30, 0.40)]
    )

    assert accepted
    assert not harness.path_is_held
    assert harness.path_target_lateral(accepted) == pytest.approx(0.08, abs=1e-3)


def test_inferred_path_remains_limited_after_hold_expiry():
    harness = _PathAcceptanceHarness()
    harness.last_path_target_lateral = 0.0
    harness.midpoint_source = "left_offset"

    accepted = harness.accept_new_path(
        [(0.70, -0.40), (1.30, -0.40)]
    )

    assert harness.path_target_lateral(accepted) == pytest.approx(-0.08, abs=1e-3)


def test_short_single_boundary_cannot_replace_established_turn():
    harness = _PathAcceptanceHarness()
    previous = [(0.40, -0.08), (0.80, -0.18), (1.20, -0.30)]
    harness.prev_path = list(previous)
    harness.last_path_target_lateral = harness.path_target_lateral(previous)
    harness.last_path_accept_time = 1.0
    harness.last_path_update_time = 1.0
    harness.accepted_path_source = "paired"
    harness.midpoint_source = "left_offset"

    # 0.34 m one-sided fragment matching the physical signature of the
    # opposite-steering event at bag t=162.17 s.
    candidate = [(0.04, 0.20), (0.08, 0.36), (0.13, 0.53)]
    accepted = harness.accept_new_path(candidate)

    assert accepted == previous
    assert harness.path_is_held
    assert harness.last_path_rejection_reason == "short_single_boundary"


def test_final_left_rejects_distant_single_boundary_path():
    harness = _PathAcceptanceHarness()
    previous = [(0.40, 0.04), (0.80, 0.12), (1.10, 0.20)]
    harness.prev_path = list(previous)
    harness.last_path_target_lateral = harness.path_target_lateral(previous)
    harness.last_path_accept_time = 1.0
    harness.last_path_update_time = 1.0
    harness.midpoint_source = "right_offset"
    harness.cone_turn_phase = "final_left"

    accepted = harness.accept_new_path(
        [(1.05, 0.10), (1.45, 0.05), (1.85, -0.05)]
    )

    assert accepted == previous
    assert harness.path_is_held
    assert harness.last_path_rejection_reason == "distant_single_boundary"


def test_final_left_rejects_nearby_single_boundary_reversal():
    harness = _PathAcceptanceHarness()
    previous = [(0.40, 0.04), (0.80, 0.12), (1.10, 0.20)]
    harness.prev_path = list(previous)
    harness.last_path_target_lateral = harness.path_target_lateral(previous)
    harness.last_path_accept_time = 1.0
    harness.last_path_update_time = 1.0
    harness.midpoint_source = "right_offset"
    harness.cone_turn_phase = "final_left"

    accepted = harness.accept_new_path(
        [(0.45, -0.08), (0.80, -0.16), (1.10, -0.22)]
    )

    assert accepted == previous
    assert harness.path_is_held
    assert harness.last_path_rejection_reason == "final_left_reversal"


def test_pending_final_left_rejects_paired_reacquire_reversal():
    harness = _PathAcceptanceHarness()
    previous = [(0.40, 0.04), (0.80, 0.12), (1.10, 0.20)]
    harness.prev_path = list(previous)
    harness.last_path_target_lateral = harness.path_target_lateral(previous)
    harness.last_path_accept_time = 1.0
    harness.last_path_update_time = 1.0
    harness.midpoint_source = "paired_reacquire"
    harness.cone_turn_phase = "final_left_pending"

    accepted = harness.accept_new_path(
        [(0.45, -0.08), (0.80, -0.16), (1.10, -0.22)]
    )

    assert accepted == previous
    assert harness.path_is_held
    assert harness.last_path_rejection_reason == "final_left_reversal"


def test_lateral_limiter_cannot_create_late_nearest_return():
    harness = _PathAcceptanceHarness()
    previous = [(0.45, 0.0), (0.80, 0.0), (1.10, 0.0)]
    harness.prev_path = list(previous)
    harness.last_path_target_lateral = 0.0
    harness.last_path_accept_time = 1.0
    harness.last_path_update_time = 1.0
    harness.accepted_path_source = "paired"
    harness.midpoint_source = "left_offset"

    # Before limiting this is ordered away from the origin. Translating it to
    # the allowed lateral target makes its far end much closer than its start,
    # so following from sample zero would briefly reverse steering.
    candidate = [
        (0.05 + 0.05 * ratio, 0.20 + 0.60 * ratio)
        for ratio in np.linspace(0.0, 1.0, 100)
    ]
    accepted = harness.accept_new_path(candidate)

    assert accepted == previous
    assert harness.path_is_held
    assert harness.last_path_rejection_reason == "late_nearest_return"


def test_path_hold_uses_elapsed_time_instead_of_scan_count():
    harness = _PathAcceptanceHarness()
    harness.prev_path = [(0.70, 0.0), (1.00, 0.0)]
    harness.last_path_accept_time = 1.0

    harness.current_scan_time = 1.39
    assert harness.hold_previous_path()
    harness.current_scan_time = 1.41
    assert harness.hold_previous_path() == []


def test_abrupt_path_direction_reversal_requires_two_scans():
    harness = _PathAcceptanceHarness()
    harness.parameters = dict(harness.parameters)
    harness.parameters["max_path_target_jump_m"] = 0.8
    harness.prev_path = [(0.50, 0.30), (1.00, 0.30)]
    harness.last_path_target_lateral = harness.path_target_lateral(
        harness.prev_path
    )
    harness.last_path_accept_time = 1.0

    candidate = [(0.50, -0.30), (1.00, -0.30)]
    first = harness.accept_new_path(candidate)
    assert first[0][1] > 0.0
    harness.current_scan_time = 1.1
    second = harness.accept_new_path(candidate)
    assert second[0][1] < 0.0


def test_paired_reacquisition_from_single_boundary_is_slew_limited():
    harness = _PathAcceptanceHarness()
    harness.prev_path = [(0.50, 0.04), (1.00, 0.04)]
    harness.last_path_target_lateral = harness.path_target_lateral(
        harness.prev_path
    )
    harness.last_path_accept_time = 1.0
    harness.last_path_update_time = 1.0
    harness.accepted_path_source = "right_offset"
    candidate = [(0.50, -0.20), (1.00, -0.20)]

    harness.current_scan_time = 1.10
    first = harness.accept_new_path(candidate)
    assert harness.midpoint_source == "paired_reacquire"
    assert harness.path_target_lateral(first) == pytest.approx(-0.025, abs=1e-3)

    harness.midpoint_source = "paired"
    harness.current_scan_time = 1.20
    second = harness.accept_new_path(candidate)
    assert harness.midpoint_source == "paired_reacquire"
    assert harness.path_target_lateral(second) == pytest.approx(-0.090, abs=1e-3)

    harness.midpoint_source = "paired"
    harness.current_scan_time = 1.30
    third = harness.accept_new_path(candidate)
    assert harness.midpoint_source == "paired_reacquire"
    assert harness.path_target_lateral(third) == pytest.approx(-0.155, abs=1e-3)

    harness.midpoint_source = "paired"
    harness.current_scan_time = 1.40
    fourth = harness.accept_new_path(candidate)
    assert harness.midpoint_source == "paired"
    assert harness.path_target_lateral(fourth) == pytest.approx(-0.20, abs=1e-3)


def test_continuous_paired_path_is_not_reacquisition_limited():
    harness = _PathAcceptanceHarness()
    harness.prev_path = [(0.50, 0.04), (1.00, 0.04)]
    harness.last_path_target_lateral = harness.path_target_lateral(
        harness.prev_path
    )
    harness.last_path_accept_time = 1.0
    harness.last_path_update_time = 1.0
    harness.accepted_path_source = "paired"
    harness.midpoint_source = "paired"

    accepted = harness.accept_new_path(
        [(0.50, -0.20), (1.00, -0.20)]
    )

    assert harness.midpoint_source == "paired"
    assert harness.path_target_lateral(accepted) == pytest.approx(-0.20, abs=1e-3)


def test_aligned_paired_reacquisition_is_not_needlessly_limited():
    harness = _PathAcceptanceHarness()
    harness.prev_path = [(0.50, 0.04), (1.00, 0.04)]
    harness.last_path_target_lateral = harness.path_target_lateral(
        harness.prev_path
    )
    harness.last_path_accept_time = 1.0
    harness.last_path_update_time = 1.0
    harness.accepted_path_source = "right_offset"
    harness.midpoint_source = "paired"

    accepted = harness.accept_new_path(
        [(0.50, -0.01), (1.00, -0.01)]
    )

    assert harness.midpoint_source == "paired"
    assert harness.path_target_lateral(accepted) == pytest.approx(-0.01, abs=1e-3)


def test_inferred_steering_rate_is_independent_of_scan_frequency():
    ten_hz = _SteeringHarness()
    ten_hz.current_scan_time = 1.10
    ten_hz_angle = ten_hz.stabilize_steering(30.0)

    twenty_hz = _SteeringHarness()
    twenty_hz.current_scan_time = 1.05
    first = twenty_hz.stabilize_steering(30.0)
    twenty_hz.current_scan_time = 1.10
    twenty_hz_angle = twenty_hz.stabilize_steering(30.0)

    assert ten_hz_angle == pytest.approx(10.0)
    assert first == pytest.approx(5.0)
    assert twenty_hz_angle == pytest.approx(10.0)


def test_final_left_sequence_holds_left_against_inferred_release():
    harness = _SteeringHarness()
    harness.midpoint_source = "paired"
    harness.update_cone_turn_phase(-6.0)
    assert harness.cone_turn_phase == "first_left"

    harness.midpoint_source = "left_offset"
    harness.update_cone_turn_phase(10.0)
    assert harness.cone_turn_phase == "right"

    harness.midpoint_source = "right_offset"
    harness.update_cone_turn_phase(-6.0)
    assert harness.cone_turn_phase == "final_left_pending"

    harness.path_is_held = True
    harness.update_cone_turn_phase(-6.0)
    assert harness.cone_turn_phase == "final_left_committed"

    harness.midpoint_source = "right_offset"
    harness.path_is_held = False
    guarded = harness.apply_cone_turn_phase_guard(5.0)
    assert guarded == pytest.approx(-12.0)


def test_final_left_fallback_ramps_only_through_measured_envelope():
    harness = _SteeringHarness()
    harness.cone_turn_phase = "final_left_committed"
    harness.midpoint_source = "paired_reacquire"

    harness.current_scan_time = 1.0
    assert harness.apply_cone_turn_phase_guard(4.0) == pytest.approx(-12.0)
    harness.current_scan_time = 1.45
    assert harness.apply_cone_turn_phase_guard(4.0) == pytest.approx(-16.5)
    harness.current_scan_time = 2.0
    assert harness.apply_cone_turn_phase_guard(4.0) == pytest.approx(-22.0)
    harness.current_scan_time = 3.0
    assert harness.apply_cone_turn_phase_guard(4.0) == pytest.approx(-22.0)


def test_fresh_paired_path_unwinds_after_venue_scale_peak():
    """Regression for fixed-8 bag 025418 frames 75~78."""
    harness = _SteeringHarness()
    harness.cone_turn_phase = "final_left_committed"
    harness.midpoint_source = "paired"
    harness.final_left_peak_steering = -22.0
    harness.final_left_measured_peak_steering = -15.1

    raw_values = [-13.6, -12.0, -10.4, -8.6]
    outputs = []
    for index, raw in enumerate(raw_values):
        harness.current_scan_time = 1.0 + 0.1 * index
        harness.update_cone_turn_phase(raw)
        outputs.append(harness.apply_cone_turn_phase_guard(raw))

    assert outputs == pytest.approx(raw_values)
    assert harness.cone_turn_phase == "final_left_releasing"


def test_paired_unwind_does_not_release_before_large_left_peak():
    harness = _SteeringHarness()
    harness.cone_turn_phase = "final_left_committed"
    harness.midpoint_source = "paired"
    harness.final_left_peak_steering = -9.0
    harness.final_left_measured_peak_steering = -9.0

    assert harness.apply_cone_turn_phase_guard(-7.0) == pytest.approx(-12.0)


def test_guarded_turn_releases_lower_raw_peak_after_strong_hold():
    """Regression for fixed-8 failure bag recorded at 03:11."""
    harness = _SteeringHarness()
    harness.cone_turn_phase = "final_left_committed"
    harness.midpoint_source = "paired"
    harness.final_left_peak_steering = -22.0
    harness.final_left_measured_peak_steering = -14.0
    harness.final_left_fallback_started_at = 0.0
    harness.final_left_strong_hold_started_at = 1.0

    harness.current_scan_time = 1.70
    harness.update_cone_turn_phase(-9.8)
    first = harness.apply_cone_turn_phase_guard(-9.8)
    harness.current_scan_time = 1.90
    harness.update_cone_turn_phase(-10.3)
    second = harness.apply_cone_turn_phase_guard(-10.3)

    assert first == pytest.approx(-22.0)
    assert second == pytest.approx(-10.3)
    assert harness.cone_turn_phase == "final_left_releasing"


def test_guarded_turn_cannot_release_before_strong_hold_duration():
    harness = _SteeringHarness()
    harness.cone_turn_phase = "final_left_committed"
    harness.midpoint_source = "paired"
    harness.final_left_peak_steering = -22.0
    harness.final_left_measured_peak_steering = -14.0
    harness.final_left_fallback_started_at = 0.0
    harness.final_left_strong_hold_started_at = 1.0

    for now, raw in ((1.30, -9.8), (1.50, -10.3)):
        harness.current_scan_time = now
        harness.update_cone_turn_phase(raw)
        assert harness.apply_cone_turn_phase_guard(raw) == pytest.approx(
            -22.0
        )

    assert harness.cone_turn_phase == "final_left_committed"


def test_release_path_loss_continues_unwind_instead_of_restoring_peak():
    harness = _SteeringHarness()
    harness.cone_turn_phase = "final_left_releasing"
    harness.midpoint_source = "right_offset"
    harness.final_left_peak_steering = -22.0
    harness.final_left_release_reference_steering = -10.0
    harness.final_left_release_reference_time = 1.0
    harness.current_scan_time = 1.5

    guarded = harness.apply_cone_turn_phase_guard(-18.0)

    assert guarded == pytest.approx(-8.5)


def test_deployed_speed_profile_boosts_only_a_good_straight():
    harness = _PathHarness()
    straight = [(0.70, 0.0), (1.10, 0.0), (1.50, 0.0)]
    curve = [(0.70, 0.0), (1.00, 0.05), (1.25, 0.30), (1.50, 0.55)]
    assert harness.compute_speed(0.0, 1.0, straight) == 21.0
    assert 9.0 <= harness.compute_speed(0.0, 1.0, curve) <= 17.0
    assert harness.compute_speed(26.0, 1.0, curve) == 9.0
    harness.midpoint_source = "right_offset"
    assert harness.compute_speed(0.0, 1.0, straight) == 9.5


def test_preview_brakes_before_current_steering_becomes_large():
    harness = _PathHarness()
    straight = [(0.70, 0.0), (1.10, 0.0), (1.50, 0.0)]
    curve = [(0.70, 0.0), (1.00, 0.05), (1.25, 0.30), (1.50, 0.55)]
    assert harness.compute_speed(0.0, 1.0, straight) == 21.0
    assert harness.compute_speed(0.0, 1.0, curve) < 17.0


def test_fixed_ten_profile_stays_ten_across_all_path_conditions():
    harness = _PathHarness()
    harness.parameters = {
        **harness.parameters,
        "cone_speed": 10.0,
        "cone_min_drive_speed": 10.0,
        "cone_straight_boost_speed": 10.0,
        "single_boundary_max_speed": 10.0,
    }
    straight = [(0.70, 0.0), (1.10, 0.0), (1.50, 0.0)]
    curve = [(0.70, 0.0), (1.00, 0.05), (1.25, 0.30)]

    assert harness.compute_speed(0.0, 1.0, straight) == 10.0
    assert harness.compute_speed(26.0, 0.4, curve) == 10.0
    harness.midpoint_source = "right_offset"
    assert harness.compute_speed(12.0, 0.5, curve) == 10.0
    harness.path_is_held = True
    assert harness.compute_speed(12.0, 0.3, curve) == 10.0


def test_sharp_curve_preview_adds_steering_without_changing_straight():
    harness = _PurePursuitHarness()
    harness.parameters = dict(harness.parameters)
    straight = [(0.50, 0.0), (0.90, 0.0), (1.30, 0.0)]
    curve = [(0.45, 0.0), (0.70, 0.10), (0.82, 0.40), (0.78, 0.70)]

    harness.parameters["sharp_curve_preview_enabled"] = False
    normal_curve = harness.pure_pursuit(curve)
    normal_straight = harness.pure_pursuit(straight)
    harness.parameters["sharp_curve_preview_enabled"] = True
    sharp_curve = harness.pure_pursuit(curve)
    sharp_straight = harness.pure_pursuit(straight)

    assert abs(sharp_curve) >= abs(normal_curve)
    assert sharp_straight == pytest.approx(normal_straight)


def test_blind_recovery_moves_but_is_strictly_bounded():
    harness = _RecoveryHarness()
    for _ in range(5):
        assert harness.publish_blind_recovery(cluster_count=4)
        harness.current_scan_time += 0.05
    assert not harness.publish_blind_recovery(cluster_count=4)
    assert all(speed == 4.0 for _, speed, _ in harness.commands)
    assert abs(harness.commands[-1][0]) < abs(harness.commands[0][0])


def test_rejected_inferred_path_extends_bounded_recovery():
    harness = _RecoveryHarness()
    harness.invalid_inferred_recovery_active = True
    for _ in range(10):
        assert harness.publish_blind_recovery(cluster_count=4)
        harness.current_scan_time += 0.08
    assert not harness.publish_blind_recovery(cluster_count=4)
    assert all(speed == 3.0 for _, speed, _ in harness.commands)
    assert all(angle == pytest.approx(10.0) for angle, _, _ in harness.commands)


def test_final_left_recovery_uses_peak_left_steering_at_creep_speed():
    harness = _RecoveryHarness()
    harness.invalid_inferred_recovery_active = True
    harness.cone_turn_phase = "final_left_committed"
    harness.last_valid_steering = -13.0
    harness.final_left_peak_steering = -17.0

    assert harness.publish_blind_recovery(cluster_count=4)
    assert harness.commands[-1][0] == pytest.approx(-17.0)
    assert harness.commands[-1][1] == pytest.approx(4.0)
