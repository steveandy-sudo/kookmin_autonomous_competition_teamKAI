import math

import numpy as np

from my_rule.cone_node import ConeNode


class _Parameter:
    def __init__(self, value):
        self.value = value


class _PurePursuitHarness:
    pure_pursuit = ConeNode.pure_pursuit
    pure_pursuit_at_distance = ConeNode.pure_pursuit_at_distance
    dynamic_lookahead = ConeNode.dynamic_lookahead

    parameters = {
        "wheelbase_m": 0.33,
        "max_steer_cmd": 26.0,
        "lookahead_min_m": 0.7,
        "lookahead_max_m": 1.45,
        "lookahead_scale": 0.12,
        "far_preview_distance_m": 0.75,
        "far_preview_weight": 0.65,
        "steering_gain": 1.18,
    }

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _GeometryHarness:
    calculate_midpoints = ConeNode.calculate_midpoints
    infer_midpoints_from_single_boundary = ConeNode.infer_midpoints_from_single_boundary
    infer_midpoints_from_richer_boundary = ConeNode.infer_midpoints_from_richer_boundary
    select_inferred_boundary = ConeNode.select_inferred_boundary
    continuous_boundary_segment = ConeNode.continuous_boundary_segment
    boundary_is_sufficient = ConeNode.boundary_is_sufficient
    nearest_gate_midpoint = ConeNode.nearest_gate_midpoint
    offset_boundary_to_center = ConeNode.offset_boundary_to_center

    parameters = {
        "pair_max_forward_delta_m": 0.30,
        "min_corridor_width_m": 0.68,
        "max_corridor_width_m": 0.98,
        "expected_corridor_width_m": 0.85,
        "allow_single_boundary_fallback": True,
        "single_boundary_min_cones": 2,
        "single_boundary_min_span_m": 0.20,
        "allow_nearest_gate_fallback": True,
        "fallback_pair_min_lateral_separation_m": 0.4,
        "fallback_pair_max_center_offset_m": 0.65,
        "group_grow_distance_m": 0.5,
        "single_boundary_switch_frames": 3,
        "min_path_midpoints": 2,
    }

    def __init__(self):
        self.midpoints_inferred = False
        self.midpoint_source = "none"
        self.active_inferred_boundary = None
        self.pending_inferred_boundary = None
        self.pending_inferred_frames = 0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _PathHarness:
    pure_pursuit_at_distance = ConeNode.pure_pursuit_at_distance
    preview_steering_demand = ConeNode.preview_steering_demand
    straight_boost_ratio = ConeNode.straight_boost_ratio
    compute_speed = ConeNode.compute_speed

    parameters = {
        "lookahead_min_m": 0.7,
        "lookahead_max_m": 1.45,
        "far_preview_distance_m": 0.75,
        "far_preview_weight": 0.65,
        "wheelbase_m": 0.33,
        "max_steer_cmd": 26.0,
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
        "single_boundary_max_speed": 9.5,
    }

    def __init__(self):
        self.path_is_held = False
        self.path_miss_count = 0
        self.midpoint_source = "paired"

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _RecoveryHarness:
    publish_blind_recovery = ConeNode.publish_blind_recovery
    parameters = {
        "blind_recovery_frames": 10,
        "blind_recovery_min_clusters": 2,
        "blind_recovery_steer_decay": 0.92,
        "cone_min_drive_speed": 9.0,
        "min_confidence": 0.3,
    }

    def __init__(self):
        self.had_valid_path = True
        self.blind_recovery_count = 0
        self.last_valid_steering = 10.0
        self.commands = []

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


def test_blind_recovery_moves_but_is_strictly_bounded():
    harness = _RecoveryHarness()
    for _ in range(10):
        assert harness.publish_blind_recovery(cluster_count=4)
    assert not harness.publish_blind_recovery(cluster_count=4)
    assert all(speed == 9.0 for _, speed, _ in harness.commands)
    assert abs(harness.commands[-1][0]) < abs(harness.commands[0][0])
