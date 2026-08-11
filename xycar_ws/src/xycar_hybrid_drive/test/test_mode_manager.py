from xycar_hybrid_drive.mode_manager import (
    DriveMode,
    HybridModeManager,
    ModeManagerConfig,
)


def update(manager, generation, **overrides):
    values = {
        "now_sec": float(generation) * 0.1,
        "canonical_generation": generation,
        "cone_generation": generation,
        "lane_available": True,
        "model_available": True,
        "path_curvature_per_m": 0.02,
        "path_turn_angle_rad": 0.02,
        "rule_angle_command": 1.0,
        "route_curve_override": None,
        "cone_confidence": 0.0,
        "cone_age_sec": 10.0,
        "cone_cluster_count": 0,
    }
    values.update(overrides)
    return manager.update(**values)


def test_curve_enters_immediately_and_exits_after_stable_straight_frames():
    manager = HybridModeManager(
        ModeManagerConfig(
            curve_exit_frames=3,
            curve_min_duration_sec=0.0,
        )
    )

    curved = update(
        manager,
        1,
        path_curvature_per_m=0.22,
        path_turn_angle_rad=0.20,
        rule_angle_command=12.0,
    )
    assert curved.mode == DriveMode.LANE_RULE_CURVE

    assert update(manager, 2).mode == DriveMode.LANE_RULE_CURVE
    assert update(manager, 3).mode == DriveMode.LANE_RULE_CURVE
    straight = update(manager, 4)
    assert straight.mode == DriveMode.MODEL_STRAIGHT


def test_cone_has_priority_after_consecutive_lidar_corridor_paths():
    manager = HybridModeManager(
        ModeManagerConfig(cone_entry_frames=3, cone_lane_recovery_frames=2)
    )
    for generation in (1, 2):
        decision = update(
            manager,
            generation,
            cone_confidence=0.8,
            cone_age_sec=0.0,
            cone_cluster_count=6,
        )
        assert decision.mode == DriveMode.MODEL_STRAIGHT
    decision = update(
        manager,
        3,
        cone_confidence=0.8,
        cone_age_sec=0.0,
        cone_cluster_count=6,
    )
    assert decision.mode == DriveMode.CONE_RULE

    manager.mode_started_sec = 0.0
    assert update(manager, 20).mode == DriveMode.CONE_RULE
    assert update(manager, 21).mode == DriveMode.MODEL_STRAIGHT


def test_missing_model_waits_in_straight_mode():
    manager = HybridModeManager(ModeManagerConfig())
    decision = update(manager, 1, model_available=False)
    assert decision.mode == DriveMode.MODEL_STRAIGHT
    assert decision.reason == "waiting for straight model"


def test_curve_rule_still_has_priority_when_model_is_missing():
    manager = HybridModeManager(ModeManagerConfig())
    decision = update(
        manager,
        1,
        model_available=False,
        path_curvature_per_m=0.22,
        path_turn_angle_rad=0.20,
        rule_angle_command=12.0,
    )
    assert decision.mode == DriveMode.LANE_RULE_CURVE


def test_curvature_noise_without_steering_evidence_stays_straight():
    manager = HybridModeManager(ModeManagerConfig())
    decision = update(
        manager,
        1,
        path_curvature_per_m=0.48,
        path_turn_angle_rad=0.03,
        rule_angle_command=3.0,
    )
    assert decision.mode == DriveMode.MODEL_STRAIGHT


def test_curvature_and_path_turn_evidence_enter_curve():
    manager = HybridModeManager(ModeManagerConfig())
    decision = update(
        manager,
        1,
        path_curvature_per_m=0.22,
        path_turn_angle_rad=0.115,
        rule_angle_command=7.0,
    )
    assert decision.mode == DriveMode.LANE_RULE_CURVE


def test_straight_geometry_exits_despite_lagged_curve_command():
    manager = HybridModeManager(
        ModeManagerConfig(
            curve_exit_frames=2,
            curve_min_duration_sec=0.0,
        )
    )
    assert update(
        manager,
        1,
        path_curvature_per_m=0.30,
        path_turn_angle_rad=0.20,
        rule_angle_command=-30.0,
    ).mode == DriveMode.LANE_RULE_CURVE
    first_straight = update(
        manager,
        2,
        path_curvature_per_m=0.08,
        path_turn_angle_rad=0.10,
        rule_angle_command=-20.0,
    )
    assert first_straight.mode == DriveMode.LANE_RULE_CURVE
    decision = update(
        manager,
        3,
        path_curvature_per_m=0.08,
        path_turn_angle_rad=0.10,
        rule_angle_command=-20.0,
    )
    assert decision.mode == DriveMode.MODEL_STRAIGHT


def test_single_straight_frame_cannot_end_a_new_curve_before_minimum_hold():
    manager = HybridModeManager(
        ModeManagerConfig(
            curve_exit_frames=1,
            curve_min_duration_sec=1.2,
        )
    )
    assert update(
        manager,
        1,
        path_curvature_per_m=0.30,
        path_turn_angle_rad=0.20,
    ).mode == DriveMode.LANE_RULE_CURVE
    early = update(
        manager,
        2,
        path_curvature_per_m=0.05,
        path_turn_angle_rad=0.02,
    )
    assert early.mode == DriveMode.LANE_RULE_CURVE
    assert early.reason == "minimum curve hold"
    late = update(
        manager,
        14,
        path_curvature_per_m=0.05,
        path_turn_angle_rad=0.02,
    )
    assert late.mode == DriveMode.MODEL_STRAIGHT


def test_route_override_selects_curve_and_straight_independent_of_lane_noise():
    manager = HybridModeManager(
        ModeManagerConfig(curve_min_duration_sec=0.0)
    )
    curved = update(
        manager,
        1,
        lane_available=False,
        path_curvature_per_m=0.0,
        path_turn_angle_rad=0.0,
        route_curve_override=True,
    )
    assert curved.mode == DriveMode.LANE_RULE_CURVE
    straight = update(
        manager,
        2,
        lane_available=False,
        path_curvature_per_m=1.0,
        path_turn_angle_rad=1.0,
        route_curve_override=False,
    )
    assert straight.mode == DriveMode.MODEL_STRAIGHT


def test_route_curve_cannot_interrupt_a_new_straight_minimum_hold():
    manager = HybridModeManager(
        ModeManagerConfig(
            curve_min_duration_sec=0.0,
            straight_min_duration_sec=0.5,
        )
    )
    assert update(
        manager,
        1,
        route_curve_override=True,
    ).mode == DriveMode.LANE_RULE_CURVE
    assert update(
        manager,
        2,
        route_curve_override=False,
    ).mode == DriveMode.MODEL_STRAIGHT
    held = update(
        manager,
        3,
        route_curve_override=True,
    )
    assert held.mode == DriveMode.MODEL_STRAIGHT
    assert held.reason == "minimum straight hold"
    assert update(
        manager,
        8,
        route_curve_override=True,
    ).mode == DriveMode.LANE_RULE_CURVE
