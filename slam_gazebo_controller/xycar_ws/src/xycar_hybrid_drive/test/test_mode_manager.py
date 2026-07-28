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
        "rule_angle_command": 1.0,
        "cone_confidence": 0.0,
        "cone_age_sec": 10.0,
        "cone_cluster_count": 0,
    }
    values.update(overrides)
    return manager.update(**values)


def test_curve_enters_immediately_and_exits_after_stable_straight_frames():
    manager = HybridModeManager(ModeManagerConfig(curve_exit_frames=3))

    curved = update(
        manager,
        1,
        path_curvature_per_m=0.22,
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


def test_missing_model_falls_back_to_lane_rule():
    manager = HybridModeManager(ModeManagerConfig())
    decision = update(manager, 1, model_available=False)
    assert decision.mode == DriveMode.LANE_RULE_CURVE
