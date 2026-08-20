from xycar_map_nav.cone_mode_latch import ConeModeConfig
from xycar_map_nav.cone_mode_latch import ConeModeEvent
from xycar_map_nav.cone_mode_latch import ConeModeLatch
from xycar_map_nav.sequential_hybrid_driver import command_timestamp_is_fresh
from xycar_map_nav.sequential_hybrid_driver import cone_approach_brake_decision
from xycar_map_nav.sequential_hybrid_driver import cone_approach_speed_limit
from xycar_map_nav.sequential_hybrid_driver import cone_brake_hold_release_ready
from xycar_map_nav.sequential_hybrid_driver import cone_disarm_hold_active
from xycar_map_nav.sequential_hybrid_driver import cone_processing_requested


def make_latch():
    return ConeModeLatch(ConeModeConfig(entry_frames=3, exit_frames=2))


def observe(latch, *, yolo=True, distance=0.8, confidence=0.8, speed=3.0):
    return latch.observe_command(
        confidence=confidence,
        speed_command=speed,
        yolo_confirmed=yolo,
        lidar_distance_m=distance,
    )


def test_cone_requires_confirmed_yolo_and_valid_command_entry():
    latch = make_latch()
    for _ in range(3):
        assert observe(latch, yolo=False) == ConeModeEvent.NONE
    assert observe(latch) == ConeModeEvent.NONE
    assert observe(latch) == ConeModeEvent.NONE
    assert observe(latch) == ConeModeEvent.STARTED
    assert latch.active


def test_cone_entry_rejects_missing_forward_distance_by_default():
    latch = make_latch()

    for _ in range(3):
        assert observe(latch, distance=float("inf")) == ConeModeEvent.NONE
    assert not latch.active


def test_configured_cone_forward_distance_gate():
    latch = ConeModeLatch(
        ConeModeConfig(
            entry_frames=1,
            entry_distance_m=3.0,
        )
    )
    assert observe(latch, distance=3.2) == ConeModeEvent.NONE
    assert observe(latch, distance=2.8) == ConeModeEvent.STARTED


def test_bag_optimized_handoff_preserves_success_and_delays_early_entry():
    latch = ConeModeLatch(
        ConeModeConfig(entry_frames=3, entry_distance_m=0.95)
    )

    # Hand-pushed monitor bag transitioned prematurely at forward x=1.320 m.
    for _ in range(3):
        assert observe(latch, distance=1.320) == ConeModeEvent.NONE

    # Competition-course success bag's three valid planner frames.
    assert observe(latch, distance=0.883) == ConeModeEvent.NONE
    assert observe(latch, distance=0.797) == ConeModeEvent.NONE
    assert observe(latch, distance=0.701) == ConeModeEvent.STARTED


def test_bag_optimized_handoff_does_not_further_delay_late_failure_case():
    latch = ConeModeLatch(
        ConeModeConfig(entry_frames=3, entry_distance_m=0.95)
    )

    # All three frames from the late collision run still qualify immediately;
    # excessive entry speed is handled by the separate dynamic brake hold.
    assert observe(latch, distance=0.513) == ConeModeEvent.NONE
    assert observe(latch, distance=0.331) == ConeModeEvent.NONE
    assert observe(latch, distance=0.173) == ConeModeEvent.STARTED


def test_yolo_presence_holds_last_cone_mode_when_command_is_invalid():
    latch = make_latch()
    for _ in range(3):
        observe(latch)
    for _ in range(20):
        assert (
            latch.update_presence(sensor_present=True)
            == ConeModeEvent.NONE
        )
    assert latch.active


def test_lidar_presence_alone_keeps_cone_mode_active():
    latch = make_latch()
    for _ in range(3):
        observe(latch)
    assert latch.update_presence(sensor_present=True) == ConeModeEvent.NONE
    assert latch.active


def test_cone_finishes_only_after_both_sensors_are_absent():
    latch = make_latch()
    for _ in range(3):
        observe(latch)
    assert latch.update_presence(sensor_present=False) == ConeModeEvent.NONE
    assert latch.active
    assert latch.update_presence(sensor_present=True) == ConeModeEvent.NONE
    assert latch.update_presence(sensor_present=False) == ConeModeEvent.NONE
    assert (
        latch.update_presence(sensor_present=False)
        == ConeModeEvent.FINISHED
    )
    assert not latch.active


def test_cone_timed_exit_requires_continuous_sensor_absence():
    latch = ConeModeLatch(
        ConeModeConfig(
            entry_frames=3,
            exit_frames=1,
            exit_absence_sec=1.0,
        )
    )
    for _ in range(3):
        observe(latch)

    assert (
        latch.update_presence(sensor_present=False, now_sec=10.0)
        == ConeModeEvent.NONE
    )
    assert (
        latch.update_presence(sensor_present=False, now_sec=10.9)
        == ConeModeEvent.NONE
    )
    assert (
        latch.update_presence(sensor_present=True, now_sec=10.95)
        == ConeModeEvent.NONE
    )
    assert (
        latch.update_presence(sensor_present=False, now_sec=20.0)
        == ConeModeEvent.NONE
    )
    assert (
        latch.update_presence(sensor_present=False, now_sec=21.0)
        == ConeModeEvent.FINISHED
    )


def test_cone_can_reenter_without_waypoint_or_lap_state():
    latch = make_latch()
    for _ in range(3):
        event = observe(latch)
    assert event == ConeModeEvent.STARTED
    latch.update_presence(sensor_present=False)
    assert (
        latch.update_presence(sensor_present=False)
        == ConeModeEvent.FINISHED
    )
    events = [observe(latch) for _ in range(3)]
    assert events[-1] == ConeModeEvent.STARTED


def test_space_disarm_hold_covers_observed_operator_pause():
    assert cone_disarm_hold_active(
        now_sec=14.1,
        disarmed_since_sec=10.0,
        hold_sec=5.0,
    )
    assert not cone_disarm_hold_active(
        now_sec=15.01,
        disarmed_since_sec=10.0,
        hold_sec=5.0,
    )


def test_cone_command_freshness_uses_lidar_derived_command_time():
    assert command_timestamp_is_fresh(
        now_sec=10.30,
        command_time_sec=10.0,
        timeout_sec=0.35,
    )
    assert not command_timestamp_is_fresh(
        now_sec=10.36,
        command_time_sec=10.0,
        timeout_sec=0.35,
    )
    assert not command_timestamp_is_fresh(
        now_sec=9.9,
        command_time_sec=10.0,
        timeout_sec=0.35,
    )
    assert not command_timestamp_is_fresh(
        now_sec=10.0,
        command_time_sec=float("-inf"),
        timeout_sec=0.35,
    )


def test_cone_planning_is_requested_while_motor_gate_is_stopped():
    assert cone_processing_requested(
        shortcut_active=False,
        cone_active=False,
        yolo_age_sec=0.20,
        yolo_timeout_sec=0.75,
    )
    assert cone_processing_requested(
        shortcut_active=False,
        cone_active=True,
        yolo_age_sec=5.0,
        yolo_timeout_sec=0.75,
    )
    assert not cone_processing_requested(
        shortcut_active=True,
        cone_active=True,
        yolo_age_sec=0.20,
        yolo_timeout_sec=0.75,
    )
    assert not cone_processing_requested(
        shortcut_active=False,
        cone_active=False,
        yolo_age_sec=0.80,
        yolo_timeout_sec=0.75,
    )


def approach_limit(**overrides):
    values = {
        "enabled": True,
        "cone_active": False,
        "shortcut_active": False,
        "yolo_frames": 0,
        "yolo_age_sec": float("inf"),
        "yolo_timeout_sec": 0.75,
        "confirmed_yolo_frames": 2,
        "yolo_confidence": 0.0,
        "strong_yolo_confidence": 0.50,
        "cluster_count": 0,
        "cluster_age_sec": float("inf"),
        "cluster_timeout_sec": 0.50,
        "first_yolo_speed_command": 15.0,
        "confirmed_speed_command": 8.0,
    }
    values.update(overrides)
    return cone_approach_speed_limit(**values)


def test_first_central_cone_yolo_caps_speed_without_steering_handoff():
    assert approach_limit(
        yolo_frames=1,
        yolo_age_sec=0.1,
        yolo_confidence=0.42,
    ) == (
        15.0,
        "first_yolo",
    )


def test_strong_first_yolo_immediately_uses_validated_cone_speed():
    assert approach_limit(
        yolo_frames=1,
        yolo_age_sec=0.1,
        yolo_confidence=0.62,
    ) == (8.0, "confirmed")


def test_second_yolo_or_fused_cluster_uses_validated_cone_speed():
    assert approach_limit(yolo_frames=2, yolo_age_sec=0.1) == (
        8.0,
        "confirmed",
    )
    assert approach_limit(cluster_count=1, cluster_age_sec=0.1) == (
        8.0,
        "confirmed",
    )


def brake_decision(**overrides):
    values = {
        "enabled": True,
        "cone_active": False,
        "shortcut_active": False,
        "cluster_count": 1,
        "cluster_age_sec": 0.1,
        "cluster_timeout_sec": 0.50,
        "cluster_forward_distance_m": 1.0,
        "vehicle_speed_mps": 0.0,
        "vehicle_speed_age_sec": 0.05,
        "vehicle_speed_timeout_sec": 0.25,
        "target_speed_mps": 0.65,
        "deceleration_mps2": 1.50,
        "response_time_sec": 0.10,
        "distance_margin_m": 0.10,
        "hard_stop_distance_m": 0.35,
        "stale_speed_stop_distance_m": 0.80,
    }
    values.update(overrides)
    return cone_approach_brake_decision(**values)


def test_success_bag_speed_does_not_trigger_unnecessary_entry_stop():
    required, distance, reason = brake_decision(
        cluster_forward_distance_m=0.797,
        vehicle_speed_mps=0.686,
    )
    assert not required
    assert 0.18 < distance < 0.19
    assert reason == "braking_distance"


def test_failure_bag_speed_triggers_entry_brake_before_handoff():
    required, distance, reason = brake_decision(
        cluster_forward_distance_m=0.714,
        vehicle_speed_mps=1.432,
    )
    assert required
    assert 0.78 < distance < 0.80
    assert reason == "braking_distance"


def test_entry_brake_has_hard_distance_and_stale_speed_fallbacks():
    assert brake_decision(
        cluster_forward_distance_m=0.34,
    ) == (True, 0.35, "hard_distance")
    assert brake_decision(
        cluster_forward_distance_m=0.79,
        vehicle_speed_age_sec=0.30,
    ) == (True, 0.80, "speed_stale")


def test_latched_brake_releases_only_with_fresh_cone_steering_at_safe_speed():
    values = {
        "cone_active": True,
        "cone_command_fresh": True,
        "vehicle_speed_mps": 0.75,
        "vehicle_speed_age_sec": 0.05,
        "vehicle_speed_timeout_sec": 0.25,
        "release_speed_mps": 0.80,
    }
    assert cone_brake_hold_release_ready(**values)
    assert not cone_brake_hold_release_ready(
        **{**values, "vehicle_speed_mps": 1.016}
    )
    assert not cone_brake_hold_release_ready(
        **{**values, "cone_command_fresh": False}
    )


def test_cone_approach_cap_does_not_override_active_missions_or_stale_data():
    assert approach_limit(
        cone_active=True,
        yolo_frames=2,
        yolo_age_sec=0.1,
    ) == (None, "none")
    assert approach_limit(
        shortcut_active=True,
        yolo_frames=2,
        yolo_age_sec=0.1,
    ) == (None, "none")
    assert approach_limit(yolo_frames=2, yolo_age_sec=0.8) == (
        None,
        "none",
    )
