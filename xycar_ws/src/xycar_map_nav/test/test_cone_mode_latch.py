from xycar_map_nav.cone_mode_latch import ConeModeConfig
from xycar_map_nav.cone_mode_latch import ConeModeEvent
from xycar_map_nav.cone_mode_latch import ConeModeLatch
from xycar_map_nav.sequential_hybrid_driver import command_timestamp_is_fresh
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


def test_cone_entry_ignores_distance_by_default():
    latch = make_latch()

    assert observe(latch, distance=float("inf")) == ConeModeEvent.NONE
    assert observe(latch, distance=float("inf")) == ConeModeEvent.NONE
    assert (
        observe(latch, distance=float("inf"))
        == ConeModeEvent.STARTED
    )


def test_optional_cone_distance_gate_restores_legacy_behavior():
    latch = ConeModeLatch(
        ConeModeConfig(
            entry_frames=1,
            entry_distance_enabled=True,
            entry_distance_m=3.0,
        )
    )
    assert observe(latch, distance=3.2) == ConeModeEvent.NONE
    assert observe(latch, distance=2.8) == ConeModeEvent.STARTED


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
