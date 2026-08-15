from xycar_map_nav.cone_mode_latch import ConeModeConfig
from xycar_map_nav.cone_mode_latch import ConeModeEvent
from xycar_map_nav.cone_mode_latch import ConeModeLatch
from xycar_map_nav.sequential_hybrid_driver import cone_disarm_hold_active


def make_latch():
    return ConeModeLatch(ConeModeConfig(entry_frames=3, exit_frames=2))


def observe(latch, *, yolo=True, distance=0.8, confidence=0.8, speed=3.0):
    return latch.observe_command(
        confidence=confidence,
        speed_command=speed,
        yolo_confirmed=yolo,
        lidar_distance_m=distance,
    )


def test_cone_requires_confirmed_yolo_lidar_entry():
    latch = make_latch()
    for _ in range(3):
        assert observe(latch, yolo=False) == ConeModeEvent.NONE
    for _ in range(3):
        assert observe(latch, distance=3.2) == ConeModeEvent.NONE
    assert observe(latch) == ConeModeEvent.NONE
    assert observe(latch) == ConeModeEvent.NONE
    assert observe(latch) == ConeModeEvent.STARTED
    assert latch.active


def test_cone_accepts_lidar_cluster_within_three_meters():
    latch = make_latch()

    assert observe(latch, distance=2.8) == ConeModeEvent.NONE
    assert observe(latch, distance=2.8) == ConeModeEvent.NONE
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
