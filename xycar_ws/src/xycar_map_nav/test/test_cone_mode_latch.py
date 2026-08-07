from xycar_map_nav.cone_mode_latch import ConeModeConfig
from xycar_map_nav.cone_mode_latch import ConeModeEvent
from xycar_map_nav.cone_mode_latch import ConeModeLatch


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
        assert observe(latch, distance=1.2) == ConeModeEvent.NONE
    assert observe(latch) == ConeModeEvent.NONE
    assert observe(latch) == ConeModeEvent.NONE
    assert observe(latch) == ConeModeEvent.STARTED
    assert latch.active


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
