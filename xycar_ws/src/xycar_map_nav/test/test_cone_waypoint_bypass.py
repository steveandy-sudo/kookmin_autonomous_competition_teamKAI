from xycar_map_nav.cone_waypoint_bypass import ConeBypassConfig
from xycar_map_nav.cone_waypoint_bypass import ConeBypassEvent
from xycar_map_nav.cone_waypoint_bypass import PreWaypointConeBypass


def make_bypass():
    return PreWaypointConeBypass(
        ConeBypassConfig(entry_frames=3, exit_frames=2)
    )


def update(bypass, confidence, speed=3.0, waypoint=0, lap=0):
    return bypass.update(
        lap_count=lap,
        target_waypoint_index=waypoint,
        confidence=confidence,
        speed_command=speed,
    )


def test_cone_must_start_and_finish_before_waypoint_one_is_skipped():
    bypass = make_bypass()
    assert update(bypass, 0.8) == ConeBypassEvent.NONE
    assert update(bypass, 0.8) == ConeBypassEvent.NONE
    assert update(bypass, 0.8) == ConeBypassEvent.STARTED
    assert bypass.active
    assert update(bypass, 0.0, speed=0.0) == ConeBypassEvent.NONE
    assert update(bypass, 0.0, speed=0.0) == ConeBypassEvent.FINISHED
    assert not bypass.active
    assert bypass.completed_lap == 0


def test_cone_is_ignored_after_waypoint_one():
    bypass = make_bypass()
    for _ in range(5):
        assert update(bypass, 0.9, waypoint=1) == ConeBypassEvent.NONE
    assert not bypass.active


def test_completed_cone_cannot_skip_twice_in_same_lap_but_resets_next_lap():
    bypass = make_bypass()
    for _ in range(3):
        event = update(bypass, 0.9)
    assert event == ConeBypassEvent.STARTED
    update(bypass, 0.0, speed=0.0)
    assert (
        update(bypass, 0.0, speed=0.0) == ConeBypassEvent.FINISHED
    )
    for _ in range(4):
        assert update(bypass, 0.9) == ConeBypassEvent.NONE
    events = [update(bypass, 0.9, lap=1) for _ in range(3)]
    assert events[-1] == ConeBypassEvent.STARTED


def test_cone_waits_for_yolo_and_one_meter_lidar_gate():
    bypass = make_bypass()
    for _ in range(4):
        assert (
            bypass.update(
                lap_count=0,
                target_waypoint_index=0,
                confidence=0.9,
                speed_command=3.0,
                yolo_confirmed=False,
                lidar_distance_m=0.8,
            )
            == ConeBypassEvent.NONE
        )
    for _ in range(4):
        assert (
            bypass.update(
                lap_count=0,
                target_waypoint_index=0,
                confidence=0.9,
                speed_command=3.0,
                yolo_confirmed=True,
                lidar_distance_m=1.2,
            )
            == ConeBypassEvent.NONE
        )
    events = [
        bypass.update(
            lap_count=0,
            target_waypoint_index=0,
            confidence=0.9,
            speed_command=3.0,
            yolo_confirmed=True,
            lidar_distance_m=0.9,
        )
        for _ in range(3)
    ]
    assert events[-1] == ConeBypassEvent.STARTED
