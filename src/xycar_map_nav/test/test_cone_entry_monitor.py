import math

from xycar_map_nav.cone_entry_monitor_core import classify_entry_stage
from xycar_map_nav.cone_entry_monitor_core import ConeEntrySnapshot
from xycar_map_nav.cone_entry_monitor_core import ConeEntryThresholds
from xycar_map_nav.cone_entry_monitor_core import entry_checks
from xycar_map_nav.cone_entry_monitor_core import physical_angle_to_command


THRESHOLDS = ConeEntryThresholds()


def ready_snapshot(**changes) -> ConeEntrySnapshot:
    values = {
        "scan_fresh": True,
        "object_stream_fresh": True,
        "yolo_confidence": 0.80,
        "yolo_frames": 2,
        "yolo_fresh": True,
        "lidar_distance_m": 2.50,
        "lidar_count": 3,
        "lidar_fresh": True,
        "command_confidence": 0.70,
        "command_speed": 10.0,
        "command_fresh": True,
        "command_frames": 2,
    }
    values.update(changes)
    return ConeEntrySnapshot(**values)


def test_entry_checks_match_integrated_three_gate_contract() -> None:
    assert entry_checks(ready_snapshot(), THRESHOLDS) == (
        True,
        True,
        True,
    )
    assert entry_checks(
        ready_snapshot(yolo_frames=1), THRESHOLDS
    ) == (False, True, True)
    assert entry_checks(
        ready_snapshot(lidar_distance_m=3.01), THRESHOLDS
    ) == (True, False, True)
    assert entry_checks(
        ready_snapshot(command_confidence=0.34), THRESHOLDS
    ) == (True, True, False)


def test_stage_explains_each_unmet_condition() -> None:
    assert "YOLO" in classify_entry_stage(
        ready_snapshot(yolo_confidence=0.0, yolo_frames=0), THRESHOLDS
    )
    assert "LiDAR" in classify_entry_stage(
        ready_snapshot(
            lidar_distance_m=math.inf,
            lidar_count=0,
            lidar_fresh=False,
        ),
        THRESHOLDS,
    )
    assert "경로" in classify_entry_stage(
        ready_snapshot(command_fresh=False), THRESHOLDS
    )
    assert "2/3" in classify_entry_stage(ready_snapshot(), THRESHOLDS)
    assert "CONE_RULE" in classify_entry_stage(
        ready_snapshot(selector_active=True), THRESHOLDS
    )


def test_cone_physical_angle_mapping_matches_selector_calibration() -> None:
    assert physical_angle_to_command(0.0) == 0.0
    assert physical_angle_to_command(4.0) == 10.0
    assert physical_angle_to_command(-10.0) == -20.0
    assert physical_angle_to_command(16.0) == 30.0
    assert physical_angle_to_command(42.0) == 42.0
