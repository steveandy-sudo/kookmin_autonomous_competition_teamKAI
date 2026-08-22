from pathlib import Path

from xycar_parking_nav.map_core import FREE, OCCUPIED, UNKNOWN, load_occupancy_map
from xycar_parking_nav.validate_map import validate


PACKAGE = Path(__file__).resolve().parents[1]


def test_runtime_map_preserves_gray_as_unknown():
    occupancy_map = load_occupancy_map(PACKAGE / "maps" / "parking_map.yaml")
    counts = occupancy_map.counts()
    assert (occupancy_map.width, occupancy_map.height) == (143, 149)
    assert occupancy_map.resolution == 0.05
    assert counts[OCCUPIED] == 1223
    assert counts[UNKNOWN] == 11622
    assert counts[FREE] == 8462


def test_original_threshold_demonstrates_gray_free_failure():
    occupancy_map = load_occupancy_map(
        PACKAGE / "maps" / "parking_map_original.yaml"
    )
    counts = occupancy_map.counts()
    assert counts[UNKNOWN] == 0
    assert counts[FREE] == 20084


def test_all_configured_mission_poses_fit_static_map():
    report = validate(
        PACKAGE / "maps" / "parking_map.yaml",
        PACKAGE / "config" / "parking_mission.yaml",
        0.095,
    )
    assert report["valid"], report["poses"]
