import math
import xml.etree.ElementTree as ET

import pytest

from xycar_gazebo_bridge.sim_control import (
    GazeboController,
    OBSTACLE_PRESETS,
    obstacle_sdf,
)


def test_set_pose_dry_run_uses_degree_yaw():
    result = GazeboController(dry_run=True).set_pose(
        "xycar_ackermann",
        x=1.0,
        y=-2.0,
        z=0.05,
        yaw_deg=90.0,
    )
    request = result["request"]
    expected = math.sqrt(0.5)
    assert f"z: {expected:.9f}" in request
    assert f"w: {expected:.9f}" in request
    assert 'name: "xycar_ackermann"' in request


@pytest.mark.parametrize("preset", sorted(OBSTACLE_PRESETS))
def test_obstacle_presets_are_valid_sdf(preset):
    root = ET.fromstring(obstacle_sdf("object_01", preset))
    assert root.tag == "sdf"
    assert root.find("./model[@name='object_01']") is not None
    assert root.find(".//collision") is not None
    assert root.find(".//visual") is not None


def test_visual_only_glass_has_no_collision():
    root = ET.fromstring(
        obstacle_sdf(
            "glass_01",
            "glass_panel",
            collision_enabled=False,
        )
    )
    assert root.find(".//collision") is None
    assert root.find(".//visual") is not None


def test_spawn_dry_run_uses_create_service():
    result = GazeboController(dry_run=True).spawn_obstacle(
        "chair_01",
        "chair",
        x=3.0,
        y=4.0,
        yaw_deg=30.0,
    )
    assert result["endpoint"] == "create"
    assert "/world/kookmin_xycar_track/create" in result["command"]
    assert "chair_01" in result["request"]


def test_invalid_entity_name_is_rejected():
    with pytest.raises(ValueError):
        GazeboController(dry_run=True).remove('bad"name')
