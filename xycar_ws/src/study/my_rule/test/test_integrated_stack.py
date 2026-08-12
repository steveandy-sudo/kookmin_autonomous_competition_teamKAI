import numpy as np
import pytest
from pathlib import Path
import yaml

from my_rule.drive_manager_node import interpolate_command
from my_rule.perception.lraspp_inference import (
    masks_from_probabilities,
    prepare_model_input,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_cone_physical_angle_uses_existing_vehicle_table():
    actual = [0.0, 4.0, 10.0, 16.0, 26.0]
    commands = [0.0, 10.0, 20.0, 30.0, 42.0]
    assert interpolate_command(4.0, actual, commands) == pytest.approx(10.0)
    assert interpolate_command(-16.0, actual, commands) == pytest.approx(-30.0)
    assert interpolate_command(26.0, actual, commands) == pytest.approx(42.0)


def test_cone_mapping_interpolates_and_clamps_at_table_edges():
    actual = [0.0, 4.0, 10.0, 16.0, 26.0]
    commands = [0.0, 10.0, 20.0, 30.0, 42.0]
    assert interpolate_command(7.0, actual, commands) == pytest.approx(15.0)
    assert interpolate_command(40.0, actual, commands) == pytest.approx(42.0)


def test_lraspp_input_contract_is_256_by_144_nchw():
    frame = np.zeros((1024, 1280, 3), dtype=np.uint8)
    tensor = prepare_model_input(frame, 256, 144)
    assert tensor.shape == (1, 3, 144, 256)
    assert tensor.flags.c_contiguous
    assert np.isfinite(tensor).all()


def test_lane_masks_are_disjoint_and_respect_confidence():
    probabilities = np.zeros((3, 2, 3), dtype=np.float32)
    probabilities[1, 0, 0] = 0.8
    probabilities[2, 1, 1] = 0.9
    probabilities[1, 0, 2] = 0.49
    white, yellow = masks_from_probabilities(
        probabilities,
        white_confidence=0.5,
        yellow_confidence=0.5,
    )
    assert white[0, 0] == 255
    assert yellow[1, 1] == 255
    assert white[0, 2] == 0
    assert not np.any((white > 0) & (yellow > 0))


def test_cone_config_allows_launch_overrides_to_take_effect():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "cone_control.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert "/**" in config
    assert "my_rule_cone_node" not in config
