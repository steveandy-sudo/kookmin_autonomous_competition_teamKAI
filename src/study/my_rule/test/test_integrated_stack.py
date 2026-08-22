import numpy as np
import pytest
from pathlib import Path
import yaml

from my_rule.drive_manager_node import DriveManagerNode, interpolate_command
from my_rule.perception.lraspp_inference import (
    masks_from_probabilities,
    prepare_model_input,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class _Parameter:
    def __init__(self, value):
        self.value = value


class _ConeManagerHarness:
    select_final_command = DriveManagerNode.select_final_command

    parameters = {
        "cone_emergency_stop_distance_m": 0.35,
        "cone_emergency_clear_distance_m": 0.50,
        "cone_emergency_clear_hold_sec": 0.30,
        "external_cone_cmd_timeout_sec": 0.50,
        "cone_fresh_stop_is_authoritative": True,
        "cone_manager_recovery_enabled": False,
    }

    def __init__(self):
        self.traffic_control_enabled = False
        self.traffic_go = True
        self.cone_mode_active = True
        self.front_distance = float("inf")
        self.cone_emergency_latched = False
        self.cone_emergency_clear_started_at = 0.0
        self.cone_angle = -12.0
        self.cone_speed = 8.0
        self.cone_confidence = 0.8
        self.cone_command_time = 1.0
        self.last_valid_cone_angle = -12.0
        self.last_valid_cone_speed = 8.0
        self.last_valid_cone_time = 1.0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])

    def static_obstacle_command(self, _now):
        return None

    def dynamic_obstacle_command(self, _now):
        return None

    def cone_target_to_command(self, angle):
        return float(angle)

    def apply_course_signal_overlay(
        self, angle, speed, state, reason, _now
    ):
        return angle, speed, state, reason


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


def test_fresh_cone_stop_cannot_replay_an_older_valid_command():
    harness = _ConeManagerHarness()
    harness.cone_command_time = 9.9
    harness.cone_speed = 0.0
    harness.cone_confidence = 0.0
    harness.last_valid_cone_time = 9.8

    angle, speed, state, reason = harness.select_final_command(10.0)

    assert (angle, speed) == (0.0, 0.0)
    assert state == "CONE_STOP"
    assert reason == "fresh_cone_stop"


def test_stale_cone_command_stops_when_manager_recovery_is_disabled():
    harness = _ConeManagerHarness()
    harness.cone_command_time = 9.0
    harness.last_valid_cone_time = 9.0

    angle, speed, state, reason = harness.select_final_command(10.0)

    assert (angle, speed) == (0.0, 0.0)
    assert state == "CONE_STOP"
    assert reason == "waiting_fresh_cone_cmd"


def test_cone_emergency_stop_requires_clearance_hold_and_fresh_path():
    harness = _ConeManagerHarness()
    harness.cone_command_time = 9.9
    harness.front_distance = 0.30
    assert harness.select_final_command(10.0)[2] == "EMERGENCY_STOP"

    harness.front_distance = 0.60
    harness.cone_command_time = 10.1
    assert harness.select_final_command(10.1)[2] == "EMERGENCY_STOP"

    harness.cone_command_time = 10.40
    angle, speed, state, _reason = harness.select_final_command(10.41)
    assert state == "CONE_SLALOM"
    assert (angle, speed) == (-12.0, 8.0)


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


def test_cone_only_launch_disables_adaptive_profile():
    source = (PACKAGE_ROOT / "launch" / "cone_only.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'if profile != "fixed"' in source
    assert "adaptive cone speed is disabled" in source
    assert 'default_value="10.0"' in source
    assert 'default_value="8.0"' in source
    assert 'DeclareLaunchArgument(\n                "cone_speed",' not in source
