from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_parameters(filename: str) -> dict:
    config = yaml.safe_load((PACKAGE_ROOT / "config" / filename).read_text())
    return config["canonical_stanley_pursuit_driver"]["ros__parameters"]


def test_simulation_defaults_remain_unchanged():
    parameters = load_parameters("canonical_stanley_pursuit.yaml")
    assert parameters["extend_fused_path_to_white"] is False
    assert parameters["temporal_path_ego_compensation_enabled"] is False
    assert parameters["control_latency_preview_sec"] == 0.10


def test_real_profile_enables_measured_delay_compensation():
    parameters = load_parameters("canonical_stanley_pursuit_real.yaml")
    assert parameters["use_sim_time"] is False
    assert parameters["extend_fused_path_to_white"] is True
    assert parameters["temporal_path_ego_compensation_enabled"] is True
    assert parameters["control_latency_preview_sec"] == 0.35
    assert parameters["curve_detection_near_x_m"] == 0.20
    assert parameters["curve_detection_far_x_m"] == 1.20
    assert parameters["curve_detection_segment_count"] == 1
    assert parameters["adaptive_curve_lookahead_enabled"] is True
    assert parameters["adaptive_curve_lookahead_m"] == 0.30
    assert parameters["adaptive_curve_minimum_path_reach_m"] == 1.0
    assert parameters["adaptive_curve_confirmation_frames"] == 2
    assert parameters["adaptive_curve_release_frames"] == 2
    assert parameters["curve_steering_multiplier_enabled"] is True
    assert parameters["curve_steering_multiplier_activation_command"] == 10.0
    assert parameters["curve_steering_multiplier"] == 1.2
    assert parameters["steering_lead_time_sec"] == 0.08
    assert parameters["steering_max_lead_command"] == 6.0
    assert parameters["cruise_speed_command"] == 22.0
    assert parameters["minimum_speed_command"] == 8.0
    assert parameters["curve_speed_command"] == 14.0
    assert parameters["degraded_path_speed_command"] == 12.0
