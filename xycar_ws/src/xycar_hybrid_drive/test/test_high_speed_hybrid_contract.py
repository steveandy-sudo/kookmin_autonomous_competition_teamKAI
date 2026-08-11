from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def parameters(name: str) -> dict:
    payload = yaml.safe_load(
        (PACKAGE / "config" / name).read_text(encoding="utf-8")
    )
    return payload["xycar_hybrid_drive"]["ros__parameters"]


def test_sim_and_real_use_straight_model_and_continuous_curve_contract():
    for name in ("hybrid_sim.yaml", "hybrid_real.yaml"):
        config = parameters(name)
        assert config["hybrid_skip_model_in_curve"] is True
        assert config["hybrid_model_speed_cap"] == 25.0
        assert config["hybrid_model_stabilizer_enabled"] is True
        assert config["hybrid_model_transition_neutral_sec"] == 0.0
        assert config["hybrid_model_transition_fast_sec"] == 0.55
        assert config["hybrid_model_transition_steering_alpha"] == 0.65
        assert config["hybrid_model_transition_rate_limit"] == 0.25
        assert config["hybrid_model_transition_steering_cap"] == 15.0
        assert config["hybrid_curve_three_level_enabled"] is False
        expected_alpha = 1.0 if name == "hybrid_sim.yaml" else 0.65
        expected_rate_limit = 42.0 if name == "hybrid_sim.yaml" else 8.0
        assert config["hybrid_curve_continuous_alpha"] == expected_alpha
        assert (
            config["hybrid_curve_continuous_rate_limit_command"]
            == expected_rate_limit
        )
        assert config["hybrid_curve_full_lock_command"] == 42.0
        assert config["hybrid_curve_pulse_duty_scale"] == 0.75
        assert config["hybrid_curve_speed_command"] == 25.0
        assert config["hybrid_sim_track_mode_override"] is False
        assert config["hybrid_sim_track_curve_feedback_enabled"] is False
        assert config["hybrid_sim_track_curve_feedback_heading_gain"] == 1.5
        expected_lookahead = 1.20 if name == "hybrid_sim.yaml" else 0.60
        assert (
            config["hybrid_sim_track_curve_feedback_lookahead_m"]
            == expected_lookahead
        )
        assert config["hybrid_sim_track_curve_exit_heading_guard_rad"] == 0.12
        assert config["hybrid_sim_track_curve_exit_cte_guard_m"] == 0.15
        assert config["hybrid_curve_exit_frames"] == 1
        assert config["hybrid_curve_min_duration_sec"] == 1.2
        expected_straight_hold = 0.5 if name == "hybrid_sim.yaml" else 0.0
        assert config["hybrid_straight_min_duration_sec"] == expected_straight_hold
        assert config["hybrid_curve_minimum_zero_frames"] == 1
        assert config["hybrid_curve_maximum_zero_frames"] == 1
        assert config["hybrid_curve_forced_pulse_min_command"] == 8.0
        assert config["hybrid_curve_stable_pulse_min_command"] == 5.0
        assert config["hybrid_curve_stable_pulse_frames"] == 2
        assert config["hybrid_curve_entry_path_angle_rad"] == 0.12
        assert config["hybrid_curve_exit_path_angle_rad"] == 0.11
        assert config["target_right_offset_m"] == 0.0
