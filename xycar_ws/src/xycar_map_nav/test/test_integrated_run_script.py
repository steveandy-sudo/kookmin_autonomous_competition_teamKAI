from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RUN_SCRIPT = PACKAGE_ROOT / "scripts" / "run_space_hybrid_test.sh"
COMPLETE_SCRIPT = PACKAGE_ROOT / "scripts" / "run_complete_space_hybrid.sh"
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "real_sequential_hybrid_drive.launch.py"


def test_integrated_run_rejects_stale_controller_stack() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")

    assert "flock -n 9" in source
    assert "check_existing_control_stack" in source
    assert "/canonical_stanley_pursuit_driver" in source
    assert "/sequential_hybrid_driver" in source


def test_integrated_run_verifies_requested_straight_speed() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")

    assert "verify_runtime_cruise_speed" in source
    assert "cruise_speed_command" in source
    assert "requested=\"$SPEED_COMMAND\"" in source
    assert "runtime_cruise_speed_command" in source


def test_integrated_run_uses_eight_as_cone_speed() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'CONE_SPEED_COMMAND="8.0"' in source
    assert '"cone_speed_command", default_value="8.0"' in launch_source


def test_integrated_run_defaults_to_requested_25_16_15_control_profile() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    complete_source = COMPLETE_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    for source in (run_source, complete_source):
        assert 'SPEED_COMMAND="${1:-${SPEED_COMMAND:-25.0}}"' in source
        assert 'CURVATURE_SPEED_CONTROL_ENABLED="${CURVATURE_SPEED_CONTROL_ENABLED:-true}"' in source
        assert 'CURVE_SPEED_COMMAND="${CURVE_SPEED_COMMAND:-16.0}"' in source
        assert 'DEGRADED_PATH_SPEED_COMMAND="${DEGRADED_PATH_SPEED_COMMAND:-15.0}"' in source
        assert 'STRAIGHT_PATH_CURVATURE_THRESHOLD="${STRAIGHT_PATH_CURVATURE_THRESHOLD:-0.24}"' in source
        assert 'STEERING_CURRENT_WEIGHT="${STEERING_CURRENT_WEIGHT:-0.35}"' in source
        assert 'STEERING_CURVE_CURRENT_WEIGHT="${STEERING_CURVE_CURRENT_WEIGHT:-0.80}"' in source
        assert "speed < 16.0 ? speed : 16.0" in source
        assert "curve < 15.0 ? curve : 15.0" in source
        assert "12.0 0.0 30.0" in source
    assert '"speed_command", default_value="25.0"' in launch_source
    assert '"curve_speed_command", default_value="16.0"' in launch_source
    assert '"degraded_path_speed_command", default_value="15.0"' in launch_source
    assert '"adaptive_curve_lookahead_m", default_value="0.30"' in launch_source
    assert '"straight_path_curvature_threshold", default_value="0.24"' in launch_source
    assert '"steering_current_weight", default_value="0.35"' in launch_source
    assert '"steering_curve_current_weight", default_value="0.80"' in launch_source
