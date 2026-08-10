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


def test_integrated_run_uses_ten_as_cone_speed() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'CONE_SPEED_COMMAND="10.0"' in source
    assert '"cone_speed_command", default_value="10.0"' in launch_source


def test_integrated_run_defaults_to_22_straight_14_curve_and_12_full_lock() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    complete_source = COMPLETE_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    for source in (run_source, complete_source):
        assert "speed < 14.0 ? speed : 14.0" in source
        assert "curve < 12.0 ? curve : 12.0" in source
        assert "12.0 0.0 30.0" in source
    assert '"speed_command", default_value="22.0"' in launch_source
    assert '"curve_speed_command", default_value="14.0"' in launch_source
    assert '"degraded_path_speed_command", default_value="12.0"' in launch_source
    assert '"adaptive_curve_lookahead_m", default_value="0.30"' in launch_source
