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


def test_integrated_run_exposes_shortcut_spatial_gate_tuning() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")
    complete_source = COMPLETE_SCRIPT.read_text(encoding="utf-8")

    for script_source in (source, complete_source):
        assert (
            'SHORTCUT_SPATIAL_GATE_MINIMUM_DISTANCE_M="'
            '${SHORTCUT_SPATIAL_GATE_MINIMUM_DISTANCE_M:-0.25}"'
            in script_source
        )
        assert (
            'SHORTCUT_SPATIAL_GATE_BLEND_DISTANCE_M="'
            '${SHORTCUT_SPATIAL_GATE_BLEND_DISTANCE_M:-0.25}"'
            in script_source
        )
    assert "export SHORTCUT_SPATIAL_GATE_MINIMUM_DISTANCE_M" in complete_source
    assert "export SHORTCUT_SPATIAL_GATE_BLEND_DISTANCE_M" in complete_source
    assert (
        'shortcut_spatial_gate_minimum_distance_m:='
        '"$SHORTCUT_SPATIAL_GATE_MINIMUM_DISTANCE_M"'
        in source
    )
    assert (
        'shortcut_spatial_gate_blend_distance_m:='
        '"$SHORTCUT_SPATIAL_GATE_BLEND_DISTANCE_M"'
        in source
    )


def test_shortcut_tuned_runtime_defaults() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert (
        'SHORTCUT_MAXIMUM_ENTRY_STEERING_SEC="'
        '${SHORTCUT_MAXIMUM_ENTRY_STEERING_SEC:-1.3}"'
        in source
    )
    assert (
        'SHORTCUT_ENTRY_SPEED_COMMAND="${SHORTCUT_ENTRY_SPEED_COMMAND:-11.0}"'
        in source
    )
    assert '"shortcut_maximum_entry_steering_sec", default_value="1.3"' in launch_source
    assert '"shortcut_entry_speed_command", default_value="11.0"' in launch_source


def test_integrated_run_defaults_to_18_straight_16_curve_and_12_full_lock() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    complete_source = COMPLETE_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    for source in (run_source, complete_source):
        assert "speed < 16.0 ? speed : 16.0" in source
        assert "curve < 12.0 ? curve : 12.0" in source
        assert "12.0 0.0 30.0" in source
    assert '"speed_command", default_value="18.0"' in launch_source
    assert '"curve_speed_command", default_value="16.0"' in launch_source
    assert '"degraded_path_speed_command", default_value="12.0"' in launch_source
    assert '"adaptive_curve_lookahead_m", default_value="0.30"' in launch_source
