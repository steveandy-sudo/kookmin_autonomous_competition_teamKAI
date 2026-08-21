from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
RUN_SCRIPT = PACKAGE_ROOT / "scripts" / "run_space_hybrid_test.sh"
COMPLETE_SCRIPT = PACKAGE_ROOT / "scripts" / "run_complete_space_hybrid.sh"
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "real_sequential_hybrid_drive.launch.py"
HYBRID_CONFIG = PACKAGE_ROOT / "config" / "sequential_hybrid_real.yaml"
OBJECT_CONFIG = (
    REPOSITORY_ROOT / "src" / "study" / "my_rule" / "config"
    / "object_detection.yaml"
)


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


def test_integrated_launch_scopes_runtime_overrides_to_exact_nodes() -> None:
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    rule_start = source.index('package="xycar_rule_drive"')
    selector_start = source.index('package="xycar_map_nav"')
    assert 'namespace="/"' in source[rule_start : rule_start + 500]
    assert 'namespace="/"' in source[selector_start : selector_start + 500]


def test_integrated_run_verifies_single_output_and_lidar_contract() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")

    assert "verify_runtime_control_contract" in source
    assert "/canonical_stanley_pursuit_driver drive_enabled False" in source
    assert "/sequential_hybrid_driver gate_arming_required True" in source
    assert "/hybrid_gate/xycar_motor_shadow" in source
    assert "wait_for_control_message" in source
    assert "  /scan \\" in source


def test_integrated_run_uses_validated_eight_as_cone_speed() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'CONE_SPEED_COMMAND="8.0"' in source
    assert '"cone_speed_command", default_value="8.0"' in launch_source


def test_integrated_cone_precompute_uses_weak_detection_without_weakening_entry():
    source = RUN_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "CONE_APPROACH_YOLO_MIN_CONFIDENCE:-0.40" in source
    assert '"cone_approach_yolo_min_confidence", default_value="0.40"' in launch_source
    assert '"cone_yolo_min_confidence": ParameterValue(' in launch_source


def test_integrated_run_accelerates_only_rule_to_cone_steering_handoff() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")

    assert (
        "RULE_TO_CONE_STEERING_RATE_COMMAND_PER_SEC:-180.0" in source
    )
    assert (
        "-p rule_to_cone_steering_rate_command_per_sec:=" in source
    )


def test_integrated_run_exposes_temporary_traffic_light_disable() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "XYCAR_TRAFFIC_LIGHT_CONTROL_ENABLED:-true" in source
    assert 'traffic_light_control_enabled:="$TRAFFIC_LIGHT_CONTROL_ENABLED"' in source
    assert '"traffic_light_control_enabled", default_value="true"' in launch_source
    assert 'LaunchConfiguration(\n                                "traffic_light_control_enabled"' in launch_source


def test_integrated_run_defaults_to_first_finish_25_11_11_profile() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    complete_source = COMPLETE_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    for source in (run_source, complete_source):
        assert 'SPEED_COMMAND="${1:-${SPEED_COMMAND:-25.0}}"' in source
        assert 'CURVATURE_SPEED_CONTROL_ENABLED="${CURVATURE_SPEED_CONTROL_ENABLED:-true}"' in source
        assert 'CURVE_SPEED_COMMAND="${CURVE_SPEED_COMMAND:-}"' in source
        assert 'DEGRADED_PATH_SPEED_COMMAND="${DEGRADED_PATH_SPEED_COMMAND:-}"' in source
        assert 'STRAIGHT_PATH_CURVATURE_THRESHOLD="${STRAIGHT_PATH_CURVATURE_THRESHOLD:-0.24}"' in source
        assert 'STEERING_CURRENT_WEIGHT="${STEERING_CURRENT_WEIGHT:-0.35}"' in source
        assert 'STEERING_CURVE_CURRENT_WEIGHT="${STEERING_CURVE_CURRENT_WEIGHT:-0.80}"' in source
        assert "speed < 11.0 ? speed : 11.0" in source
        assert "curve < 15.0 ? curve : 15.0" in source
        assert "12.0 0.0 30.0" in source
    assert '"speed_command", default_value="25.0"' in launch_source
    assert '"curve_speed_command", default_value="11.0"' in launch_source
    assert '"degraded_path_speed_command", default_value="11.0"' in launch_source
    assert '"adaptive_curve_lookahead_m", default_value="0.30"' in launch_source
    assert '"straight_path_curvature_threshold", default_value="0.24"' in launch_source
    assert '"steering_current_weight", default_value="0.35"' in launch_source
    assert '"steering_curve_current_weight", default_value="0.80"' in launch_source


def test_low_speed_profile_derives_curve_defaults_from_requested_cap() -> None:
    for script in (RUN_SCRIPT, COMPLETE_SCRIPT):
        source = script.read_text(encoding="utf-8")

        assert "(speed < 11.0 ? speed : 11.0)" in source
        assert "(curve < 15.0 ? curve : 15.0)" in source
        assert source.index('SPEED_COMMAND="$(awk') < source.index(
            "curve_default=\"$(awk"
        )


def test_both_real_car_scripts_expose_the_two_shortcut_strategies() -> None:
    for script in (RUN_SCRIPT, COMPLETE_SCRIPT):
        source = script.read_text(encoding="utf-8")

        assert "--shortcut-mode" in source
        assert "w1|yellow_count" in source
        assert "--shortcut-yellow-count" in source
        assert "--shortcut-angle" in source
        assert "--shortcut-return-sec" in source


def test_integrated_launch_starts_only_the_selected_shortcut_strategy() -> None:
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("shortcut_strategy", default_value="w1")' in source
    assert '"\' == \'w1\'"' in source
    assert '"\' == \'yellow_count\'"' in source
    assert "yellow_count_shortcut_control.launch.py" in source


def test_integrated_yellow_count_trigger_is_selectable_with_safe_default() -> None:
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"shortcut_yellow_count_pass_target", default_value="2"' in source
    assert '"yellow_pass_target": LaunchConfiguration(' in source
    assert '"shortcut_yellow_count_pass_target"' in source
    assert '"yellow_visible_frames": "2"' in source
    assert '"yellow_absent_frames": "1"' in source


def test_integrated_vehicle_classes_use_avoidance_speed_cap() -> None:
    object_config = OBJECT_CONFIG.read_text(encoding="utf-8")
    hybrid_config = HYBRID_CONFIG.read_text(encoding="utf-8")

    assert (
        "class_aliases: [red_car=red_car, green_car=green_car]"
        in object_config
    )
    assert "required_classes: [red_car, green_car," in object_config
    assert (
        "vehicle_class_names: [obstacle_vehicle, red_car, green_car]"
        in hybrid_config
    )
    assert (
        "vehicle_red_car_avoidance_speed_limit_command: 20.0"
        in hybrid_config
    )
    assert (
        "vehicle_green_car_avoidance_speed_limit_command: 20.0"
        in hybrid_config
    )
