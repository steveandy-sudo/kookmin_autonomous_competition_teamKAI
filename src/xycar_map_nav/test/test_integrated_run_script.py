from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
RUN_SCRIPT = PACKAGE_ROOT / "scripts" / "run_space_hybrid_test.sh"
COMPLETE_SCRIPT = PACKAGE_ROOT / "scripts" / "run_complete_space_hybrid.sh"
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "real_sequential_hybrid_drive.launch.py"
HYBRID_CONFIG = PACKAGE_ROOT / "config" / "sequential_hybrid_real.yaml"
SELECTOR_DRIVER = (
    PACKAGE_ROOT / "xycar_map_nav" / "sequential_hybrid_driver.py"
)
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


def test_complete_run_prompts_for_curve_multiplier_first() -> None:
    source = COMPLETE_SCRIPT.read_text(encoding="utf-8")

    multiplier_prompt = source.index(
        '"S자/곡선 조향 배수 (1.0=증폭 없음)"'
    )
    speed_prompt = source.index('"주행 속도 command')
    assert multiplier_prompt < speed_prompt
    assert source.count('prompt_float CURVE_STEERING_MULTIPLIER \\\n') == 1
    assert (
        '"S자/곡선 조향 배수 (1.0=증폭 없음)" 1.0 0.0 3.0'
        in source
    )


def test_integrated_run_exposes_independent_overall_speed_limit() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    complete_source = COMPLETE_SCRIPT.read_text(encoding="utf-8")

    for source in (run_source, complete_source):
        assert (
            'OVERALL_SPEED_LIMIT_COMMAND="${OVERALL_SPEED_LIMIT_COMMAND:-20.0}"'
            in source
        )
    assert (
        "overall_speed_limit_command: $OVERALL_SPEED_LIMIT_COMMAND"
        in run_source
    )
    assert '-p speed_command:="$OVERALL_SPEED_LIMIT_COMMAND"' in run_source


def test_integrated_run_prompts_for_vehicle_avoidance_offsets() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    complete_source = COMPLETE_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    config_source = HYBRID_CONFIG.read_text(encoding="utf-8")

    assert 'VEHICLE_LEFT_OFFSET_M="${VEHICLE_LEFT_OFFSET_M:-0.13}"' in run_source
    assert 'VEHICLE_RIGHT_OFFSET_M="${VEHICLE_RIGHT_OFFSET_M:-0.13}"' in run_source
    assert 'prompt_float VEHICLE_LEFT_OFFSET_M \\\n' in complete_source
    assert 'prompt_float VEHICLE_RIGHT_OFFSET_M \\\n' in complete_source
    assert '"오른쪽 장애물 감지 시 왼쪽 회피 이동량 [m]" 0.13' in complete_source
    assert '"왼쪽 장애물 감지 시 오른쪽 회피 이동량 [m]" 0.13' in complete_source
    assert 'export VEHICLE_LEFT_OFFSET_M VEHICLE_RIGHT_OFFSET_M' in complete_source
    assert '"vehicle_left_offset_m", default_value="0.13"' in launch_source
    assert '"vehicle_right_offset_m", default_value="0.13"' in launch_source
    assert "vehicle_left_offset_m: 0.13" in config_source
    assert "vehicle_right_offset_m: 0.13" in config_source


def test_integrated_run_waits_for_measured_lane_center_on_avoidance_return() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    config_source = HYBRID_CONFIG.read_text(encoding="utf-8")

    assert 'VEHICLE_RETURN_CROSS_TRACK_ERROR_M:-0.08' in run_source
    assert 'VEHICLE_RETURN_REQUIRED_FRAMES:-3' in run_source
    assert 'VEHICLE_RETURN_STRAIGHT_PP_WEIGHT:-0.45' in run_source
    assert 'vehicle_return_cross_track_error_m:="$VEHICLE_RETURN_CROSS_TRACK_ERROR_M"' in run_source
    assert 'vehicle_return_required_frames:="$VEHICLE_RETURN_REQUIRED_FRAMES"' in run_source
    assert 'vehicle_return_straight_pure_pursuit_weight:="$VEHICLE_RETURN_STRAIGHT_PP_WEIGHT"' in run_source
    assert '"vehicle_return_cross_track_error_m", default_value="0.08"' in launch_source
    assert '"vehicle_return_required_frames", default_value="3"' in launch_source
    assert '"vehicle_return_straight_pure_pursuit_weight"' in launch_source
    assert "vehicle_return_cross_track_error_m: 0.08" in config_source
    assert "vehicle_return_required_frames: 3" in config_source


def test_integrated_run_exposes_shortcut_left_lane_preposition_offset() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    config_source = HYBRID_CONFIG.read_text(encoding="utf-8")

    assert (
        'SHORTCUT_LEFT_LANE_OFFSET_M="${SHORTCUT_LEFT_LANE_OFFSET_M:-0.05}"'
        in run_source
    )
    assert (
        'shortcut_left_lane_offset_m:="$SHORTCUT_LEFT_LANE_OFFSET_M"'
        in run_source
    )
    assert (
        '"shortcut_left_lane_offset_m", default_value="0.05"'
        in launch_source
    )
    assert "shortcut_left_lane_offset_m: 0.05" in config_source


def test_integrated_curve_response_defaults_match_real_profile() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    complete_source = COMPLETE_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'CURVE_STEERING_MULTIPLIER:-1.0' in run_source
    assert '"Curve control latency preview [s]" 0.35' in run_source
    assert '"곡선 제어 지연 예측 시간 [s]" 0.35' in complete_source
    assert '"curve_steering_multiplier", default_value="1.0"' in launch_source
    assert (
        '"curve_control_latency_preview_sec", default_value="0.35"'
        in launch_source
    )
    assert (
        '"yellow_curve_reversal_preview_far_x_m",\n'
        '                default_value="1.00"'
        in launch_source
    )


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


def test_integrated_run_supports_direct_xbin_rule_path() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert (
        'RULE_PERCEPTION_BACKEND="${XYCAR_RULE_PERCEPTION_BACKEND:-direct_xbin}"'
        in run_source
    )
    assert "direct_xbin)" in run_source
    assert "RULE_READY_TOPIC=/perception/xbin_direct_centerline" in run_source
    assert "LANE_DIRECT_CANONICAL_ENABLED=false" in run_source
    assert "LANE_START_CANONICAL_ADAPTER=false" in run_source
    assert "RULE_EXTERNAL_PATH_ENABLED=true" in run_source
    assert (
        'rule_external_path_topic:=/perception/xbin_direct_centerline'
        in run_source
    )
    assert (
        '"lane_direct_centerline_enabled", default_value="false"'
        in launch_source
    )
    assert (
        '"rule_external_path_enabled", default_value="false"'
        in launch_source
    )
    assert '"external_path_enabled": ParameterValue(' in launch_source


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


def test_integrated_cone_exit_rejects_stale_cached_commands() -> None:
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    config_source = HYBRID_CONFIG.read_text(encoding="utf-8")
    selector_source = SELECTOR_DRIVER.read_text(encoding="utf-8")
    branch_start = selector_source.rindex("elif self.cone_bypass.active:")
    branch_end = selector_source.index(
        "elif (\n            self.avoidance_state.controls_vehicle",
        branch_start,
    )
    cone_branch = selector_source[branch_start:branch_end]

    assert '"cone_exit_absence_sec", default_value="0.25"' in launch_source
    assert '"cone_sensor_presence_timeout_sec", default_value="0.25"' in launch_source
    assert "cone_exit_absence_sec: 0.25" in config_source
    assert "cone_sensor_presence_timeout_sec: 0.25" in config_source
    assert "and cone_command_fresh" in cone_branch
    assert "last_valid_cone_command" in cone_branch
    assert "command_timestamp_is_fresh" in cone_branch
    assert 'reason="cone LiDAR command invalid or stale"' in cone_branch
    assert "self.last_valid_cone_command = (0.0, 0.0)" in selector_source
    assert (
        'self.last_valid_cone_command_time = float("-inf")'
        in selector_source
    )


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
    assert '"shortcut_entry_speed_command": ParameterValue(' in launch_source
    assert '"shortcut_yolo_absence_frames": 1' in launch_source
    assert "shortcut_yolo_absence_frames: 1" in HYBRID_CONFIG.read_text(
        encoding="utf-8"
    )
    assert 'VEHICLE_YOLO_TIMEOUT_SEC:-0.50' in source
    assert '"vehicle_yolo_timeout_sec", default_value="0.50"' in launch_source


def test_integrated_run_exposes_temporary_shortcut_disable() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")

    assert 'START_SHORTCUT="${XYCAR_START_SHORTCUT:-true}"' in source
    assert 'start_shortcut:="$START_SHORTCUT"' in source
    assert 'echo "Shortcut control: $START_SHORTCUT"' in source


def test_integrated_drive_uses_yolo_only_for_traffic_signal_classes() -> None:
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")
    object_config = OBJECT_CONFIG.read_text(encoding="utf-8")

    assert '"startup_signal_hsv_enabled": False' in launch_source
    assert '"traffic_signal_cv_enabled": False' in launch_source
    assert "traffic_signal_cv_enabled: false" in object_config


def test_integrated_run_defaults_to_20_12_11_profile() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    complete_source = COMPLETE_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    for source in (run_source, complete_source):
        assert 'SPEED_COMMAND="${1:-${SPEED_COMMAND:-20.0}}"' in source
        assert 'CURVATURE_SPEED_CONTROL_ENABLED="${CURVATURE_SPEED_CONTROL_ENABLED:-true}"' in source
        assert 'CURVE_SPEED_COMMAND="${CURVE_SPEED_COMMAND:-}"' in source
        assert 'DEGRADED_PATH_SPEED_COMMAND="${DEGRADED_PATH_SPEED_COMMAND:-}"' in source
        assert 'STRAIGHT_PATH_CURVATURE_THRESHOLD="${STRAIGHT_PATH_CURVATURE_THRESHOLD:-0.24}"' in source
        assert 'STEERING_CURRENT_WEIGHT="${STEERING_CURRENT_WEIGHT:-0.35}"' in source
        assert 'STEERING_CURVE_CURRENT_WEIGHT="${STEERING_CURVE_CURRENT_WEIGHT:-0.80}"' in source
        assert 'YELLOW_CURVE_REVERSAL_PREVIEW_FAR_X_M="${YELLOW_CURVE_REVERSAL_PREVIEW_FAR_X_M:-1.00}"' in source
        assert "speed < 12.0 ? speed : 12.0" in source
        assert "curve < 11.0 ? curve : 11.0" in source
        assert "12.0 0.0 30.0" in source
    assert '"speed_command", default_value="20.0"' in launch_source
    assert '"curve_speed_command", default_value="12.0"' in launch_source
    assert '"degraded_path_speed_command", default_value="11.0"' in launch_source
    assert 'DEGRADED_PATH_MINIMUM_SPAN_M="${DEGRADED_PATH_MINIMUM_SPAN_M:-0.40}"' in run_source
    assert 'DEGRADED_PATH_RELEASE_SPAN_M="${DEGRADED_PATH_RELEASE_SPAN_M:-0.60}"' in run_source
    assert 'DEGRADED_PATH_CONFIRMATION_FRAMES="${DEGRADED_PATH_CONFIRMATION_FRAMES:-2}"' in run_source
    assert 'DEGRADED_PATH_RELEASE_FRAMES="${DEGRADED_PATH_RELEASE_FRAMES:-2}"' in run_source
    assert '"degraded_path_minimum_span_m", default_value="0.40"' in launch_source
    assert '"degraded_path_release_span_m", default_value="0.60"' in launch_source
    assert '"degraded_path_confirmation_frames", default_value="2"' in launch_source
    assert '"degraded_path_release_frames", default_value="2"' in launch_source
    assert '"adaptive_curve_lookahead_m", default_value="0.30"' in launch_source
    assert '"straight_path_curvature_threshold", default_value="0.24"' in launch_source
    assert '"steering_current_weight", default_value="0.35"' in launch_source
    assert '"steering_curve_current_weight", default_value="0.80"' in launch_source
    assert '"stanley_gain", default_value="1.30"' in launch_source
    assert '"Curve Stanley cross-track gain" 1.30' in run_source
    assert '"곡선 Stanley 횡오차 gain" 1.30' in complete_source


def test_integrated_run_enables_only_post_mission_s_entry_guard() -> None:
    run_source = RUN_SCRIPT.read_text(encoding="utf-8")
    launch_source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'S_CURVE_ENTRY_GUARD_ENABLED="${S_CURVE_ENTRY_GUARD_ENABLED:-true}"' in run_source
    assert 'S_CURVE_ENTRY_SPEED_CAP_COMMAND="${S_CURVE_ENTRY_SPEED_CAP_COMMAND:-11.0}"' in run_source
    assert (
        'S_CURVE_ENTRY_RED_CAR_SPEED_CAP_COMMAND="'
        '${S_CURVE_ENTRY_RED_CAR_SPEED_CAP_COMMAND:-13.0}"'
        in run_source
    )
    assert 's_curve_entry_guard_enabled:="$S_CURVE_ENTRY_GUARD_ENABLED"' in run_source
    assert 's_curve_entry_speed_cap_command:="$S_CURVE_ENTRY_SPEED_CAP_COMMAND"' in run_source
    assert (
        's_curve_entry_red_car_speed_cap_command:="'
        '$S_CURVE_ENTRY_RED_CAR_SPEED_CAP_COMMAND"'
        in run_source
    )
    assert "S_CURVE_ENTRY_MAXIMUM_RIGHT_ANGLE_COMMAND" not in run_source
    assert "s_curve_entry_maximum_right_angle_command" not in launch_source
    assert (
        '"s_curve_entry_guard_enabled", default_value="true"'
        in launch_source
    )
    assert (
        '"s_curve_entry_speed_cap_command", default_value="11.0"'
        in launch_source
    )
    assert (
        '"s_curve_entry_curve_speed_margin_command",\n'
        '                default_value="1.00",'
        in launch_source
    )
    assert (
        '"s_curve_entry_red_car_speed_cap_command",\n'
        '                default_value="13.0",'
        in launch_source
    )
    assert (
        '"s_curve_entry_minimum_curve_distance_m",\n'
        '                default_value="1.50",'
        in launch_source
    )


def test_low_speed_profile_derives_curve_defaults_from_requested_cap() -> None:
    for script in (RUN_SCRIPT, COMPLETE_SCRIPT):
        source = script.read_text(encoding="utf-8")

        assert "(speed < 12.0 ? speed : 12.0)" in source
        assert "(curve < 11.0 ? curve : 11.0)" in source
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
