from xycar_map_nav.space_drive_gate import active_drive_mode
from xycar_map_nav.space_drive_gate import classify_vesc_health
from xycar_map_nav.space_drive_gate import estimate_message_rate_hz
from xycar_map_nav.space_drive_gate import format_avoidance_basis
from xycar_map_nav.space_drive_gate import stop_reason_text


def test_normal_model_and_rule_modes_are_exclusive():
    assert active_drive_mode("RL", "RL/IMITATION") == "MODEL"
    assert active_drive_mode("RULE", "RULE") == "RULE"
    assert active_drive_mode("SHORTCUT", "SHORTCUT_W1_SEARCH") == "SHORTCUT"
    assert active_drive_mode("TRAFFIC_LIGHT", "RED_4") == "TRAFFIC_LIGHT"


def test_cone_and_avoidance_override_base_mode_display():
    assert active_drive_mode("CONE_RULE", "CONE_RULE") == "CONE"
    assert (
        active_drive_mode("YOLO_LIDAR_AVOIDANCE", "AVOID_LEFT")
        == "AVOIDANCE_LEFT"
    )


def test_tracking_is_the_single_active_display_mode():
    assert (
        active_drive_mode("RL", "YOLO_TRACKING(1.25m)")
        == "AVOIDANCE_TRACK"
    )


def test_message_rate_uses_inter_message_intervals():
    assert estimate_message_rate_hz([]) == 0.0
    assert estimate_message_rate_hz([1.0]) == 0.0
    assert estimate_message_rate_hz([1.0, 1.25, 1.5]) == 4.0


def test_avoidance_basis_explains_direction_from_clearance():
    text = format_avoidance_basis(
        "AVOIDANCE_LEFT",
        [3.0, 0.2, 1.1, 0.86, 0.95, 0.42, 1.1, 0.0, 0.2],
        target_label="cone",
        yolo_min_confidence=0.50,
        entry_distance_m=1.20,
        minimum_side_clearance_m=0.70,
    )
    assert "YOLO=0.86>=0.50" in text
    assert "좌측여유=0.95m" in text
    assert "좌회피" in text


def test_avoidance_basis_explains_blocked_state():
    text = format_avoidance_basis(
        "AVOIDANCE_BLOCKED",
        [2.0, 0.0, 0.8, 0.75, 0.4, 0.5, 0.8, 0.0, 0.2],
        target_label="cone",
        yolo_min_confidence=0.50,
        entry_distance_m=1.20,
        minimum_side_clearance_m=0.70,
    )
    assert "노란 중앙선 좌우판단 대기" in text


def test_avoidance_basis_reports_opposite_side_of_yellow_divider_obstacle():
    text = format_avoidance_basis(
        "AVOIDANCE_RIGHT",
        [
            4.0,
            -0.2,
            1.0,
            0.90,
            1.2,
            0.8,
            1.0,
            0.1,
            0.2,
            -1.0,
            -1.0,
            1.0,
        ],
        target_label="cone",
        yolo_min_confidence=0.50,
        entry_distance_m=1.20,
        minimum_side_clearance_m=0.70,
    )
    assert "노란 중앙선 기준" in text
    assert "장애물=왼쪽" in text
    assert "오른쪽으로 회피" in text


def test_vesc_health_reports_low_voltage_derating_and_latch():
    limited = classify_vesc_health(
        diagnostic_message="low-voltage propulsion derating",
        diagnostic_level=1,
        values={
            "guard_state": "low_voltage_limited",
            "voltage_input": "7.200",
            "voltage_output_scale": "0.800",
            "fault_code": "0",
        },
    )
    assert limited[0] == "LOW_VOLTAGE_LIMITED"
    assert "7.20V" in limited[1]
    assert "80%" in limited[1]

    stopped = classify_vesc_health(
        diagnostic_message="motor output fault-latched",
        diagnostic_level=2,
        values={
            "guard_state": "fault_latched",
            "voltage_input": "6.000",
            "voltage_output_scale": "0.000",
            "fault_code": "0",
        },
    )
    assert stopped[0] == "LOW_VOLTAGE_STOP"
    assert "저전압 보호 차단" in stopped[1]


def test_stop_reason_always_explains_common_gate_stops():
    assert "SPACE" in stop_reason_text(
        output_reason="SPACE_STOP",
        selector_state="RUNNING",
        selector_reason="rule command",
        vesc_state="NORMAL",
        vesc_reason="normal",
    )
    assert "후보 명령" in stop_reason_text(
        output_reason="CANDIDATE_STALE",
        selector_state="RUNNING",
        selector_reason="rule command",
        vesc_state="NORMAL",
        vesc_reason="normal",
    )
    assert "cone command stale" in stop_reason_text(
        output_reason="SELECTOR_STOP",
        selector_state="SENSOR_STOP",
        selector_reason="cone command stale",
        vesc_state="NORMAL",
        vesc_reason="normal",
    )
    assert "저전압" in stop_reason_text(
        output_reason="SPACE_RUN",
        selector_state="RUNNING",
        selector_reason="rule command",
        vesc_state="LOW_VOLTAGE_STOP",
        vesc_reason="저전압 보호 차단",
    )
