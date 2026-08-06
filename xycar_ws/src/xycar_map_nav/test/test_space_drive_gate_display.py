from xycar_map_nav.space_drive_gate import active_drive_mode
from xycar_map_nav.space_drive_gate import estimate_message_rate_hz
from xycar_map_nav.space_drive_gate import format_avoidance_basis


def test_normal_model_and_rule_modes_are_exclusive():
    assert active_drive_mode("RL", "RL/IMITATION") == "MODEL"
    assert active_drive_mode("RULE", "RULE") == "RULE"


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
    assert "양쪽 공간 부족" in text


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
