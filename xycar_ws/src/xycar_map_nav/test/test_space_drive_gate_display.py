from xycar_map_nav.space_drive_gate import active_drive_mode
from xycar_map_nav.space_drive_gate import estimate_message_rate_hz


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
