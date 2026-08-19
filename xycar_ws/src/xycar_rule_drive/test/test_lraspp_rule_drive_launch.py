from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "lraspp_rule_drive_real.launch.py"


def test_lraspp_rule_launch_connects_new_canonical_to_rule_driver():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "lane_seg_lraspp_low_latency_real.launch.py" in source
    assert '"canonical_topic": "/perception/canonical_road_image"' in source
    assert 'default_value="false"' in source
    assert '"command_on_canonical": True' in source
    assert '"sitl_bypass_path_enabled": False' in source
    assert 'executable="rule_command_adapter"' in source
    assert '"shadow_motor_topic": "/rule_drive/base_motor_shadow"' in source
    assert '"drive_enabled": False' in source


def test_lraspp_rule_launch_keeps_real_car_tuning_adjustable():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    for argument in (
        "speed_command",
        "straight_speed_command",
        "turn_speed_command",
        "slowdown_start_angle_command",
        "full_slowdown_angle_command",
        "target_left_offset_m",
        "lookahead_distance_m",
        "pure_pursuit_weight",
        "pure_pursuit_control_x_m",
        "stanley_control_x_m",
        "stanley_gain",
        "stanley_softening_mps",
        "straight_pure_pursuit_weight",
        "straight_stanley_gain",
        "straight_stanley_softening_mps",
        "opposed_stanley_weight",
        "control_latency_preview_sec",
        "curve_control_latency_preview_sec",
        "curve_control_latency_minimum_hold_sec",
        "angle_command_min",
        "angle_command_max",
        "perception_rate_hz",
        "obstacle_shift_m",
        "obstacle_release_delay_sec",
    ):
        assert re.search(
            rf'DeclareLaunchArgument\(\s*"{argument}"',
            source,
        )
