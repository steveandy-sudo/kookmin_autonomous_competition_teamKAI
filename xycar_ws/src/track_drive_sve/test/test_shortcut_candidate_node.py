from track_drive_sve.shortcut_candidate_node import limit_shortcut_angle
from track_drive_sve.shortcut_core import ShortcutCore


def test_shortcut_command_uses_integrated_steering_envelope():
    assert limit_shortcut_angle(-100.0, 42.0) == -42.0
    assert limit_shortcut_angle(100.0, 42.0) == 42.0
    assert limit_shortcut_angle(12.5, 42.0) == 12.5


def test_semantic_entry_handoff_skips_the_timed_enter_phase():
    core = ShortcutCore()

    core.start_cruise(12.5)

    assert core.phase == "cruise"
    assert core.phase_start_sec == 12.5
