from track_drive_sve.shortcut_candidate_node import limit_shortcut_angle


def test_shortcut_command_uses_integrated_steering_envelope():
    assert limit_shortcut_angle(-100.0, 42.0) == -42.0
    assert limit_shortcut_angle(100.0, 42.0) == 42.0
    assert limit_shortcut_angle(12.5, 42.0) == 12.5
