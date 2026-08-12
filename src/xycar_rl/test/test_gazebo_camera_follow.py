from xycar_rl.gazebo_camera_follow import follow_command
from xycar_rl.gazebo_camera_follow import offset_command


def test_follow_command_targets_xycar_model():
    command = follow_command("xycar_ackermann")
    assert "/gui/follow" in command
    assert 'data: "xycar_ackermann"' in command


def test_offset_command_uses_chase_camera_position():
    command = offset_command(-2.5, 0.0, 1.6)
    assert "/gui/follow/offset" in command
    assert "x: -2.500000 y: 0.000000 z: 1.600000" in command
