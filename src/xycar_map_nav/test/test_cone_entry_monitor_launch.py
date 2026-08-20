from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "cone_entry_monitor.launch.py"
RUN_SCRIPT = PACKAGE_ROOT / "scripts" / "run_cone_entry_monitor.sh"


def test_monitor_launch_structurally_disconnects_motor_output() -> None:
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert "real_hybrid_test_sensors.launch.py" not in source
    assert "xycar_vesc_driver" not in source
    assert '"wide_camera", "wide_camera.launch.py"' in source
    assert '"xycar_lidar", "xycar_lidar.launch.py"' in source
    assert '"drive_enabled": "false"' in source
    assert '"cone_speed_command": "10.0"' in source
    assert '"traffic_light_control_enabled": "false"' in source
    assert '"start_shortcut": "false"' in source
    assert '"vehicle_avoidance_enabled": "false"' in source


def test_one_command_script_rejects_existing_motor_stack() -> None:
    source = RUN_SCRIPT.read_text(encoding="utf-8")

    assert "/xycar_vesc_driver" in source
    assert "/space_drive_gate" in source
    assert "cone_entry_monitor.launch.py" in source
    assert 'ROS_DOMAIN_ID:-7' in source
