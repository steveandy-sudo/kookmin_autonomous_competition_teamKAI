import pytest
from launch import LaunchContext

from xycar_map_nav.real_odom_launch import make_real_odom_nodes


def _context(**overrides):
    values = {
        "odom_source": "vesc_imu",
        "use_command_odom": "false",
        "use_sim_time": "false",
        "start_native_vesc_driver": "true",
        "vesc_driver_config": "driver.yaml",
        "vesc_port": "/dev/ttyMOTOR",
        "vesc_drive_enabled": "false",
        "vesc_imu_odom_params_file": "vesc.yaml",
        "vesc_topic": "/vehicle/vesc_state",
        "imu_topic": "/imu",
        "command_odom_params_file": "command.yaml",
        "use_imu_yaw": "true",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def _executables(nodes):
    return [node.node_executable for node in nodes]


def test_measured_odom_starts_native_driver_and_one_fusion_node():
    nodes = make_real_odom_nodes(_context())
    assert _executables(nodes) == [
        "xycar_vesc_driver",
        "vesc_imu_odom_node",
    ]


def test_measured_odom_can_use_an_existing_native_driver():
    nodes = make_real_odom_nodes(
        _context(start_native_vesc_driver="false")
    )
    assert _executables(nodes) == ["vesc_imu_odom_node"]


def test_command_fallback_uses_native_motor_and_command_odom():
    nodes = make_real_odom_nodes(_context(odom_source="command"))
    assert _executables(nodes) == [
        "xycar_vesc_driver",
        "command_odom_node",
    ]


def test_legacy_command_switch_overrides_measured_default():
    nodes = make_real_odom_nodes(_context(use_command_odom="true"))
    assert _executables(nodes) == [
        "xycar_vesc_driver",
        "command_odom_node",
    ]


def test_external_mode_starts_only_native_motor_driver():
    nodes = make_real_odom_nodes(_context(odom_source="external"))
    assert _executables(nodes) == ["xycar_vesc_driver"]


def test_invalid_odom_source_is_rejected():
    with pytest.raises(RuntimeError, match="odom_source"):
        make_real_odom_nodes(_context(odom_source="both"))
