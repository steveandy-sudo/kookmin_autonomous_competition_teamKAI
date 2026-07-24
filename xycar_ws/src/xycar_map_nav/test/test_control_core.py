import math

from xycar_map_nav.control_core import (
    DynamicAvoidanceConfig,
    DynamicVehicleRule,
    nearest_path_index,
    pure_pursuit_command,
    steering_command_for_curvature,
)


def test_pure_pursuit_and_calibrated_steering_sign():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.5), (3.0, 1.0)]
    command = pure_pursuit_command(
        path,
        0,
        vehicle_x=0.0,
        vehicle_y=0.0,
        vehicle_yaw=0.0,
        lookahead_m=1.5,
        lateral_offset_m=0.0,
        closed=False,
    )
    assert command.curvature_per_m > 0.0
    steering = steering_command_for_curvature(
        command.curvature_per_m,
        [-42.0, 0.0, 42.0],
        [1.5, 0.0, -1.9],
    )
    assert steering < 0.0


def test_nearest_index_does_not_jump_to_distant_route_section():
    points = [(float(index), 0.0) for index in range(100)]
    index = nearest_path_index(
        points,
        90.0,
        0.0,
        previous_index=10,
        closed=False,
        search_back=2,
        search_ahead=5,
    )
    assert index == 15


def test_dynamic_vehicle_rule_sequence_and_meter_offsets():
    rule = DynamicVehicleRule(
        DynamicAvoidanceConfig(offset_rate_mps=1.0, clear_reset_sec=0.2)
    )
    state = rule.update(
        now_sec=0.0, front_count=1, behind_count=0, armed=True
    )
    assert state.mode == "AVOID_RIGHT"
    state = rule.update(
        now_sec=0.2, front_count=1, behind_count=0, armed=True
    )
    assert state.lateral_offset_m < 0.0
    state = rule.update(
        now_sec=0.4, front_count=0, behind_count=1, armed=True
    )
    assert state.mode == "PASS_LEFT"
    state = rule.update(
        now_sec=0.6, front_count=0, behind_count=2, armed=True
    )
    assert state.mode == "RETURN_CENTER"
    for step in range(7, 15):
        state = rule.update(
            now_sec=step * 0.1,
            front_count=0,
            behind_count=0,
            armed=True,
        )
    assert state.mode == "NORMAL"
    assert math.isclose(state.lateral_offset_m, 0.0, abs_tol=0.02)


def test_dynamic_rule_disarms_outside_configured_segment():
    rule = DynamicVehicleRule(DynamicAvoidanceConfig(offset_rate_mps=1.0))
    rule.update(now_sec=0.0, front_count=1, behind_count=0, armed=True)
    state = rule.update(
        now_sec=0.2, front_count=1, behind_count=0, armed=False
    )
    assert state.mode == "NORMAL"
