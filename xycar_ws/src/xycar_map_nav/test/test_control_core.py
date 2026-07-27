import math

from xycar_map_nav.control_core import (
    alignment_limited_speed_command,
    DynamicAvoidanceConfig,
    DynamicVehicleRule,
    filtered_steering_command,
    nearest_path_index,
    pure_pursuit_command,
    rate_limited_speed_command,
    stanley_path_command,
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


def test_stanley_steers_back_before_crossing_straight_path():
    path = [(index * 0.1, 0.0) for index in range(80)]
    command = stanley_path_command(
        path,
        10,
        vehicle_x=1.0,
        vehicle_y=0.12,
        vehicle_yaw=0.0,
        speed_mps=0.6,
        lateral_offset_m=0.0,
        closed=False,
        wheelbase_m=0.32,
        front_axle_offset_m=0.16,
        steering_delay_sec=0.10,
        stanley_gain=0.85,
        stanley_softening_mps=0.45,
        heading_gain=1.0,
        curvature_feedforward_gain=0.9,
        heading_window_m=0.4,
        heading_preview_m=0.0,
        curvature_window_m=0.55,
        curvature_preview_m=0.28,
        maximum_steering_angle_rad=0.62,
    )
    assert command.cross_track_error_m > 0.0
    assert command.curvature_per_m < 0.0


def test_stanley_uses_heading_and_curve_feedforward():
    radius = 2.0
    path = [
        (
            radius * math.sin(index * 0.02),
            radius * (1.0 - math.cos(index * 0.02)),
        )
        for index in range(80)
    ]
    command = stanley_path_command(
        path,
        10,
        vehicle_x=path[10][0],
        vehicle_y=path[10][1],
        vehicle_yaw=0.20,
        speed_mps=0.8,
        lateral_offset_m=0.0,
        closed=False,
        wheelbase_m=0.32,
        front_axle_offset_m=0.0,
        steering_delay_sec=0.0,
        stanley_gain=0.85,
        stanley_softening_mps=0.45,
        heading_gain=1.0,
        curvature_feedforward_gain=1.0,
        heading_window_m=0.4,
        heading_preview_m=0.0,
        curvature_window_m=0.55,
        curvature_preview_m=0.28,
        maximum_steering_angle_rad=0.62,
    )
    assert command.path_curvature_per_m > 0.35
    assert command.curvature_per_m > 0.0


def test_stanley_cross_track_correction_softens_with_speed():
    path = [(index * 0.1, 0.0) for index in range(80)]
    common = dict(
        points=path,
        nearest_index=10,
        vehicle_x=1.0,
        vehicle_y=0.10,
        vehicle_yaw=0.0,
        lateral_offset_m=0.0,
        closed=False,
        wheelbase_m=0.32,
        front_axle_offset_m=0.16,
        steering_delay_sec=0.10,
        stanley_gain=0.85,
        stanley_softening_mps=0.45,
        heading_gain=1.0,
        curvature_feedforward_gain=0.9,
        heading_window_m=0.4,
        heading_preview_m=0.0,
        curvature_window_m=0.55,
        curvature_preview_m=0.28,
        maximum_steering_angle_rad=0.62,
    )
    slow = stanley_path_command(speed_mps=0.2, **common)
    fast = stanley_path_command(speed_mps=1.2, **common)
    assert abs(fast.steering_angle_rad) < abs(slow.steering_angle_rad)


def test_steering_filter_limits_large_straight_reversal():
    filtered = filtered_steering_command(
        -30.0,
        30.0,
        dt_sec=0.05,
        rate_limit_command_per_sec=150.0,
        time_constant_sec=0.10,
    )
    assert math.isclose(filtered, 22.5)
    curve_filtered = filtered_steering_command(
        -30.0,
        30.0,
        dt_sec=0.05,
        rate_limit_command_per_sec=300.0,
        time_constant_sec=0.04,
    )
    assert curve_filtered < filtered


def test_alignment_speed_waits_for_curve_exit_to_settle():
    aligned = alignment_limited_speed_command(
        10.0,
        3.0,
        cross_track_error_m=0.02,
        heading_error_rad=0.04,
        cross_track_soft_m=0.05,
        cross_track_hard_m=0.20,
        heading_soft_rad=0.08,
        heading_hard_rad=0.35,
    )
    unsettled = alignment_limited_speed_command(
        10.0,
        3.0,
        cross_track_error_m=0.02,
        heading_error_rad=0.25,
        cross_track_soft_m=0.05,
        cross_track_hard_m=0.20,
        heading_soft_rad=0.08,
        heading_hard_rad=0.35,
    )
    assert math.isclose(aligned, 10.0)
    assert 3.0 < unsettled < aligned


def test_speed_rate_limit_accelerates_slowly_and_brakes_quickly():
    accelerating = rate_limited_speed_command(
        10.0,
        3.0,
        dt_sec=0.05,
        acceleration_rate_command_per_sec=5.0,
        deceleration_rate_command_per_sec=30.0,
    )
    braking = rate_limited_speed_command(
        3.0,
        10.0,
        dt_sec=0.05,
        acceleration_rate_command_per_sec=5.0,
        deceleration_rate_command_per_sec=30.0,
    )
    assert math.isclose(accelerating, 3.25)
    assert math.isclose(braking, 8.5)


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
