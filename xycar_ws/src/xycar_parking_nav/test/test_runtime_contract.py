from ast import literal_eval
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def _yaml(relative_path: str):
    with (PACKAGE / relative_path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def test_nav2_ackermann_reverse_contract():
    config = _yaml("config/nav2_parking.yaml")
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    controller = config["controller_server"]["ros__parameters"]["FollowPath"]
    forward_planner = config["planner_server"]["ros__parameters"]["ForwardGrid"]
    forward_controller = config["controller_server"]["ros__parameters"][
        "FollowPathForward"
    ]
    reverse_controller = config["controller_server"]["ros__parameters"][
        "FollowPathReverse"
    ]

    assert planner["motion_model_for_search"] == "REEDS_SHEPP"
    assert forward_planner["motion_model_for_search"] == "DUBIN"
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    endpoint_curvatures = adapter["steering_map_curvatures"]
    worst_side_radius = 1.0 / min(
        abs(endpoint_curvatures[0]), abs(endpoint_curvatures[-1])
    )
    assert planner["minimum_turning_radius"] >= worst_side_radius
    assert controller["plugin"] == "nav2_mppi_controller::MPPIController"
    assert controller["motion_model"] == "Ackermann"
    assert controller["vx_min"] < 0.0 < controller["vx_max"]
    assert forward_controller["vx_min"] == 0.0
    assert forward_controller["vx_max"] > 0.0
    assert reverse_controller["vx_min"] < 0.0
    assert reverse_controller["vx_max"] == 0.0
    assert forward_controller["PathAngleCritic"]["forward_preference"] is True
    assert reverse_controller["enforce_path_inversion"] is False
    assert controller["enforce_path_inversion"] is True
    assert controller["AckermannConstraints"]["min_turning_r"] >= worst_side_radius
    assert controller["CostCritic"]["consider_footprint"] is True
    assert "PreferForwardCritic" not in controller["critics"]
    smoother = config["velocity_smoother"]["ros__parameters"]
    assert smoother["scale_velocities"] is True
    assert adapter["input_topic"] == "/cmd_vel_nav"


def test_amcl_updates_while_stationary_for_pre_drive_gate():
    amcl = _yaml("config/nav2_parking.yaml")["amcl"]["ros__parameters"]
    manager = _yaml("config/mission_manager.yaml")["parking_mission_manager"][
        "ros__parameters"
    ]

    assert amcl["update_min_d"] == 0.0
    assert amcl["update_min_a"] == 0.0
    assert amcl["set_initial_pose"] is False
    assert amcl["always_reset_initial_pose"] is False
    assert manager["amcl_nomotion_update_service"] == "/request_nomotion_update"
    assert 0.0 < manager["nomotion_update_period_sec"] <= 0.25
    assert (
        0.0
        < manager["nomotion_update_pose_silence_sec"]
        < manager["maximum_pose_age_sec"]
    )


def test_amcl_tolerates_moderate_reproduced_map_mismatch():
    amcl = _yaml("config/nav2_parking.yaml")["amcl"]["ros__parameters"]
    manager = _yaml("config/mission_manager.yaml")["parking_mission_manager"][
        "ros__parameters"
    ]

    assert amcl["do_beamskip"] is True
    assert 0.35 <= amcl["beam_skip_distance"] <= 0.50
    assert 0.18 <= amcl["sigma_hit"] <= 0.25
    assert amcl["laser_likelihood_max_dist"] >= 1.25
    assert 0.04 < manager["maximum_xy_variance"] <= 0.09
    assert 0.08 < manager["maximum_yaw_variance"] <= 0.16


def test_mission_manager_runtime_limits_are_in_a_ros_parameter_file():
    manager = _yaml("config/mission_manager.yaml")["parking_mission_manager"][
        "ros__parameters"
    ]
    goal_checker = _yaml("config/nav2_parking.yaml")["controller_server"][
        "ros__parameters"
    ]["parking_goal_checker"]
    transit_goal_checker = _yaml("config/nav2_parking.yaml")["controller_server"][
        "ros__parameters"
    ]["transit_goal_checker"]

    assert manager["required_stable_samples"] >= 5
    assert (
        manager["required_stable_samples"] * manager["nomotion_update_period_sec"]
        < manager["maximum_pose_age_sec"]
    )
    # Covers the 3 s stationary parking holds without permitting an
    # indefinitely stale localization estimate during motion.
    assert 3.0 < manager["maximum_pose_age_sec"] <= 4.0
    assert manager["localization_loss_cancel_sec"] <= 1.0
    assert manager["forward_failure_reverse_fallback_enabled"] is True
    assert manager["forward_no_progress_timeout_sec"] <= 3.0
    assert manager["retry_delay_sec"] <= 0.25
    assert manager["retry_delay_sec"] < manager["nav2_activation_timeout_sec"] <= 10.0
    assert goal_checker["xy_goal_tolerance"] <= manager["goal_position_tolerance_m"]
    assert 0.40 <= transit_goal_checker["xy_goal_tolerance"] <= 0.45
    assert transit_goal_checker["yaw_goal_tolerance"] >= 3.0
    assert 0.08 < goal_checker["xy_goal_tolerance"] <= 0.13
    assert 0.08 < goal_checker["yaw_goal_tolerance"] <= 0.16
    assert transit_goal_checker["xy_goal_tolerance"] < manager[
        "transit_goal_position_tolerance_m"
    ]
    assert transit_goal_checker["yaw_goal_tolerance"] <= manager[
        "transit_goal_yaw_tolerance_rad"
    ]
    assert 0.42 <= manager["transit_pass_radius_m"] <= 0.50
    assert manager["transit_pass_maximum_miss_distance_m"] >= 0.70
    assert manager["transit_pass_lateral_tolerance_m"] >= 0.50
    assert manager["replan_authorization_grace_sec"] >= manager["retry_delay_sec"]
    assert manager["automatic_recovery_enabled"] is True
    assert 0.10 <= manager["automatic_recovery_delay_sec"] <= 0.50


def test_all_parking_launches_load_mission_manager_parameter_file():
    for relative_path in (
        "launch/parking_real.launch.py",
        "launch/parking_navigation.launch.py",
    ):
        source = (PACKAGE / relative_path).read_text(encoding="utf-8")
        manager_node = source.split('executable="mission_manager"', 1)[1].split(
            'executable="cmd_vel_adapter"', 1
        )[0]
        assert 'LaunchConfiguration("manager_params")' in manager_node


def test_mission_has_three_minute_budget_with_fast_transitions():
    manager = _yaml("config/mission_manager.yaml")["parking_mission_manager"][
        "ros__parameters"
    ]
    steps = _yaml("config/parking_mission.yaml")["mission"]["steps"]
    parking_steps = [step for step in steps if step.get("parking_goal", False)]
    transit_steps = [
        step for step in steps
        if not step.get("parking_goal", False) and step["name"] != "START_RETURN"
    ]

    assert manager["mission_time_limit_sec"] == 180.0
    assert manager["mission_time_limit_enforced"] is False
    assert 0.0 < manager["time_warning_remaining_sec"] <= 30.0
    assert manager["time_log_period_sec"] <= 10.0
    assert len(parking_steps) == 2
    assert all(step["hold_sec"] >= 3.0 for step in parking_steps)
    assert all(step["hold_sec"] <= 0.15 for step in transit_steps)
    assert sum(step["hold_sec"] for step in steps) <= 11.2 + 1.0e-9


def test_planning_footprint_is_permissive_but_stop_shield_keeps_physical_body():
    nav2 = _yaml("config/nav2_parking.yaml")
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    global_footprint = literal_eval(
        nav2["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    )
    local_footprint = literal_eval(
        nav2["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    )

    assert global_footprint == local_footprint
    assert min(point[0] for point in global_footprint) > adapter["footprint_minimum_x"]
    assert max(point[0] for point in global_footprint) < adapter["footprint_maximum_x"]
    assert max(abs(point[1]) for point in global_footprint) < adapter["footprint_half_width"]
    global_padding = nav2["global_costmap"]["global_costmap"]["ros__parameters"][
        "footprint_padding"
    ]
    local_padding = nav2["local_costmap"]["local_costmap"]["ros__parameters"][
        "footprint_padding"
    ]
    assert global_padding == local_padding
    assert global_padding == 0.0
    assert adapter["steering_settle_tolerance_command"] <= 3.0
    for costmap_name in ("global_costmap", "local_costmap"):
        params = nav2[costmap_name][costmap_name]["ros__parameters"]
        padded_inscribed_radius = min(
            max(point[0] for point in global_footprint) + global_padding,
            max(abs(point[1]) for point in global_footprint) + global_padding,
        )
        assert params["inflation_layer"]["inflation_radius"] >= padded_inscribed_radius


def test_kookmin_gazebo_real_vehicle_geometry_contract():
    """Freeze the measured geometry used by generate_kookmin_track.py."""

    nav2 = _yaml("config/nav2_parking.yaml")
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    mission = _yaml("config/parking_mission.yaml")["mission"]
    footprint = literal_eval(
        nav2["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
    )

    assert mission["base_from_reference_x_m"] == 0.16
    assert min(point[0] for point in footprint) == -0.28
    assert max(point[0] for point in footprint) == 0.06
    assert max(abs(point[1]) for point in footprint) == 0.10
    assert adapter["footprint_minimum_x"] == -0.46
    assert adapter["footprint_maximum_x"] == 0.11
    assert adapter["footprint_half_width"] == 0.18
    assert adapter["speed_gain_mps_per_command"] == 0.080612


def test_stop_shield_projects_beyond_nav2_braking_distance():
    nav2 = _yaml("config/nav2_parking.yaml")
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    speed = max(
        adapter["maximum_forward_command"],
        adapter["maximum_reverse_command"],
    ) * adapter["speed_gain_mps_per_command"]
    physical_stop = (
        speed * adapter["reaction_time_sec"]
        + speed * speed / (2.0 * adapter["braking_deceleration_mps2"])
    )

    assert adapter["minimum_projection_m"] >= physical_stop
    assert math.isfinite(physical_stop)


def test_runtime_logs_explain_step_and_motor_stop_context():
    manager = _yaml("config/mission_manager.yaml")["parking_mission_manager"][
        "ros__parameters"
    ]
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    manager_source = (PACKAGE / "xycar_parking_nav/mission_manager.py").read_text(
        encoding="utf-8"
    )
    adapter_source = (PACKAGE / "xycar_parking_nav/cmd_vel_adapter.py").read_text(
        encoding="utf-8"
    )

    assert 0.5 <= manager["diagnostic_log_period_sec"] <= 2.0
    assert 0.5 <= adapter["diagnostic_log_period_sec"] <= 2.0
    assert adapter["mission_state_topic"] == "/parking/mission_state"
    assert adapter["vesc_state_topic"] == "/vehicle/vesc_state"
    assert "오차/완료조건" in manager_source
    assert "cmd_vel나이" in adapter_source
    assert "VESC=%s" in adapter_source


def test_mission_visualizes_nominal_route_targets_and_recorded_parking_spaces():
    manager = _yaml("config/mission_manager.yaml")["parking_mission_manager"][
        "ros__parameters"
    ]
    manager_source = (PACKAGE / "xycar_parking_nav/mission_manager.py").read_text(
        encoding="utf-8"
    )

    assert manager["visualization_topic"] == "/parking/mission_visualization"
    assert manager["parking_box_length_m"] >= 0.70
    assert manager["parking_box_width_m"] >= 0.45
    assert "nominal_route" in manager_source
    assert '"A_PARK"' in manager_source
    assert '"B_PARK"' in manager_source
    assert "current_target" in manager_source


def test_real_vehicle_uses_continuous_command_four():
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    controller = _yaml("config/nav2_parking.yaml")["controller_server"][
        "ros__parameters"
    ]["FollowPath"]
    launch_source = (PACKAGE / "launch/parking_real.launch.py").read_text(
        encoding="utf-8"
    )

    assert adapter["minimum_moving_command"] == 4.0
    assert adapter["maximum_forward_command"] == 4.0
    assert adapter["maximum_reverse_command"] == 4.0
    assert 0.30 <= adapter["transient_nav2_zero_hold_sec"] <= 0.45
    assert "minimum_command_pulse_enabled" not in adapter
    assert "pulse_minimum_on_sec" not in adapter
    assert "maximum_pulse_duty_cycle" not in adapter
    minimum_physical_speed = (
        adapter["minimum_moving_command"]
        * adapter["speed_gain_mps_per_command"]
    )
    assert max(abs(controller["vx_min"]), controller["vx_max"]) <= minimum_physical_speed
    assert adapter["reaction_time_sec"] >= 4.0 * adapter["timer_period_sec"]
    assert (
        adapter["transient_nav2_zero_hold_sec"]
        * adapter["minimum_moving_command"]
        * adapter["speed_gain_mps_per_command"]
        < 0.15
    )
    assert '"acceleration_slew_enabled": True' in launch_source
    assert '"deceleration_limit_mps2": 1.5' in launch_source


def test_forward_lidar_stop_triggers_bounded_reverse_and_replan():
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]
    manager = _yaml("config/mission_manager.yaml")["parking_mission_manager"][
        "ros__parameters"
    ]
    adapter_source = (
        PACKAGE / "xycar_parking_nav/cmd_vel_adapter.py"
    ).read_text(encoding="utf-8")
    manager_source = (
        PACKAGE / "xycar_parking_nav/mission_manager.py"
    ).read_text(encoding="utf-8")

    assert adapter["obstacle_reverse_recovery_enabled"] is True
    reverse_distance_upper_bound = (
        adapter["obstacle_reverse_duration_sec"]
        * adapter["minimum_moving_command"]
        * adapter["speed_gain_mps_per_command"]
    )
    assert 0.19 <= reverse_distance_upper_bound <= 0.21
    assert adapter["obstacle_reverse_settle_sec"] >= adapter[
        "direction_change_dwell_sec"
    ]
    assert manager["obstacle_reverse_replan_delay_sec"] >= (
        adapter["obstacle_reverse_settle_sec"]
        + adapter["obstacle_reverse_duration_sec"]
    )
    assert manager["obstacle_reverse_authorization_sec"] > manager[
        "obstacle_reverse_replan_delay_sec"
    ]
    assert adapter["obstacle_recovery_request_topic"] == manager[
        "obstacle_recovery_request_topic"
    ]
    assert "desired.speed_command > 0.0" in adapter_source
    assert "obstacle_reverse_active" in adapter_source
    assert "handle.cancel_goal_async()" in manager_source
    assert "lidar_obstacle_reverse_recovery" in manager_source


def test_behavior_trees_have_direction_specific_plugins_and_safe_recovery():
    expected_plugins = {
        "ackermann_navigate_to_pose.xml": ("GridBased", "FollowPath"),
        "ackermann_forward_navigate_to_pose.xml": (
            "ForwardGrid",
            "FollowPathForward",
            "transit_goal_checker",
        ),
        "ackermann_forward_precise_navigate_to_pose.xml": (
            "ForwardGrid",
            "FollowPathForward",
            "parking_goal_checker",
        ),
        "ackermann_reverse_transit_navigate_to_pose.xml": (
            "GridBased",
            "FollowPath",
            "transit_goal_checker",
        ),
        "ackermann_bidirectional_precise_navigate_to_pose.xml": (
            "GridBased",
            "FollowPath",
            "parking_goal_checker",
        ),
        "ackermann_reverse_parking_navigate_to_pose.xml": (
            "GridBased",
            "FollowPathReverse",
            "parking_goal_checker",
        ),
    }
    expected_plugins["ackermann_navigate_to_pose.xml"] = (
        "GridBased",
        "FollowPath",
        "parking_goal_checker",
    )
    parking_update_trees = {
        "ackermann_navigate_to_pose.xml",
    }
    for filename, (planner_id, controller_id, goal_checker_id) in expected_plugins.items():
        tree = ET.parse(PACKAGE / "behavior_trees" / filename)
        tags = {element.tag for element in tree.iter()}
        rate_controller = tree.find(".//RateController")
        navigation_recovery = tree.find(
            ".//RecoveryNode[@name='NavigateRecovery']"
        )
        compute = tree.find(".//ComputePathToPose")
        follow = tree.find(".//FollowPath")

        assert not {"Spin", "BackUp", "DriveOnHeading", "Wait"} & tags
        assert {"ComputePathToPose", "FollowPath"} <= tags
        assert compute is not None and compute.attrib["planner_id"] == planner_id
        assert follow is not None and follow.attrib["controller_id"] == controller_id
        assert follow.attrib["goal_checker_id"] == goal_checker_id
        if filename in parking_update_trees:
            # Only direction boundaries / parking poses reconnect against the
            # static map. Ordinary transit keeps one nominal segment path.
            assert rate_controller is not None
            assert float(rate_controller.attrib["hz"]) <= 2.0
            assert tree.find(".//PipelineSequence") is not None
        else:
            assert rate_controller is None
            assert tree.find(".//Sequence") is not None
        assert navigation_recovery is not None
        minimum_retries = (
            1 if filename == "ackermann_forward_navigate_to_pose.xml" else 6
        )
        assert int(navigation_recovery.attrib["number_of_retries"]) >= minimum_retries


def test_lidar_is_reserved_for_the_independent_emergency_stop():
    nav2 = _yaml("config/nav2_parking.yaml")
    adapter = _yaml("config/cmd_vel_adapter.yaml")["parking_cmd_vel_adapter"][
        "ros__parameters"
    ]

    for costmap_name in ("local_costmap", "global_costmap"):
        params = nav2[costmap_name][costmap_name]["ros__parameters"]
        obstacle = params["obstacle_layer"]
        scan = obstacle["scan"]
        assert obstacle["enabled"] is False
        assert scan["topic"] == "/slam/scan_filtered"
        assert 0.25 <= scan["observation_persistence"] <= 0.50

    assert adapter["scan_topic"] == "/slam/scan_filtered"
    assert adapter["minimum_projection_m"] >= 0.14
    assert adapter["collision_margin_m"] >= 0.045


def test_waypoint_route_localization_matches_imported_branch_contract():
    localizer = _yaml("config/route_scan_localizer.yaml")[
        "parking_route_scan_localizer"
    ]["ros__parameters"]
    route = _yaml("config/waypoint_route.yaml")["parking_waypoint_route"][
        "ros__parameters"
    ]
    drive_launch = (
        PACKAGE / "launch/parking_waypoint_drive.launch.py"
    ).read_text(encoding="utf-8")
    capture_launch = (
        PACKAGE / "launch/parking_waypoint_capture.launch.py"
    ).read_text(encoding="utf-8")
    rviz_source = (PACKAGE / "rviz/parking_nav.rviz").read_text(
        encoding="utf-8"
    )
    manager_source = (
        PACKAGE / "xycar_parking_nav/mission_manager.py"
    ).read_text(encoding="utf-8")

    assert localizer["path_topic"] == route["path_topic"]
    assert localizer["scan_topic"] == "/slam/scan_filtered"
    assert localizer["required_consistent_scans"] == 4
    assert localizer["initial_route_search_distance_m"] == pytest.approx(2.0)
    assert localizer["minimum_score_margin"] == pytest.approx(0.06)
    assert localizer["minimum_inlier_ratio"] == pytest.approx(0.45)
    assert route["inflation_radius_m"] == pytest.approx(0.23)
    assert route["parking_snap_radius_m"] == pytest.approx(0.45)
    assert '"initial_pose_publish_count": "0"' in drive_launch
    assert '"require_route_localization": "true"' in drive_launch
    assert "age.ready and self.route_localization_ready" in manager_source
    assert "_current_step_requires_aligned_completion" in manager_source
    assert "aligned_completion_waiting_for_localization" in manager_source
    assert '"capture_enabled": True' in capture_launch
    assert '"drive_enabled": "false"' in capture_launch
    assert '"reference_mission_yaml"' in capture_launch
    assert "/parking/reference_locations" in rviz_source


def test_mppi_prefers_a_free_detour_before_emergency_stop():
    nav2 = _yaml("config/nav2_parking.yaml")
    controllers = nav2["controller_server"]["ros__parameters"]

    for controller_name in (
        "FollowPathForward",
        "FollowPath",
        "FollowPathReverse",
    ):
        controller = controllers[controller_name]
        assert controller["CostCritic"]["cost_weight"] >= 8.0
        assert controller["PathAlignCritic"]["cost_weight"] >= 18.0
        assert controller["PathFollowCritic"]["cost_weight"] >= 12.0
        assert controller["CostCritic"]["consider_footprint"] is True

    for costmap_name in ("local_costmap", "global_costmap"):
        params = nav2[costmap_name][costmap_name]["ros__parameters"]
        assert params["inflation_layer"]["inflation_radius"] >= 0.10

    planners = nav2["planner_server"]["ros__parameters"]
    assert planners["ForwardGrid"]["cost_penalty"] >= 3.0
    assert planners["GridBased"]["cost_penalty"] >= 2.5


def test_global_planner_balances_fast_detours_and_motion_stability():
    planners = _yaml("config/nav2_parking.yaml")["planner_server"]["ros__parameters"]
    planner = planners["GridBased"]
    forward = planners["ForwardGrid"]

    assert planner["motion_model_for_search"] == "REEDS_SHEPP"
    assert planner["reverse_penalty"] == 1.0
    assert planner["change_penalty"] >= 4.0
    assert forward["motion_model_for_search"] == "DUBIN"
    assert 1.05 <= planner["non_straight_penalty"] <= 1.10
    assert 0.0 < planner["retrospective_penalty"] <= 0.015
    assert planner["cache_obstacle_heuristic"] is False
    assert planner["max_planning_time"] <= 2.0


def test_only_explicit_parking_steps_allow_reverse():
    steps = _yaml("config/parking_mission.yaml")["mission"]["steps"]
    reverse_steps = {
        step["name"] for step in steps if step.get("allow_reverse", False)
    }
    assert reverse_steps == {
        "A_REVERSE_ARC_1",
        "A_REVERSE_ARC_2",
        "A_REVERSE_ARC_3",
        "A_REVERSE_CUSP",
        "A_PARK",
        "B_REVERSE_ARC_1",
        "B_REVERSE_ARC_2",
        "B_REVERSE_CUSP",
        "B_PARK",
        "B_EXIT_FORWARD_ARC_1",
        "START_RETURN_ALIGN_2",
    }


def test_failed_forward_transit_has_controlled_reverse_fallback():
    manager = _yaml("config/mission_manager.yaml")["parking_mission_manager"][
        "ros__parameters"
    ]
    source = (PACKAGE / "xycar_parking_nav" / "mission_manager.py").read_text(
        encoding="utf-8"
    )

    assert manager["forward_failure_reverse_fallback_enabled"] is True
    assert "is_reverse_fallback_candidate(step)" in source
    assert "wrapped.status == GoalStatus.STATUS_ABORTED" in source
    assert "self.reverse_fallback_behavior_tree" in source
    assert "self.precise_reverse_behavior_tree" in source
    assert "self.parking_reverse_behavior_tree" in source
    assert "self.reverse_only_transit_behavior_tree" in source


def test_b_parking_bypasses_nav2_abort_with_closed_loop_direct_reverse():
    manager = _yaml("config/mission_manager.yaml")["parking_mission_manager"][
        "ros__parameters"
    ]
    source = (PACKAGE / "xycar_parking_nav" / "mission_manager.py").read_text(
        encoding="utf-8"
    )

    assert manager["direct_reverse_parking_enabled"] is True
    assert manager["direct_reverse_parking_goal_names"] == ["B_PARK"]
    assert manager["direct_cmd_vel_topic"] == "/cmd_vel_nav"
    assert manager["direct_reverse_speed_mps"] == pytest.approx(0.322448)
    assert "direct_reverse_parking_command" in source
    assert "self.direct_parking_active" in source


def test_forward_transit_skips_holds_but_precision_boundaries_remain_marked():
    steps = _yaml("config/parking_mission.yaml")["mission"]["steps"]
    moving_transit = [
        step
        for step in steps
        if not step.get("allow_reverse", False)
        and not step.get("precise_goal", False)
        and not step.get("parking_goal", False)
    ]
    precise = {step["name"] for step in steps if step.get("precise_goal", False)}

    assert all(step["hold_sec"] == 0.0 for step in moving_transit)
    assert {
        "A_FORWARD_CUSP",
        "A_ENTRY",
        "B_ENTRY",
        "B_FORWARD_CORRECTION",
        "B_PARK",
        "START_RETURN",
    } <= precise


def test_mission_uses_field_measured_reference_centers():
    mission = _yaml("config/parking_mission.yaml")["mission"]
    amcl = _yaml("config/nav2_parking.yaml")["amcl"]["ros__parameters"]
    initial = mission["initial_pose"]
    steps = {step["name"]: step for step in mission["steps"]}

    assert (initial["x"], initial["y"], initial["yaw"]) == (
        1.790142252,
        0.777545195,
        -3.038,
    )
    assert (steps["A_PARK"]["x"], steps["A_PARK"]["y"], steps["A_PARK"]["yaw"]) == (
        -0.015964721,
        4.104640247,
        0.021,
    )
    assert (steps["B_PARK"]["x"], steps["B_PARK"]["y"], steps["B_PARK"]["yaw"]) == (
        2.059160896,
        3.185632433,
        -1.503,
    )
    assert (
        steps["START_RETURN"]["x"],
        steps["START_RETURN"]["y"],
        steps["START_RETURN"]["yaw"],
    ) == (initial["x"], initial["y"], initial["yaw"])
    assert (
        amcl["initial_pose"]["x"],
        amcl["initial_pose"]["y"],
        amcl["initial_pose"]["yaw"],
    ) == (1.631, 0.761, -3.038)
    assert "A_CLEAR" in steps
    assert steps["A_CLEAR"]["y"] >= 4.8
    assert "A_APPROACH" in steps
    assert {"A_ROUTE_ARC_1", "A_ROUTE_STRAIGHT", "A_ROUTE_ALIGN"} <= set(steps)
    assert {"A_TURN_IN", "A_FORWARD_CUSP", "A_REVERSE_CUSP"} <= set(steps)
    assert {"A_REVERSE_ARC_1", "A_REVERSE_ARC_2", "A_REVERSE_ARC_3"} <= set(steps)
    assert {"A_CLEAR_ARC_1", "A_CLEAR_ARC_2"} <= set(steps)
    assert {"B_REVERSE_CUSP", "B_EXIT_FORWARD_CUSP"} <= set(steps)
    assert steps["B_REVERSE_ARC_2"]["x"] >= 2.15
    assert steps["B_REVERSE_CUSP"]["x"] >= 2.12
    # The B correction pose is southeast of B_PARK. It gives a forward-only
    # approach and leaves one reverse-only parking path away from y=4.0.
    b_correction = steps["B_FORWARD_CORRECTION"]
    b_correction_base = (
        b_correction["x"] + mission["base_from_reference_x_m"] * math.cos(b_correction["yaw"]),
        b_correction["y"] + mission["base_from_reference_x_m"] * math.sin(b_correction["yaw"]),
    )
    assert math.isclose(b_correction_base[0], 2.850, abs_tol=0.002)
    assert math.isclose(b_correction_base[1], 2.400, abs_tol=0.002)
    assert math.isclose(b_correction["yaw"], -0.600, abs_tol=1.0e-9)
    assert {"B_REVERSE_ARC_1", "B_REVERSE_ARC_2"} <= set(steps)
    assert "B_FORWARD_CORRECTION" in steps
    assert {"B_EXIT_FORWARD_ARC_1", "B_EXIT_FORWARD_ARC_2"} <= set(steps)
    assert {
        "START_RETURN_STRAIGHT",
        "START_RETURN_ARC",
        "START_RETURN_DIAGONAL",
        "START_RETURN_ALIGN_1",
        "START_RETURN_ALIGN_2",
    } <= set(steps)


def test_start_route_heading_changes_fit_the_measured_turning_radius():
    mission = _yaml("config/parking_mission.yaml")["mission"]
    steps = {step["name"]: step for step in mission["steps"]}
    ordered = [mission["initial_pose"]] + [
        steps[name]
        for name in ("A_ROUTE_ARC_1", "A_ROUTE_STRAIGHT", "A_ROUTE_ALIGN")
    ]
    offset = float(mission["base_from_reference_x_m"])
    base_poses = [
        (
            pose["x"] + offset * math.cos(pose["yaw"]),
            pose["y"] + offset * math.sin(pose["yaw"]),
            pose["yaw"],
        )
        for pose in ordered
    ]

    for start, goal in zip(base_poses, base_poses[1:]):
        chord = math.hypot(goal[0] - start[0], goal[1] - start[1])
        yaw_delta = abs(
            math.atan2(math.sin(goal[2] - start[2]), math.cos(goal[2] - start[2]))
        )
        minimum_arc_chord = 2.0 * 0.67 * math.sin(0.5 * yaw_delta)
        assert chord + 0.02 >= minimum_arc_chord


def test_launches_use_humble_safe_python_boolean_spelling():
    for name in ("parking_navigation.launch.py", "parking_real.launch.py"):
        source = (PACKAGE / "launch" / name).read_text(encoding="utf-8")
        assert '"use_composition": "False"' in source
        assert '"use_respawn": "False"' in source
        assert '"use_composition": "false"' not in source


def test_real_launch_runs_terminal_sensor_preflight_by_default():
    source = (PACKAGE / "launch/parking_real.launch.py").read_text(
        encoding="utf-8"
    )
    setup_source = (PACKAGE / "setup.py").read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("enable_preflight", default_value="true")' in source
    assert 'DeclareLaunchArgument("preflight_timeout_sec", default_value="20.0")' in source
    assert 'executable="parking_preflight"' in source
    assert '"drive_enabled": ParameterValue(' in source
    assert "parking_preflight = xycar_parking_nav.parking_preflight:main" in setup_source


def test_parking_preflight_status_labels_are_korean():
    source = (PACKAGE / "xycar_parking_nav/parking_preflight.py").read_text(
        encoding="utf-8"
    )

    assert all(label in source for label in ("[정상]", "[대기]", "[실패]", "[준비완료]"))
    assert not any(label in source for label in ("[OK]", "[WAIT]", "[FAIL]", "[READY]"))


def test_rviz_has_no_visual_slam_or_legacy_path_topics():
    source = (PACKAGE / "rviz/parking_nav.rviz").read_text(encoding="utf-8")
    assert "slam_toolbox" not in source.lower()
    assert "/map_nav/" not in source
    assert "/amcl_pose" in source
    assert "/parking/mission_goals" in source
    assert "/parking/mission_visualization" in source
    assert "Parking Spaces And Nominal Route" in source
    assert "/plan" in source
