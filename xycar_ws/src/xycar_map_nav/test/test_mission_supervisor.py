from xycar_map_nav.mission_supervisor import (
    camera_box_lidar_sector,
    MissionMode,
    MissionSupervisor,
    MissionSupervisorConfig,
    scan_sector_distance,
)


def observe_empty(supervisor, now_sec, traffic_color="unknown"):
    supervisor.observe_objects(
        now_sec=now_sec,
        cone_count=0,
        cone_max_confidence=0.0,
        vehicle_count=0,
        vehicle_max_confidence=0.0,
        vehicle_lidar_distance_m=float("inf"),
        traffic_color=traffic_color,
    )


def test_global_path_is_the_default_mode():
    supervisor = MissionSupervisor(MissionSupervisorConfig())

    decision = supervisor.decide(
        now_sec=1.0,
        cone_command_ready=False,
        dynamic_rule_mode="NORMAL",
    )

    assert decision.mode == MissionMode.GLOBAL_PATH


def test_cone_requires_camera_lidar_distance_and_command():
    supervisor = MissionSupervisor(MissionSupervisorConfig())
    for now in (1.0, 1.1):
        supervisor.observe_objects(
            now_sec=now,
            cone_count=2,
            cone_max_confidence=0.8,
            vehicle_count=0,
            vehicle_max_confidence=0.0,
            vehicle_lidar_distance_m=float("inf"),
            traffic_color="unknown",
        )
    supervisor.observe_cone_lidar(
        now_sec=1.1,
        count=3,
        nearest_distance_m=0.51,
    )
    assert (
        supervisor.decide(
            now_sec=1.1,
            cone_command_ready=True,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.GLOBAL_PATH
    )

    supervisor.observe_cone_lidar(
        now_sec=1.2,
        count=3,
        nearest_distance_m=0.50,
    )
    assert (
        supervisor.decide(
            now_sec=1.2,
            cone_command_ready=False,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.GLOBAL_PATH
    )
    assert (
        supervisor.decide(
            now_sec=1.2,
            cone_command_ready=True,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.CONE_RULE
    )


def test_cone_planner_gate_starts_on_first_camera_detection():
    supervisor = MissionSupervisor(
        MissionSupervisorConfig(cone_processing_hold_sec=1.5)
    )
    assert not supervisor.cone_processing_requested(0.0)

    supervisor.observe_objects(
        now_sec=1.0,
        cone_count=1,
        cone_max_confidence=0.8,
        vehicle_count=0,
        vehicle_max_confidence=0.0,
        vehicle_lidar_distance_m=float("inf"),
        traffic_color="unknown",
    )

    # Planning wakes on one YOLO frame, while control transfer still needs
    # the stricter two-frame camera + LiDAR + command condition.
    assert supervisor.cone_processing_requested(1.0)
    assert supervisor.cone_processing_requested(2.49)
    assert not supervisor.cone_processing_requested(2.51)


def test_cone_exit_uses_minimum_duration_and_clear_hold():
    supervisor = MissionSupervisor(MissionSupervisorConfig())
    for now in (1.0, 1.1):
        supervisor.observe_objects(
            now_sec=now,
            cone_count=2,
            cone_max_confidence=0.8,
            vehicle_count=0,
            vehicle_max_confidence=0.0,
            vehicle_lidar_distance_m=float("inf"),
            traffic_color="unknown",
        )
    supervisor.observe_cone_lidar(
        now_sec=1.1,
        count=2,
        nearest_distance_m=0.4,
    )
    assert (
        supervisor.decide(
            now_sec=1.1,
            cone_command_ready=True,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.CONE_RULE
    )

    observe_empty(supervisor, 1.2)
    supervisor.observe_cone_lidar(
        now_sec=1.2,
        count=0,
        nearest_distance_m=float("inf"),
    )
    assert (
        supervisor.decide(
            now_sec=2.2,
            cone_command_ready=False,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.CONE_RULE
    )
    assert (
        supervisor.decide(
            now_sec=3.0,
            cone_command_ready=False,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.GLOBAL_PATH
    )


def test_dynamic_vehicle_requires_camera_and_associated_lidar_range():
    supervisor = MissionSupervisor(MissionSupervisorConfig())
    for now, distance in ((1.0, 2.5), (1.1, 2.3)):
        supervisor.observe_objects(
            now_sec=now,
            cone_count=0,
            cone_max_confidence=0.0,
            vehicle_count=1,
            vehicle_max_confidence=0.9,
            vehicle_lidar_distance_m=distance,
            traffic_color="unknown",
        )
    decision = supervisor.decide(
        now_sec=1.1,
        cone_command_ready=False,
        dynamic_rule_mode="NORMAL",
    )

    assert decision.mode == MissionMode.DYNAMIC_VEHICLE_RULE


def test_lidar_obstacle_rule_overrides_global_path_without_camera_model():
    supervisor = MissionSupervisor(MissionSupervisorConfig())

    decision = supervisor.decide(
        now_sec=1.0,
        cone_command_ready=False,
        dynamic_rule_mode="NORMAL",
        lidar_obstacle_rule_mode="BYPASS_LEFT",
    )

    assert decision.mode == MissionMode.LIDAR_OBSTACLE_RULE
    assert decision.reason == "lidar_path_obstacle_bypass_left"


def test_dynamic_override_waits_for_rule_to_return_normal():
    supervisor = MissionSupervisor(MissionSupervisorConfig())
    for now in (1.0, 1.1):
        supervisor.observe_objects(
            now_sec=now,
            cone_count=0,
            cone_max_confidence=0.0,
            vehicle_count=1,
            vehicle_max_confidence=0.9,
            vehicle_lidar_distance_m=1.5,
            traffic_color="unknown",
        )
    supervisor.decide(
        now_sec=1.1,
        cone_command_ready=False,
        dynamic_rule_mode="AVOID_RIGHT",
    )
    observe_empty(supervisor, 1.2)

    assert (
        supervisor.decide(
            now_sec=2.0,
            cone_command_ready=False,
            dynamic_rule_mode="RETURN_CENTER",
        ).mode
        == MissionMode.DYNAMIC_VEHICLE_RULE
    )
    assert (
        supervisor.decide(
            now_sec=2.0,
            cone_command_ready=False,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.DYNAMIC_VEHICLE_RULE
    )
    assert (
        supervisor.decide(
            now_sec=2.6,
            cone_command_ready=False,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.GLOBAL_PATH
    )


def test_red_or_yellow_latches_stop_until_confirmed_green():
    supervisor = MissionSupervisor(MissionSupervisorConfig())
    observe_empty(supervisor, 1.0, "yellow")
    observe_empty(supervisor, 1.1, "yellow")
    assert (
        supervisor.decide(
            now_sec=1.1,
            cone_command_ready=False,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.TRAFFIC_STOP
    )

    observe_empty(supervisor, 1.2, "unknown")
    assert (
        supervisor.decide(
            now_sec=1.2,
            cone_command_ready=False,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.TRAFFIC_STOP
    )

    observe_empty(supervisor, 1.3, "green")
    observe_empty(supervisor, 1.4, "green")
    assert (
        supervisor.decide(
            now_sec=1.4,
            cone_command_ready=False,
            dynamic_rule_mode="NORMAL",
        ).mode
        == MissionMode.GLOBAL_PATH
    )


def test_lane_intervention_is_disabled_by_default():
    supervisor = MissionSupervisor(MissionSupervisorConfig())
    supervisor.observe_lane_risk(now_sec=1.0, risky=True)

    decision = supervisor.decide(
        now_sec=1.0,
        cone_command_ready=False,
        dynamic_rule_mode="NORMAL",
    )

    assert decision.mode == MissionMode.GLOBAL_PATH


def test_left_camera_box_projects_to_positive_lidar_angles():
    minimum, maximum = camera_box_lidar_sector(
        xmin=100,
        xmax=200,
        image_width=1000,
        horizontal_fov_deg=100.0,
        padding_deg=0.0,
    )

    assert minimum > 0.0
    assert maximum > minimum


def test_scan_distance_requires_multiple_points_and_rejects_outlier():
    distance = scan_sector_distance(
        ranges=[float("inf"), 1.0, 1.2, 8.0, float("nan")],
        angle_min=-0.2,
        angle_increment=0.1,
        range_min=0.1,
        range_max=10.0,
        sector_min_angle=-0.11,
        sector_max_angle=0.11,
        minimum_points=2,
    )

    assert distance == 1.1
