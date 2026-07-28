from dataclasses import replace

import cv2
import numpy as np
import pytest

from my_rule.drive_manager_node import (
    DriveManagerNode,
    DynamicObstacleEstimate,
    StaticObstacleEstimate,
    choose_static_avoidance_direction,
    dynamic_follow_speed,
    inferred_target_speed_mps,
    initial_traffic_go,
    obstacle_ttc,
    signal_approach_speed,
)
from my_rule.perception.object_perception import (
    DetectionRecord,
    cone_modalities_match,
    detection_side_counts,
    filter_detections,
    green_hsv_evidence_in_box,
    normalize_class_name,
    restrict_yellow_mask_to_hints,
)


class _Parameter:
    def __init__(self, value):
        self.value = value


class _ConeGateHarness:
    cone_entry_gate = DriveManagerNode.cone_entry_gate

    parameters = {
        "external_cone_cmd_timeout_sec": 0.5,
        "cone_cluster_timeout_sec": 0.5,
        "cone_mode_presence_min_clusters": 2,
        "require_yolo_cone_for_entry": True,
        "yolo_cone_timeout_sec": 0.75,
        "yolo_cone_required_frames": 2,
        "yolo_cone_min_count": 2,
    }

    def __init__(self):
        self.cone_command_time = 10.0
        self.cone_entry_frames = 3
        self.cone_cluster_time = 10.0
        self.cone_cluster_count = 4
        self.cone_cluster_left_count = 2
        self.cone_cluster_right_count = 2
        self.yolo_cone_time = 10.0
        self.yolo_cone_frames = 2
        self.yolo_cone_count = 2
        self.yolo_cone_left_count = 1
        self.yolo_cone_right_count = 1

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _TrafficHarness:
    update_traffic_state = DriveManagerNode.update_traffic_state

    parameters = {
        "traffic_min_confidence": 0.5,
        "traffic_signal_max_center_y_ratio": 0.55,
        "traffic_signal_min_area_ratio": 0.00005,
        "traffic_required_frames": 2,
        "course_signal_activation_mode": "mission_state",
    }

    def __init__(self):
        self.traffic_control_enabled = True
        self.traffic_startup_only = False
        self.traffic_go = False
        self.traffic_red_frames = 0
        self.traffic_green_frames = 0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _Logger:
    def info(self, _message):
        pass


class _BoolMessage:
    def __init__(self, data):
        self.data = data


class _StartupTrafficHarness(_TrafficHarness):
    on_startup_green = DriveManagerNode.on_startup_green

    def __init__(self):
        super().__init__()
        self.traffic_startup_only = True
        self.wait_for_green_at_start = True
        self.course_signal_control_enabled = False
        self.course_signal_armed = False
        self.course_signal_state = "SEARCHING"

    def get_logger(self):
        return _Logger()


class _CourseSignalHarness:
    update_course_signal_state = DriveManagerNode.update_course_signal_state
    apply_course_signal_overlay = DriveManagerNode.apply_course_signal_overlay

    parameters = {
        "traffic_min_confidence": 0.5,
        "traffic_signal_max_center_y_ratio": 0.55,
        "traffic_signal_min_area_ratio": 0.00005,
        "course_signal_red_confirm_frames": 2,
        "course_signal_green_confirm_frames": 2,
        "course_signal_yellow_confirm_frames": 2,
        "course_signal_yellow_action": "ignore",
        "course_signal_yellow_commit_area_ratio": 0.012,
        "course_signal_observation_timeout_sec": 0.8,
        "course_signal_slowdown_area_ratio": 0.005,
        "course_signal_stop_area_ratio": 0.015,
        "course_signal_lost_action": "hold",
    }

    def __init__(self):
        self.traffic_go = True
        self.course_signal_control_enabled = True
        self.course_signal_armed = True
        self.course_signal_state = "SEARCHING"
        self.course_signal_color = "unknown"
        self.course_signal_area_ratio = 0.0
        self.course_signal_last_seen_time = 0.0
        self.course_signal_red_frames = 0
        self.course_signal_green_frames = 0
        self.course_signal_yellow_frames = 0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _MissionPriorityHarness:
    select_final_command = DriveManagerNode.select_final_command
    static_obstacle_command = DriveManagerNode.static_obstacle_command

    parameters = {
        "enable_static_obstacle_handling": True,
        "static_obstacle_timeout_sec": 0.75,
    }

    def __init__(self):
        self.traffic_control_enabled = False
        self.traffic_go = True
        self.traffic_red_frames = 0
        self.static_obstacle_confirmed = True
        self.static_obstacle_time = 10.0
        self.static_obstacle_state = "CLEAR"
        self.static_obstacle_estimate = StaticObstacleEstimate(
            detected=True,
            in_path=True,
            close=True,
            sensor_confirmed=True,
            position="center",
            distance_m=0.9,
            target_speed_mps=0.0,
            reason="car+lidar",
        )
        self.front_distance = 0.9
        self.cone_mode_active = True

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])


class _DynamicDecisionHarness:
    dynamic_overtake_candidate_ready = (
        DriveManagerNode.dynamic_overtake_candidate_ready
    )
    dynamic_obstacle_moves_into_pass_lane = (
        DriveManagerNode.dynamic_obstacle_moves_into_pass_lane
    )

    parameters = {
        "dynamic_obstacle_required_frames": 4,
        "dynamic_obstacle_allow_multi_vehicle": False,
        "dynamic_obstacle_multi_vehicle_gap_m": 0.8,
        "dynamic_obstacle_curve_steer_threshold_command": 8.0,
        "dynamic_obstacle_allow_curve_start": False,
        "dynamic_obstacle_min_commit_distance_m": 1.1,
        "dynamic_obstacle_prepare_distance_m": 2.4,
        "dynamic_obstacle_min_ttc_sec": 3.0,
        "dynamic_obstacle_lateral_motion_threshold_ratio_s": 0.08,
    }

    def __init__(self):
        self.dynamic_obstacle_abort_hold_until = 0.0
        self.lane_angle = 0.0

    def get_parameter(self, name):
        return _Parameter(self.parameters[name])

    def dynamic_lane_path_ready(self):
        return True


def _record(name, confidence, xmin, ymin, xmax, ymax):
    return DetectionRecord(
        class_name=name,
        class_id=0,
        confidence=confidence,
        xmin=xmin,
        ymin=ymin,
        xmax=xmax,
        ymax=ymax,
    )


def test_object_model_class_names_are_normalized_without_aliasing():
    assert normalize_class_name("Cone") == "cone"
    assert normalize_class_name(" yellow-centerline ") == "yellow_centerline"
    assert normalize_class_name("Car") == "car"


def test_detection_record_exposes_box_geometry_for_sensor_fusion():
    record = _record("Car", 0.9, 12, 20, 52, 65)
    assert record.width == 40
    assert record.height == 45
    assert record.area == 1800


def test_each_object_class_uses_its_own_confidence_threshold():
    records = [
        _record("Cone", 0.49, 0, 0, 10, 10),
        _record("Cone", 0.51, 0, 0, 10, 10),
        _record("Car", 0.46, 0, 0, 10, 10),
        _record("unknown", 0.99, 0, 0, 10, 10),
    ]
    accepted = filter_detections(
        records,
        {"cone": 0.50, "car": 0.45},
    )
    assert [record.class_name for record in accepted] == ["cone", "car"]


def test_startup_hsv_measures_only_the_exact_yolo_box():
    hsv = np.zeros((40, 60, 3), dtype=np.uint8)
    hsv[10:20, 20:30] = (80, 220, 220)
    hsv[0:10, 0:10] = (80, 220, 220)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    evidence = green_hsv_evidence_in_box(
        bgr,
        (20, 10, 30, 20),
    )
    assert evidence.valid
    assert evidence.green_pixels == 100
    assert evidence.pixel_ratio == 1.0

    outside = green_hsv_evidence_in_box(
        bgr,
        (35, 25, 55, 35),
    )
    assert outside.valid
    assert outside.green_pixels == 0
    assert outside.pixel_ratio == 0.0


def test_course_signal_bbox_profile_preserves_speed_then_brakes_to_zero():
    speed, progress, stop = signal_approach_speed(8.0, 0.003, 0.005, 0.015)
    assert speed == 8.0
    assert progress == 0.0
    assert not stop

    speed, progress, stop = signal_approach_speed(8.0, 0.010, 0.005, 0.015)
    assert speed == pytest.approx(4.0)
    assert progress == pytest.approx(0.5)
    assert not stop

    speed, progress, stop = signal_approach_speed(8.0, 0.015, 0.005, 0.015)
    assert speed == 0.0
    assert progress == 1.0
    assert stop


def test_cone_image_side_counts_ignore_the_center_deadband():
    cones = [
        _record("cone", 0.9, 10, 40, 30, 70),
        _record("cone", 0.9, 170, 40, 190, 70),
        _record("cone", 0.9, 95, 40, 105, 70),
    ]
    assert detection_side_counts(
        cones,
        image_width=200,
        class_name="cone",
        center_deadband_ratio=0.08,
    ) == (3, 1, 1)


def test_cone_entry_requires_both_modalities_and_matching_sides():
    assert cone_modalities_match(
        yolo_total=2,
        yolo_left=1,
        yolo_right=1,
        lidar_total=4,
        lidar_left=2,
        lidar_right=2,
        minimum_yolo_count=2,
        minimum_lidar_count=2,
    )
    assert not cone_modalities_match(
        yolo_total=2,
        yolo_left=1,
        yolo_right=1,
        lidar_total=3,
        lidar_left=3,
        lidar_right=0,
        minimum_yolo_count=2,
        minimum_lidar_count=2,
    )
    assert not cone_modalities_match(
        yolo_total=1,
        yolo_left=1,
        yolo_right=0,
        lidar_total=4,
        lidar_left=2,
        lidar_right=2,
        minimum_yolo_count=2,
        minimum_lidar_count=2,
    )


def test_drive_manager_enters_cones_only_after_fresh_yolo_and_lidar():
    harness = _ConeGateHarness()
    assert harness.cone_entry_gate(10.1, 3) == (
        True,
        "yolo+lidar_confirmed",
    )
    harness.yolo_cone_frames = 1
    assert harness.cone_entry_gate(10.1, 3) == (
        False,
        "waiting_yolo_cones",
    )
    harness.yolo_cone_frames = 2
    harness.cone_cluster_right_count = 0
    assert harness.cone_entry_gate(10.1, 3) == (
        False,
        "waiting_sensor_side_match",
    )


def test_green_releases_and_red_relatches_after_temporal_confirmation():
    harness = _TrafficHarness()
    green = [_record("green", 0.9, 40, 10, 70, 40)]
    harness.update_traffic_state(green, 200, 100)
    assert not harness.traffic_go
    harness.update_traffic_state(green, 200, 100)
    assert harness.traffic_go

    yellow = [_record("yellow", 0.99, 40, 10, 70, 40)]
    harness.update_traffic_state(yellow, 200, 100)
    assert harness.traffic_go

    red = [_record("red", 0.9, 40, 10, 70, 40)]
    harness.update_traffic_state(red, 200, 100)
    assert harness.traffic_go
    harness.update_traffic_state(red, 200, 100)
    assert not harness.traffic_go


def test_startup_hsv_releases_once_and_later_yolo_red_cannot_relatch():
    harness = _StartupTrafficHarness()
    green = [_record("green", 0.9, 40, 10, 70, 40)]
    harness.update_traffic_state(green, 200, 100)
    harness.update_traffic_state(green, 200, 100)
    assert not harness.traffic_go

    harness.on_startup_green(_BoolMessage(True))
    assert harness.traffic_go

    red = [_record("red", 0.9, 40, 10, 70, 40)]
    harness.update_traffic_state(red, 200, 100)
    harness.update_traffic_state(red, 200, 100)
    assert harness.traffic_go


def test_later_red_brakes_without_discarding_lane_steering():
    harness = _CourseSignalHarness()
    red = [_record("red", 0.9, 20, 5, 30, 15)]
    harness.update_course_signal_state(red, 100, 100, 10.0)
    harness.update_course_signal_state(red, 100, 100, 10.1)
    assert harness.course_signal_state == "RED_APPROACH"
    result = harness.apply_course_signal_overlay(
        12.0, 8.0, "LANE_FOLLOW", "lane", 10.2
    )
    assert result[0] == 12.0
    assert result[1] == pytest.approx(4.0)
    assert result[2:] == (
        "COURSE_RED_APPROACH",
        "bbox_area=0.01000 brake=0.50",
    )


def test_later_green_releases_red_and_ignored_yellow_preserves_state():
    harness = _CourseSignalHarness()
    harness.course_signal_state = "RED_HOLD"
    yellow = [_record("yellow", 0.9, 20, 5, 30, 15)]
    harness.update_course_signal_state(yellow, 100, 100, 10.0)
    harness.update_course_signal_state(yellow, 100, 100, 10.1)
    assert harness.course_signal_state == "RED_HOLD"

    green = [_record("green", 0.9, 20, 5, 30, 15)]
    harness.update_course_signal_state(green, 100, 100, 10.2)
    harness.update_course_signal_state(green, 100, 100, 10.3)
    assert harness.course_signal_state == "GREEN_PASS"
    assert harness.apply_course_signal_overlay(
        -7.0, 8.0, "LANE_FOLLOW", "lane", 10.4
    ) == (-7.0, 8.0, "LANE_FOLLOW", "lane")


def test_only_integrated_mode_needs_green_before_initial_departure():
    assert not initial_traffic_go(True, True)
    assert initial_traffic_go(True, False)
    assert initial_traffic_go(False, True)


def test_confirmed_obstacle_safety_gate_remains_active_in_cone_mode():
    harness = _MissionPriorityHarness()
    assert harness.select_final_command(10.1) == (
        0.0,
        0.0,
        "STATIC_OBSTACLE_HOLD",
        "confirmed_car_in_cone_mode",
    )


def test_static_motion_estimate_separates_fixed_and_moving_car():
    fixed = inferred_target_speed_mps(
        previous_distance_m=2.0,
        current_distance_m=2.0 - 8.0 * 0.080612,
        dt_sec=1.0,
        ego_speed_command=8.0,
        speed_gain_mps_per_command=0.080612,
    )
    moving = inferred_target_speed_mps(
        previous_distance_m=2.0,
        current_distance_m=1.60,
        dt_sec=1.0,
        ego_speed_command=8.0,
        speed_gain_mps_per_command=0.080612,
    )
    assert fixed == pytest.approx(0.0)
    assert moving == pytest.approx(0.244896)


def test_static_obstacle_chooses_opposite_or_wider_clear_lane():
    assert choose_static_avoidance_direction(
        obstacle_position="left",
        left_clear=True,
        right_clear=True,
        left_distance_m=1.4,
        right_distance_m=0.9,
    ) == "right"
    assert choose_static_avoidance_direction(
        obstacle_position="center",
        left_clear=True,
        right_clear=True,
        left_distance_m=1.4,
        right_distance_m=0.9,
    ) == "left"
    assert choose_static_avoidance_direction(
        obstacle_position="right",
        left_clear=False,
        right_clear=False,
        left_distance_m=1.4,
        right_distance_m=0.9,
    ) is None


def test_dynamic_following_uses_time_gap_and_brakes_for_fast_closure():
    cruise, target_gap = dynamic_follow_speed(
        obstacle_distance_m=3.0,
        closing_speed_mps=0.0,
        estimated_vehicle_speed_command=3.0,
        current_speed_command=8.0,
        speed_to_mps_scale=0.080612,
        standstill_gap_m=0.65,
        target_time_gap_sec=2.0,
        follow_kp=0.8,
        closing_gain=4.0,
        minimum_speed_command=3.0,
        maximum_speed_command=8.0,
    )
    assert target_gap == pytest.approx(1.939792)
    assert cruise > 3.0

    braking, braking_gap = dynamic_follow_speed(
        obstacle_distance_m=1.2,
        closing_speed_mps=0.4,
        estimated_vehicle_speed_command=3.0,
        current_speed_command=8.0,
        speed_to_mps_scale=0.080612,
        standstill_gap_m=0.65,
        target_time_gap_sec=2.0,
        follow_kp=0.8,
        closing_gain=4.0,
        minimum_speed_command=3.0,
        maximum_speed_command=8.0,
    )
    assert braking_gap > target_gap
    assert braking == 3.0
    assert obstacle_ttc(1.2, 0.4) == pytest.approx(3.0)


def test_dynamic_pass_requires_stable_vehicle_and_reacts_to_cut_in():
    harness = _DynamicDecisionHarness()
    stable = DynamicObstacleEstimate(
        detected=True,
        sensor_confirmed=True,
        distance_m=2.0,
        position="center",
        vehicle_count=1,
        lane_stable_frames=4,
        ttc_sec=5.0,
    )
    assert harness.dynamic_overtake_candidate_ready(stable, 10.0)
    assert not harness.dynamic_overtake_candidate_ready(
        replace(stable, position="transitioning", lane_stable_frames=0),
        10.0,
    )
    assert harness.dynamic_obstacle_moves_into_pass_lane(
        replace(stable, lateral_velocity_ratio_s=-0.05),
        "left",
    )
    assert not harness.dynamic_obstacle_moves_into_pass_lane(
        replace(stable, lateral_velocity_ratio_s=-0.05),
        "right",
    )


def test_yolo_yellow_hint_rejects_a_distant_false_component():
    mask = np.zeros((72, 128), dtype=np.uint8)
    cv2.rectangle(mask, (57, 5), (62, 66), 255, -1)
    cv2.rectangle(mask, (10, 12), (20, 60), 255, -1)
    cv2.rectangle(mask, (92, 18), (102, 55), 255, -1)
    hint = _record(
        "yellow_centerline",
        0.9,
        220,
        0,
        300,
        144,
    )
    selected, applied = restrict_yellow_mask_to_hints(
        mask,
        [hint],
        image_width=512,
        horizontal_margin_px=6,
        minimum_retained_fraction=0.20,
    )
    assert applied
    assert np.count_nonzero(selected[:, 57:63]) > 0
    assert np.count_nonzero(selected[:, 10:21]) == 0
    assert np.count_nonzero(selected[:, 92:103]) == 0


def test_yolo_yellow_hint_falls_back_if_it_would_erase_the_lane():
    mask = np.zeros((72, 128), dtype=np.uint8)
    cv2.rectangle(mask, (57, 5), (62, 66), 255, -1)
    cv2.rectangle(mask, (10, 12), (20, 60), 255, -1)
    hint = _record(
        "yellow_centerline",
        0.9,
        470,
        0,
        500,
        144,
    )
    selected, applied = restrict_yellow_mask_to_hints(
        mask,
        [hint],
        image_width=512,
        horizontal_margin_px=2,
    )
    assert not applied
    assert np.array_equal(selected, mask)
