from xycar_map_nav.lidar_obstacle import LidarPathObstacle
from xycar_map_nav.yolo_lidar_avoidance import (
    ShortcutAvoidanceSuppression,
    ShortcutAvoidanceSuppressionConfig,
    YoloLidarAvoidanceConfig,
    YoloLidarAvoidanceController,
    YoloLidarAvoidanceMode,
)
from xycar_map_nav.sequential_hybrid_driver import is_avoidance_detection
from xycar_map_nav.sequential_hybrid_driver import (
    avoidance_speed_limit_for_vehicle_class,
)
from xycar_map_nav.sequential_hybrid_driver import (
    preferred_avoidance_mode_from_yellow_reference,
)
from xycar_map_nav.sequential_hybrid_driver import (
    shortcut_suppresses_vehicle_class,
)
from xycar_map_nav.sequential_hybrid_driver import (
    straight_road_side_decision_allowed,
)


def obstacle(left=1.2, right=0.5, distance=1.0, lateral=0.0):
    return LidarPathObstacle(
        distance_m=distance,
        lateral_m=lateral,
        width_m=0.35,
        point_count=6,
        x_vehicle_m=distance,
        y_vehicle_m=lateral,
        left_clearance_m=left,
        right_clearance_m=right,
    )


def test_shortcut_suppression_releases_after_two_post_handoff_left_frames():
    suppression = ShortcutAvoidanceSuppression(
        ShortcutAvoidanceSuppressionConfig(
            release_left_angle_command=-8.0,
            release_required_frames=2,
        )
    )

    suppression.start_shortcut()
    assert suppression.active
    assert not suppression.observe_rule_angle(-20.0)
    assert suppression.left_frames == 0

    suppression.start_rule_handoff()
    assert not suppression.observe_rule_angle(-9.0)
    assert suppression.left_frames == 1
    assert not suppression.observe_rule_angle(-7.9)
    assert suppression.left_frames == 0
    assert not suppression.observe_rule_angle(-12.0)
    assert suppression.observe_rule_angle(-10.0)
    assert not suppression.active


def test_shortcut_suppression_can_be_disabled():
    suppression = ShortcutAvoidanceSuppression(
        ShortcutAvoidanceSuppressionConfig(enabled=False)
    )
    suppression.start_shortcut()
    suppression.start_rule_handoff()
    assert not suppression.active
    assert not suppression.observe_rule_angle(-42.0)


def test_shortcut_suppresses_only_green_car():
    assert shortcut_suppresses_vehicle_class(
        "green_car",
        suppression_active=True,
    )
    assert not shortcut_suppresses_vehicle_class(
        "red_car",
        suppression_active=True,
    )
    assert not shortcut_suppresses_vehicle_class(
        "green_car",
        suppression_active=False,
    )


def test_red_and_green_car_use_separate_avoidance_speed_limits():
    common = {
        "default_speed_limit_command": 8.0,
        "red_car_speed_limit_command": 8.0,
        "green_car_speed_limit_command": 15.0,
    }
    assert avoidance_speed_limit_for_vehicle_class(
        "red_car",
        **common,
    ) == 8.0
    assert avoidance_speed_limit_for_vehicle_class(
        "green_car",
        **common,
    ) == 15.0
    assert avoidance_speed_limit_for_vehicle_class(
        "obstacle_vehicle",
        **common,
    ) == 8.0


def test_selected_vehicle_class_speed_persists_during_avoidance():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(
            immediate_on_yolo=True,
            preferred_side_required_frames=1,
            yolo_required_frames=1,
            speed_limit_command=8.0,
        )
    )
    controller.observe_yolo(
        now_sec=0.0,
        detected=True,
        confidence=0.9,
        lidar_distance_m=2.0,
        preferred_mode=YoloLidarAvoidanceMode.AVOID_LEFT,
        target_class_name="green_car",
        speed_limit_command=15.0,
    )
    state = controller.step(
        now_sec=0.1,
        dt_sec=0.1,
        obstacle=None,
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_LEFT
    assert state.target_class_name == "green_car"
    assert state.speed_limit_command == 15.0


def test_cone_is_only_an_avoidance_candidate_in_temporary_test_mode():
    common = {
        "class_name": "cone",
        "confidence": 0.8,
        "vehicle_names": {"car", "obstacle_vehicle"},
        "vehicle_min_confidence": 0.45,
        "cone_min_confidence": 0.50,
    }
    assert not is_avoidance_detection(
        **common,
        cone_as_vehicle_obstacle=False,
    )
    assert is_avoidance_detection(
        **common,
        cone_as_vehicle_obstacle=True,
    )


def test_cone_avoidance_uses_its_own_confidence_threshold():
    assert not is_avoidance_detection(
        class_name="cone",
        confidence=0.49,
        vehicle_names={"car"},
        vehicle_min_confidence=0.20,
        cone_as_vehicle_obstacle=True,
        cone_min_confidence=0.50,
    )


def test_yellow_reference_maps_object_to_opposite_avoidance_side():
    yellow_x_by_y = [100.0 + 0.25 * row for row in range(144)]
    assert preferred_avoidance_mode_from_yellow_reference(
        object_x=80.0,
        object_y=100.0,
        yellow_x_by_y=yellow_x_by_y,
        deadband_px=6.0,
    ) == YoloLidarAvoidanceMode.AVOID_RIGHT
    assert preferred_avoidance_mode_from_yellow_reference(
        object_x=140.0,
        object_y=100.0,
        yellow_x_by_y=yellow_x_by_y,
        deadband_px=6.0,
    ) == YoloLidarAvoidanceMode.AVOID_LEFT
    assert preferred_avoidance_mode_from_yellow_reference(
        object_x=128.0,
        object_y=100.0,
        yellow_x_by_y=yellow_x_by_y,
        deadband_px=6.0,
    ) is None


def test_side_decision_requires_fresh_straight_rule_and_yellow_geometry():
    common = dict(
        rule_command_age_sec=0.05,
        yellow_reference_age_sec=0.05,
        yellow_reference_rmse_px=1.5,
        yellow_reference_span_ratio=0.50,
        maximum_rule_angle_command=8.0,
        rule_command_timeout_sec=0.25,
        yellow_reference_timeout_sec=0.40,
        maximum_yellow_rmse_px=3.0,
        minimum_yellow_span_ratio=0.20,
    )
    assert straight_road_side_decision_allowed(
        rule_angle_command=5.0,
        **common,
    )
    assert not straight_road_side_decision_allowed(
        rule_angle_command=12.0,
        **common,
    )
    assert not straight_road_side_decision_allowed(
        rule_angle_command=5.0,
        **{**common, "yellow_reference_rmse_px": 4.0},
    )


def test_curve_gate_clears_uncommitted_avoidance_side():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(preferred_side_required_frames=1)
    )
    controller.observe_yolo(
        now_sec=0.0,
        detected=True,
        confidence=0.9,
        lidar_distance_m=2.0,
        preferred_mode=YoloLidarAvoidanceMode.AVOID_LEFT,
    )
    assert controller.preferred_mode == YoloLidarAvoidanceMode.AVOID_LEFT
    controller.observe_yolo(
        now_sec=0.1,
        detected=True,
        confidence=0.9,
        lidar_distance_m=2.0,
        preferred_mode=None,
        side_decision_allowed=False,
    )
    assert controller.preferred_mode is None


def test_curve_gate_does_not_flip_an_active_avoidance_side():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(
            immediate_on_yolo=True,
            preferred_side_required_frames=1,
            yolo_required_frames=1,
        )
    )
    controller.observe_yolo(
        now_sec=0.0,
        detected=True,
        confidence=0.9,
        lidar_distance_m=2.0,
        preferred_mode=YoloLidarAvoidanceMode.AVOID_LEFT,
    )
    state = controller.step(
        now_sec=0.0,
        dt_sec=0.1,
        obstacle=None,
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_LEFT
    controller.observe_yolo(
        now_sec=0.1,
        detected=True,
        confidence=0.9,
        lidar_distance_m=1.5,
        preferred_mode=YoloLidarAvoidanceMode.AVOID_RIGHT,
        side_decision_allowed=False,
    )
    assert controller.preferred_mode == YoloLidarAvoidanceMode.AVOID_LEFT


def test_oscillating_camera_side_is_not_accepted_from_one_frame():
    controller = YoloLidarAvoidanceController(YoloLidarAvoidanceConfig())
    controller.observe_yolo(
        now_sec=0.0,
        detected=True,
        confidence=0.9,
        lidar_distance_m=2.0,
        preferred_mode=YoloLidarAvoidanceMode.AVOID_RIGHT,
    )
    controller.observe_yolo(
        now_sec=0.1,
        detected=True,
        confidence=0.9,
        lidar_distance_m=2.0,
        preferred_mode=YoloLidarAvoidanceMode.AVOID_LEFT,
    )
    assert controller.preferred_mode is None


def arm(
    controller,
    distance=1.0,
    preferred_mode=YoloLidarAvoidanceMode.AVOID_LEFT,
):
    controller.observe_yolo(
        now_sec=0.0,
        detected=True,
        confidence=0.9,
        lidar_distance_m=distance,
        preferred_mode=preferred_mode,
    )
    controller.observe_yolo(
        now_sec=0.1,
        detected=True,
        confidence=0.9,
        lidar_distance_m=distance,
        preferred_mode=preferred_mode,
    )


def test_yolo_tracks_before_entry_distance_then_chooses_clear_side():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(entry_distance_m=1.2)
    )
    arm(controller, distance=2.0)
    state = controller.step(
        now_sec=0.1,
        dt_sec=0.1,
        obstacle=obstacle(distance=2.0),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.YOLO_TRACKING
    assert not state.controls_vehicle

    controller.update_lidar_distance(1.0)
    state = controller.step(
        now_sec=0.2,
        dt_sec=0.1,
        obstacle=obstacle(left=1.4, right=0.4),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_LEFT
    assert state.lateral_offset_m > 0.0
    assert state.controls_vehicle


def test_left_camera_obstacle_keeps_right_avoidance_when_right_is_clear():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(entry_distance_m=1.2)
    )
    for now in (0.0, 0.1):
        controller.observe_yolo(
            now_sec=now,
            detected=True,
            confidence=0.9,
            lidar_distance_m=1.0,
            preferred_mode=YoloLidarAvoidanceMode.AVOID_RIGHT,
        )
    state = controller.step(
        now_sec=0.2,
        dt_sec=0.1,
        obstacle=obstacle(left=1.4, right=0.8),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_RIGHT


def test_left_camera_obstacle_waits_when_right_side_is_too_narrow():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(entry_distance_m=1.2)
    )
    for now in (0.0, 0.1):
        controller.observe_yolo(
            now_sec=now,
            detected=True,
            confidence=0.9,
            lidar_distance_m=1.0,
            preferred_mode=YoloLidarAvoidanceMode.AVOID_RIGHT,
        )
    state = controller.step(
        now_sec=0.2,
        dt_sec=0.1,
        obstacle=obstacle(left=1.4, right=0.5),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR


def test_lidar_lateral_position_does_not_override_yellow_reference():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(entry_distance_m=1.2)
    )
    for now in (0.0, 0.1):
        controller.observe_yolo(
            now_sec=now,
            detected=True,
            confidence=0.9,
            lidar_distance_m=1.0,
            preferred_mode=YoloLidarAvoidanceMode.AVOID_RIGHT,
        )
    state = controller.step(
        now_sec=0.2,
        dt_sec=0.1,
        obstacle=obstacle(left=1.2, right=1.2, lateral=-0.12),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_RIGHT


def test_missing_camera_side_waits_even_when_lidar_space_is_clear():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(entry_distance_m=1.2)
    )
    for now in (0.0, 0.1):
        controller.observe_yolo(
            now_sec=now,
            detected=True,
            confidence=0.9,
            lidar_distance_m=1.0,
            preferred_mode=None,
        )
    state = controller.step(
        now_sec=0.2,
        dt_sec=0.1,
        obstacle=obstacle(left=1.2, right=1.2, lateral=0.2),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR


def test_immediate_yolo_avoidance_uses_camera_preferred_side():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(immediate_on_yolo=True)
    )
    for now in (0.0, 0.1):
        controller.observe_yolo(
            now_sec=now,
            detected=True,
            confidence=0.9,
            lidar_distance_m=6.0,
            preferred_mode=YoloLidarAvoidanceMode.AVOID_LEFT,
        )
    state = controller.step(
        now_sec=0.1,
        dt_sec=0.1,
        obstacle=obstacle(left=1.2, right=1.2, distance=6.0),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_LEFT
    assert state.lateral_offset_m > 0.0
    assert state.controls_vehicle


def test_immediate_yolo_works_without_matching_lidar_return():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(
            immediate_on_yolo=True,
            preferred_side_required_frames=1,
            yolo_required_frames=1,
        )
    )
    controller.observe_yolo(
        now_sec=0.0,
        detected=True,
        confidence=0.9,
        lidar_distance_m=float("inf"),
        preferred_mode=YoloLidarAvoidanceMode.AVOID_LEFT,
    )
    assert controller.state().mode == YoloLidarAvoidanceMode.AVOID_LEFT
    state = controller.step(
        now_sec=0.0,
        dt_sec=0.1,
        obstacle=None,
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_LEFT


def test_immediate_yolo_rearms_during_return_without_distance_gate():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(
            immediate_on_yolo=True,
            preferred_side_required_frames=1,
            yolo_required_frames=1,
            minimum_avoid_sec=0.0,
            clear_hold_sec=0.0,
            return_hold_sec=1.0,
        )
    )
    controller.observe_yolo(
        now_sec=0.0,
        detected=True,
        confidence=0.9,
        lidar_distance_m=float("inf"),
        preferred_mode=YoloLidarAvoidanceMode.AVOID_RIGHT,
    )
    assert controller.state().mode == YoloLidarAvoidanceMode.AVOID_RIGHT
    controller.step(
        now_sec=1.2,
        dt_sec=0.1,
        obstacle=None,
        cone_active=False,
    )
    state = controller.step(
        now_sec=1.3,
        dt_sec=0.1,
        obstacle=None,
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.RETURN_CENTER

    controller.observe_yolo(
        now_sec=1.4,
        detected=True,
        confidence=0.9,
        lidar_distance_m=8.0,
        preferred_mode=YoloLidarAvoidanceMode.AVOID_RIGHT,
    )
    state = controller.step(
        now_sec=1.4,
        dt_sec=0.1,
        obstacle=None,
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_RIGHT


def test_immediate_yolo_does_not_wait_for_lidar_side_clearance():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(
            immediate_on_yolo=True,
            preferred_side_required_frames=1,
            yolo_required_frames=1,
        )
    )
    controller.observe_yolo(
        now_sec=0.0,
        detected=True,
        confidence=0.9,
        lidar_distance_m=0.8,
        preferred_mode=YoloLidarAvoidanceMode.AVOID_RIGHT,
    )
    state = controller.step(
        now_sec=0.0,
        dt_sec=0.1,
        obstacle=obstacle(left=0.2, right=0.2),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_RIGHT


def test_immediate_yolo_keeps_driving_until_straight_side_is_decided():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(
            immediate_on_yolo=True,
            preferred_side_required_frames=1,
            yolo_required_frames=1,
        )
    )
    controller.observe_yolo(
        now_sec=0.0,
        detected=True,
        confidence=0.9,
        lidar_distance_m=0.8,
        preferred_mode=None,
        side_decision_allowed=False,
    )
    state = controller.step(
        now_sec=0.0,
        dt_sec=0.1,
        obstacle=obstacle(left=1.2, right=1.2),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.YOLO_TRACKING
    assert not state.controls_vehicle
    assert state.lateral_offset_m == 0.0

    controller.observe_yolo(
        now_sec=0.1,
        detected=True,
        confidence=0.9,
        lidar_distance_m=0.7,
        preferred_mode=YoloLidarAvoidanceMode.AVOID_RIGHT,
        side_decision_allowed=True,
    )
    state = controller.step(
        now_sec=0.1,
        dt_sec=0.1,
        obstacle=obstacle(left=1.2, right=1.2),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_RIGHT
    assert state.controls_vehicle
    assert state.lateral_offset_m < 0.0


def test_immediate_avoidance_keeps_its_side_across_short_detection_gap():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(
            immediate_on_yolo=True,
            yolo_timeout_sec=2.5,
        )
    )
    for now in (0.0, 0.1):
        controller.observe_yolo(
            now_sec=now,
            detected=True,
            confidence=0.9,
            lidar_distance_m=6.0,
            preferred_mode=YoloLidarAvoidanceMode.AVOID_LEFT,
        )
    controller.step(
        now_sec=0.1,
        dt_sec=0.1,
        obstacle=obstacle(left=1.2, right=1.2, distance=6.0),
        cone_active=False,
    )
    state = controller.step(
        now_sec=2.2,
        dt_sec=0.1,
        obstacle=None,
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_LEFT

    controller.observe_yolo(
        now_sec=2.2,
        detected=True,
        confidence=0.9,
        lidar_distance_m=2.3,
        preferred_mode=YoloLidarAvoidanceMode.AVOID_RIGHT,
    )
    state = controller.step(
        now_sec=2.3,
        dt_sec=0.1,
        obstacle=None,
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.AVOID_LEFT


def test_both_sides_blocked_waits_instead_of_turning():
    controller = YoloLidarAvoidanceController(YoloLidarAvoidanceConfig())
    arm(controller)
    state = controller.step(
        now_sec=0.2,
        dt_sec=0.1,
        obstacle=obstacle(left=0.4, right=0.5),
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.WAIT_SIDE_CLEAR
    assert state.speed_limit_command == 0.0
    assert state.lateral_offset_m == 0.0


def test_clear_hold_returns_to_center_and_releases_to_base_policy():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(
            yolo_timeout_sec=0.2,
            minimum_avoid_sec=0.1,
            clear_hold_sec=0.2,
            return_hold_sec=0.1,
            offset_rate_mps=1.0,
        )
    )
    arm(controller)
    controller.step(
        now_sec=0.2,
        dt_sec=0.1,
        obstacle=obstacle(),
        cone_active=False,
    )
    controller.step(
        now_sec=0.4,
        dt_sec=0.1,
        obstacle=None,
        cone_active=False,
    )
    state = controller.step(
        now_sec=0.61,
        dt_sec=0.1,
        obstacle=None,
        cone_active=False,
    )
    assert state.mode == YoloLidarAvoidanceMode.RETURN_CENTER
    for index in range(10):
        state = controller.step(
            now_sec=0.72 + 0.1 * index,
            dt_sec=0.1,
            obstacle=None,
            cone_active=False,
        )
        if state.mode == YoloLidarAvoidanceMode.IDLE:
            break
    assert state.mode == YoloLidarAvoidanceMode.IDLE


def test_cone_has_priority_and_cancels_obstacle_override():
    controller = YoloLidarAvoidanceController(YoloLidarAvoidanceConfig())
    arm(controller)
    controller.step(
        now_sec=0.2,
        dt_sec=0.1,
        obstacle=obstacle(),
        cone_active=False,
    )
    state = controller.step(
        now_sec=0.3,
        dt_sec=0.1,
        obstacle=obstacle(),
        cone_active=True,
    )
    assert state.mode == YoloLidarAvoidanceMode.IDLE
    assert state.lateral_offset_m == 0.0
