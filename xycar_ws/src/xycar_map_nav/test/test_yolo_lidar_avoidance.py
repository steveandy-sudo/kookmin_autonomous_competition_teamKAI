from xycar_map_nav.lidar_obstacle import LidarPathObstacle
from xycar_map_nav.yolo_lidar_avoidance import (
    YoloLidarAvoidanceConfig,
    YoloLidarAvoidanceController,
    YoloLidarAvoidanceMode,
)
from xycar_map_nav.sequential_hybrid_driver import is_avoidance_detection
from xycar_map_nav.sequential_hybrid_driver import (
    preferred_avoidance_mode_from_yellow_reference,
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


def test_immediate_yolo_waits_when_lidar_has_no_matching_return():
    controller = YoloLidarAvoidanceController(
        YoloLidarAvoidanceConfig(immediate_on_yolo=True)
    )
    for now in (0.0, 0.1):
        controller.observe_yolo(
            now_sec=now,
            detected=True,
            confidence=0.9,
            lidar_distance_m=float("inf"),
            preferred_mode=YoloLidarAvoidanceMode.AVOID_LEFT,
        )
    assert controller.state().mode == YoloLidarAvoidanceMode.YOLO_TRACKING


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
