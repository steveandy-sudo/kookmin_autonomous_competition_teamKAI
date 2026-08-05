from xycar_map_nav.lidar_obstacle import LidarPathObstacle
from xycar_map_nav.yolo_lidar_avoidance import (
    YoloLidarAvoidanceConfig,
    YoloLidarAvoidanceController,
    YoloLidarAvoidanceMode,
)


def obstacle(left=1.2, right=0.5, distance=1.0):
    return LidarPathObstacle(
        distance_m=distance,
        lateral_m=0.0,
        width_m=0.35,
        point_count=6,
        x_vehicle_m=distance,
        y_vehicle_m=0.0,
        left_clearance_m=left,
        right_clearance_m=right,
    )


def arm(controller, distance=1.0):
    controller.observe_yolo(
        now_sec=0.0,
        detected=True,
        confidence=0.9,
        lidar_distance_m=distance,
    )
    controller.observe_yolo(
        now_sec=0.1,
        detected=True,
        confidence=0.9,
        lidar_distance_m=distance,
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
        obstacle=None,
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
