import math

from xycar_map_nav.lidar_obstacle import (
    detect_path_obstacle,
    forward_route_distance,
    LidarBypassConfig,
    LidarObstacleBypassRule,
    LidarPathObstacle,
    LidarPathObstacleConfig,
)


def straight_route(count=61, spacing=0.05):
    return tuple((index * spacing, 0.0) for index in range(count))


def scan_with_points(points):
    ranges = [float("inf")] * 181
    for x, y in points:
        angle = math.atan2(y, x)
        index = int(round(math.degrees(angle))) + 90
        ranges[index] = math.hypot(x, y)
    return ranges


def detect(ranges, config=None):
    return detect_path_obstacle(
        ranges=ranges,
        angle_min=math.radians(-90.0),
        angle_increment=math.radians(1.0),
        range_min=0.05,
        range_max=8.0,
        route_points=straight_route(),
        nearest_index=0,
        vehicle_x=0.0,
        vehicle_y=0.0,
        vehicle_yaw=0.0,
        closed=False,
        config=config or LidarPathObstacleConfig(),
    )


def obstacle(lateral=0.0, left_clearance=1.5, right_clearance=1.5):
    return LidarPathObstacle(
        distance_m=0.90,
        lateral_m=lateral,
        width_m=0.20,
        point_count=7,
        x_vehicle_m=0.90,
        y_vehicle_m=lateral,
        left_clearance_m=left_clearance,
        right_clearance_m=right_clearance,
    )


def test_detects_multi_ray_obstacle_on_global_path():
    points = [
        (0.90, 0.90 * math.tan(math.radians(angle)))
        for angle in range(-5, 6)
    ]

    result = detect(scan_with_points(points))

    assert result is not None
    assert result.point_count >= 5
    assert 0.80 <= result.distance_m <= 1.0
    assert abs(result.lateral_m) < 0.03
    assert result.width_m >= 0.09


def test_rejects_single_lidar_ray_and_side_wall():
    assert detect(scan_with_points([(0.90, 0.0)])) is None

    side_points = [
        (0.80 + 0.03 * index, 0.35)
        for index in range(7)
    ]
    assert detect(scan_with_points(side_points)) is None


def test_obstacle_on_left_selects_right_bypass():
    rule = LidarObstacleBypassRule(
        LidarBypassConfig(required_frames=2, offset_rate_mps=1.0)
    )
    route = straight_route(count=80, spacing=0.10)

    first = rule.update(
        now_sec=0.0,
        observation=obstacle(lateral=0.10),
        path_index=0,
        route_points=route,
        closed=False,
        enabled=True,
    )
    second = rule.update(
        now_sec=0.1,
        observation=obstacle(lateral=0.10),
        path_index=0,
        route_points=route,
        closed=False,
        enabled=True,
    )

    assert first.mode == "NORMAL"
    assert second.mode == "BYPASS_RIGHT"
    assert second.lateral_offset_m < 0.0


def test_centered_obstacle_uses_clearer_side():
    rule = LidarObstacleBypassRule(
        LidarBypassConfig(required_frames=1, offset_rate_mps=1.0)
    )

    state = rule.update(
        now_sec=0.1,
        observation=obstacle(
            lateral=0.0,
            left_clearance=1.5,
            right_clearance=0.6,
        ),
        path_index=0,
        route_points=straight_route(),
        closed=False,
        enabled=True,
    )

    assert state.mode == "BYPASS_LEFT"


def test_bypass_memory_holds_until_route_progress_passes_obstacle():
    rule = LidarObstacleBypassRule(
        LidarBypassConfig(
            required_frames=1,
            offset_rate_mps=1.0,
            clear_hold_sec=0.25,
        )
    )
    route = straight_route(count=80, spacing=0.10)
    rule.update(
        now_sec=0.0,
        observation=obstacle(),
        path_index=0,
        route_points=route,
        closed=False,
        enabled=True,
    )

    before = rule.update(
        now_sec=0.4,
        observation=None,
        path_index=10,
        route_points=route,
        closed=False,
        enabled=True,
    )
    after = rule.update(
        now_sec=0.5,
        observation=None,
        path_index=18,
        route_points=route,
        closed=False,
        enabled=True,
    )

    assert before.mode.startswith("BYPASS")
    assert after.mode == "RETURN_CENTER"


def test_forward_route_distance_handles_closed_route_wrap():
    route = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))

    assert math.isclose(
        forward_route_distance(route, 3, 1, closed=True),
        2.0,
    )
    assert forward_route_distance(route, 2, 1, closed=False) == 0.0
