import math
from pathlib import Path

import pytest

from my_rule.ego_cone_tracker import (
    EgoConeTracker,
    PlanarPose,
    merge_points,
    reproject_points,
    vehicle_to_world,
    world_to_vehicle,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_vehicle_world_transform_round_trip_with_sensor_offset():
    pose = PlanarPose(x=1.2, y=-0.4, yaw=math.radians(32.0))
    sensor_point = (0.85, 0.31)

    world = vehicle_to_world(
        sensor_point,
        pose,
        origin_x_offset_m=0.42,
    )
    restored = world_to_vehicle(
        world,
        pose,
        origin_x_offset_m=0.42,
    )

    assert restored == pytest.approx(sensor_point)


def test_reprojection_moves_static_cone_opposite_to_vehicle_motion():
    previous = PlanarPose(x=0.0, y=0.0, yaw=0.0)
    current = PlanarPose(x=0.20, y=0.0, yaw=0.0)

    projected = reproject_points(
        [(1.00, 0.35)],
        previous,
        current,
        origin_x_offset_m=0.42,
    )

    assert projected == pytest.approx([(0.80, 0.35)])


def test_semantic_cones_survive_only_the_bounded_prediction_window():
    tracker = EgoConeTracker(ttl_sec=0.35, sensor_x_offset_m=0.42)
    initial_pose = PlanarPose(0.0, 0.0, 0.0)
    moved_pose = PlanarPose(0.10, 0.0, 0.0)

    seeded = tracker.update(
        pose=initial_pose,
        semantic_points=[(1.0, 0.42), (1.0, -0.42)],
        lidar_points=[],
        now_sec=0.0,
    )
    predicted = tracker.update(
        pose=moved_pose,
        semantic_points=[],
        lidar_points=[],
        now_sec=0.34,
    )
    expired = tracker.update(
        pose=moved_pose,
        semantic_points=[],
        lidar_points=[],
        now_sec=0.351,
    )

    assert seeded.observed_count == 2
    assert seeded.predicted_count == 0
    assert predicted.observed_count == 0
    assert predicted.predicted_count == 2
    assert len(predicted.points) == 2
    assert predicted.points[0] == pytest.approx((0.90, 0.42))
    assert predicted.points[1] == pytest.approx((0.90, -0.42))
    assert expired.cones == ()


def test_raw_lidar_refreshes_known_track_but_cannot_seed_one():
    tracker = EgoConeTracker(ttl_sec=0.35, sensor_x_offset_m=0.42)
    pose = PlanarPose(0.0, 0.0, 0.0)

    raw_only = tracker.update(
        pose=pose,
        semantic_points=[],
        lidar_points=[(1.0, 0.4)],
        now_sec=0.0,
    )
    assert raw_only.cones == ()

    tracker.update(
        pose=pose,
        semantic_points=[(1.0, 0.4)],
        lidar_points=[],
        now_sec=0.01,
    )
    refreshed = tracker.update(
        pose=PlanarPose(0.10, 0.0, 0.0),
        semantic_points=[],
        lidar_points=[(0.90, 0.4)],
        now_sec=0.30,
    )
    still_alive = tracker.update(
        pose=PlanarPose(0.20, 0.0, 0.0),
        semantic_points=[],
        lidar_points=[],
        now_sec=0.60,
    )

    assert refreshed.observed_count == 1
    assert refreshed.predicted_count == 0
    assert still_alive.predicted_count == 1


def test_large_odom_jump_clears_old_landmarks_before_reseeding():
    tracker = EgoConeTracker(
        ttl_sec=0.35,
        maximum_pose_jump_m=0.30,
        maximum_pose_jump_yaw_deg=45.0,
    )
    tracker.update(
        pose=PlanarPose(0.0, 0.0, 0.0),
        semantic_points=[(1.0, 0.4)],
        lidar_points=[],
        now_sec=0.0,
    )

    jumped = tracker.update(
        pose=PlanarPose(1.0, 0.0, 0.0),
        semantic_points=[],
        lidar_points=[],
        now_sec=0.1,
    )

    assert jumped.pose_jump_reset
    assert jumped.cones == ()


def test_merge_points_keeps_fresh_measurement_over_nearby_prediction():
    merged = merge_points(
        [(0.80, 0.40)],
        [(0.82, 0.41), (1.20, -0.40)],
        minimum_separation_m=0.08,
    )

    assert len(merged) == 2
    assert merged[0] == pytest.approx((0.80, 0.40))
    assert merged[1] == pytest.approx((1.20, -0.40))


def test_real_config_enables_odom_tracking_without_fixed_turn_guard():
    config = (PACKAGE_ROOT / "config" / "cone_control.yaml").read_text(
        encoding="utf-8"
    )

    assert "ego_cone_tracking_enabled: true" in config
    assert "ego_cone_track_ttl_sec: 0.30" in config
    assert "ego_cone_predicted_speed: 4.0" in config
    assert "cone_turn_sequence_guard_enabled: false" in config
