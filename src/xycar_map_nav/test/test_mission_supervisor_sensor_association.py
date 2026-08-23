import math

from xycar_map_nav.mission_supervisor import lidar_target_matches_camera_sector
from xycar_map_nav.mission_supervisor import scan_sector_distance


def test_vehicle_target_must_match_camera_angle_and_distance():
    assert lidar_target_matches_camera_sector(
        target_x_m=1.0,
        target_y_m=0.1,
        camera_sector=(-0.15, 0.15),
        camera_sector_distance_m=1.0,
        distance_tolerance_m=0.20,
    )
    assert not lidar_target_matches_camera_sector(
        target_x_m=1.0,
        target_y_m=0.5,
        camera_sector=(-0.15, 0.15),
        camera_sector_distance_m=1.0,
        distance_tolerance_m=0.20,
    )
    assert not lidar_target_matches_camera_sector(
        target_x_m=1.5,
        target_y_m=0.0,
        camera_sector=(-0.15, 0.15),
        camera_sector_distance_m=1.0,
        distance_tolerance_m=0.20,
    )


def test_traffic_light_sector_always_vetoes_vehicle_association():
    assert not lidar_target_matches_camera_sector(
        target_x_m=1.0,
        target_y_m=0.0,
        camera_sector=(-0.20, 0.20),
        camera_sector_distance_m=1.0,
        excluded_sectors=((-0.05, 0.05),),
    )


def test_sector_distance_ignores_traffic_light_rays():
    angle_increment = math.radians(1.0)
    ranges = [3.0] * 21
    ranges[9] = 0.7
    ranges[10] = 0.7
    ranges[14] = 1.2
    ranges[15] = 1.2
    distance = scan_sector_distance(
        ranges=ranges,
        angle_min=math.radians(-10.0),
        angle_increment=angle_increment,
        range_min=0.1,
        range_max=5.0,
        sector_min_angle=math.radians(-5.0),
        sector_max_angle=math.radians(5.0),
        minimum_points=2,
        excluded_sectors=(
            (math.radians(-2.0), math.radians(2.0)),
        ),
    )
    assert distance == 1.2
