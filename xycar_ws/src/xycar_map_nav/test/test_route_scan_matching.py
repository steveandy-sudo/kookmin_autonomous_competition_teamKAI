import math

import cv2
import numpy as np

from xycar_map_nav.route_scan_matching import (
    LikelihoodField,
    MatchConfig,
    load_likelihood_field,
    match_scan_to_route,
    scan_points_in_base,
)


def _field_with_endpoint_patterns(
    scan_points,
    pose_x_values,
    *,
    resolution=0.05,
):
    height = 240
    width = 240
    origin_x = -6.0
    origin_y = -6.0
    occupied = np.zeros((height, width), dtype=bool)
    for pose_x in pose_x_values:
        for point_x, point_y in scan_points:
            world_x = pose_x + point_x
            world_y = point_y
            column = int(math.floor((world_x - origin_x) / resolution))
            bottom_row = int(
                math.floor((world_y - origin_y) / resolution)
            )
            row = height - 1 - bottom_row
            if 0 <= row < height and 0 <= column < width:
                occupied[row, column] = True
    distance = cv2.distanceTransform(
        np.logical_not(occupied).astype(np.uint8),
        cv2.DIST_L2,
        5,
    )
    return LikelihoodField(
        distance_m=distance * resolution,
        known=np.ones((height, width), dtype=bool),
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=0.0,
    )


def _asymmetric_scan():
    upper = [(0.6 + index * 0.04, 1.0) for index in range(28)]
    lower = [(0.5 + index * 0.05, -1.4) for index in range(24)]
    diagonal = [
        (1.8 + index * 0.03, -0.4 + index * 0.025)
        for index in range(20)
    ]
    return np.asarray(upper + lower + diagonal, dtype=np.float32)


def _config():
    return MatchConfig(
        route_sample_spacing_m=0.20,
        lateral_search_m=0.20,
        lateral_step_m=0.10,
        yaw_search_rad=math.radians(10.0),
        yaw_step_rad=math.radians(5.0),
        refinement_xy_m=0.10,
        refinement_xy_step_m=0.05,
        refinement_yaw_rad=math.radians(3.0),
        refinement_yaw_step_rad=math.radians(1.0),
        minimum_inlier_ratio=0.70,
        maximum_mean_distance_m=0.10,
        minimum_score_margin=0.05,
        ambiguity_separation_m=1.0,
    )


def test_trinary_gray_pixel_is_unknown_for_scan_matching(tmp_path):
    image_path = tmp_path / "map.pgm"
    cv2.imwrite(
        str(image_path),
        np.asarray([[0, 205, 254]], dtype=np.uint8),
    )
    yaml_path = tmp_path / "map.yaml"
    yaml_path.write_text(
        "image: map.pgm\n"
        "mode: trinary\n"
        "resolution: 0.05\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )

    field = load_likelihood_field(yaml_path)

    assert field.known.tolist() == [[True, False, True]]


def test_unique_route_location_is_accepted():
    scan_points = _asymmetric_scan()
    field = _field_with_endpoint_patterns(scan_points, [1.2])
    route = [(value, 0.0) for value in np.linspace(-4.0, 4.0, 81)]

    match = match_scan_to_route(
        field,
        route,
        scan_points,
        closed=False,
        config=_config(),
    )

    assert match.accepted
    assert abs(match.x - 1.2) <= 0.10
    assert abs(match.y) <= 0.10
    assert abs(match.yaw) <= math.radians(3.0)
    assert match.inlier_ratio >= 0.90


def test_repeated_route_pattern_is_rejected_as_ambiguous():
    scan_points = _asymmetric_scan()
    field = _field_with_endpoint_patterns(scan_points, [-2.0, 2.0])
    route = [(value, 0.0) for value in np.linspace(-4.0, 4.0, 81)]

    match = match_scan_to_route(
        field,
        route,
        scan_points,
        closed=False,
        config=_config(),
    )

    assert not match.accepted
    assert match.score_margin < 0.05


def test_scan_points_apply_laser_extrinsic():
    points = scan_points_in_base(
        [1.0, float("inf"), 2.0],
        angle_min=0.0,
        angle_increment=math.pi / 2.0,
        range_min=0.1,
        range_max=3.0,
        laser_x=0.1,
        laser_y=0.2,
        laser_yaw=math.pi / 2.0,
        maximum_points=10,
    )

    assert points.shape == (2, 2)
    assert np.allclose(points[0], [0.1, 1.2], atol=1.0e-6)
    assert np.allclose(points[1], [0.1, -1.8], atol=1.0e-6)
