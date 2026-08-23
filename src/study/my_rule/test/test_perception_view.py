import numpy as np

from my_rule.perception_view_node import (
    infer_drive_mode,
    metric_to_topdown_pixels,
    project_laser_xy,
    rear_axle_to_laser,
    roi_points,
)


def test_auto_mode_follows_drive_manager_state():
    assert infer_drive_mode("LANE_FOLLOW / cone_mode=0") == "lane"
    assert infer_drive_mode("CONE_SLALOM / cone_mode=1") == "cone"
    assert infer_drive_mode("CONE_RECOVERY / waiting") == "cone"


def test_explicit_mode_overrides_state():
    assert infer_drive_mode("CONE_SLALOM", "lane") == "lane"
    assert infer_drive_mode("LANE_FOLLOW", "cone") == "cone"


def test_roi_ratios_scale_to_image():
    ratios = [0.25, 0.50, 0.75, 0.50, 1.0, 1.0, 0.0, 1.0]
    points = roi_points(1280, 1024, ratios)
    assert points.tolist() == [
        [320, 512],
        [960, 512],
        [1280, 1024],
        [0, 1024],
    ]


def test_topdown_coordinates_are_forward_up_and_left_is_image_left():
    pixels = metric_to_topdown_pixels(
        [(0.0, 0.0), (2.0, 0.0), (1.0, 0.5)],
        width=400,
        height=300,
        forward_range_m=2.0,
        lateral_range_m=2.0,
    )
    assert pixels[0].tolist() == [200, 299]
    assert pixels[1].tolist() == [200, 0]
    assert pixels[2, 0] < 200


def test_rear_axle_path_is_shifted_into_laser_frame():
    points = rear_axle_to_laser([(0.42, 0.0), (1.42, -0.2)], 0.42)
    assert np.allclose(points, [[0.0, 0.0], [1.0, -0.2]])


def test_rectified_projection_uses_positive_camera_depth():
    points = np.asarray([(1.0, 0.0), (-1.0, 0.0)])
    # Map laser X to camera optical Z and laser Y to camera optical X.
    rotation = np.asarray(
        [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ]
    )
    matrix = np.asarray(
        [
            [100.0, 0.0, 320.0],
            [0.0, 100.0, 240.0],
            [0.0, 0.0, 1.0],
        ]
    )
    pixels, valid = project_laser_xy(
        points,
        rotation,
        np.zeros(3),
        matrix,
    )
    assert valid.tolist() == [True, False]
    assert np.allclose(pixels[0], [320.0, 240.0])
