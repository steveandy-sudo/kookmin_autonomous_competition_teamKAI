import numpy as np

from my_lane.bev_path_centerline_node import (
    bev_pixels_to_metric_path,
)


def convert(values):
    return bev_pixels_to_metric_path(
        values,
        bev_width=640,
        bev_height=480,
        lateral_range_m=1.4,
        forward_range_m=1.5,
        vehicle_x_px=320.0,
        vehicle_y_px=479.0,
    )


def test_center_pixels_convert_to_zero_lateral_and_sorted_forward():
    path = convert([320.0, 400.0, 320.0, 300.0, 320.0, 200.0])
    assert path.shape == (3, 2)
    assert np.allclose(path[:, 1], 0.0)
    assert np.all(np.diff(path[:, 0]) > 0.0)


def test_left_pixels_are_positive_vehicle_lateral():
    path = convert([300.0, 400.0, 300.0, 300.0, 300.0, 200.0])
    assert np.all(path[:, 1] > 0.0)


def test_invalid_pixel_array_returns_empty_path():
    assert convert([1.0, 2.0, 3.0]).shape == (0, 2)
