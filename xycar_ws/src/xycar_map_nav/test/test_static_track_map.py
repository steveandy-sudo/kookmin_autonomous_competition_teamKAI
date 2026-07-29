import numpy as np

from xycar_map_nav.static_track_map_node import (
    display_occupancy_from_image,
)


def test_track_lines_are_darker_than_the_background_in_rviz():
    image = np.full((2, 3, 3), 96, dtype=np.uint8)
    image[0, 1] = 255

    occupancy = display_occupancy_from_image(image)

    assert occupancy.shape == (2, 3)
    assert occupancy[0, 0] == 18
    assert occupancy[0, 1] == 88
