import unittest

import cv2
import numpy as np

from xycar_perception.canonical_road import (
    make_canonical_road_image_from_masks,
)
from xycar_perception.yolo_lane_segmenter import merge_lane_instance_masks


class YoloCanonicalTest(unittest.TestCase):
    def test_instance_masks_merge_by_lane_class(self):
        masks = np.zeros((3, 20, 30), dtype=np.float32)
        masks[0, 2:18, 3:6] = 1.0
        masks[1, 4:16, 14:17] = 1.0
        masks[2, 2:18, 24:27] = 1.0

        white, yellow = merge_lane_instance_masks(
            masks,
            np.array([0, 1, 0], dtype=np.float32),
            (40, 60),
        )

        self.assertEqual(white.shape, (40, 60))
        self.assertGreater(int(np.count_nonzero(white)), 0)
        self.assertGreater(int(np.count_nonzero(yellow)), 0)
        self.assertEqual(int(np.count_nonzero(white & yellow)), 0)

    def test_binary_masks_produce_fixed_canonical_contract(self):
        white = np.zeros((220, 640), dtype=np.uint8)
        yellow = np.zeros_like(white)
        cv2.line(white, (130, 0), (180, 219), 255, 8)
        cv2.line(white, (510, 0), (460, 219), 255, 8)
        for row in range(20, 210, 45):
            cv2.line(yellow, (320, row), (320, row + 20), 255, 7)

        canonical, canonical_white, canonical_yellow = (
            make_canonical_road_image_from_masks(
                white,
                yellow,
                lateral_m_per_px=1.4 / 640.0,
                forward_m_per_px=1.5 / 220.0,
                lateral_range_m=1.4,
                forward_range_m=1.5,
                output_width=256,
                output_height=144,
                line_width_px=5,
                bottom_ignore_m=0.0,
            )
        )

        colors = {tuple(pixel) for pixel in canonical.reshape(-1, 3)}
        self.assertTrue(
            colors.issubset(
                {(36, 36, 36), (255, 255, 255), (0, 220, 255)}
            )
        )
        self.assertGreater(int(np.count_nonzero(canonical_white)), 0)
        self.assertGreater(int(np.count_nonzero(canonical_yellow)), 0)
        self.assertEqual(
            int(np.count_nonzero(canonical_white & canonical_yellow)), 0
        )

    def test_priority_profile_strictly_normalizes_thick_yolo_masks(self):
        white = np.zeros((220, 640), dtype=np.uint8)
        yellow = np.zeros_like(white)
        cv2.line(white, (500, 0), (440, 219), 255, 42)
        cv2.line(yellow, (330, 15), (300, 205), 255, 34)

        _, filtered_white, filtered_yellow = make_canonical_road_image_from_masks(
            white,
            yellow,
            lateral_m_per_px=1.4 / 640.0,
            forward_m_per_px=1.5 / 220.0,
            white_max_component_thickness_px=28.0,
            yellow_max_component_thickness_px=24.0,
            geometry_filter_enabled=True,
            bottom_ignore_m=0.0,
        )
        _, priority_white, priority_yellow = make_canonical_road_image_from_masks(
            white,
            yellow,
            lateral_m_per_px=1.4 / 640.0,
            forward_m_per_px=1.5 / 220.0,
            white_max_component_thickness_px=0.0,
            yellow_max_component_thickness_px=0.0,
            geometry_filter_enabled=True,
            white_min_line_span_px=14,
            yellow_min_line_span_px=7,
            min_line_elongation=1.8,
            min_line_verticality=0.30,
            white_max_components_per_side=3,
            yellow_max_components=5,
            white_max_mask_fraction=0.08,
            yellow_max_mask_fraction=0.06,
            max_line_fit_rmse_px=4.0,
            bottom_ignore_m=0.0,
        )

        self.assertEqual(int(np.count_nonzero(filtered_white)), 0)
        self.assertEqual(int(np.count_nonzero(filtered_yellow)), 0)
        self.assertGreater(int(np.count_nonzero(priority_white)), 0)
        self.assertGreater(int(np.count_nonzero(priority_yellow)), 0)
        self.assertLess(int(np.count_nonzero(priority_white)), 1000)
        self.assertLess(int(np.count_nonzero(priority_yellow)), 1000)


if __name__ == "__main__":
    unittest.main()
