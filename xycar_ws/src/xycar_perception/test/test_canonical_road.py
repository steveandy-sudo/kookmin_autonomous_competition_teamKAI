import unittest
from pathlib import Path

import cv2
import numpy as np
import yaml

from xycar_perception.canonical_road import make_canonical_road_image


class CanonicalRoadTest(unittest.TestCase):
    def make_bev(self, height, white_value, white_width, yellow_value, yellow_width):
        image = np.full((height, 640, 3), 70, dtype=np.uint8)
        start_y = max(0, height - 115)
        cv2.line(
            image,
            (150, start_y),
            (150, height - 12),
            (white_value, white_value, white_value),
            white_width,
        )
        cv2.line(
            image,
            (490, start_y),
            (490, height - 12),
            (white_value, white_value, white_value),
            white_width,
        )
        for y in range(start_y, height - 12, 24):
            cv2.line(
                image,
                (320, y),
                (320, min(y + 12, height - 12)),
                (0, yellow_value, yellow_value),
                yellow_width,
            )
        return image

    def canonical(self, image):
        return make_canonical_road_image(
            image,
            lateral_m_per_px=0.0022,
            forward_m_per_px=0.01,
            lateral_range_m=1.4,
            forward_range_m=1.2,
            output_width=256,
            output_height=144,
            line_width_px=5,
            bottom_ignore_m=0.08,
        )

    def test_different_bev_ranges_colors_and_widths_share_representation(self):
        sim = self.make_bev(220, white_value=255, white_width=10, yellow_value=255, yellow_width=9)
        real = self.make_bev(120, white_value=125, white_width=4, yellow_value=150, yellow_width=4)

        sim_image, sim_white, sim_yellow = self.canonical(sim)
        real_image, real_white, real_yellow = self.canonical(real)

        self.assertEqual(sim_image.shape, (144, 256, 3))
        white_union = np.count_nonzero((sim_white > 0) | (real_white > 0))
        white_intersection = np.count_nonzero((sim_white > 0) & (real_white > 0))
        yellow_union = np.count_nonzero((sim_yellow > 0) | (real_yellow > 0))
        yellow_intersection = np.count_nonzero((sim_yellow > 0) & (real_yellow > 0))
        self.assertGreater(white_intersection / white_union, 0.90)
        self.assertGreater(yellow_intersection / yellow_union, 0.65)
        self.assertLess(
            abs(np.count_nonzero(sim_yellow) - np.count_nonzero(real_yellow))
            / np.count_nonzero(sim_yellow),
            0.20,
        )

    def test_canonical_colors_are_fixed(self):
        image, _, _ = self.canonical(
            self.make_bev(120, white_value=160, white_width=5, yellow_value=170, yellow_width=5)
        )
        colors = {tuple(pixel) for pixel in image.reshape(-1, 3)}
        self.assertTrue(colors.issubset({(36, 36, 36), (255, 255, 255), (0, 220, 255)}))

    def test_invalid_bev_fill_edge_does_not_become_a_white_lane(self):
        image = np.full((120, 640, 3), 95, dtype=np.uint8)
        valid = np.full((120, 640), 255, dtype=np.uint8)
        polygon = np.array([[0, 120], [0, 75], [90, 120]], dtype=np.int32)
        cv2.fillPoly(image, [polygon], (70, 70, 70))
        cv2.fillPoly(valid, [polygon], 0)
        valid = cv2.erode(valid, np.ones((5, 5), dtype=np.uint8))

        _, white, yellow = make_canonical_road_image(
            image,
            valid_mask=valid,
            lateral_m_per_px=0.0022,
            forward_m_per_px=0.01,
            lateral_range_m=1.4,
            forward_range_m=1.2,
            output_width=256,
            output_height=144,
            line_width_px=5,
            bottom_ignore_m=0.0,
        )

        self.assertEqual(int(np.count_nonzero(white)), 0)
        self.assertEqual(int(np.count_nonzero(yellow)), 0)

    def test_real_config_matches_measured_lane_pipeline(self):
        config_path = Path(__file__).resolve().parents[1] / "config" / "camera_perception_real.yaml"
        with config_path.open(encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)

        params = config["xycar_camera_perception"]["ros__parameters"]
        self.assertEqual(params["image_topic"], "/wide_camera/rect/image_raw")
        self.assertFalse(params["enable_rectify"])
        self.assertEqual(params["publish_rate_limit_hz"], 0.0)
        self.assertEqual(params["bev_width"], 640)
        self.assertEqual(params["bev_height"], 220)
        self.assertAlmostEqual(params["src_tl_x_ratio"], 0.357)
        self.assertAlmostEqual(params["src_tr_x_ratio"], 0.777)
        self.assertAlmostEqual(params["src_bl_x_ratio"], 0.170)
        self.assertAlmostEqual(params["src_br_x_ratio"], 0.965)
        self.assertAlmostEqual(params["src_top_y_ratio"], 0.520)
        self.assertAlmostEqual(params["src_bottom_y_ratio"], 0.625)
        self.assertAlmostEqual(params["lateral_m_per_px"], 0.39 / 256.0)
        self.assertEqual(params["white_s_max"], 120)
        self.assertEqual(params["white_v_min"], 145)
        self.assertEqual(params["yellow_h_min"], 14)
        self.assertEqual(params["yellow_h_max"], 45)
        self.assertEqual(params["yellow_s_min"], 60)
        self.assertEqual(params["yellow_v_min"], 80)

    def test_sim_config_does_not_apply_real_fisheye_rectification(self):
        config_path = Path(__file__).resolve().parents[1] / "config" / "camera_perception.yaml"
        with config_path.open(encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)

        params = config["xycar_camera_perception"]["ros__parameters"]
        self.assertEqual(params["image_topic"], "/image_raw")
        self.assertFalse(params["enable_rectify"])
        self.assertEqual(params["publish_rate_limit_hz"], 0.0)


if __name__ == "__main__":
    unittest.main()
