import unittest

import cv2
import numpy as np

from lane_seg_control.bag_montage_exporter import (
    make_montage,
    nearest_motor_command,
)
from lane_seg_control.canonical_adapter_node import (
    build_bev_geometry,
    warp_semantic_masks,
)
from lane_seg_control.lane_seg_inference_node import (
    class_roles,
    merge_instance_masks,
)
from lane_seg_control.lraspp_inference_node import (
    masks_from_probabilities,
    prepare_model_input,
)
from lane_seg_control.white_lane_fitter import (
    compose_fitted_canonical,
    fit_white_lane_boundaries,
    fit_yellow_centerline_reference,
    render_white_lane_fit_debug,
)
from xycar_perception.canonical_road import make_canonical_road_image_from_masks


class LaneSegIntegrationTest(unittest.TestCase):
    def test_bag_montage_has_three_fixed_width_panels(self):
        rectified = np.zeros((1024, 1280, 3), dtype=np.uint8)
        bev = np.zeros((660, 640, 3), dtype=np.uint8)
        canonical = np.zeros((144, 256, 3), dtype=np.uint8)
        montage = make_montage(rectified, bev, canonical)
        self.assertEqual(montage.shape, (512, 1920, 3))

    def test_bag_montage_adds_canonical_fit_debug_panel(self):
        rectified = np.zeros((1024, 1280, 3), dtype=np.uint8)
        bev = np.zeros((660, 640, 3), dtype=np.uint8)
        canonical = np.zeros((144, 256, 3), dtype=np.uint8)
        montage = make_montage(rectified, bev, canonical, canonical)
        self.assertEqual(montage.shape, (512, 2560, 3))

    def test_bag_montage_adds_command_annotation_banner(self):
        rectified = np.zeros((1024, 1280, 3), dtype=np.uint8)
        bev = np.zeros((660, 640, 3), dtype=np.uint8)
        canonical = np.zeros((144, 256, 3), dtype=np.uint8)
        montage = make_montage(
            rectified,
            bev,
            canonical,
            canonical,
            "REC_CMD steer=+1.0 | RL raw steer=-2.0",
        )
        self.assertEqual(montage.shape, (556, 2560, 3))

    def test_nearest_motor_command_honors_timestamp_tolerance(self):
        stamps = np.asarray([100, 200, 300], dtype=np.int64)
        commands = np.asarray([[1, 3], [2, 4], [3, 5]], dtype=np.float32)
        command = nearest_motor_command(212, stamps, commands, 20)
        self.assertEqual(command, (2.0, 4.0, -0.000012))
        self.assertIsNone(nearest_motor_command(250, stamps, commands, 20))

    def test_canonical_white_fit_preserves_yellow(self):
        height, width = 144, 256
        white = np.zeros((height, width), dtype=np.uint8)
        yellow = np.zeros_like(white)
        for y_start, y_end in ((6, 30), (38, 64), (72, 98), (106, 140)):
            points_left = []
            points_right = []
            for y in range(y_start, y_end):
                left_x = int(round(54.0 + 0.0018 * (y - 72.0) ** 2))
                right_x = int(round(198.0 - 0.10 * (y - 72.0)))
                points_left.append((left_x, y))
                points_right.append((right_x, y))
            cv2.polylines(
                white,
                [np.asarray(points_left, dtype=np.int32)],
                False,
                255,
                5,
            )
            cv2.polylines(
                white,
                [np.asarray(points_right, dtype=np.int32)],
                False,
                255,
                5,
            )
        cv2.line(yellow, (126, 10), (130, 134), 255, 3)
        yellow_before = yellow.copy()

        reference = fit_yellow_centerline_reference(yellow)
        result = fit_white_lane_boundaries(
            white,
            divider_x_by_y=reference.x_by_y,
        )
        self.assertTrue(result.left.valid)
        self.assertTrue(result.right.valid)
        self.assertEqual(result.left.degree, 2)
        self.assertEqual(result.right.degree, 1)

        road, fitted_white = compose_fitted_canonical(
            result.mask,
            yellow,
            np.full_like(white, 255),
        )
        np.testing.assert_array_equal(yellow, yellow_before)
        self.assertGreater(np.count_nonzero(fitted_white), 0)
        self.assertTrue(np.all(road[yellow > 0] == (0, 220, 255)))

        debug = render_white_lane_fit_debug(
            np.full((height, width, 3), 36, dtype=np.uint8),
            white,
            yellow,
            result,
        )
        self.assertEqual(debug.shape, (height, width, 3))

    def test_yellow_reference_classifies_white_on_each_side(self):
        height, width = 144, 256
        white = np.zeros((height, width), dtype=np.uint8)
        yellow = np.zeros_like(white)
        cv2.line(white, (30, 138), (30, 6), 255, 5)
        cv2.line(white, (112, 138), (112, 6), 255, 5)
        cv2.line(yellow, (70, 50), (84, 90), 255, 5)

        reference = fit_yellow_centerline_reference(yellow)
        self.assertTrue(reference.valid)
        self.assertEqual(reference.component_count, 1)
        self.assertGreater(np.count_nonzero(reference.mask[0]), 0)
        self.assertGreater(np.count_nonzero(reference.mask[-1]), 0)

        result = fit_white_lane_boundaries(
            white,
            divider_x_by_y=reference.x_by_y,
        )
        self.assertTrue(result.left.valid)
        self.assertTrue(result.right.valid)
        self.assertLess(float(np.median(result.left.centers[:, 0])), 60.0)
        self.assertGreater(float(np.median(result.right.centers[:, 0])), 100.0)
        self.assertLess(float(np.median(result.right.centers[:, 0])), 128.0)

    def test_multiple_yellow_dashes_form_one_full_height_line(self):
        yellow = np.zeros((144, 256), dtype=np.uint8)
        cv2.line(yellow, (118, 20), (120, 45), 255, 4)
        cv2.line(yellow, (124, 80), (127, 110), 255, 4)

        reference = fit_yellow_centerline_reference(yellow)
        self.assertTrue(reference.valid)
        self.assertEqual(reference.component_count, 2)
        self.assertTrue(np.all(np.count_nonzero(reference.mask, axis=1) > 0))
        delta_x = np.diff(reference.x_by_y)
        np.testing.assert_allclose(delta_x, delta_x[0], atol=1e-4)

        road, _ = compose_fitted_canonical(
            np.zeros_like(yellow),
            yellow,
            np.full_like(yellow, 255),
        )
        self.assertEqual(np.count_nonzero(road[0] == (0, 220, 255)), 0)
        self.assertGreater(np.count_nonzero(reference.mask[0]), 0)

    def test_missing_yellow_makes_exactly_one_white_lane(self):
        for expected_side, x in (("left", 48), ("right", 202)):
            with self.subTest(expected_side=expected_side):
                white = np.zeros((144, 256), dtype=np.uint8)
                cv2.line(white, (x, 138), (x, 6), 255, 5)
                result = fit_white_lane_boundaries(white)
                selected = (
                    result.left if expected_side == "left" else result.right
                )
                other = (
                    result.right if expected_side == "left" else result.left
                )
                self.assertTrue(selected.valid)
                self.assertFalse(other.valid)
                self.assertEqual(
                    cv2.connectedComponents(result.mask, connectivity=8)[0] - 1,
                    1,
                )

    def test_sparse_one_sided_white_becomes_one_extended_right_line(self):
        white = np.zeros((144, 256), dtype=np.uint8)
        yellow = np.zeros_like(white)
        cv2.line(yellow, (80, 55), (80, 90), 255, 5)
        cv2.line(white, (108, 68), (110, 77), 255, 5)
        cv2.line(white, (114, 94), (116, 103), 255, 5)
        reference = fit_yellow_centerline_reference(yellow)

        result = fit_white_lane_boundaries(
            white,
            divider_x_by_y=reference.x_by_y,
        )
        self.assertFalse(result.left.valid)
        self.assertTrue(result.right.valid)
        self.assertEqual(result.right.degree, 1)
        self.assertGreater(np.count_nonzero(result.mask[0]), 0)
        components = cv2.connectedComponents(result.mask, connectivity=8)[0] - 1
        self.assertEqual(components, 1)

    def test_close_sparse_fragments_fit_one_extended_line_on_either_side(self):
        height, width = 144, 256
        yellow = np.zeros((height, width), dtype=np.uint8)
        cv2.line(yellow, (128, 45), (128, 100), 255, 4)
        reference = fit_yellow_centerline_reference(yellow)

        for side, x in (("left", 52), ("right", 204)):
            with self.subTest(side=side):
                white = np.zeros((height, width), dtype=np.uint8)
                cv2.line(white, (x, 123), (x + 1, 127), 255, 4)
                cv2.line(white, (x + 2, 129), (x + 3, 135), 255, 4)
                result = fit_white_lane_boundaries(
                    white,
                    divider_x_by_y=reference.x_by_y,
                )
                selected = result.left if side == "left" else result.right
                other = result.right if side == "left" else result.left
                self.assertTrue(selected.valid)
                self.assertTrue(selected.used_fallback)
                self.assertFalse(other.valid)
                self.assertGreater(np.count_nonzero(result.mask[0]), 0)
                components = (
                    cv2.connectedComponents(result.mask, connectivity=8)[0] - 1
                )
                self.assertEqual(components, 1)

    def test_detached_same_side_component_contributes_to_single_line(self):
        height, width = 144, 256
        yellow = np.zeros((height, width), dtype=np.uint8)
        cv2.line(yellow, (128, 30), (128, 138), 255, 4)
        reference = fit_yellow_centerline_reference(yellow)

        for side, upper_x, lower_x in (
            ("left", 76, 38),
            ("right", 180, 218),
        ):
            with self.subTest(side=side):
                white = np.zeros((height, width), dtype=np.uint8)
                cv2.line(white, (upper_x, 18), (upper_x, 72), 255, 5)
                cv2.line(white, (lower_x, 112), (lower_x, 124), 255, 5)
                result = fit_white_lane_boundaries(
                    white,
                    divider_x_by_y=reference.x_by_y,
                )
                selected = result.left if side == "left" else result.right
                other = result.right if side == "left" else result.left
                self.assertTrue(selected.valid)
                self.assertFalse(other.valid)
                self.assertEqual(selected.degree, 1)
                detached_distance = np.hypot(
                    selected.centers[:, 0] - lower_x,
                    selected.centers[:, 1] - 118,
                )
                self.assertLess(float(np.min(detached_distance)), 3.0)
                components = (
                    cv2.connectedComponents(result.mask, connectivity=8)[0] - 1
                )
                self.assertEqual(components, 1)

    def test_six_white_centers_do_not_overfit_a_quadratic(self):
        white = np.zeros((144, 256), dtype=np.uint8)
        yellow = np.zeros_like(white)
        cv2.line(yellow, (110, 10), (110, 138), 255, 4)
        for x, y in ((185, 18), (183, 35), (180, 52), (174, 69), (163, 86), (145, 120)):
            cv2.circle(white, (x, y), 3, 255, -1)
        reference = fit_yellow_centerline_reference(yellow)
        result = fit_white_lane_boundaries(
            white,
            divider_x_by_y=reference.x_by_y,
        )
        self.assertTrue(result.right.valid)
        self.assertEqual(result.right.degree, 1)
        self.assertEqual(result.right.centers.shape[0], 6)

    def test_unsupported_white_fragment_is_not_copied_to_output(self):
        white = np.zeros((144, 256), dtype=np.uint8)
        cv2.line(white, (180, 80), (200, 80), 255, 2)
        result = fit_white_lane_boundaries(white)
        self.assertFalse(result.right.valid)
        self.assertEqual(np.count_nonzero(result.mask), 0)

    def test_lraspp_preprocessing_is_rgb_imagenet_nchw(self):
        frame = np.zeros((2, 2, 3), dtype=np.uint8)
        frame[:] = (255, 0, 0)
        model_input = prepare_model_input(frame, 2, 2)
        self.assertEqual(model_input.shape, (1, 3, 2, 2))
        expected_rgb = np.array(
            [
                (0.0 - 0.485) / 0.229,
                (0.0 - 0.456) / 0.224,
                (1.0 - 0.406) / 0.225,
            ]
        )
        np.testing.assert_allclose(model_input[0, :, 0, 0], expected_rgb)

    def test_lraspp_probabilities_make_disjoint_lane_masks(self):
        probabilities = np.zeros((3, 2, 3), dtype=np.float32)
        probabilities[0] = 0.8
        probabilities[:, 0, 0] = (0.05, 0.90, 0.05)
        probabilities[:, 1, 2] = (0.05, 0.05, 0.90)
        probabilities[:, 0, 1] = (0.35, 0.40, 0.25)
        white, yellow = masks_from_probabilities(
            probabilities,
            white_confidence=0.5,
            yellow_confidence=0.5,
        )
        self.assertEqual(white[0, 0], 255)
        self.assertEqual(yellow[1, 2], 255)
        self.assertEqual(white[0, 1], 0)
        self.assertEqual(np.count_nonzero(white & yellow), 0)

    def test_model_class_aliases_are_accepted(self):
        self.assertEqual(
            class_roles({0: "white-boundary", 1: "yellow_centerline"}),
            {0: "white", 1: "yellow"},
        )

    def test_yellow_uses_a_separate_confidence_threshold(self):
        masks = np.ones((3, 16, 16), dtype=np.float32)
        white, yellow, white_count, yellow_count = merge_instance_masks(
            masks,
            np.array([0, 1, 1]),
            np.array([0.25, 0.39, 0.41]),
            (16, 16),
            {0: "white", 1: "yellow"},
            confidence_threshold=0.20,
            role_confidence_thresholds={"white": 0.20, "yellow": 0.40},
        )
        self.assertEqual((white_count, yellow_count), (1, 1))
        self.assertGreater(np.count_nonzero(white), 0)
        self.assertGreater(np.count_nonzero(yellow), 0)

    def test_masks_reach_fixed_canonical_contract(self):
        masks = np.zeros((2, 1024, 1280), dtype=np.float32)
        cv2.line(masks[0], (310, 610), (570, 490), 1.0, 18)
        cv2.line(masks[0], (1130, 620), (830, 490), 1.0, 18)
        cv2.line(masks[1], (710, 610), (710, 500), 1.0, 12)
        white, yellow, white_count, yellow_count = merge_instance_masks(
            masks,
            np.array([0, 1]),
            np.array([0.9, 0.8]),
            (1024, 1280),
            {0: "white", 1: "yellow"},
            confidence_threshold=0.2,
        )
        self.assertEqual((white_count, yellow_count), (1, 1))

        geometry = build_bev_geometry(
            1280,
            1024,
            source_ratios=(
                0.437418509,
                0.476969898,
                0.662089539,
                0.478676707,
                0.916280746,
                0.614458919,
                0.214667964,
                0.597002029,
            ),
            destination_ratios=(0.15, 0.85, 0.0, 2.0 / 3.0),
            bev_width=640,
            bev_height=220,
        )
        image = np.full((1024, 1280, 3), 36, dtype=np.uint8)
        _, bev_white, bev_yellow, valid = warp_semantic_masks(
            image,
            white,
            yellow,
            geometry,
            valid_lateral_margin_px=6,
            valid_erode_px=4,
        )
        canonical, canonical_white, canonical_yellow = (
            make_canonical_road_image_from_masks(
                bev_white,
                bev_yellow,
                valid_mask=valid,
                lateral_m_per_px=0.0021875,
                forward_m_per_px=0.006818182,
                preserve_white_mask=True,
                min_component_area_px=1,
                geometry_filter_enabled=False,
                bottom_ignore_m=0.0,
            )
        )
        self.assertEqual(canonical.shape, (144, 256, 3))
        self.assertGreater(np.count_nonzero(canonical_white), 0)
        self.assertGreater(np.count_nonzero(canonical_yellow), 0)
        self.assertTrue(
            {tuple(pixel) for pixel in canonical.reshape(-1, 3)}.issubset(
                {(36, 36, 36), (255, 255, 255), (0, 220, 255)}
            )
        )

    def test_full_bev_height_is_preserved_in_canonical(self):
        white = np.zeros((480, 640), dtype=np.uint8)
        yellow = np.zeros_like(white)
        cv2.line(white, (80, 8), (560, 8), 255, 3)
        cv2.line(yellow, (80, 470), (560, 470), 255, 3)

        _, canonical_white, canonical_yellow = (
            make_canonical_road_image_from_masks(
                white,
                yellow,
                valid_mask=np.full_like(white, 255),
                lateral_m_per_px=1.4 / 640.0,
                forward_m_per_px=1.5 / 480.0,
                lateral_range_m=1.4,
                forward_range_m=1.5,
                preserve_white_mask=True,
                min_component_area_px=1,
                geometry_filter_enabled=False,
                top_ignore_m=0.0,
                bottom_ignore_m=0.0,
            )
        )

        self.assertGreater(np.count_nonzero(canonical_white[:8]), 0)
        self.assertGreater(np.count_nonzero(canonical_yellow[-8:]), 0)

    def test_extended_bev_keeps_near_field_yellow(self):
        geometry = build_bev_geometry(
            1280,
            1024,
            source_ratios=(
                472.0 / 1280.0,
                494.0 / 1024.0,
                906.0 / 1280.0,
                486.0 / 1024.0,
                1272.0 / 1280.0,
                612.0 / 1024.0,
                46.0 / 1280.0,
                622.0 / 1024.0,
            ),
            destination_ratios=(
                80.0 / 640.0,
                560.0 / 640.0,
                0.0,
                479.0 / 660.0,
            ),
            bev_width=640,
            bev_height=660,
        )
        image = np.zeros((1024, 1280, 3), dtype=np.uint8)
        white = np.zeros((1024, 1280), dtype=np.uint8)
        yellow = np.zeros_like(white)
        cv2.line(yellow, (742, 689), (742, 757), 255, 8)
        _, _, bev_yellow, _ = warp_semantic_masks(
            image,
            white,
            yellow,
            geometry,
            valid_lateral_margin_px=0,
            valid_erode_px=0,
            clip_to_source_polygon=False,
        )
        self.assertEqual(bev_yellow.shape, (660, 640))
        self.assertGreater(np.count_nonzero(bev_yellow[480:]), 0)


if __name__ == "__main__":
    unittest.main()
