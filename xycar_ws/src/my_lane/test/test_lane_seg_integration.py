import unittest
from pathlib import Path

import cv2
import numpy as np

from my_lane.bag_montage_exporter import (
    make_montage,
    nearest_motor_command,
)
from my_lane.camera_input import (
    CameraRectifier,
    decode_compressed_bgr,
    scale_camera_matrix,
)
from my_lane.canonical_adapter_node import (
    CanonicalRenderConfig,
    build_bev_geometry,
    render_canonical_from_bev_masks,
    warp_semantic_masks,
    warp_semantic_masks_only,
)
from my_lane.lane_seg_inference_node import (
    class_roles,
    merge_instance_masks,
)
from my_lane.lraspp_inference_node import (
    masks_from_probabilities,
    prepare_model_input,
)
from my_lane.white_lane_fitter import (
    compose_fitted_canonical,
    fit_white_lane_boundaries,
    fit_yellow_centerline_reference,
    normalize_yellow_fragments,
    render_white_lane_fit_debug,
)
from my_road.canonical_road import make_canonical_road_image_from_masks


class LaneSegIntegrationTest(unittest.TestCase):
    def test_compressed_decode_and_camera_matrix_scaling(self):
        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        frame[:, :32] = (10, 90, 180)
        ok, encoded = cv2.imencode(".jpg", frame)
        self.assertTrue(ok)
        decoded = decode_compressed_bgr(encoded.tobytes())
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.shape, frame.shape)

        matrix = np.asarray(
            [[400.0, 0.0, 320.0], [0.0, 420.0, 240.0], [0.0, 0.0, 1.0]]
        )
        scaled = scale_camera_matrix(matrix, (640, 480), (320, 240))
        np.testing.assert_allclose(
            scaled,
            np.asarray(
                [
                    [200.0, 0.0, 160.0],
                    [0.0, 210.0, 120.0],
                    [0.0, 0.0, 1.0],
                ]
            ),
        )

    def test_camera_rectifier_reuses_cached_maps(self):
        calibration = (
            Path(__file__).resolve().parents[2]
            / "my_road"
            / "config"
            / "wide_camera_fisheye_1280x1024_20260708.yaml"
        )
        rectifier = CameraRectifier(calibration, 0.3)
        frame = np.zeros((96, 128, 3), dtype=np.uint8)
        first = rectifier.rectify(frame)
        first_map = rectifier.map1
        second = rectifier.rectify(frame)
        self.assertEqual(first.shape, frame.shape)
        self.assertIs(first_map, rectifier.map1)
        np.testing.assert_array_equal(first, second)

    def test_camera_rectifier_directly_outputs_model_resolution(self):
        calibration = (
            Path(__file__).resolve().parents[2]
            / "my_road"
            / "config"
            / "wide_camera_fisheye_1280x1024_20260708.yaml"
        )
        rectifier = CameraRectifier(calibration, 0.3)
        frame = np.zeros((1024, 1280, 3), dtype=np.uint8)
        cv2.line(frame, (180, 1000), (540, 300), (255, 255, 255), 12)
        cv2.line(frame, (1100, 1000), (750, 300), (0, 220, 255), 12)

        direct = rectifier.rectify_to_size(frame, 256, 144)
        first_map = rectifier.scaled_maps[(1280, 1024, 256, 144)][0]
        repeated = rectifier.rectify_to_size(frame, 256, 144)

        self.assertEqual(direct.shape, (144, 256, 3))
        self.assertIs(
            first_map,
            rectifier.scaled_maps[(1280, 1024, 256, 144)][0],
        )
        np.testing.assert_array_equal(direct, repeated)

    def test_direct_model_rectification_matches_full_resolution_geometry(self):
        calibration = (
            Path(__file__).resolve().parents[2]
            / "my_road"
            / "config"
            / "wide_camera_fisheye_1280x1024_20260708.yaml"
        )
        rectifier = CameraRectifier(calibration, 0.3)
        frame = np.zeros((1024, 1280, 3), dtype=np.uint8)
        for x in range(80, 1280, 120):
            cv2.line(frame, (x, 0), (x, 1023), (255, 255, 255), 3)
        for y in range(64, 1024, 96):
            cv2.line(frame, (0, y), (1279, y), (255, 255, 255), 3)

        full_then_resize = cv2.resize(
            rectifier.rectify(frame),
            (256, 144),
            interpolation=cv2.INTER_AREA,
        )
        direct = rectifier.rectify_to_size(frame, 256, 144)
        full_edges = cv2.Canny(full_then_resize, 40, 120)
        direct_edges = cv2.Canny(direct, 40, 120)
        tolerance = np.ones((3, 3), dtype=np.uint8)
        covered = cv2.dilate(full_edges, tolerance) > 0
        direct_edge_pixels = direct_edges > 0
        overlap_ratio = np.count_nonzero(
            direct_edge_pixels & covered
        ) / max(1, np.count_nonzero(direct_edge_pixels))
        self.assertGreater(overlap_ratio, 0.95)

    def test_direct_mask_warp_is_pixel_identical_to_adapter_warp(self):
        height, width = 144, 256
        image = np.full((height, width, 3), 48, dtype=np.uint8)
        white = np.zeros((height, width), dtype=np.uint8)
        yellow = np.zeros_like(white)
        cv2.line(white, (32, 140), (78, 62), 255, 5)
        cv2.line(white, (224, 140), (180, 62), 255, 5)
        cv2.line(yellow, (126, 140), (130, 82), 255, 5)
        geometry = build_bev_geometry(
            width,
            height,
            source_ratios=(
                0.442578,
                0.480781,
                0.688281,
                0.480781,
                0.919141,
                0.614189,
                0.190625,
                0.614189,
            ),
            destination_ratios=(
                0.205714,
                0.794286,
                0.0,
                0.666666667,
            ),
            bev_width=640,
            bev_height=660,
        )
        _, legacy_white, legacy_yellow, legacy_valid = warp_semantic_masks(
            image,
            white,
            yellow,
            geometry,
            valid_lateral_margin_px=0,
            valid_erode_px=0,
            clip_to_source_polygon=False,
        )
        direct_white, direct_yellow, direct_valid = (
            warp_semantic_masks_only(
                white,
                yellow,
                geometry,
                valid_lateral_margin_px=0,
                valid_erode_px=0,
                clip_to_source_polygon=False,
            )
        )
        np.testing.assert_array_equal(direct_white, legacy_white)
        np.testing.assert_array_equal(direct_yellow, legacy_yellow)
        np.testing.assert_array_equal(direct_valid, legacy_valid)

    def test_direct_canonical_render_is_pixel_identical_to_legacy_steps(self):
        white = np.zeros((660, 640), dtype=np.uint8)
        yellow = np.zeros_like(white)
        valid = np.full_like(white, 255)
        cv2.line(white, (90, 650), (205, 80), 255, 12)
        cv2.line(white, (550, 650), (430, 80), 255, 12)
        cv2.line(yellow, (315, 650), (320, 500), 255, 9)
        cv2.line(yellow, (321, 420), (327, 270), 255, 9)
        config = CanonicalRenderConfig(
            lateral_m_per_px=1.4 / 640.0,
            forward_m_per_px=1.5 / 660.0,
            lateral_range_m=1.4,
            forward_range_m=1.5,
            output_width=256,
            output_height=144,
            background_gray=36,
            line_width_px=5,
            white_fit_enabled=True,
            white_fit_window_count=9,
            white_fit_margin_px=24,
            white_fit_min_pixels=4,
            white_fit_min_centers=2,
            white_fit_min_span_px=8,
            white_fit_residual_px=6.0,
            white_fit_line_width_px=5,
            yellow_divider_enabled=True,
            yellow_divider_min_pixels=3,
            yellow_divider_residual_px=6.0,
            yellow_divider_line_width_px=5,
            yellow_normalize_enabled=False,
            yellow_normalize_line_width_px=5,
            yellow_normalize_min_area_px=3,
            yellow_normalize_smoothing_rows=5,
        )

        direct = render_canonical_from_bev_masks(
            white,
            yellow,
            valid,
            config,
        )
        raw = make_canonical_road_image_from_masks(
            white,
            yellow,
            valid_mask=valid,
            lateral_m_per_px=config.lateral_m_per_px,
            forward_m_per_px=config.forward_m_per_px,
            lateral_range_m=config.lateral_range_m,
            forward_range_m=config.forward_range_m,
            output_width=config.output_width,
            output_height=config.output_height,
            background_gray=config.background_gray,
            line_width_px=config.line_width_px,
            min_component_area_px=1,
            white_max_component_thickness_px=0.0,
            yellow_max_component_thickness_px=0.0,
            geometry_filter_enabled=False,
            preserve_white_mask=True,
            top_ignore_m=0.0,
            bottom_ignore_m=0.0,
            return_stages=True,
        )
        reference = fit_yellow_centerline_reference(
            raw.yellow_mask,
            min_pixels=config.yellow_divider_min_pixels,
            residual_threshold_px=config.yellow_divider_residual_px,
            line_width_px=config.yellow_divider_line_width_px,
        )
        fitted = fit_white_lane_boundaries(
            raw.white_mask,
            window_count=config.white_fit_window_count,
            window_margin_px=config.white_fit_margin_px,
            min_pixels_per_window=config.white_fit_min_pixels,
            min_centers=config.white_fit_min_centers,
            min_span_px=config.white_fit_min_span_px,
            residual_threshold_px=config.white_fit_residual_px,
            line_width_px=config.white_fit_line_width_px,
            divider_x_by_y=reference.x_by_y,
        )
        legacy_road, legacy_white = compose_fitted_canonical(
            fitted.mask,
            raw.yellow_mask,
            raw.valid_mask,
            background_gray=config.background_gray,
        )
        np.testing.assert_array_equal(direct.stages.road_image, legacy_road)
        np.testing.assert_array_equal(
            direct.stages.white_mask,
            legacy_white,
        )
        np.testing.assert_array_equal(
            direct.stages.yellow_mask,
            raw.yellow_mask,
        )
        np.testing.assert_array_equal(
            direct.stages.valid_mask,
            raw.valid_mask,
        )

    def test_yellow_normalizer_preserves_dash_gaps_and_width(self):
        yellow = np.zeros((144, 256), dtype=np.uint8)
        cv2.line(yellow, (124, 12), (132, 54), 255, 13)
        cv2.line(yellow, (131, 86), (137, 132), 255, 11)

        normalized = normalize_yellow_fragments(
            yellow,
            line_width_px=3,
            min_component_area_px=3,
        )

        component_count = cv2.connectedComponents(
            (normalized > 0).astype(np.uint8),
            connectivity=8,
        )[0] - 1
        self.assertEqual(component_count, 2)
        self.assertEqual(np.count_nonzero(normalized[66:76]), 0)
        self.assertLess(np.count_nonzero(normalized), np.count_nonzero(yellow))
        occupied_rows = np.flatnonzero(np.any(normalized > 0, axis=1))
        row_widths = np.count_nonzero(normalized[occupied_rows] > 0, axis=1)
        self.assertLessEqual(int(np.percentile(row_widths, 90)), 5)

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
                0.442578,
                0.480781,
                0.688281,
                0.480781,
                0.919141,
                0.614189,
                0.190625,
                0.614189,
            ),
            destination_ratios=(
                0.205714,
                0.794286,
                0.0,
                0.666666667,
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
