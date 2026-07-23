import math
from types import SimpleNamespace
import unittest

from track_drive.integration.camera_cone_adapter import (
    camera_cone_source_is_fresh,
    count_camera_cones,
    detector_output_is_valid,
)


def detection(class_id, score):
    return SimpleNamespace(class_id=class_id, score=score)


class DetectorOutputValidityTest(unittest.TestCase):
    def test_accepts_current_finite_class_scores(self):
        self.assertTrue(
            detector_output_is_valid(
                class_scores=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                raw_shape="1x10x8400",
                expected_class_count=6,
            )
        )

    def test_rejects_missing_short_or_nonfinite_output(self):
        self.assertFalse(
            detector_output_is_valid(
                class_scores=[0.1] * 6,
                raw_shape="",
                expected_class_count=6,
            )
        )
        self.assertFalse(
            detector_output_is_valid(
                class_scores=[0.1] * 5,
                raw_shape="1x10x8400",
                expected_class_count=6,
            )
        )
        self.assertFalse(
            detector_output_is_valid(
                class_scores=[0.1, 0.2, math.nan, 0.4, 0.5, 0.6],
                raw_shape="1x10x8400",
                expected_class_count=6,
            )
        )


class CameraConeCountingTest(unittest.TestCase):
    def test_counts_only_configured_cones_at_threshold(self):
        self.assertEqual(
            count_camera_cones(
                detections=[
                    detection(0, 0.35),
                    detection(0, 0.9),
                    detection(0, 0.349),
                    detection(1, 0.9),
                ],
                cone_class_ids=[0],
                minimum_confidence=0.35,
            ),
            2,
        )

    def test_empty_successful_detection_list_means_zero_cones(self):
        self.assertEqual(
            count_camera_cones(
                detections=[],
                cone_class_ids=[0],
                minimum_confidence=0.35,
            ),
            0,
        )

    def test_malformed_detection_or_configuration_is_invalid(self):
        self.assertIsNone(
            count_camera_cones(
                detections=[detection(0, math.nan)],
                cone_class_ids=[0],
                minimum_confidence=0.35,
            )
        )
        self.assertIsNone(
            count_camera_cones(
                detections=[object()],
                cone_class_ids=[0],
                minimum_confidence=0.35,
            )
        )
        self.assertIsNone(
            count_camera_cones(
                detections=[],
                cone_class_ids=[],
                minimum_confidence=0.35,
            )
        )


class CameraConeFreshnessTest(unittest.TestCase):
    def test_accepts_nonnegative_count_at_timeout_boundary(self):
        self.assertTrue(
            camera_cone_source_is_fresh(
                now_sec=10.2,
                last_receive_sec=10.0,
                timeout_sec=0.2,
                source_count=0,
            )
        )

    def test_rejects_negative_stale_missing_or_future_source(self):
        self.assertFalse(
            camera_cone_source_is_fresh(
                now_sec=10.1,
                last_receive_sec=10.0,
                timeout_sec=0.2,
                source_count=-1,
            )
        )
        self.assertFalse(
            camera_cone_source_is_fresh(
                now_sec=10.3,
                last_receive_sec=10.0,
                timeout_sec=0.2,
                source_count=4,
            )
        )
        self.assertFalse(
            camera_cone_source_is_fresh(
                now_sec=10.0,
                last_receive_sec=None,
                timeout_sec=0.2,
                source_count=4,
            )
        )
        self.assertFalse(
            camera_cone_source_is_fresh(
                now_sec=10.0,
                last_receive_sec=10.1,
                timeout_sec=0.2,
                source_count=4,
            )
        )


if __name__ == "__main__":
    unittest.main()
