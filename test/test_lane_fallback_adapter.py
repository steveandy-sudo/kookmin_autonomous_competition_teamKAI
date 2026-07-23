import math
from types import SimpleNamespace
import unittest

from track_drive.integration.lane_fallback_adapter import (
    centerline_values_are_valid,
    lane_fallback_command_values_are_valid,
    lane_path_evidence_is_valid,
    lane_fallback_source_is_fresh,
)


def point(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def segment(segment_type, confidence=0.8, point_count=3):
    return SimpleNamespace(
        type=segment_type,
        confidence=confidence,
        points=[point(float(index)) for index in range(point_count)],
    )


class CenterlineValuesTest(unittest.TestCase):
    def test_accepts_minimum_valid_centerline(self):
        self.assertTrue(
            centerline_values_are_valid(
                points=[point(), point(1.0), point(2.0)],
                confidence=0.25,
                minimum_point_count=3,
                minimum_confidence=0.25,
            )
        )

    def test_rejects_too_few_points_or_low_confidence(self):
        common = {
            "minimum_point_count": 3,
            "minimum_confidence": 0.25,
        }
        self.assertFalse(
            centerline_values_are_valid(
                points=[point(), point(1.0)],
                confidence=0.9,
                **common,
            )
        )
        self.assertFalse(
            centerline_values_are_valid(
                points=[point(), point(1.0), point(2.0)],
                confidence=0.249,
                **common,
            )
        )

    def test_rejects_nonfinite_or_malformed_values(self):
        valid_points = [point(), point(1.0), point(2.0)]
        for confidence in (math.nan, math.inf, -math.inf, 1.01):
            with self.subTest(confidence=confidence):
                self.assertFalse(
                    centerline_values_are_valid(
                        points=valid_points,
                        confidence=confidence,
                        minimum_point_count=3,
                        minimum_confidence=0.25,
                    )
                )

        invalid_points = [point(), point(1.0, math.nan), point(2.0)]
        self.assertFalse(
            centerline_values_are_valid(
                points=invalid_points,
                confidence=0.9,
                minimum_point_count=3,
                minimum_confidence=0.25,
            )
        )
        self.assertFalse(
            centerline_values_are_valid(
                points=[point(), point(1.0), object()],
                confidence=0.9,
                minimum_point_count=3,
                minimum_confidence=0.25,
            )
        )


class LanePathEvidenceTest(unittest.TestCase):
    def test_accepts_one_valid_yellow_segment(self):
        self.assertTrue(
            lane_path_evidence_is_valid(
                segments=[segment(3)],
                minimum_point_count=3,
                minimum_confidence=0.25,
            )
        )

    def test_accepts_two_valid_white_segments(self):
        self.assertTrue(
            lane_path_evidence_is_valid(
                segments=[segment(1), segment(2)],
                minimum_point_count=3,
                minimum_confidence=0.25,
            )
        )

    def test_rejects_one_white_segment(self):
        self.assertFalse(
            lane_path_evidence_is_valid(
                segments=[segment(1)],
                minimum_point_count=3,
                minimum_confidence=0.25,
            )
        )

    def test_rejects_invalid_yellow_and_incomplete_white_pair(self):
        self.assertFalse(
            lane_path_evidence_is_valid(
                segments=[
                    segment(3, confidence=0.1),
                    segment(1),
                ],
                minimum_point_count=3,
                minimum_confidence=0.25,
            )
        )


class LaneFallbackCommandValuesTest(unittest.TestCase):
    def test_accepts_valid_angle_at_physical_limit(self):
        for angle in (-26.0, 0.0, 26.0):
            with self.subTest(angle=angle):
                self.assertTrue(
                    lane_fallback_command_values_are_valid(
                        steering_angle_deg=angle,
                        command_valid=True,
                        max_steering_angle_deg=26.0,
                    )
                )

    def test_rejects_invalid_nonfinite_or_out_of_range_angle(self):
        cases = (
            (0.0, False),
            (math.nan, True),
            (math.inf, True),
            (-26.01, True),
            (26.01, True),
        )
        for angle, command_valid in cases:
            with self.subTest(
                angle=angle,
                command_valid=command_valid,
            ):
                self.assertFalse(
                    lane_fallback_command_values_are_valid(
                        steering_angle_deg=angle,
                        command_valid=command_valid,
                        max_steering_angle_deg=26.0,
                    )
                )

    def test_rejects_invalid_physical_limit(self):
        self.assertFalse(
            lane_fallback_command_values_are_valid(
                steering_angle_deg=0.0,
                command_valid=True,
                max_steering_angle_deg=0.0,
            )
        )


class CenterlineFreshnessTest(unittest.TestCase):
    def test_freshness_includes_timeout_boundary(self):
        self.assertTrue(
            lane_fallback_source_is_fresh(
                now_sec=10.4,
                last_receive_sec=10.0,
                timeout_sec=0.4,
                source_values_valid=True,
            )
        )

    def test_rejects_invalid_stale_missing_or_future_source(self):
        common = {
            "now_sec": 10.5,
            "timeout_sec": 0.4,
        }
        self.assertFalse(
            lane_fallback_source_is_fresh(
                last_receive_sec=10.0,
                source_values_valid=True,
                **common,
            )
        )
        self.assertFalse(
            lane_fallback_source_is_fresh(
                last_receive_sec=None,
                source_values_valid=True,
                **common,
            )
        )
        self.assertFalse(
            lane_fallback_source_is_fresh(
                last_receive_sec=10.4,
                source_values_valid=False,
                **common,
            )
        )
        self.assertFalse(
            lane_fallback_source_is_fresh(
                now_sec=10.0,
                last_receive_sec=10.1,
                timeout_sec=0.4,
                source_values_valid=True,
            )
        )


if __name__ == "__main__":
    unittest.main()
