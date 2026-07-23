import math
from types import SimpleNamespace
import unittest

from track_drive.lane_fallback_controller import (
    LaneFallbackParameters,
    compute_lane_fallback_steering,
)


def point(x, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


class LaneFallbackControllerTest(unittest.TestCase):
    def setUp(self):
        self.parameters = LaneFallbackParameters()

    def test_straight_path_produces_zero_physical_angle(self):
        result = compute_lane_fallback_steering(
            [point(0.4), point(0.8), point(1.5)],
            self.parameters,
        )
        self.assertTrue(result.valid)
        self.assertAlmostEqual(result.steering_angle_deg, 0.0)

    def test_left_path_uses_negative_physical_angle(self):
        result = compute_lane_fallback_steering(
            [
                point(0.4, 0.05),
                point(0.8, 0.15),
                point(1.5, 0.35),
            ],
            self.parameters,
        )
        self.assertTrue(result.valid)
        self.assertLess(result.steering_angle_deg, 0.0)

    def test_right_path_uses_positive_physical_angle(self):
        result = compute_lane_fallback_steering(
            [
                point(0.4, -0.05),
                point(0.8, -0.15),
                point(1.5, -0.35),
            ],
            self.parameters,
        )
        self.assertTrue(result.valid)
        self.assertGreater(result.steering_angle_deg, 0.0)

    def test_angle_is_limited_to_physical_range(self):
        parameters = LaneFallbackParameters(
            steering_gain=10.0,
            max_steering_angle_deg=26.0,
        )
        result = compute_lane_fallback_steering(
            [
                point(0.2, 1.0),
                point(0.4, 1.0),
                point(0.8, 1.0),
            ],
            parameters,
        )
        self.assertTrue(result.valid)
        self.assertEqual(result.steering_angle_deg, -26.0)

    def test_rejects_short_or_sparse_path(self):
        sparse = compute_lane_fallback_steering(
            [point(0.4), point(0.8)],
            self.parameters,
        )
        short = compute_lane_fallback_steering(
            [point(0.2), point(0.4), point(0.6)],
            self.parameters,
        )
        self.assertFalse(sparse.valid)
        self.assertFalse(short.valid)

    def test_rejects_nonfinite_or_malformed_point(self):
        for invalid_point in (
            point(0.8, math.nan),
            point(math.inf, 0.0),
            object(),
        ):
            with self.subTest(invalid_point=invalid_point):
                result = compute_lane_fallback_steering(
                    [point(0.4), invalid_point, point(1.5)],
                    self.parameters,
                )
                self.assertFalse(result.valid)

    def test_rejects_invalid_parameter_relationships(self):
        with self.assertRaises(ValueError):
            LaneFallbackParameters(
                near_lookahead_m=0.8,
                minimum_path_distance_m=0.7,
            )


if __name__ == "__main__":
    unittest.main()
