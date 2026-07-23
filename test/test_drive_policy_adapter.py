import math
import unittest

from track_drive.integration.drive_policy_adapter import (
    drive_policy_source_is_fresh,
    policy_debug_values_are_valid,
)


class DrivePolicyValuesTest(unittest.TestCase):
    def test_finite_final_steering_in_range_is_valid(self):
        self.assertTrue(
            policy_debug_values_are_valid(
                [0.25, 12.0, 10.0, 4.0]
            )
        )
        self.assertTrue(
            policy_debug_values_are_valid(
                [math.nan, math.inf, -42.0, math.nan]
            )
        )
        self.assertTrue(
            policy_debug_values_are_valid(
                [math.nan, -math.inf, 42.0, math.nan]
            )
        )

    def test_short_or_invalid_final_steering_is_invalid(self):
        cases = (
            [],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, math.nan, 4.0],
            [0.0, 0.0, math.inf, 4.0],
            [0.0, 0.0, -42.01, 4.0],
            [0.0, 0.0, 42.01, 4.0],
            [0.0, 0.0, "angle", 4.0],
            "not-an-array",
            None,
        )

        for values in cases:
            with self.subTest(values=values):
                self.assertFalse(policy_debug_values_are_valid(values))


class DrivePolicyFreshnessTest(unittest.TestCase):
    def test_freshness_includes_timeout_boundary(self):
        self.assertTrue(
            drive_policy_source_is_fresh(
                now_sec=20.5,
                last_receive_sec=20.0,
                timeout_sec=0.5,
                source_values_valid=True,
            )
        )
        self.assertFalse(
            drive_policy_source_is_fresh(
                now_sec=20.500001,
                last_receive_sec=20.0,
                timeout_sec=0.5,
                source_values_valid=True,
            )
        )

    def test_missing_invalid_or_reversed_source_is_not_fresh(self):
        cases = (
            (20.0, None, 0.5, True),
            (20.0, 20.0, 0.5, False),
            (19.0, 20.0, 0.5, True),
            (20.0, 20.0, -0.1, True),
            (math.inf, 20.0, 0.5, True),
        )

        for now_sec, last_receive_sec, timeout_sec, valid in cases:
            with self.subTest(
                now_sec=now_sec,
                last_receive_sec=last_receive_sec,
                timeout_sec=timeout_sec,
                valid=valid,
            ):
                self.assertFalse(
                    drive_policy_source_is_fresh(
                        now_sec=now_sec,
                        last_receive_sec=last_receive_sec,
                        timeout_sec=timeout_sec,
                        source_values_valid=valid,
                    )
                )
