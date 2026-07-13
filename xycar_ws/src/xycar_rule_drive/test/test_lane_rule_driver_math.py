import unittest

from xycar_rule_drive.lane_rule_driver import (
    apply_steering_only,
    inverse_lookup_table,
    interpolate_clamped,
    make_point,
    midpoint_biased_toward_first,
    offset_polyline,
    propagate_path,
    pure_pursuit_curvature,
)


COMMANDS = [-42.0, -40.0, -35.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 35.0, 40.0, 42.0]
CURVATURES = [
    1.502435,
    1.383494,
    1.174860,
    0.922781,
    0.552809,
    0.194230,
    0.0,
    -0.556883,
    -0.959829,
    -1.369323,
    -1.601706,
    -1.853397,
    -1.939236,
]


class LaneRuleDriverMathTest(unittest.TestCase):
    def test_steering_only_forces_zero_speed(self):
        self.assertEqual(apply_steering_only(3.0, True), 0.0)
        self.assertEqual(apply_steering_only(3.0, False), 3.0)

    def test_midpoint_bias_moves_away_from_white_boundary(self):
        self.assertAlmostEqual(
            midpoint_biased_toward_first(0.0, -0.40, 0.02),
            -0.18,
        )
        self.assertAlmostEqual(
            midpoint_biased_toward_first(0.0, 0.40, 0.02),
            0.18,
        )

    def test_straight_motion_moves_saved_path_toward_vehicle(self):
        path = [make_point(0.5, 0.1), make_point(1.0, 0.1)]

        propagated = propagate_path(path, 0.4, 0.0, 0.5)

        self.assertAlmostEqual(propagated[0].x, 0.3)
        self.assertAlmostEqual(propagated[1].x, 0.8)
        self.assertAlmostEqual(propagated[0].y, 0.1)

    def test_turning_motion_rotates_saved_path_into_new_vehicle_frame(self):
        path = [make_point(1.0, 0.0)]

        propagated = propagate_path(path, 0.5, 1.0, 0.4)

        self.assertLess(propagated[0].x, path[0].x)
        self.assertLess(propagated[0].y, path[0].y)

    def test_right_offset_of_straight_yellow_line_builds_lane_center(self):
        yellow = [make_point(x_value, 0.0) for x_value in (0.2, 0.4, 0.6)]

        path = offset_polyline(yellow, 0.20)

        self.assertEqual(len(path), len(yellow))
        for point in path:
            self.assertAlmostEqual(point.y, -0.20)

    def test_right_offset_follows_curve_normal(self):
        yellow = [
            make_point(0.2, 0.0),
            make_point(0.4, -0.1),
            make_point(0.6, -0.3),
        ]

        path = offset_polyline(yellow, 0.20)

        self.assertEqual(len(path), len(yellow))
        self.assertLess(path[1].x, yellow[1].x)
        self.assertLess(path[1].y, yellow[1].y)

    def test_rear_axle_reference_reduces_false_center_origin_curvature(self):
        center_curvature = pure_pursuit_curvature(0.30, -0.06, 0.0)
        rear_axle_curvature = pure_pursuit_curvature(0.30, -0.06, -0.16)

        self.assertLess(abs(rear_axle_curvature), abs(center_curvature))
        self.assertAlmostEqual(center_curvature, -1.282051, places=5)
        self.assertAlmostEqual(rear_axle_curvature, -0.557621, places=5)

    def test_inverse_map_recovers_measured_commands(self):
        curvatures, commands = inverse_lookup_table(COMMANDS, CURVATURES)
        for expected_command, curvature in zip(COMMANDS, CURVATURES):
            actual_command = interpolate_clamped(curvature, curvatures, commands)
            self.assertAlmostEqual(actual_command, expected_command)

    def test_inverse_map_interpolates_curvature(self):
        curvatures, commands = inverse_lookup_table(COMMANDS, CURVATURES)
        lower = COMMANDS.index(10.0)
        upper = COMMANDS.index(20.0)
        curvature = (CURVATURES[lower] + CURVATURES[upper]) / 2.0
        self.assertAlmostEqual(
            interpolate_clamped(curvature, curvatures, commands),
            15.0,
        )

    def test_inverse_map_clamps_unreachable_curvature(self):
        curvatures, commands = inverse_lookup_table(COMMANDS, CURVATURES)
        self.assertEqual(interpolate_clamped(-10.0, curvatures, commands), 42.0)
        self.assertEqual(interpolate_clamped(10.0, curvatures, commands), -42.0)


if __name__ == "__main__":
    unittest.main()
