import unittest

import math

from xycar_gazebo_bridge.xycar_motor_bridge import (
    apply_command_deadzone,
    first_order_response,
    interpolate_clamped,
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


class MotorBridgeMathTest(unittest.TestCase):
    def test_profile_points_are_preserved(self):
        for command, curvature in zip(COMMANDS, CURVATURES):
            self.assertEqual(
                interpolate_clamped(command, COMMANDS, CURVATURES),
                curvature,
            )

    def test_intermediate_command_is_linearly_interpolated(self):
        lower = COMMANDS.index(10.0)
        upper = COMMANDS.index(20.0)
        expected = (CURVATURES[lower] + CURVATURES[upper]) / 2.0
        actual = interpolate_clamped(15.0, COMMANDS, CURVATURES)
        self.assertAlmostEqual(actual, expected)

    def test_unmeasured_commands_are_clamped_to_measured_range(self):
        self.assertEqual(
            interpolate_clamped(-100.0, COMMANDS, CURVATURES),
            CURVATURES[0],
        )
        self.assertEqual(
            interpolate_clamped(100.0, COMMANDS, CURVATURES),
            CURVATURES[-1],
        )

    def test_high_angle_measurements_preserve_left_right_asymmetry(self):
        self.assertAlmostEqual(CURVATURES[0], 1.502435)
        self.assertAlmostEqual(CURVATURES[-1], -1.939236)
        self.assertGreater(abs(CURVATURES[-1]), abs(CURVATURES[0]))

    def test_measured_speed_deadzone_starts_at_command_three(self):
        for command in (-2.99, -2.0, 0.0, 1.0, 2.0, 2.99):
            self.assertEqual(apply_command_deadzone(command, 3.0), 0.0)
        self.assertEqual(apply_command_deadzone(3.0, 3.0), 3.0)
        self.assertEqual(apply_command_deadzone(-3.0, 3.0), -3.0)

    def test_first_order_response_is_timer_rate_independent(self):
        actual = first_order_response(0.0, 1.0, 0.19, 0.19)
        self.assertAlmostEqual(actual, 1.0 - math.exp(-1.0))
        self.assertEqual(first_order_response(0.2, 1.0, 0.0, 0.19), 0.2)
        self.assertEqual(first_order_response(0.2, 1.0, 0.01, 0.0), 1.0)

    def test_invalid_lookup_table_is_rejected(self):
        invalid_tables = [
            ([0.0], [0.0]),
            ([0.0, 1.0], [0.0]),
            ([0.0, 0.0], [0.0, 1.0]),
        ]
        for inputs, outputs in invalid_tables:
            with self.assertRaises(ValueError):
                interpolate_clamped(0.0, inputs, outputs)


if __name__ == "__main__":
    unittest.main()
