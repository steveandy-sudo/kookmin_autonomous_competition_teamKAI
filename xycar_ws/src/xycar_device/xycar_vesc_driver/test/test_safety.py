import unittest

from xycar_vesc_driver.safety import (
    LATCHED,
    LIMITED,
    NORMAL,
    VoltageGuard,
    slew,
)


class MotorSafetyTest(unittest.TestCase):
    def test_voltage_guard_limits_then_latches(self):
        guard = VoltageGuard()
        self.assertEqual(guard.update(8.8, 0, 0.0), NORMAL)
        self.assertEqual(guard.output_scale, 1.0)
        self.assertEqual(guard.update(7.4, 0, 0.1), LIMITED)
        self.assertFalse(guard.acceleration_allowed)
        self.assertTrue(guard.output_allowed)
        self.assertAlmostEqual(guard.output_scale, 1.4 / 1.5)
        self.assertEqual(guard.update(6.75, 0, 0.2), LIMITED)
        self.assertAlmostEqual(guard.output_scale, 0.5)
        self.assertEqual(guard.update(6.0, 0, 0.3), LATCHED)
        self.assertFalse(guard.output_allowed)
        self.assertEqual(guard.output_scale, 0.0)

    def test_voltage_guard_needs_stable_manual_clear(self):
        guard = VoltageGuard(recovery_stable_sec=2.0)
        guard.update(6.2, 2, 0.0)
        self.assertEqual(
            guard.clear(0.1), (False, "VESC fault_code is still 2")
        )
        guard.update(9.0, 0, 1.0)
        success, message = guard.clear(1.1)
        self.assertFalse(success)
        self.assertIn("wait", message)
        success, _ = guard.clear(3.1)
        self.assertTrue(success)
        self.assertEqual(guard.state, NORMAL)

    def test_slew_uses_acceleration_and_deceleration_limits(self):
        self.assertAlmostEqual(slew(0.0, 1.0, 0.1, 0.5, 2.0), 0.05)
        self.assertAlmostEqual(slew(1.0, 0.0, 0.1, 0.5, 2.0), 0.8)
        self.assertEqual(slew(0.02, 0.0, 0.1, 0.5, 2.0), 0.0)
