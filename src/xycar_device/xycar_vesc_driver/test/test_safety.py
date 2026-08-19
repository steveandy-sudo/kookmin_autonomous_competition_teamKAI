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
        self.assertEqual(guard.update(7.5, 0, 0.1), LIMITED)
        self.assertFalse(guard.acceleration_allowed)
        self.assertTrue(guard.output_allowed)
        self.assertAlmostEqual(guard.output_scale, 1.0)
        self.assertEqual(guard.update(7.0, 0, 0.2), LIMITED)
        self.assertAlmostEqual(guard.output_scale, 2.0 / 3.0)
        self.assertEqual(guard.update(6.75, 0, 0.3), LIMITED)
        self.assertAlmostEqual(guard.output_scale, 0.5)
        self.assertEqual(guard.update(6.2, 0, 0.4), LIMITED)
        self.assertAlmostEqual(guard.output_scale, 2.0 / 15.0)
        self.assertEqual(guard.update(6.0, 0, 0.5), LATCHED)
        self.assertFalse(guard.output_allowed)
        self.assertEqual(guard.output_scale, 0.0)

    def test_hardware_fault_needs_stable_manual_clear(self):
        guard = VoltageGuard(recovery_stable_sec=2.0)
        guard.update(8.8, 3, 0.0)
        self.assertEqual(
            guard.clear(0.1), (False, "VESC fault_code is still 3")
        )
        guard.update(9.0, 0, 1.0)
        success, message = guard.clear(1.1)
        self.assertFalse(success)
        self.assertIn("wait", message)
        success, _ = guard.clear(3.1)
        self.assertTrue(success)
        self.assertEqual(guard.state, NORMAL)

    def test_low_voltage_latch_auto_recovers_after_stable_voltage(self):
        guard = VoltageGuard(
            auto_recover=True,
            recovery_voltage=8.0,
            recovery_stable_sec=3.0,
        )
        self.assertEqual(guard.update(6.0, 0, 0.0), LATCHED)
        self.assertEqual(guard.update(8.2, 0, 1.0), LATCHED)
        self.assertEqual(guard.update(8.2, 0, 3.9), LATCHED)
        self.assertEqual(guard.update(8.2, 0, 4.0), NORMAL)
        self.assertTrue(guard.output_allowed)

    def test_default_recovery_stability_is_three_seconds(self):
        self.assertEqual(VoltageGuard().recovery_stable_sec, 1.0)

    def test_under_voltage_fault_auto_recovers_after_stable_voltage(self):
        guard = VoltageGuard(
            auto_recover=True,
            recovery_voltage=8.0,
            recovery_stable_sec=3.0,
        )
        self.assertEqual(guard.update(8.8, 2, 0.0), LATCHED)
        self.assertEqual(guard.update(8.8, 0, 1.0), LATCHED)
        self.assertEqual(guard.update(8.8, 0, 3.9), LATCHED)
        self.assertEqual(guard.update(8.8, 0, 4.0), NORMAL)
        self.assertTrue(guard.output_allowed)

    def test_recovery_timer_restarts_after_voltage_dip(self):
        guard = VoltageGuard(
            auto_recover=True,
            recovery_voltage=8.0,
            recovery_stable_sec=3.0,
        )
        self.assertEqual(guard.update(8.8, 2, 0.0), LATCHED)
        self.assertEqual(guard.update(8.2, 0, 1.0), LATCHED)
        self.assertEqual(guard.update(7.9, 0, 3.5), LATCHED)
        self.assertEqual(guard.update(8.2, 0, 4.0), LATCHED)
        self.assertEqual(guard.update(8.2, 0, 6.9), LATCHED)
        self.assertEqual(guard.update(8.2, 0, 7.0), NORMAL)

    def test_non_voltage_vesc_fault_does_not_auto_recover(self):
        guard = VoltageGuard(
            auto_recover=True,
            recovery_voltage=8.0,
            recovery_stable_sec=3.0,
        )
        self.assertEqual(guard.update(8.8, 3, 0.0), LATCHED)
        self.assertEqual(guard.update(8.8, 0, 1.0), LATCHED)
        self.assertEqual(guard.update(8.8, 0, 4.1), LATCHED)
        success, _ = guard.clear(4.1)
        self.assertTrue(success)
        self.assertEqual(guard.state, NORMAL)

    def test_slew_uses_acceleration_and_deceleration_limits(self):
        self.assertAlmostEqual(slew(0.0, 1.0, 0.1, 0.5, 2.0), 0.05)
        self.assertAlmostEqual(slew(1.0, 0.0, 0.1, 0.5, 2.0), 0.8)
        self.assertEqual(slew(0.02, 0.0, 0.1, 0.5, 2.0), 0.0)

    def test_slew_does_not_cross_zero_during_direction_change(self):
        self.assertAlmostEqual(slew(0.4, -1.0, 0.1, 0.5, 2.0), 0.2)
        self.assertEqual(slew(0.1, -1.0, 0.1, 0.5, 2.0), 0.0)
