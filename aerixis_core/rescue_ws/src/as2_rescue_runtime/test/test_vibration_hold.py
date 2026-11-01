"""Vibration estimate through short IMU gaps (2026-10-31): Gazebo froze ~1 s under load, the IMU of
every simulated drone stopped at once, "vibration_rms unknown" withdrew readiness and the mission
tree aborted all 4 drones at take-off. A gap keeps the last estimate for VIBRATION_HOLD_S only."""
import math
import unittest

from as2_rescue_runtime.runtime_node import VIBRATION_HOLD_S, vibration_estimate


def samples(t0, t1, rate=50.0, amplitude=0.0):
    out, n = [], int((t1 - t0) * rate)
    for k in range(n):
        out.append((t0 + k / rate, [amplitude * (-1) ** k, 0.0, 9.81]))
    return out


class VibrationHoldTest(unittest.TestCase):
    def test_estimate_from_enough_samples(self):
        value, held = vibration_estimate(samples(9.0, 10.0, amplitude=0.5), 10.0)
        self.assertAlmostEqual(value, 0.5, places=3)
        self.assertEqual(held, (value, 10.0))

    def test_too_few_samples_without_history_is_unknown(self):
        value, held = vibration_estimate(samples(9.9, 10.0), 10.0)
        self.assertTrue(math.isnan(value))
        self.assertIsNone(held)

    def test_short_gap_keeps_last_estimate(self):
        _, held = vibration_estimate(samples(9.0, 10.0, amplitude=0.3), 10.0)
        # Simulator frozen for 1.2 s: only stale samples left.
        value, held2 = vibration_estimate(samples(9.0, 10.0, amplitude=0.3), 11.2, held)
        self.assertAlmostEqual(value, 0.3, places=3)
        self.assertEqual(held2, held)

    def test_long_gap_becomes_unknown(self):
        _, held = vibration_estimate(samples(9.0, 10.0, amplitude=0.3), 10.0)
        value, _ = vibration_estimate([], 10.0 + VIBRATION_HOLD_S + 0.1, held)
        self.assertTrue(math.isnan(value))

    def test_new_samples_replace_held_value(self):
        _, held = vibration_estimate(samples(9.0, 10.0, amplitude=0.3), 10.0)
        value, held2 = vibration_estimate(samples(11.0, 12.0, amplitude=2.0), 12.0, held)
        self.assertAlmostEqual(value, 2.0, places=3)
        self.assertEqual(held2, (value, 12.0))


if __name__ == '__main__':
    unittest.main()
