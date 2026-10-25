"""AS2's PX4 platform publishes BatteryState.percentage on a 0..100 scale: the runtime must read it
as such, or the 25 % reserve abort never fires (2026-10-25), and treat "no reading" as unknown."""
import math
import unittest

from sensor_msgs.msg import BatteryState

from as2_rescue_runtime.runtime_node import battery_fraction
from as2_rescue_runtime.safety import Limits


def battery(percentage, present=True):
    msg = BatteryState()
    msg.percentage = percentage
    msg.present = present
    return msg


class BatteryFractionTests(unittest.TestCase):
    def test_as2_percent_scale(self):
        self.assertAlmostEqual(battery_fraction(battery(80.0)), 0.8)
        self.assertAlmostEqual(battery_fraction(battery(18.0)), 0.18)

    def test_low_battery_is_under_the_reserve(self):
        self.assertLess(battery_fraction(battery(18.0)), Limits().battery_reserve)
        self.assertGreater(battery_fraction(battery(60.0)), Limits().battery_reserve)

    def test_unknown_reading_is_none(self):
        self.assertIsNone(battery_fraction(battery(-100.0)))           # PX4 remaining -1
        self.assertIsNone(battery_fraction(battery(math.nan)))
        self.assertIsNone(battery_fraction(battery(50.0, present=False)))

    def test_ros_scale_and_clamp(self):
        self.assertAlmostEqual(battery_fraction(battery(0.4), scale=1.0), 0.4)
        self.assertEqual(battery_fraction(battery(130.0)), 1.0)


if __name__ == '__main__':
    unittest.main()
