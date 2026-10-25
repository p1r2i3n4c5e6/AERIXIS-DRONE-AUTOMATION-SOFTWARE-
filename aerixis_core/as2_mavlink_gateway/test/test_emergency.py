"""Which GCS commands take the onboard emergency path (FC guardian) instead of AS2."""
import unittest

from as2_mavlink_gateway.protocol import emergency_action

P = (0.0,) * 7


class EmergencyRoutingTest(unittest.TestCase):
    def test_rtl_any_component(self):
        for component in (0, 1, 190):
            self.assertEqual(emergency_action(20, 1, component, P, 1), 'RTL')

    def test_land_to_autopilot_or_broadcast_is_emergency(self):
        self.assertEqual(emergency_action(21, 1, 1, P, 1), 'LAND')
        self.assertEqual(emergency_action(21, 0, 0, P, 1), 'LAND')

    def test_land_to_as2_component_keeps_as2_path(self):
        self.assertIsNone(emergency_action(21, 1, 190, P, 1))

    def test_all_drones_and_other_drones(self):
        self.assertEqual(emergency_action(20, 0, 1, P, 3), 'RTL')     # broadcast: every aircraft
        self.assertIsNone(emergency_action(20, 2, 1, P, 3))           # addressed to drone with system 2

    def test_set_mode_auto_rtl_and_land(self):
        self.assertEqual(emergency_action(176, 1, 190, (1.0, 4.0, 5.0, 0, 0, 0, 0), 1), 'RTL')
        self.assertEqual(emergency_action(176, 1, 190, (1.0, 4.0, 6.0, 0, 0, 0, 0), 1), 'LAND')
        self.assertIsNone(emergency_action(176, 1, 190, (1.0, 6.0, 0.0, 0, 0, 0, 0), 1))  # OFFBOARD
        self.assertIsNone(emergency_action(176, 1, 190, (1.0, 3.0, 0.0, 0, 0, 0, 0), 1))  # POSCTL

    def test_other_commands(self):
        self.assertIsNone(emergency_action(400, 1, 1, P, 1))          # arm/disarm
        self.assertIsNone(emergency_action(22, 1, 1, P, 1))           # takeoff


if __name__ == '__main__':
    unittest.main()
