import json
import unittest

from as2_mavlink_gateway.protocol import (
    enu_to_global,
    enu_to_ned,
    global_to_enu,
    manual_control_to_enu,
    mission_progress,
    mission_to_as2_json,
    ned_to_enu,
)


HOME = (47.397742, 8.545594, 488.0)


class ProtocolTest(unittest.TestCase):
    def test_local_frame_conversions_round_trip(self):
        self.assertEqual(enu_to_ned((2.0, 3.0, 4.0)), (3.0, 2.0, -4.0))
        self.assertEqual(ned_to_enu((3.0, 2.0, -4.0)), (2.0, 3.0, 4.0))
        global_position = enu_to_global(12.0, -7.0, 2.5, *HOME)
        local_position = global_to_enu(*global_position, *HOME)
        for actual, expected in zip(local_position, (12.0, -7.0, 2.5)):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_manual_control_is_bounded(self):
        forward, left, up, yaw = manual_control_to_enu(2000, -2000, 1500, -2000)
        self.assertEqual((forward, left, up, yaw), (1.0, 1.0, 0.5, 0.5))
        self.assertEqual(manual_control_to_enu(0, 0, 500, 0), (0, 0, 0, 0))

    def test_mission_translation_and_field_validation(self):
        items = [
            {'command': 22, 'frame': 6, 'x': 0, 'y': 0, 'z': 2.0},
            {'command': 16, 'frame': 4, 'x': 5.0, 'y': 6.0, 'z': 2.0},
            {'command': 21, 'frame': 4, 'x': 0.0, 'y': 0.0, 'z': 0.0},
        ]
        mission = json.loads(mission_to_as2_json('drone0', items, HOME))
        self.assertEqual(mission['target'], 'drone0')
        # Default: the waypoints become one smooth trajectory (the gateway's use_trajectory).
        self.assertEqual([i['behavior'] for i in mission['plan']],
                         ['takeoff', 'trajectory_generation', 'land'])
        self.assertEqual(mission['plan'][1]['args']['path'], [[5.0, 6.0, 2.0]])
        stop_and_go = json.loads(mission_to_as2_json('drone0', items, HOME, use_trajectory=False))
        self.assertEqual([i['behavior'] for i in stop_and_go['plan']],
                         ['takeoff', 'go_to', 'land'])
        items[1]['x'] = 31.0
        with self.assertRaisesRegex(ValueError, 'outside'):
            mission_to_as2_json('drone0', items, HOME)

    def test_mission_rejects_unsupported_commands(self):
        items = [
            {'command': 22, 'frame': 6, 'x': 0, 'y': 0, 'z': 2.0},
            {'command': 16, 'frame': 4, 'x': 2.0, 'y': 2.0, 'z': 2.0},
            {'command': 17, 'frame': 4, 'x': 0.0, 'y': 0.0, 'z': 0.0},
            {'command': 21, 'frame': 4, 'x': 0.0, 'y': 0.0, 'z': 0.0},
        ]
        with self.assertRaisesRegex(ValueError, 'Unsupported'):
            mission_to_as2_json('drone0', items, HOME)

    def test_mission_progress_from_as2_status(self):
        status = '{"done_items":2,"pending_items":3,"current_item":{"behavior":"go_to"}}'
        self.assertEqual(mission_progress(status), (2, 6))
        self.assertEqual(mission_progress('{"done_items":5,"pending_items":0}'), (5, 5))


if __name__ == '__main__':
    unittest.main()


class FlightModeTests(unittest.TestCase):
    """GCS mode buttons (2026-10-25): heartbeat shows PX4's real mode; requests map to PX4 modes."""

    def test_heartbeat_custom_mode_from_nav_state(self):
        from as2_mavlink_gateway.protocol import px4_custom_mode
        self.assertEqual(px4_custom_mode(2), 3 << 16)                    # POSCTL
        self.assertEqual(px4_custom_mode(1), 2 << 16)                    # ALTCTL
        self.assertEqual(px4_custom_mode(14), 6 << 16)                   # OFFBOARD
        self.assertEqual(px4_custom_mode(4), (4 << 16) | (3 << 24))      # AUTO_LOITER
        self.assertEqual(px4_custom_mode(5), (4 << 16) | (5 << 24))      # AUTO_RTL
        self.assertEqual(px4_custom_mode(18), (4 << 16) | (6 << 24))     # AUTO_LAND
        self.assertIsNone(px4_custom_mode(99))

    def test_requested_mode(self):
        from as2_mavlink_gateway.protocol import requested_mode
        self.assertEqual(requested_mode([1, 3, 0]), 'POSCTL')
        self.assertEqual(requested_mode([1, 2, 0]), 'ALTCTL')
        self.assertEqual(requested_mode([1, 6, 0]), 'OFFBOARD')
        self.assertIsNone(requested_mode([1, 4, 5]))     # AUTO RTL: emergency path, not here
