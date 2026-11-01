"""Several packages per drone (2026-11-01): which actuator set moves to which value for each drop."""
import unittest

from as2_rescue_runtime.payload_node import loaded_package, package_number, release_slots, servo_list


class PayloadSlotTests(unittest.TestCase):
    def test_single_servo_is_the_old_behaviour(self):
        self.assertEqual(release_slots('', 2, 1.0, -1.0), [dict(set=2, step=1, drops=1, value=1.0)])
        self.assertEqual(release_slots('', 0, 1.0), [])              # adapter not configured

    def test_one_servo_per_package(self):
        slots = release_slots('1:1,2:1,3:1', 0, 1.0, -1.0)
        self.assertEqual([(s['set'], s['value']) for s in slots], [(1, 1.0), (2, 1.0), (3, 1.0)])

    def test_stepped_servo_moves_evenly_to_the_release_end(self):
        slots = release_slots('4:4', 0, 1.0, -1.0)
        self.assertEqual([s['set'] for s in slots], [4, 4, 4, 4])
        self.assertEqual([s['value'] for s in slots], [-0.5, 0.0, 0.5, 1.0])

    def test_mixed_servos_go_servo_by_servo(self):
        slots = release_slots('1:1,2:3', 0, 1.0, -1.0)
        self.assertEqual([(s['set'], s['step']) for s in slots], [(1, 1), (2, 1), (2, 2), (2, 3)])
        self.assertAlmostEqual(slots[1]['value'], -1 / 3, places=3)

    def test_invalid_profiles(self):
        for spec in ('7:1', '1:0', '1:21', '1:2,1:3', 'aux1'):
            with self.assertRaises(ValueError):
                servo_list(spec)
        self.assertEqual(release_slots('9:1', 0, 1.0), [])           # invalid -> disabled, not a crash
        with self.assertRaises(ValueError):
            release_slots('1:1', 0, 2.0)

    def test_package_numbers(self):
        self.assertEqual(package_number('kit_001'), 1)
        self.assertEqual(package_number('kit_001#3'), 3)


    def test_loaded_packages_are_counted_per_mission(self):
        load = dict(mission_id='m2', payload_id='kit', count=3, confirmed_loaded=True)
        self.assertEqual(loaded_package(load, {}, 'm2', 3)['payload_id'], 'kit#1')
        done = {'t1': dict(mission='m2', payload_id='kit#1'), 't2': dict(mission='m2', payload_id='kit#2')}
        self.assertEqual(loaded_package(load, done, 'm2', 3)['payload_id'], 'kit#3')
        done['t3'] = dict(mission='m2', payload_id='kit#3')
        self.assertIsNone(loaded_package(load, done, 'm2', 3))                     # all dropped
        self.assertEqual(loaded_package(dict(load, count=5), {}, 'm2', 3)['count'], 3)  # capped at capacity
        self.assertIsNone(loaded_package(load, {}, 'm2', 0))                       # no servo

    def test_a_package_id_from_an_earlier_mission_is_refused(self):
        # SITL 2026-11-01: a reused ID counted the earlier drop and the load started at #2.
        load = dict(mission_id='m2', payload_id='kit', count=3, confirmed_loaded=True)
        self.assertIsNone(loaded_package(load, {'old': dict(mission='m1', payload_id='kit')}, 'm2', 3))
        self.assertEqual(loaded_package(dict(load, count=1), {}, 'm2', 3)['payload_id'], 'kit')


if __name__ == '__main__':
    unittest.main()
