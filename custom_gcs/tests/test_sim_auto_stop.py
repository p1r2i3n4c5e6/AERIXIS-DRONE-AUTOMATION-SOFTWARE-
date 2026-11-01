"""Simulation auto-stop (2026-11-01): PX4 SITL + Gazebo stop once every mission has ended and every
drone is landed and disarmed for AUTO_STOP_GRACE_S; never on a fresh idle simulation, never while
a drone flies or its state is unknown, never when switched off or in Field mode."""
import tempfile
import unittest
from pathlib import Path

import sim_control
from sim_control import AUTO_STOP_GRACE_S, SimControl, fleet_finished


def drone(i, active=False, armed=False, state=1, fresh=True, age=0.2):
    return dict(drone_id='drone%d' % i, label='Sim %d' % (i + 1), fresh=fresh,
                data=dict(active=active, ros=dict(platform=dict(armed=armed, state=state), platform_age_s=age)))


class AutoStopTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.saved = sim_control.STATE_FILE
        sim_control.STATE_FILE = Path(self.tmp.name) / 'simulation.json'
        self.sim = SimControl()
        self.sim.state = 'running'
        self.stops = []
        self.sim.stop = lambda reason='': self.stops.append(reason)
        self.sim.running_components = lambda: {'px4': True}

    def tearDown(self):
        sim_control.STATE_FILE = self.saved
        self.tmp.cleanup()

    def run_for(self, drones, seconds, start=0.0, sim_fleet=True):
        t = start
        while t <= start + seconds:
            self.sim.auto_stop_tick(drones, sim_fleet, now=t)
            t += 2.0
        return t

    def test_stops_60_s_after_all_missions_ended_and_drones_landed(self):
        t = self.run_for([drone(0, active=True, armed=True, state=3), drone(1)], 10)
        self.assertEqual(self.stops, [])
        t = self.run_for([drone(0), drone(1, state=0)], AUTO_STOP_GRACE_S - 4, start=t)
        self.assertEqual(self.stops, [])
        self.assertIsNotNone(self.sim.auto_status(now=t)['stops_in_s'])
        self.run_for([drone(0), drone(1, state=0)], 8, start=t)
        self.assertEqual(len(self.stops), 1)
        self.assertIn('stopped automatically', self.stops[0])

    def test_a_fresh_idle_simulation_is_never_stopped(self):
        self.run_for([drone(0), drone(1)], 3 * AUTO_STOP_GRACE_S)
        self.assertEqual(self.stops, [])

    def test_not_while_a_drone_is_armed_flying_or_unknown(self):
        t = self.run_for([drone(0, active=True)], 4)
        for case in ([drone(0, armed=True)], [drone(0, state=3)], [drone(0, fresh=False)], [drone(0, age=10)],
                     [dict(drone_id='drone0', fresh=True, data=dict(active=False))]):
            t = self.run_for(case, 2 * AUTO_STOP_GRACE_S, start=t)
            self.assertEqual(self.stops, [], case)
            self.assertFalse(fleet_finished(case)[0])

    def test_a_new_mission_resets_the_countdown(self):
        t = self.run_for([drone(0, active=True)], 4)
        t = self.run_for([drone(0)], AUTO_STOP_GRACE_S - 10, start=t)
        t = self.run_for([drone(0, active=True, armed=True, state=3)], 4, start=t)
        self.run_for([drone(0)], AUTO_STOP_GRACE_S - 10, start=t)
        self.assertEqual(self.stops, [])

    def test_off_keep_running_and_field_mode(self):
        self.sim.set_auto_stop(False)
        t = self.run_for([drone(0, active=True)], 4)
        t = self.run_for([drone(0)], 2 * AUTO_STOP_GRACE_S, start=t)
        self.assertEqual(self.stops, [])
        self.sim.set_auto_stop(True)
        self.sim.keep_running()
        t = self.run_for([drone(0)], 2 * AUTO_STOP_GRACE_S, start=t)
        self.assertEqual(self.stops, [])
        self.sim.auto['skip'] = False
        self.run_for([drone(0)], 2 * AUTO_STOP_GRACE_S, start=t, sim_fleet=False)
        self.assertEqual(self.stops, [])

    def test_setting_is_saved(self):
        self.sim.set_auto_stop(False)
        self.assertFalse(SimControl().settings['auto_stop'])


if __name__ == '__main__':
    unittest.main()
