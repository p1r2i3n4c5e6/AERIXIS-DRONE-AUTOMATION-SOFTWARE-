"""Mission archive and mission time (2026-11-01): ~/AERIXIS_Missions/{Simulation,Real_Hardware}/
Mission_N/<drone>/..., opened by the first start, closed CLOSE_AFTER_S after the last drone ended."""
import csv
import json
import tempfile
import time
import unittest
from pathlib import Path

from mission_archive import MissionArchive, duration_text


def status(active, state='RUNNING', start=1000.0, end=None, mission='m1', op='SurveyWaypoint',
           pos=(0.0, 0.0, 15.0), tags=(), payload=None):
    rescue = dict(active_operation=op if active else None, vehicle=dict(position=list(pos), speed=2.0, battery=0.9, armed=active),
                  gps=dict(latitude=28.67, longitude=77.49, altitude=230.0),
                  geotags=[dict(id=f'{mission}-L{i}', label=f'L{i}', latitude=28.67 + i * 1e-4, longitude=77.49, local=[i, 0, 0])
                           for i in tags])
    return dict(active=active, execution=dict(mission_id=mission, state=state, started_at=start, finished_at=end, detail='x'),
                ros=dict(rescue=dict(data=rescue), links=dict(payload=payload) if payload else {}))


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        root = Path(self.tmp.name)
        self.sim_state, self.px4 = root / 'as2-sitl', root / 'tmp'
        self.archive = MissionArchive(root / 'AERIXIS_Missions', self.sim_state, self.px4, close_after_s=30)

    def tearDown(self):
        self.tmp.cleanup()

    def fly(self, sim_mode=True, t0=1000.0):
        a = self.archive
        a.note_stage(dict(drone_id='drone0', action='stage', xml='<root/>', gcs_label='Arena search (search)',
                          route_bundle=dict(x=1)))
        drones = lambda active, state, end, x, tags, pk: [
            dict(drone_id='drone0', label='Sim 1', fresh=True, data=status(active, state, t0, end, pos=(x, 0, 15), tags=tags)),
            dict(drone_id='drone2', label='Sim 3', fresh=True, data=status(active, state, t0 + 10, end, mission='m2', op='FlyToTarget',
                                                                          payload=dict(loaded=10, remaining=pk)))]
        a.tick(drones(False, 'VALIDATED', None, 0, (), 10), sim_mode, now=t0 - 5)
        self.assertIsNone(a.current)                               # nothing runs: no mission yet
        for k in range(10):
            a.tick(drones(True, 'RUNNING', None, 2.0 * k, (1,) if k > 3 else (), 10 - k // 2), sim_mode, now=t0 + 1 + k)
        status_now = a.status(now=t0 + 11)
        self.assertTrue(status_now['running'])
        self.assertGreaterEqual(status_now['mission'], 1)
        end = t0 + 600
        a.tick(drones(False, 'SUCCESS', end, 18, (1, 2), 0), sim_mode, now=end + 1)
        reset = drones(False, 'SUCCESS', end, 18, (1, 2), 0)
        reset[1]['data']['ros']['links']['payload'] = dict(loaded=0, remaining=0)   # drone resets after the mission
        a.tick(reset, sim_mode, now=end + 2)
        self.assertTrue(a.status(now=end + 2)['running'])          # closes only after CLOSE_AFTER_S
        return a.tick(drones(False, 'SUCCESS', end, 18, (1, 2), 0), sim_mode, now=end + 40)

    def test_simulation_mission_folder_and_time(self):
        summary = self.fly(True)
        folder = Path(summary['folder'])
        self.assertEqual(folder.parent.name, 'Simulation')
        self.assertEqual(folder.name, 'Mission_1')
        self.assertEqual(summary['mission_time'], '10:00')        # first start -> last end
        self.assertEqual(summary['geotags'], 2)
        self.assertEqual(summary['packages_delivered'], 10)
        d2 = json.loads((Path(summary['folder']) / 'drone2' / 'status.json').read_text())
        self.assertEqual((d2['packages_delivered'], d2['packages_loaded']), (10, 10))
        d0 = json.loads((folder / 'drone0' / 'status.json').read_text())
        self.assertEqual(d0['result'], 'SUCCESS')
        self.assertEqual(d0['mission_name'], 'Arena search (search)')
        self.assertEqual(d0['duration'], '10:00')
        rows = list(csv.reader(open(folder / 'drone0' / 'track.csv')))
        self.assertGreater(len(rows), 5)
        self.assertEqual(len(json.loads((folder / 'drone0' / 'geotags.json').read_text())), 2)
        self.assertEqual((folder / 'drone0' / 'tree.xml').read_text(), '<root/>')
        self.assertIn('Mission time  10:00', (folder / 'summary.txt').read_text())
        self.assertFalse(self.archive.status()['running'])

    def test_numbering_and_real_hardware_folder(self):
        self.fly(True)
        second = self.fly(True, t0=5000.0)
        self.assertEqual(Path(second['folder']).name, 'Mission_2')
        real = self.fly(False, t0=9000.0)
        self.assertEqual(Path(real['folder']).parent.name, 'Real_Hardware')
        self.assertEqual(Path(real['folder']).name, 'Mission_1')
        self.assertIn('stay on each aircraft', real['onboard_logs'])
        history = self.archive.history()
        self.assertEqual([h['mode'] for h in history], ['Real_Hardware', 'Simulation', 'Simulation'])

    def test_simulation_copies_the_onboard_files(self):
        missions = self.sim_state / 'missions' / 'drone0'
        missions.mkdir(parents=True)
        (missions / 'm1.xml').write_text('<root/>')
        (missions / 'm1.events.jsonl').write_text('{"e":1}\n' * 1000)
        logs = self.sim_state / 'logs'
        logs.mkdir()
        (logs / 'drone0_onboard.log').write_text('[x] [INFO] [100.0] [old]\n[x] [INFO] [1000.5] [in]\nno stamp follows\n'
                                                 '[x] [INFO] [9999.0] [after]\n')
        summary = self.fly(True)
        deadline = time.time() + 10
        while self.archive.copying and time.time() < deadline:
            time.sleep(0.05)
        out = Path(summary['folder']) / 'drone0' / 'onboard'
        self.assertTrue((out / 'm1.xml').is_file())
        self.assertTrue((out / 'm1.events.jsonl.gz').is_file())
        self.assertEqual((out / 'drone0_onboard.log').read_text(), '[x] [INFO] [1000.5] [in]\nno stamp follows\n')

    def test_duration_text(self):
        self.assertEqual(duration_text(65), '1:05')
        self.assertEqual(duration_text(3725), '1:02:05')
        self.assertEqual(duration_text(None), '—')


if __name__ == '__main__':
    unittest.main()
