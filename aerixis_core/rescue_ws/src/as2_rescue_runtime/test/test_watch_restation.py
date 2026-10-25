"""Mission 2 watch next to a peer's orbit: keep-out zones, re-station choice, hold-and-resume,
never entering the margin, bounded wait (no ROS)."""
import math
from pathlib import Path
import tempfile
import unittest

from as2_rescue_runtime.durable import Journal
from as2_rescue_runtime.engine import Engine
from as2_rescue_runtime.radio import encode
from as2_rescue_runtime.safety import Evidence
from as2_rescue_runtime.swarm import PeerPeople, keep_out_zones, person_claim, recent_tags
from as2_rescue_runtime.vision_ops import PASSING_WAIT_S, ROUTE_REPLY_S, WATCH_HOLD_S
from as2_rescue_runtime.watch import STATION_CLEARANCE_M, choose_station, leg_blocker
from as2_rescue_planner.watch import parse
from test_vision_ops import Clock, Driver

# The 2-drone SITL geometry: drone1 watches (0,20); drone0 orbits person B at (-5,16), r = 10 m.
PLANNED = [0.0, 20.0, 12.0]
AREA = '-35,5;35,5;-35,25;35,25;0,5;35,15;-35,15;drone0=0,-20'
B = [-5.0, 16.0]
ORBIT = dict(owner='drone0', kind='orbit', label='L7', xy=B, radius_m=25.0)
CLAIM = dict(owner='drone0', label='L7', since=990.0, xy=B, r=10.0, age_s=0.1)


def peer_at(xy, radius=15.0, owner='drone0'):
    return dict(owner=owner, kind='peer', xy=list(xy), radius_m=radius)


class KeepOutZoneTests(unittest.TestCase):
    OWN = dict(position=[0.0, 0.0, 12.0], frame_id='earth', sigma_m=1.0, speed=0.0)

    def peer(self, **kw):
        value = dict(owner='drone0', position=[10.0, 0.0, 12.0], frame_id='earth', stamp=100.0, sigma_m=2.0, speed=1.0)
        value.update(kw)
        return value

    def test_orbit_zone_adds_the_orbit_radius_to_the_gate_margin(self):
        zones = keep_out_zones(self.OWN, [self.peer()], [dict(owner='drone0', label='L1', xy=[20.0, 5.0], r=12.0)],
                               100.2, 10.0, 1.0, buffer_m=3.0)
        peer, orbit = zones
        # minimum separation + own sigma + peer sigma + peer speed x (age + one period) + buffer
        self.assertEqual((peer['kind'], peer['xy']), ('peer', [10.0, 0.0]))
        self.assertAlmostEqual(peer['radius_m'], 10 + 1 + 2 + 1.0 * 1.2 + 3, places=1)
        self.assertEqual((orbit['kind'], orbit['xy'], orbit['label']), ('orbit', [20.0, 5.0], 'L1'))
        self.assertAlmostEqual(orbit['radius_m'], 12 + peer['radius_m'], places=1)

    def test_claim_without_position_is_centred_on_its_peer_one_orbit_diameter_wider(self):
        zones = keep_out_zones(self.OWN, [self.peer()], [dict(owner='drone0', label='L1', xy=None, r=10.0)],
                               100.2, 10.0, 1.0)
        self.assertEqual(zones[1]['xy'], [10.0, 0.0])
        self.assertAlmostEqual(zones[1]['radius_m'], 20 + zones[0]['radius_m'], places=1)

    def test_unknown_radius_uses_the_default_and_unusable_peers_are_skipped(self):
        zones = keep_out_zones(self.OWN, [self.peer(stamp=90.0), self.peer(owner='drone2', frame_id='map')],
                               [dict(owner='drone0', label='L1', xy=[0.0, 30.0], r=None)], 100.0, 10.0, 1.0,
                               default_radius_m=10.0, default_sigma_m=2.5)
        # Both peer frames unusable (too old / other frame): only the claim, with default radius and sigma.
        self.assertEqual([z['kind'] for z in zones], ['orbit'])
        self.assertAlmostEqual(zones[0]['radius_m'], 10 + 10 + 1 + 2.5 + 3, places=1)

    def test_claim_carries_the_orbit_radius_over_the_mesh(self):
        supervisor = dict(stamp=100.0, mission_id='m1',
                          claim=dict(label='L1', position=[5.0, 6.0], since=99.0, mission_id='m1', radius_m=9.2))
        sent = person_claim(supervisor, 100.1)
        self.assertEqual(sent['r'], 10)  # rounded up, never down
        people = PeerPeople()
        people.update('drone0', dict(stamp=100.0, claim=sent), 100.0)
        self.assertEqual(people.claims(100.1)[0]['r'], 10.0)
        overhead = dict(supervisor, claim=dict(supervisor['claim'], radius_m=0.0))
        self.assertNotIn('r', person_claim(overhead, 100.1))
        frame = dict(owner='drone12', position=[123.456789012, -98.7654321, 25.123456789], frame_id='earth',
                     stamp=1729580000.123456, sigma_m=2.5, speed=1.234567,
                     claim=dict(label='L12', since=1729579990.12, xy=[123.4, -98.7], r=10),
                     tags=[[100.1, -50.2], [110.3, -60.4], [120.5, -70.6], [130.7, -80.8]])
        self.assertLessEqual(len(encode(dict(sender='drone12', kind='peer', payload=frame), b'k' * 32, 0.0, ttl=5)), 2)

    def test_handled_people_ride_with_the_geotags(self):
        supervisor = dict(geotags=[dict(local=[1.0, 2.0, 0.0])], handled=[[-5.04, 16.02]])
        self.assertEqual(recent_tags(supervisor), [[1.0, 2.0], [-5.0, 16.0]])


class StationChoiceTests(unittest.TestCase):
    OPTIONS = parse(AREA)

    def test_leg_never_enters_a_zone_and_from_inside_only_moves_out(self):
        zone = dict(xy=[0.0, 0.0], radius_m=10.0)
        self.assertIsNotNone(leg_blocker([-20, 1], [20, 1], [zone]))  # crosses it
        self.assertIsNone(leg_blocker([-20, 11], [20, 11], [zone]))  # passes outside
        self.assertIsNone(leg_blocker([3, 0], [30, 0], [zone]))  # inside, moving straight out
        self.assertIsNotNone(leg_blocker([3, 0], [-30, 0], [zone]))  # inside, moving deeper
        self.assertIsNotNone(leg_blocker([20, 0], [11, 0], [dict(zone, radius_m=12.0)]))  # ends inside

    def test_planned_station_first_while_it_is_clear(self):
        self.assertEqual(choose_station([16, -3], PLANNED, self.OPTIONS, [])[0], PLANNED[:2])

    def test_busy_peers_station_when_its_orbit_covers_the_planned_one(self):
        choice = choose_station([16, -3], PLANNED, self.OPTIONS, [ORBIT], busy=['drone0'])
        self.assertEqual(choice, ([0.0, -20.0], 'station of drone0 (busy with a person)'))

    def test_otherwise_nearest_clear_alternate_in_own_share(self):
        # (0,5) lies inside the orbit zone, (-35,25) is clear but its leg crosses the orbit.
        xy, why = choose_station([16, -3], PLANNED, self.OPTIONS, [ORBIT])
        self.assertEqual((xy, why), ([35.0, 25.0], 'alternate station in own share'))
        self.assertGreaterEqual(math.dist(xy, B), ORBIT['radius_m'] + STATION_CLEARANCE_M)
        rejected = choose_station([16, -3], PLANNED, self.OPTIONS, [ORBIT], rejected=[[35.0, 25.0]])
        self.assertEqual(rejected[0], [35.0, 15.0])

    def test_waits_for_a_passing_peer_before_taking_a_worse_station(self):
        crossing = peer_at([8.0, -12.0], radius=6.0)  # sits on the leg (16,-3) -> (0,-20)
        waiting = choose_station([16, -3], PLANNED, self.OPTIONS, [ORBIT, crossing], busy=['drone0'], patient=True)
        self.assertIsNone(waiting[0])
        self.assertIn('waiting for drone0 to pass', waiting[1])
        impatient = choose_station([16, -3], PLANNED, self.OPTIONS, [ORBIT, crossing], busy=['drone0'])
        self.assertEqual(impatient[1], 'alternate station in own share')

    def test_nothing_clear_gives_none(self):
        everywhere = dict(owner='drone0', kind='orbit', xy=[0.0, 0.0], radius_m=500.0)
        self.assertIsNone(choose_station([600, 0], PLANNED, self.OPTIONS, [everywhere], busy=['drone0']))


class WatchEngineTests(unittest.TestCase):
    """The runtime flying the watch, with the vehicle following the commanded velocity."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.e = Evidence(self.clock)
        self.d = Driver()
        self.d.evidence = self.e
        self.routes = []
        approve = self.d.request_route

        def request_route(command, pose, frame):
            self.routes.append((command, list(pose)))
            if self.refuse is None or not self.refuse(command, pose):
                approve(command, pose, frame)
            else:
                self.e.put('route', dict(approved=False, path_clear=False, mission_id=command['mission_id'],
                                         command_id=command['command_id'], reason='Re-station goal is not a staged watch station'))
        self.d.request_route = request_route
        self.refuse = None
        self.engine = Engine(self.e, Journal(Path(self.tmp.name) / 'j.json'), self.d, self.clock)
        self.engine.owner = 'mission'
        self.engine.identity = 'drone1'
        self.position = [16.0, -3.0, 12.0]
        self.zones = [ORBIT, peer_at([-15.0, 16.0])]
        self.claims = [CLAIM]
        self.fresh = self.safe = True
        self.n = 0
        self.closest = float('inf')
        self.put()

    def tearDown(self):
        self.tmp.cleanup()

    def put(self):
        self.e.put('vehicle', dict(position=list(self.position), yaw=0.0, frame_id='earth', position_valid=True,
                                   connected=True, speed=0.0, tilt_deg=0, body_rate_dps=0, vibration_rms=.1,
                                   battery=.8, hold_ack=True, motion_idle=True, platform_state=3))
        self.e.put('swarm', dict(stamp=self.clock(), required=True, peers_fresh=self.fresh, separation_safe=self.safe,
                                 peer_claims=list(self.claims), peer_tags=[], keep_out=list(self.zones)))

    def update(self):
        """Publish changed evidence (a sample with an unchanged stamp is ignored)."""
        self.clock.value += 0.05
        self.put()

    def command(self, operation, **extra):
        self.n += 1
        value = dict(schema_version=1, stamp=self.clock(), mission_id='mission', run_id='run',
                     command_id='c%d' % self.n, operation=operation, timeout_s=900)
        value.update(extra)
        return self.engine.submit(value)

    def scan(self, **extra):
        return self.command('ScanArea', duration_s=600.0, yaw_rate_dps=12.0, pose='0;20;12', area=AREA,
                            speed=1.0, segment_id='drone1_station', **extra)

    def margin(self):
        """Smallest distance to the edge of any zone (negative: inside one)."""
        return min((math.dist(self.position[:2], z['xy']) - z['radius_m'] for z in self.zones), default=float('inf'))

    def fly(self, seconds, dt=0.1, until=None):
        result = None
        for _ in range(int(seconds / dt)):
            result = self.engine.tick()
            if result or (until and until()):
                break
            velocity = [c for c in self.d.calls if isinstance(c, tuple) and c[0] == 'velocity']
            if velocity and self.d.moving:
                vx, vy, _ = velocity[-1][1]
                self.position[0] += vx * dt
                self.position[1] += vy * dt
            self.clock.value += dt
            self.put()
            self.closest = min(self.closest, self.margin())
        return result

    def watch(self):
        return self.engine.vision.watch

    def test_station_leg_stops_short_of_a_peer_orbit_and_does_not_record_the_segment(self):
        self.command('SurveyWaypoint', segment_id='drone1_station', pose='0;20;12', speed=1.0)
        result = self.engine.tick()
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertIn('STATION_BLOCKED: drone0 orbit around L7', result['reason'])
        self.assertIn('stop', self.d.calls)
        self.assertNotIn(('move', [0.0, 20.0, 12.0]), self.d.calls)
        self.assertNotIn('drone1_station', self.engine.journal.state['missions'].get('mission', {}).get('completed', []))
        self.assertEqual(self.watch()['state'], 'stopped short')

    def test_station_leg_unchanged_without_an_orbit_and_gate_as_strict_as_before(self):
        self.zones = [peer_at([-15.0, 16.0])]  # a peer aircraft alone never stops the planned leg
        self.claims = []
        self.update()
        self.command('SurveyWaypoint', segment_id='drone1_station', pose='0;20;12', speed=1.0)
        self.assertIsNone(self.engine.tick())
        self.assertIn(('move', [0.0, 20.0, 12.0]), self.d.calls)
        self.safe = False
        self.update()
        result = self.engine.tick()
        self.assertEqual((result['status'], result['reason']), ('FAILURE', 'Peer freshness/separation unavailable'))

    def test_watch_moves_to_the_busy_peers_station_without_entering_any_zone(self):
        self.scan()
        self.assertIsNone(self.fly(120, until=lambda: (self.watch() or {}).get('state') == 'watching'))
        self.assertEqual(self.watch()['note'], 'watch re-stationed (peer orbit nearby): station of drone0 (busy with a person)')
        self.assertLess(math.dist(self.position[:2], [0, -20]), 1.0)
        self.assertGreaterEqual(self.closest, 0.0)
        legs = {c['command_id'] for c, _ in self.routes}
        self.assertEqual(legs, {'c1:restation:1'})
        self.assertTrue(all(c['operation'] == 'Restation' and pose == [0.0, -20.0, 12.0] for c, pose in self.routes))
        self.assertIsNone(self.fly(5))  # keeps watching there

    def test_leg_stops_short_of_a_peer_crossing_it_and_resumes_once_clear(self):
        self.scan()
        self.fly(8)  # leg approved and flying south-west
        self.assertEqual(self.watch()['state'], 're-stationing')
        ahead = [self.position[0] + 0.4 * (0 - self.position[0]), self.position[1] + 0.4 * (-20 - self.position[1])]
        self.zones = [ORBIT, peer_at(ahead, radius=3.0)]  # a peer turns up on the leg
        self.fly(3)
        self.assertGreaterEqual(self.closest, 0.0)
        self.assertIn(self.watch()['state'], ('choosing', 'waiting'))
        stopped = list(self.position)
        self.fly(5)
        self.assertLess(math.dist(stopped, self.position), 1.0)  # holds, does not push on
        self.zones = [ORBIT]  # the peer has passed
        self.assertIsNone(self.fly(120, until=lambda: self.watch()['state'] == 'watching'))
        self.assertLess(math.dist(self.position[:2], [0, -20]), 1.0)
        self.assertGreaterEqual(self.closest, 0.0)

    def test_holds_clear_when_no_station_is_clear_and_resumes_when_the_claim_is_released(self):
        self.position = [60.0, 60.0, 12.0]
        self.zones = [dict(ORBIT, xy=[0.0, 0.0], radius_m=60.0)]
        self.update()
        self.scan()
        self.fly(5)
        self.assertEqual(self.watch()['state'], 'holding')
        self.assertIn('no clear watch station', self.watch()['note'])
        self.assertLess(math.dist(self.position[:2], [60, 60]), 0.5)
        self.assertFalse(self.routes)  # never asked to move
        self.zones = []  # orbit finished: claim released
        self.claims = []
        self.assertIsNone(self.fly(200, until=lambda: self.watch()['state'] == 'watching'))
        self.assertEqual(self.watch()['note'], 'watching planned station')
        self.assertLess(math.dist(self.position[:2], PLANNED[:2]), 1.0)

    def test_gives_up_after_the_bounded_wait(self):
        self.position = [60.0, 60.0, 12.0]
        self.zones = [dict(ORBIT, xy=[0.0, 0.0], radius_m=60.0)]
        self.update()
        self.scan()
        result = self.fly(WATCH_HOLD_S + 5)
        self.assertEqual(result['status'], 'FAILURE')
        self.assertIn('WATCH_BLOCKED', result['reason'])
        self.assertIn('stop', self.d.calls)

    def test_route_refusal_skips_that_station(self):
        self.refuse = lambda command, pose: pose[:2] == [0.0, -20.0]
        self.scan()
        self.assertIsNone(self.fly(150, until=lambda: (self.watch() or {}).get('state') == 'watching'))
        self.assertLess(math.dist(self.position[:2], [35, 25]), 1.0)
        self.assertGreaterEqual(self.closest, 0.0)
        first = [c for c, p in self.routes if p[:2] == [0.0, -20.0]]
        self.assertTrue(first)

    def test_claim_appearing_over_the_station_moves_the_watch(self):
        self.position = list(PLANNED)
        self.zones, self.claims = [], []
        self.update()
        self.scan()
        self.fly(3)
        self.assertEqual(self.watch()['note'], 'watching planned station')
        self.zones, self.claims = [ORBIT], [CLAIM]
        self.assertIsNone(self.fly(120, until=lambda: 'watch re-stationed' in self.watch()['note']))
        # Starting inside the orbit zone, every leg only took it farther out.
        self.assertGreater(math.dist(self.position[:2], B), ORBIT['radius_m'])

    def test_separation_loss_fails_a_flying_leg_but_never_starts_one(self):
        self.safe = False
        self.update()
        self.scan()
        self.assertIsNone(self.fly(5))
        self.assertLess(math.dist(self.position[:2], [16, -3]), 0.5)
        self.safe = True
        self.fly(8)
        self.assertEqual(self.watch()['state'], 're-stationing')
        self.safe = False
        result = self.fly(2)
        self.assertEqual((result['status'], result['reason']), ('FAILURE', 'Peer freshness/separation unavailable'))

    def test_passing_wait_is_bounded_then_another_station_is_taken(self):
        self.zones = [ORBIT, peer_at([8.0, -12.0], radius=6.0)]  # parked on the leg to drone0's station
        self.update()
        self.scan()
        self.fly(PASSING_WAIT_S - 2)
        self.assertEqual(self.watch()['state'], 'waiting')
        self.assertIsNone(self.fly(150, until=lambda: self.watch()['state'] == 'watching'))
        self.assertIn('alternate station in own share', self.watch()['note'])
        self.assertGreaterEqual(self.closest, 0.0)

    def test_single_aircraft_watch_is_unchanged(self):
        self.e.samples.pop('swarm')
        self.position = list(PLANNED)
        self.put = lambda: self.e.put('vehicle', dict(position=list(self.position), yaw=0.0, frame_id='earth',
                                                      position_valid=True, connected=True, speed=0.0, tilt_deg=0,
                                                      body_rate_dps=0, vibration_rms=.1, battery=.8, hold_ack=True,
                                                      motion_idle=True, platform_state=3))
        self.update()
        self.command('ScanArea', duration_s=2.0, yaw_rate_dps=12.0, pose='0;20;12', speed=1.0)
        result = self.fly(5)
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertFalse(self.routes)


class HandledPersonTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.e = Evidence(self.clock)
        self.d = Driver()
        self.d.evidence = self.e
        self.engine = Engine(self.e, Journal(Path(self.tmp.name) / 'j.json'), self.d, self.clock)
        self.engine.owner = 'mission'

    def tearDown(self):
        self.tmp.cleanup()

    def put(self, label):
        self.e.put('vehicle', dict(position=[0, 0, 10], yaw=0.0, frame_id='earth', position_valid=True,
                                   connected=True, speed=0.0, tilt_deg=0, body_rate_dps=0, vibration_rms=.1,
                                   battery=.8, hold_ack=True, motion_idle=True, platform_state=3))
        self.e.put('vision', dict(fresh=True, locked=True, label=label, suppressed=False, err_x=0.0, err_y=0.0,
                                  gimbal=dict(yaw=0.0, pitch=-45.0, age_s=0.05)))

    def submit(self, operation, command_id, **extra):
        self.engine.submit(dict(schema_version=1, stamp=self.clock(), mission_id='mission', run_id='run',
                                command_id=command_id, operation=operation, timeout_s=60, **extra))

    def test_orbited_person_is_published_as_handled_and_not_detected_again(self):
        self.put('L1')
        self.submit('OrbitTarget', 'c1', radius_m=12.0, laps=1.0)
        self.assertIsNone(self.engine.tick())
        self.assertEqual(self.engine.vision.claim_state()['radius_m'], 12.0)
        self.submit('CancelOperation', 'c1')
        self.submit('ReleaseTargetLock', 'c2')
        self.assertEqual(self.engine.tick()['status'], 'SUCCESS')
        self.assertEqual(self.engine.vision.handled_points, [[10.0, 0.0]])
        self.clock.value += 1.0
        self.put('L9')  # the tracker re-selects the same person under a new label
        self.assertFalse(self.engine.flags()['HumanDetected'])


if __name__ == '__main__':
    unittest.main()
