"""Fleet person de-confliction: claims, peer exclusion and the tie-break (no ROS)."""
import math
from pathlib import Path
import tempfile
import unittest

from as2_rescue_runtime.durable import Journal
from as2_rescue_runtime.engine import Engine
from as2_rescue_runtime.radio import encode
from as2_rescue_runtime.safety import Evidence
from as2_rescue_runtime.swarm import PeerPeople, person_claim, recent_tags
from as2_rescue_runtime.vision_ops import CLAIM_TIE_WINDOW_S, DETECTION_HOLD_S, claim_wins
from test_vision_ops import Clock, Driver


def supervisor(now, claim=None, mission='m1', geotags=()):
    return dict(stamp=now, mission_id=mission, claim=claim, geotags=list(geotags))


class ClaimMessageTests(unittest.TestCase):
    def test_claim_published_only_while_fresh_and_for_the_current_mission(self):
        claim = dict(label='L1', position=[10.04, -3.26], since=99.5, mission_id='m1')
        self.assertEqual(person_claim(supervisor(100.0, claim), 100.2),
                         dict(label='L1', since=99.5, xy=[10.0, -3.3]))
        self.assertIsNone(person_claim(supervisor(100.0, claim), 101.0))  # runtime silent
        self.assertIsNone(person_claim(supervisor(100.0, dict(claim, mission_id='old')), 100.1))
        self.assertIsNone(person_claim(supervisor(100.0, None), 100.1))
        self.assertIsNone(person_claim(None, 100.1))
        # No estimate yet: claim still shown, but peers cannot match it by position.
        self.assertNotIn('xy', person_claim(supervisor(100.0, dict(claim, position=None)), 100.1))

    def test_peer_claim_expires_and_is_released_by_a_frame_without_it(self):
        people = PeerPeople(claim_ttl=3.0)
        people.update('drone0', dict(stamp=100.0, claim=dict(label='L1', since=99.0, xy=[5, 6])), 100.0)
        claims = people.claims(100.5)
        self.assertEqual([(c['owner'], c['label'], c['xy']) for c in claims], [('drone0', 'L1', [5.0, 6.0])])
        self.assertEqual(people.claims(103.5), [])  # not renewed for longer than the TTL
        people.update('drone0', dict(stamp=104.0, claim=dict(label='L1', since=99.0, xy=[5, 6])), 104.0)
        self.assertEqual(len(people.claims(104.1)), 1)  # renewed
        people.update('drone0', dict(stamp=103.0), 104.2)  # older frame: ignored
        self.assertEqual(len(people.claims(104.3)), 1)
        people.update('drone0', dict(stamp=104.4), 104.4)  # released
        self.assertEqual(people.claims(104.5), [])
        people.update('drone1', dict(stamp=105.0, claim=dict(label='L2', since='bad')), 105.0)
        self.assertEqual(people.claims(105.1), [])

    def test_peer_geotags_accumulate_without_duplicates(self):
        tags = [dict(local=[i * 10.0, 1.0, 0.0]) for i in range(6)]
        self.assertEqual(recent_tags(supervisor(1.0, geotags=tags)), [[20.0, 1.0], [30.0, 1.0], [40.0, 1.0], [50.0, 1.0]])
        people = PeerPeople(tag_ttl=60.0)
        people.update('drone0', dict(stamp=1.0, tags=[[0, 0], [10, 0]]), 1.0)
        people.update('drone0', dict(stamp=2.0, tags=[[10.3, 0], [20, 0]]), 2.0)  # 10.3 is the same tag
        self.assertEqual(sorted(t['xy'][0] for t in people.tags(3.0)), [0.0, 10.0, 20.0])
        self.assertEqual(people.tags(70.0), [])

    def test_peer_frame_with_claim_and_tags_stays_small_for_lora(self):
        frame = dict(owner='drone12', position=[123.456789012, -98.7654321, 25.123456789], frame_id='earth',
                     stamp=1729580000.123456, sigma_m=2.5, speed=1.234567,
                     claim=dict(label='L12', since=1729579990.12, xy=[123.4, -98.7]),
                     tags=[[100.1, -50.2], [110.3, -60.4], [120.5, -70.6], [130.7, -80.8]])
        message = dict(sender='drone12', kind='peer', payload=frame)
        self.assertLessEqual(len(encode(message, b'k' * 32, 0.0, ttl=5)), 2)


class TieBreakTests(unittest.TestCase):
    def test_lower_id_wins_inside_the_window_earlier_claim_wins_outside(self):
        mine = dict(since=100.0)
        self.assertTrue(claim_wins(dict(owner='drone0', since=100.5), mine, 'drone1'))
        self.assertFalse(claim_wins(dict(owner='drone1', since=100.5), mine, 'drone0'))
        self.assertTrue(claim_wins(dict(owner='drone1', since=100.0 - CLAIM_TIE_WINDOW_S - .1), mine, 'drone0'))
        self.assertFalse(claim_wins(dict(owner='drone0', since=100.0 + CLAIM_TIE_WINDOW_S + .1), mine, 'drone1'))
        self.assertTrue(claim_wins(dict(owner='drone9', since=500.0), None, 'drone0'))  # no claim of mine

    def test_both_sides_agree_on_one_winner(self):
        for a, b in ((100.0, 100.4), (100.0, 103.0), (103.0, 100.0), (100.0, 100.0)):
            first = dict(owner='drone0', since=a)
            second = dict(owner='drone1', since=b)
            self.assertNotEqual(claim_wins(second, first, 'drone0'), claim_wins(first, second, 'drone1'))


class EngineClaimTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.e = Evidence(self.clock)
        self.d = Driver()
        self.d.evidence = self.e
        self.engine = Engine(self.e, Journal(Path(self.tmp.name) / 'j.json'), self.d, self.clock)
        self.engine.owner = 'mission'
        self.engine.identity = 'drone1'
        self.n = 0

    def tearDown(self):
        self.tmp.cleanup()

    def step(self, dt=0.05):
        self.clock.value += dt

    def put(self, position=(0, 0, 10), label='L1', pitch=-45.0, gyaw=0.0, locked=True, err=(0.0, 0.0)):
        # Person 10 m east of an east-facing aircraft at 10 m when pitch is -45 and gyaw 0.
        self.e.put('vehicle', dict(position=list(position), yaw=0.0, frame_id='earth', position_valid=True,
                                   connected=True, speed=0.0, tilt_deg=0, body_rate_dps=0, vibration_rms=.1,
                                   battery=.8, hold_ack=True, motion_idle=True, platform_state=3))
        self.e.put('vision', dict(fresh=True, locked=locked, label=label if locked else None, suppressed=False,
                                  err_x=err[0], err_y=err[1], gimbal=dict(yaw=gyaw, pitch=pitch, age_s=0.05)))

    def peers(self, claims=(), tags=()):
        self.e.put('swarm', dict(stamp=self.clock(), peer_claims=list(claims), peer_tags=list(tags)))

    def command(self, operation, **extra):
        self.n += 1
        value = dict(schema_version=1, stamp=self.clock(), mission_id='mission', run_id='run',
                     command_id='c%d' % self.n, operation=operation, timeout_s=60)
        value.update(extra)
        return self.engine.submit(value)

    def tick(self):
        result = self.engine.tick()
        self.engine.vision.housekeeping(self.clock())
        return result

    def releases(self):
        return [c for c in self.d.calls if isinstance(c, tuple) and c[0] == 'vision' and c[1]['cmd'] == 'release']

    def test_binding_a_person_claims_them_until_released(self):
        self.put()
        self.command('OrbitTarget', radius_m=10.0, laps=1.0)
        self.assertIsNone(self.tick())
        claim = self.engine.vision.claim_state()
        self.assertEqual((claim['label'], claim['mission_id']), ('L1', 'mission'))
        self.assertAlmostEqual(claim['position'][0], 10.0, places=1)
        self.assertAlmostEqual(claim['since'], 1000.0)
        self.command('CancelOperation', command_id='c1')
        self.command('ReleaseTargetLock')
        self.assertEqual(self.tick()['status'], 'SUCCESS')
        self.assertIsNone(self.engine.vision.claim_state())

    def test_return_to_launch_and_mission_change_release_the_claim(self):
        self.put()
        self.command('TrackTarget')
        self.tick()
        self.assertIsNotNone(self.engine.vision.claim_state())
        self.command('CancelOperation', command_id='c1')
        self.command('ReturnToLaunch')
        self.assertIsNone(self.engine.vision.claim_state())
        self.engine.vision.claim = dict(label='L1', position=[1, 1], since=1.0, mission_id='other')
        self.assertIsNone(self.engine.vision.claim_state())  # never published for another mission

    def test_person_claimed_by_a_peer_is_not_a_detection_and_is_released(self):
        self.put()
        self.assertTrue(self.engine.flags()['HumanDetected'])
        self.step(DETECTION_HOLD_S)  # past the hand-over hold of that detection
        self.put()
        self.peers([dict(owner='drone0', label='L7', since=990.0, xy=[13.0, 4.0])])  # 5 m away
        self.assertFalse(self.engine.flags()['HumanDetected'])
        # Watching (ScanArea running): the lock is dropped and the gimbal looks away anyway.
        self.command('ScanArea', duration_s=60)
        self.tick()
        self.assertTrue(self.releases())
        self.step()
        self.peers([dict(owner='drone0', label='L7', since=990.0, xy=[30.0, 0.0])])  # someone else
        self.assertTrue(self.engine.flags()['HumanDetected'])

    def test_estimate_is_trusted_only_once_the_person_is_centred(self):
        # New selection, gimbal still at the search attitude: the person sits off-centre, the
        # estimate would point 5+ m away from them. With peer claims around, wait for centring.
        self.peers([dict(owner='drone0', label='L7', since=990.0, xy=[30.0, 0.0])])
        self.put(err=(0.6, 0.1))
        self.assertFalse(self.engine.flags()['HumanDetected'])
        self.step()
        self.peers([dict(owner='drone0', label='L7', since=990.0, xy=[30.0, 0.0])])
        self.put(err=(0.05, 0.0))
        self.assertTrue(self.engine.flags()['HumanDetected'])

    def test_just_detected_person_is_not_released_before_the_response_binds(self):
        self.put()
        self.assertTrue(self.engine.flags()['HumanDetected'])
        self.step()
        # Estimate noise moves the person next to a peer claim between detection and binding.
        self.peers([dict(owner='drone0', label='L7', since=990.0, xy=[13.0, 4.0])])
        self.put()
        self.engine.vision.housekeeping(self.clock())
        self.assertEqual(self.releases(), [])
        # Binding claims the person; the tie-break then decides (drone0 claimed first).
        self.command('OrbitTarget', radius_m=10.0, laps=1.0)
        self.tick()
        self.assertEqual(self.engine.vision.yielded['owner'], 'drone0')

    def test_peer_geotag_excludes_the_person(self):
        self.put()
        self.peers(tags=[dict(owner='drone0', xy=[11.0, 1.0])])
        self.assertFalse(self.engine.flags()['HumanDetected'])
        self.step()
        self.peers(tags=[dict(owner='drone0', xy=[40.0, 1.0])])
        self.assertTrue(self.engine.flags()['HumanDetected'])

    def test_simultaneous_claim_higher_id_gives_way_and_moves_clear(self):
        self.put()
        self.command('OrbitTarget', radius_m=10.0, laps=1.0)
        self.tick()
        self.step()
        # drone0 locked the same person 0.5 s later (inside the window): drone0 keeps it.
        self.peers([dict(owner='drone0', label='L3', since=1000.5, xy=[10.5, 0.5])])
        self.put()
        self.assertIsNone(self.tick())
        self.assertIsNone(self.engine.vision.claim)
        self.assertEqual(self.engine.vision.yielded['owner'], 'drone0')
        self.assertIsNone(self.engine.incident_label)
        self.step()
        self.put()
        self.assertIsNone(self.tick())
        cmd = self.engine.active
        self.assertIn('yielded_at', cmd)
        self.assertNotIn('label', cmd)
        self.assertTrue(self.releases())
        # Waits outside drone0's orbit circle (10 m radius) with a 10 m margin.
        self.assertGreaterEqual(math.hypot(cmd['retreat'][0] - 10.5, cmd['retreat'][1] - 0.5), 19.9)
        velocity = [c for c in self.d.calls if c[0] == 'velocity'][-1]
        self.assertLess(velocity[1][0], 0.0)  # moving west, away from the person
        # A person nobody has claimed shows up: this drone orbits them instead.
        self.step()
        self.peers([dict(owner='drone0', label='L3', since=1000.5, xy=[10.5, 0.5])])
        self.put(label='L2', gyaw=180.0)  # 10 m west
        self.assertIsNone(self.tick())
        self.assertEqual(cmd.get('label'), 'L2')
        self.assertNotIn('yielded_at', cmd)
        self.assertAlmostEqual(self.engine.vision.claim['position'][0], -10.0, places=1)

    def test_lower_id_keeps_the_person(self):
        self.engine.identity = 'drone0'
        self.put()
        self.command('OrbitTarget', radius_m=10.0, laps=1.0)
        self.tick()
        self.step()
        self.peers([dict(owner='drone1', label='L3', since=999.8, xy=[10.5, 0.5])])
        self.put()
        self.assertIsNone(self.tick())
        self.assertEqual(self.engine.vision.claim['label'], 'L1')
        self.assertIsNone(self.engine.vision.yielded)
        self.assertNotIn('yielded_at', self.engine.active)

    def test_earlier_claim_keeps_the_person_even_against_a_lower_id(self):
        self.put()
        self.command('OrbitTarget', radius_m=10.0, laps=1.0)
        self.tick()
        self.step(5.0)
        self.peers([dict(owner='drone0', label='L3', since=1005.0, xy=[10.0, 0.0])])
        self.put()
        self.tick()
        self.assertEqual(self.engine.vision.claim['label'], 'L1')

    def test_approach_gives_way_by_failing(self):
        self.put()
        self.command('ApproachOverhead', speed=1.0)
        self.tick()
        self.step()
        self.peers([dict(owner='drone0', label='L3', since=1000.2, xy=[10.0, 0.0])])
        self.put()
        self.tick()  # de-confliction runs after the operation tick
        self.step()
        self.put()
        result = self.tick()
        self.assertEqual(result['status'], 'FAILURE')
        self.assertIn('YIELDED', result['reason'])

    def test_give_way_search_is_bounded(self):
        self.put()
        self.command('TrackTarget', timeout_s=600)
        self.tick()
        self.step()
        self.peers([dict(owner='drone0', label='L3', since=1000.1, xy=[10.0, 0.0])])
        self.put()
        self.tick()
        result = None
        for _ in range(130):
            self.step(1.0)
            self.peers([dict(owner='drone0', label='L3', since=1000.1, xy=[10.0, 0.0])])
            self.put(locked=False)
            result = self.tick()
            if result:
                break
        self.assertEqual(result['status'], 'FAILURE')
        self.assertIn('nobody else found', result['reason'])


if __name__ == '__main__':
    unittest.main()
