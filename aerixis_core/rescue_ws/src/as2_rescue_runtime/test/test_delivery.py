"""Mission 7 delivery drones (2026-11-01): AcquireTarget claims the nearest geotagged person nobody
serves (search drones' geotags heard over the mesh), FlyToTarget flies over it, DropAtTarget
releases once; the served person stays claimed so the other delivery drone takes the next one."""
import unittest

from as2_rescue_runtime.safety import Limits
from as2_rescue_runtime.vision_ops import TARGET_SETTLE_S
import test_vision_ops as base   # module import: pytest must not collect its tests twice


class DeliveryTests(unittest.TestCase):
    setUp, tearDown, step, put, command = (base.EngineVisionTests.setUp, base.EngineVisionTests.tearDown,
                                           base.EngineVisionTests.step, base.EngineVisionTests.put,
                                           base.EngineVisionTests.command)

    def mesh(self, tags=(), claims=()):
        """Swarm status as swarm_agent publishes it: peers' geotags and live claims."""
        self.e.put('swarm', dict(stamp=self.clock(), required=True, peers_fresh=True, separation_safe=True,
                                 owned=False, peers=[], keep_out=[],
                                 peer_tags=[dict(owner='drone0', xy=list(xy)) for xy in tags],
                                 peer_claims=[dict(c) for c in claims]))

    def on_ground(self, tags=(), claims=(), position=(0, 0, 0.2)):
        self.step(0.5)
        self.put(position=position, locked=False)
        self.mesh(tags, claims)

    def acquire(self, tags, claims=()):
        self.on_ground(tags, claims)
        self.command('AcquireTarget')
        self.assertIsNone(self.engine.tick())
        for _ in range(int(TARGET_SETTLE_S / 0.5) + 2):
            self.on_ground(tags, claims)
            result = self.engine.tick()
            if result:
                return result
        self.fail('AcquireTarget did not finish')

    def test_waits_on_the_ground_until_a_geotag_arrives(self):
        self.on_ground()
        self.command('AcquireTarget')
        for _ in range(5):
            self.assertIsNone(self.engine.tick())
            self.on_ground()
        self.assertIsNone(self.engine.vision.claim)
        result = None
        for _ in range(int(TARGET_SETTLE_S / 0.5) + 3):        # a search drone geotags someone
            self.on_ground([(10, 0)])
            result = self.engine.tick() or result
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertEqual(self.engine.target, 'T10_0')

    def test_claims_the_nearest_unserved_person(self):
        result = self.acquire([(30, 0), (10, 0), (-40, 5)])
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertEqual(self.engine.target, 'T10_0')
        self.assertEqual(self.engine.vision.claim_state()['position'], [10.0, 0.0])

    def test_a_person_another_drone_serves_is_skipped(self):
        claim = dict(owner='drone3', label='T10_0', since=self.clock() - 10, xy=[10.0, 0.0])
        self.acquire([(10, 0), (30, 0)], claims=[claim])
        self.assertEqual(self.engine.target, 'T30_0')

    def test_simultaneous_claim_lower_drone_id_keeps_it_the_other_takes_the_next(self):
        # This drone is drone3; drone2 claims the same person in the same instant: drone2 keeps it.
        self.engine.identity = 'drone3'
        tags = [(10, 0), (30, 0)]
        self.on_ground(tags)
        self.command('AcquireTarget')
        self.assertIsNone(self.engine.tick())
        mine = self.engine.vision.claim
        rival = dict(owner='drone2', label='T10_0', since=mine['since'], xy=[10.0, 0.0])
        for _ in range(int(TARGET_SETTLE_S / 0.5) + 4):
            self.on_ground(tags, [rival])
            self.engine.vision.housekeeping(self.clock())     # deconflict, as runtime_node does
            result = self.engine.tick()
            if result:
                break
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertEqual(self.engine.target, 'T30_0')

    def fly_to(self, target=(10.0, 0.0)):
        self.acquire([target])
        self.step(); self.put(position=(0, 0, 15), locked=False); self.mesh([target])
        self.command('FlyToTarget', speed=2.0)
        self.assertIsNone(self.engine.tick())
        velocity = [c for c in self.d.calls if c[0] == 'velocity'][-1]
        self.assertGreater(velocity[1][0], 0.5)                  # towards the target (east)
        for _ in range(40):
            self.step(0.1); self.put(position=(target[0], target[1], 15), locked=False); self.mesh([target])
            result = self.engine.tick()
            if result:
                return result
        self.fail('FlyToTarget did not arrive')

    def test_flies_over_the_claimed_person(self):
        self.assertEqual(self.fly_to()['status'], 'SUCCESS')

    def test_flight_stops_if_the_target_is_taken(self):
        self.acquire([(10, 0)])
        self.engine.vision.release_claim()                       # lost to a peer
        self.step(); self.put(position=(0, 0, 15), locked=False); self.mesh([(10, 0)])
        self.command('FlyToTarget', speed=2.0)
        self.assertIn('TARGET_TAKEN', self.engine.tick()['reason'])

    def test_drops_once_over_the_target_and_keeps_it_claimed(self):
        self.engine.safety.limits = Limits(overhead_envelope_verified=True, allow_operator_load=True,
                                           min_overhead_agl_m=5.0, max_overhead_agl_m=20.0)
        self.fly_to()
        self.step(); self.put(position=(10.2, 0.1, 15), locked=False); self.mesh([(10, 0)])
        self.e.put('payload', dict(operator_confirmed=True, consumed=False, payload_id='kit-1'))
        self.command('DropAtTarget')
        self.assertTrue(self.engine.flags()['OverheadDropReady'])
        self.assertIsNone(self.engine.tick())
        self.assertFalse(any(c[0] == 'release' for c in self.d.calls))
        self.step(0.2); self.put(position=(10.2, 0.1, 15), locked=False); self.mesh([(10, 0)])
        self.assertIsNone(self.engine.tick())
        self.assertTrue(any(c[0] == 'release' for c in self.d.calls))
        self.step(); self.put(position=(10.2, 0.1, 15), locked=False); self.mesh([(10, 0)])
        self.e.put('payload', dict(transaction_id=self.engine.last_release, operator_only=True,
                                   command_sent=True, payload_id='kit-1'))
        result = self.engine.tick()
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertTrue(any(c[0] == 'report' and c[1]['kind'] == 'delivery' for c in self.d.calls))
        # ReturnToLaunch clears the live claim; the served person stays claimed for the peers.
        self.engine.vision.release_claim()
        self.assertEqual(self.engine.vision.claim_state()['label'], 'T10_0')

    def test_no_drop_off_target_or_without_envelope(self):
        self.engine.safety.limits = Limits(overhead_envelope_verified=False, allow_operator_load=True,
                                           min_overhead_agl_m=5.0, max_overhead_agl_m=20.0)
        self.fly_to()
        self.step(); self.put(position=(10, 0, 15), locked=False); self.mesh([(10, 0)])
        self.e.put('payload', dict(operator_confirmed=True, consumed=False, payload_id='kit-1'))
        self.command('DropAtTarget')
        self.assertIn('envelope', self.engine.tick()['reason'])
        self.engine.safety.limits = Limits(overhead_envelope_verified=True, allow_operator_load=True,
                                           min_overhead_agl_m=5.0, max_overhead_agl_m=20.0)
        self.step(); self.put(position=(13, 0, 15), locked=False); self.mesh([(10, 0)])
        self.e.put('payload', dict(operator_confirmed=True, consumed=False, payload_id='kit-1'))
        self.command('DropAtTarget')
        self.assertFalse(self.engine.flags()['OverheadDropReady'])
        self.assertIn('Not steady over', self.engine.tick()['reason'])
        self.assertFalse(any(c[0] == 'release' for c in self.d.calls))


if __name__ == '__main__':
    unittest.main()
