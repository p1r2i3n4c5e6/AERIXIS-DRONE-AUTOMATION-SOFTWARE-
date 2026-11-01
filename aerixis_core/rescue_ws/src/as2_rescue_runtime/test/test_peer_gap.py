"""Fleet survey legs hold position through a short peer-data gap (SITL 2026-11-01: one late mesh
frame / a paused swarm-agent status ended drone1's search after 2 of 9 lanes, twice)."""
import math
import unittest

from as2_rescue_runtime.engine import PEER_GAP_HOLD_S
import test_avoidance_layer as base   # module import: pytest must not collect its tests twice


class PeerGapTests(unittest.TestCase):
    setUp, tearDown, put, leg = (base.AvoidanceLayerTests.setUp, base.AvoidanceLayerTests.tearDown,
                                 base.AvoidanceLayerTests.put, base.AvoidanceLayerTests.leg)

    def vehicle_only(self, peers_fresh=None):
        """Vehicle evidence; swarm status missing (None) or present but with stale peers (False)."""
        self.e.put('vehicle', dict(position=[10.0, 0.0, 12.0], velocity=[0.0, 0.0, 0.0], yaw=0.0, frame_id='earth',
                                   position_valid=True, connected=True, speed=0.0, tilt_deg=0, body_rate_dps=0,
                                   vibration_rms=.1, battery=.8, hold_ack=True, motion_idle=True, platform_state=3))
        if peers_fresh is not None:
            self.e.put('swarm', dict(stamp=self.clock(), required=True, peers_fresh=peers_fresh, separation_safe=False,
                                     owned=False, peers=[], keep_out=[], neighbours=[]))

    def moves(self):
        return [c for c in self.d.calls if isinstance(c, tuple) and c[0] == 'move']

    def start_leg(self):
        self.put()
        self.leg()
        self.engine.tick(); self.engine.tick()
        self.assertEqual(len(self.moves()), 1)

    def gap(self, seconds, peers_fresh):
        result = None
        for _ in range(int(seconds / 0.1)):
            self.clock.value += 0.1
            self.vehicle_only(peers_fresh)
            result = self.engine.tick()
            if result:
                return result
        return result

    def test_short_gaps_hold_position_and_the_leg_continues(self):
        self.start_leg()
        for peers_fresh in (None, False):                 # swarm status paused / a peer frame late
            self.assertIsNone(self.gap(2.0, peers_fresh))
            self.assertIn('stop', self.d.calls)
            self.assertIn('holding position', self.engine.waiting)
            self.assertTrue(self.engine.flags()['SeparationSafe'])   # the tree's gates stay satisfied
            moves = len(self.moves())
            self.clock.value += 0.1
            self.put()
            self.engine.tick(); self.engine.tick()
            self.assertEqual(len(self.moves()), moves + 1)  # leg sent again
            self.assertFalse(self.engine.flags()['PeerGapHold'])

    def test_a_long_gap_still_ends_the_leg(self):
        self.start_leg()
        result = self.gap(PEER_GAP_HOLD_S + 2, None)
        self.assertEqual(result['status'], 'FAILURE')
        self.assertIn('Peer freshness/separation unavailable', result['reason'])

    def test_fresh_data_saying_too_close_fails_at_once(self):
        self.start_leg()
        self.clock.value += 0.1
        self.put()
        self.e.samples['swarm'] = (dict(self.e.get('swarm'), separation_safe=False), self.clock(), self.clock())
        self.assertIn('Peer freshness/separation unavailable', self.engine.tick()['reason'])

    def test_a_later_single_drone_mission_does_not_need_peers(self):
        self.put()
        self.engine.flags()                               # a fleet mission's swarm status was seen
        self.assertTrue(self.engine.peer_required)
        self.engine.submit(dict(schema_version=1, stamp=self.clock(), mission_id='solo', run_id='r2', command_id='s1',
                                operation='SurveyWaypoint', segment_id='solo1', pose='30;0;12', speed=1.5, timeout_s=300))
        for _ in range(3):
            self.clock.value += 0.1
            self.vehicle_only(None)
            result = self.engine.tick()
            self.assertFalse(result and result.get('status') == 'FAILURE', result)


if __name__ == '__main__':
    unittest.main()


class EdgeGoalTests(unittest.TestCase):
    """AI avoidance: a lane end on the area's edge (unreachable inside the shield's edge margin)."""
    def test_held_still_close_to_the_goal_counts_as_reached(self):
        from as2_rescue_runtime.avoidance import AvoidanceLayer, STALL_TIME_S
        layer = AvoidanceLayer()
        vehicle = dict(position=[138.0, -131.5, 15.0], velocity=[0.0, 0.0, 0.0], yaw=0.0)
        swarm = dict(stamp=100.0, neighbours=[])
        cmd = {}
        _, _, reached = layer.leg_command(cmd, [98.0, -131.5], [140.0, -131.5, 15.0], 2.0, vehicle, swarm, 1.0, 100.0)
        self.assertFalse(reached)
        _, _, reached = layer.leg_command(cmd, [98.0, -131.5], [140.0, -131.5, 15.0], 2.0, vehicle, swarm, 1.0, 100.0 + STALL_TIME_S)
        self.assertTrue(reached)

    def test_still_moving_or_far_is_not_reached(self):
        from as2_rescue_runtime.avoidance import AvoidanceLayer
        layer = AvoidanceLayer()
        swarm = dict(stamp=100.0, neighbours=[])
        moving = dict(position=[138.0, -131.5, 15.0], velocity=[1.0, 0.0, 0.0], yaw=0.0)
        far = dict(position=[130.0, -131.5, 15.0], velocity=[0.0, 0.0, 0.0], yaw=0.0)
        for vehicle in (moving, far):
            cmd = {}
            for t in (100.0, 110.0):
                _, _, reached = layer.leg_command(cmd, [98.0, -131.5], [140.0, -131.5, 15.0], 2.0, vehicle, swarm, 1.0, t)
            self.assertFalse(reached)
