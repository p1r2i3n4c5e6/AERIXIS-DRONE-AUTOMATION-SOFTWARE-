"""Onboard AI avoidance layer: OFF changes nothing, ON flies legs around peers (no ROS needed)."""
import math
from pathlib import Path
import tempfile
import unittest

from as2_rescue_runtime.avoidance import AvoidanceLayer, GATE_BUFFER_M
from as2_rescue_runtime.durable import Journal
from as2_rescue_runtime.engine import Engine
from as2_rescue_runtime.safety import Evidence


class Clock:
    def __init__(self):
        self.value = 1000.0

    def __call__(self):
        return self.value


class Driver:
    idle = True

    def __init__(self):
        self.calls, self.result = [], None

    def stop(self):
        self.calls.append('stop')

    def hold(self):
        return True

    def track(self, target):
        pass

    def move(self, pose, frame, speed, facing=None):
        self.calls.append(('move', pose))
        self.result = 'RUNNING'

    def velocity(self, linear, yaw, frame):
        self.calls.append(('velocity', [round(v, 3) for v in linear], round(yaw, 3)))

    def motion_result(self):
        return self.result

    def request_route(self, command, pose, frame):
        self.evidence.put('route', dict(approved=True, path_clear=True, mission_id=command['mission_id'],
                                        command_id=command['command_id'], target_pose=pose, frame_id=frame))

    def checkpoint(self, *a):
        pass


class AvoidanceLayerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.e = Evidence(self.clock)
        self.d = Driver()
        self.d.evidence = self.e
        self.engine = Engine(self.e, Journal(Path(self.tmp.name) / 'j.json'), self.d, self.clock)
        self.engine.avoidance = AvoidanceLayer()                 # the shipped brain
        self.n = 0

    def tearDown(self):
        self.tmp.cleanup()

    def put(self, position=(0.0, 0.0, 12.0), velocity=(1.5, 0.0, 0.0), peers=()):
        self.e.put('vehicle', dict(position=list(position), velocity=list(velocity), yaw=0.0, frame_id='earth',
                                   position_valid=True, connected=True, speed=math.hypot(*velocity[:2]), tilt_deg=0,
                                   body_rate_dps=0, vibration_rms=.1, battery=.8, hold_ack=True, motion_idle=True,
                                   platform_state=3))
        self.e.put('gps', dict(valid=True, latitude=47.0, longitude=8.0, altitude=500.0, horizontal_sigma_m=1.0))
        self.e.put('swarm', dict(stamp=self.clock(), required=True, peers_fresh=True, separation_safe=True,
                                 owned=False, peers=[], keep_out=[],
                                 neighbours=[dict(owner='drone1', xy=list(p[:2]), z=12.0, velocity=list(v), sigma_m=1.0,
                                                  age_s=0.1) for p, v in peers]))

    def leg(self, pose='60;0;12'):
        self.n += 1
        self.engine.submit(dict(schema_version=1, stamp=self.clock(), mission_id='m', run_id='r', command_id='c%d' % self.n,
                                operation='SurveyWaypoint', segment_id='leg%d' % self.n, pose=pose, speed=1.5,
                                timeout_s=300, swarm_id='g'))

    def velocities(self):
        return [c for c in self.d.calls if c[0] == 'velocity']

    def test_off_is_the_usual_goto(self):
        self.put()
        self.leg()
        self.engine.tick(); self.engine.tick()
        self.assertIn(('move', [60.0, 0.0, 12.0]), self.d.calls)
        self.assertEqual(self.velocities(), [])

    def test_on_bends_right_for_a_head_on_peer_and_finishes_at_the_waypoint(self):
        self.assertEqual(self.engine.avoidance.mode, 'brain')
        self.engine.avoidance.set_enabled(True)
        self.put(peers=[((30.0, 0.5), (-1.5, 0.0))])
        self.leg()
        self.engine.tick(); self.engine.tick()
        self.assertFalse([c for c in self.d.calls if c[0] == 'move'])  # no GoTo: the layer flies the leg
        vy = self.velocities()[-1][1][1]
        self.assertLess(vy, -0.05)                                       # right of +x is -y
        self.assertTrue(self.engine.avoidance.status['active'])
        # Arrive: stop and the leg succeeds like a GoTo would.
        self.clock.value += 0.1
        self.put(position=(59.4, 0.3, 12.1), velocity=(0.3, 0.0, 0.0))
        result = self.engine.tick()
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertIn('stop', self.d.calls)

    def test_switched_off_mid_leg_hands_over_to_goto(self):
        self.engine.avoidance.set_enabled(True)
        self.put()
        self.leg()
        self.engine.tick(); self.engine.tick()
        self.assertTrue(self.velocities())
        self.engine.avoidance.set_enabled(False)
        self.clock.value += 0.1
        self.put(position=(5.0, 0.0, 12.0))
        self.engine.tick()
        self.assertIn(('move', [60.0, 0.0, 12.0]), self.d.calls)

    def test_vehicle_data_as_old_as_the_readiness_check_allows_does_not_fail_the_leg(self):
        # SITL 2026-10-23 (loaded laptop): readiness accepted 0.5-1.0 s old data, the leg code read
        # it with a 0.5 s limit, got None: "'NoneType' object is not subscriptable".
        for on in (False, True):
            self.engine.avoidance.set_enabled(on)
            self.put()
            self.leg()
            self.engine.tick(); self.engine.tick()
            self.clock.value += 0.7
            self.e.put('swarm', dict(stamp=self.clock(), required=True, peers_fresh=True, separation_safe=True,
                                     owned=False, peers=[], keep_out=[], neighbours=[]))
            result = self.engine.tick()
            self.assertIsNone(result, (on, result))
            self.engine.active = None

    def test_ai_return_flies_home_at_mission_height_without_climbing(self):
        self.engine.avoidance.set_enabled(True)
        self.put(position=(0.0, 0.0, 0.2), velocity=(0.0, 0.0, 0.0))
        self.engine.set_home(self.e.get('vehicle'))
        self.engine.vision.return_altitude_m = 16.0
        self.clock.value += 0.1
        self.put(position=(40.0, 5.0, 12.0), velocity=(0.0, 0.0, 0.0))
        self.engine.submit(dict(schema_version=1, stamp=self.clock(), mission_id='m', run_id='r', command_id='rtl',
                                operation='ReturnToLaunch', speed=2.0, timeout_s=300))
        self.engine.tick(); self.engine.tick()
        self.assertFalse([c for c in self.d.calls if c[0] == 'move'])            # no GoTo, no climb leg
        vx, vy, vz = self.velocities()[-1][1]
        self.assertLess(vx, 0.0)                                                 # heading home (west)
        self.assertAlmostEqual(vz, 0.0, places=2)                                # stays at 12 m
        self.clock.value += 0.1
        self.put(position=(0.4, 0.3, 12.0), velocity=(0.2, 0.0, 0.0))
        self.assertIn('mission height', self.engine.tick()['reason'])

    def test_sitl_stuck_run_takeoff_not_armed_then_return_and_land_on_the_ground_end_at_once(self):
        # SITL 2026-10-23 (AI on): PX4 refused to arm, the take-off ran 90 s on the ground, then
        # ReturnToLaunch waited for ever to "arrive" from the ground.
        self.engine.avoidance.set_enabled(True)
        self.put(position=(0.0, 0.0, 0.2), velocity=(0.0, 0.0, 0.0))
        self.e.get('vehicle')['armed'] = False
        self.d.vertical = lambda op, height, speed: self.d.calls.append(('vertical', op))
        self.engine.submit(dict(schema_version=1, stamp=self.clock(), mission_id='m', run_id='r', command_id='to',
                                operation='ManagedTakeoff', height=15, speed=0.5, timeout_s=300))
        self.engine.tick()
        self.assertIn(('vertical', 'ManagedTakeoff'), self.d.calls)
        result = None
        for _ in range(100):
            self.clock.value += 0.1
            self.put(position=(0.0, 0.0, 0.2), velocity=(0.0, 0.0, 0.0))
            self.e.get('vehicle')['armed'] = False
            result = self.engine.tick()
            if result:
                break
        self.assertEqual(result['status'], 'FAILURE')
        self.assertIn('did not arm', result['reason'])
        self.assertLess(self.clock.value - 1000.0, 9.0)
        for op in ('ReturnToLaunch', 'ManagedLand'):
            self.engine.submit(dict(schema_version=1, stamp=self.clock(), mission_id='m', run_id='r', command_id=op,
                                    operation=op, speed=1.0, timeout_s=300))
            result = self.engine.tick()
            self.assertEqual(result['status'], 'SUCCESS', result)
            self.assertIn('Already on the ground', result['reason'])
        self.assertFalse(self.velocities())

    def test_a_refused_leg_fails_with_the_reason_instead_of_hovering(self):
        # SITL 2026-10-23: a leg the route check refused hovered silently for 3 min.
        self.put()
        def refuse(command, pose, frame):
            self.e.put('route', dict(approved=False, path_clear=False, mission_id=command['mission_id'],
                                     command_id=command['command_id'], reason='Requested transit crosses outside/excluded area'))
        self.d.request_route = refuse
        self.leg()
        result = None
        for _ in range(80):
            self.clock.value += 0.1
            self.put()
            result = self.engine.tick()
            if result:
                break
            self.assertIn('outside/excluded', self.engine.waiting)
        self.assertEqual(result['status'], 'FAILURE')
        self.assertIn('Route refused: Requested transit crosses', result['reason'])
        self.assertLess(self.clock.value - 1000.0, 6.0)
        self.assertEqual(self.engine.waiting, '')

    def test_takeoff_without_climb_stops(self):
        self.put(position=(0.0, 0.0, 0.2), velocity=(0.0, 0.0, 0.0))
        self.d.vertical = lambda op, height, speed: None
        self.engine.submit(dict(schema_version=1, stamp=self.clock(), mission_id='m', run_id='r', command_id='to',
                                operation='ManagedTakeoff', height=15, speed=0.5, timeout_s=300))
        result = None
        for _ in range(250):
            self.clock.value += 0.1
            self.put(position=(0.0, 0.0, 0.3), velocity=(0.0, 0.0, 0.0))
            self.e.get('vehicle')['armed'] = True
            result = self.engine.tick()
            if result:
                break
        self.assertIn('No climb', result['reason'])

    def test_landing_after_an_ai_return_waits_for_the_velocity_handover(self):
        self.put()
        self.d.idle = False                                    # velocity mode still handing back
        calls = []
        self.d.vertical = lambda op, height, speed: calls.append(op)
        self.engine.submit(dict(schema_version=1, stamp=self.clock(), mission_id='m', run_id='r', command_id='land',
                                operation='ManagedLand', speed=0.5, timeout_s=300))
        self.assertIsNone(self.engine.tick())
        self.assertEqual(calls, [])
        self.d.idle = True
        self.clock.value += 1.0
        self.put()
        self.engine.tick()
        self.assertEqual(calls, ['ManagedLand'])

    def test_a_model_for_another_brain_version_falls_back_to_orca(self):
        import numpy as np
        path = Path(self.tmp.name) / 'old.npz'
        np.savez(path, w0=np.zeros((39, 64)), b0=np.zeros(64), w1=np.zeros((64, 64)), b1=np.zeros(64),
                 w2=np.zeros((64, 2)), b2=np.zeros(2))
        layer = AvoidanceLayer(str(path))
        self.assertEqual(layer.mode, 'orca')
        self.assertIn('not usable', layer.status['note'])
        self.assertTrue(layer.set_enabled(True))

    def test_ai_return_after_a_leg_reverses_gently_and_keeps_the_nose(self):
        # SITL 2026-10-23: reversing 1.35 m/s in one step with a 180 deg yaw tripped the stability monitor.
        layer = AvoidanceLayer()
        layer.set_enabled(True)
        cmd = {}
        vehicle = dict(position=[40.0, 0.0, 15.0], velocity=[1.35, 0.0, 0.0], yaw=0.0)
        out = []
        for k in range(30):
            velocity, heading, _ = layer.leg_command(cmd, [40.0, 0.0], [0.0, 0.0, 15.0], 2.0, vehicle, None, 0.1,
                                                     100.0 + 0.1 * k, keep_heading=True)
            out.append(velocity[0])
            self.assertEqual(heading, 0.0)
        steps = [abs(b - a) for a, b in zip([1.35] + out, out)]
        self.assertLessEqual(max(steps), 0.15 + 1e-9)          # 1.5 m/s^2 at 10 Hz
        self.assertLess(out[-1], -1.0)                          # still gets going home within 3 s

    def test_neighbour_radius_is_the_gate_margin_plus_buffer(self):
        layer = AvoidanceLayer()
        vehicle = dict(position=[0, 0, 12], velocity=[1.5, 0, 0])
        swarm = dict(stamp=100.0, neighbours=[dict(xy=[40, 0], z=12, velocity=[-1.5, 0], sigma_m=1.0, age_s=0.2)])
        n = layer.neighbours(vehicle, swarm, 1.0, 100.0)[0]
        # 10 m + both sigmas + both speeds x (message age 0.2 s + link latency 0.2 s) + buffer
        self.assertAlmostEqual(n['radius'], 10.0 + 1.0 + 1.0 + (1.5 + 1.5) * (0.2 + 0.2) + GATE_BUFFER_M)
        self.assertAlmostEqual(n['age'], 0.2)
        self.assertEqual(n['pos'], (40 - 1.5 * 0.2, 0.0))                          # extrapolated by its age
        # The simulator the brain was trained in uses the same rule (as2_rescue_avoidance.orca).
        from as2_rescue_avoidance import orca
        trained = orca.neighbour_for_gate((40, 0), (-1.5, 0), 0.2, 1.0, 1.5, 1.0, 10.0)
        self.assertAlmostEqual(trained['radius'], n['radius'])
        self.assertEqual(GATE_BUFFER_M, orca.GATE_BUFFER_M)
        # RTK fleet (3 m): about half the radius, so passes are about half as wide.
        rtk = AvoidanceLayer(min_separation_m=3.0).neighbours(dict(vehicle), dict(swarm, neighbours=[
            dict(swarm['neighbours'][0], sigma_m=0.1)]), 0.1, 100.0)[0]
        self.assertAlmostEqual(rtk['radius'], 3.0 + 0.2 + 3.0 * 0.4 + GATE_BUFFER_M)
        # Peers more than the separation above/below (e.g. still on the ground) are not avoided.
        for z, seen in ((40, 0), (0.2, 0), (4.0, 1)):
            swarm['neighbours'][0].update(z=z)
            self.assertEqual(len(layer.neighbours(vehicle, swarm, 1.0, 100.0)), seen, z)


if __name__ == '__main__':
    unittest.main()
