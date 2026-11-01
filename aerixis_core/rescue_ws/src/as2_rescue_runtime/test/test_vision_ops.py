"""Gimbal-guided operations for the five A8-tracker missions (no ROS, fake driver)."""
import math
from pathlib import Path
import tempfile
import unittest

from as2_rescue_runtime.durable import Journal
from as2_rescue_runtime.engine import Engine
from as2_rescue_runtime.safety import Evidence, Limits
from as2_rescue_runtime.vision_ops import (approach_velocity, orbit_velocity, overhead,
                                           target_estimate)


class Clock:
    def __init__(self):
        self.value = 1000.0

    def __call__(self):
        return self.value


class Driver:
    def __init__(self):
        self.calls = []
        self.result = None
        self.moving = False

    @property
    def idle(self):
        return not self.moving

    def stop(self):
        self.calls.append('stop')
        self.moving = False

    def hold(self):
        return True

    def track(self, target):
        pass

    def velocity(self, linear, yaw, frame):
        self.calls.append(('velocity', [round(v, 3) for v in linear], round(yaw, 3)))
        self.moving = True
        return True

    def vision_command(self, value):
        self.calls.append(('vision', value))

    def move(self, pose, frame, speed, facing=None):
        self.calls.append(('move', pose))
        self.result = 'RUNNING'
        self.moving = True

    def motion_result(self):
        return self.result

    def release(self, *args):
        self.calls.append(('release', args))

    def report(self, value):
        self.calls.append(('report', value))

    def request_route(self, command, pose, frame):
        self.evidence.put('route', dict(approved=True, path_clear=True, mission_id=command['mission_id'],
                                        command_id=command['command_id'], target_pose=pose, frame_id=frame))

    def vertical(self, *args):
        self.result = 'RUNNING'


class GeometryTests(unittest.TestCase):
    def test_range_from_pitch_and_bearing_from_gimbal_yaw(self):
        # 10 m above ground, gimbal 45 deg down, 90 deg right of a north-facing aircraft.
        point, distance, agl = target_estimate([0, 0, 10], math.pi / 2, dict(pitch=-45, yaw=90), (0, 0), 0)
        self.assertAlmostEqual(distance, 10, places=6)
        self.assertAlmostEqual(point[0], 10, places=6)  # east
        self.assertAlmostEqual(point[1], 0, places=6)

    def test_horizon_is_unobservable(self):
        self.assertIsNone(target_estimate([0, 0, 10], 0, dict(pitch=-1, yaw=0), (0, 0), 0))

    def test_nadir_image_error_shifts_estimate(self):
        point, _, _ = target_estimate([0, 0, 10], 0, dict(pitch=-90, yaw=0), (0.0, -0.5), 0)
        self.assertGreater(point[0], 1.0)  # image top = camera bearing (east)

    def test_approach_and_orbit_laws(self):
        vx, vy = approach_velocity([0, 0, 10], [20, 0, 0], 1.5)
        self.assertAlmostEqual(vx, 1.5)
        self.assertAlmostEqual(vy, 0.0)
        vx, vy, _, error = orbit_velocity([10, 0, 10], [0, 0, 0], 10, 1.0)
        self.assertLess(error, 1e-9)
        self.assertAlmostEqual(vx, 0.0, places=6)
        self.assertAlmostEqual(vy, 1.0, places=6)  # counter-clockwise tangent


class EngineVisionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.e = Evidence(self.clock)
        self.d = Driver()
        self.d.evidence = self.e
        self.engine = Engine(self.e, Journal(Path(self.tmp.name) / 'j.json'), self.d, self.clock)
        self.engine.owner = 'mission'
        self.n = 0

    def tearDown(self):
        self.tmp.cleanup()

    def step(self, dt=0.05):
        self.clock.value += dt

    def put(self, position=(0, 0, 10), yaw=0.0, speed=0.0, label='L1', pitch=-45.0, gyaw=0.0,
            err=(0.0, 0.0), locked=True, gps=True):
        self.e.put('vehicle', dict(position=list(position), yaw=yaw, frame_id='earth', position_valid=True,
                                   connected=True, speed=speed, tilt_deg=0, body_rate_dps=0,
                                   vibration_rms=.1, battery=.8, hold_ack=True,
                                   motion_idle=True, platform_state=3))
        self.e.put('vision', dict(fresh=True, locked=locked, label=label if locked else None,
                                  suppressed=False, err_x=err[0], err_y=err[1],
                                  gimbal=dict(yaw=gyaw, pitch=pitch, age_s=0.05)))
        if gps:
            self.e.put('gps', dict(valid=True, latitude=47.0, longitude=8.0, altitude=500.0,
                                   horizontal_sigma_m=1.0))

    def command(self, operation, **extra):
        self.n += 1
        value = dict(schema_version=1, stamp=self.clock(), mission_id='mission', run_id='run',
                     command_id='c%d' % self.n, operation=operation, timeout_s=60, **extra)
        return self.engine.submit(value)

    def geotag(self, **view):
        """GeotagOverhead over several camera frames (it fuses samples for GEOTAG_WINDOW_S)."""
        self.command('GeotagOverhead')
        for _ in range(60):
            result = self.engine.tick()
            if result:
                return result
            self.step(0.1)
            self.put(**view)
        self.fail('geotag did not finish')

    def test_approach_flies_towards_person_then_succeeds_overhead(self):
        self.put(pitch=-45)
        self.assertTrue(self.engine.flags()['HumanDetected'])
        self.command('ApproachOverhead', speed=1.0, lost_timeout_s=3.0)
        self.assertIsNone(self.engine.tick())
        velocity = [c for c in self.d.calls if c[0] == 'velocity'][-1]
        self.assertGreater(velocity[1][0], 0.5)  # east, towards the person 10 m ahead
        for _ in range(40):
            self.step()
            self.put(position=(10, 0, 10), pitch=-89.0)
            result = self.engine.tick()
            if result:
                break
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertIn('stop', self.d.calls)

    def swarm(self, safe=True, fresh=True):
        self.e.put('swarm', dict(stamp=self.clock(), required=True, peers_fresh=fresh, separation_safe=safe,
                                 owned=False, peers=[], keep_out=[]))

    def test_approach_holds_while_a_peer_is_too_close_then_continues(self):
        # Fleet: a peer inside the separation margin (or stale peer data) stops the approach in place.
        self.put(pitch=-45); self.swarm()
        self.command('ApproachOverhead', speed=1.0, lost_timeout_s=3.0)
        self.assertIsNone(self.engine.tick())
        self.assertGreater([c for c in self.d.calls if c[0] == 'velocity'][-1][1][0], 0.5)
        for safe, fresh in ((False, True), (True, False)):
            self.step(); self.put(pitch=-45); self.swarm(safe=safe, fresh=fresh)
            self.assertIsNone(self.engine.tick())
            self.assertEqual([c for c in self.d.calls if c[0] == 'velocity'][-1][1][:2], [0.0, 0.0])
            self.assertIn('holding ApproachOverhead', self.engine.vision.hold['note'])
        self.step(); self.put(pitch=-45); self.swarm()
        self.assertIsNone(self.engine.tick())                       # separation back: approach resumes
        self.assertGreater([c for c in self.d.calls if c[0] == 'velocity'][-1][1][0], 0.5)
        self.assertIsNone(self.engine.vision.hold)

    def test_orbit_hold_is_bounded_and_does_not_count_laps(self):
        from as2_rescue_runtime.vision_ops import SEPARATION_HOLD_S
        self.put(pitch=-45); self.swarm()
        self.command('OrbitTarget', radius_m=10.0, laps=1.0, speed=2.0, lost_timeout_s=600)
        self.engine.active['deadline'] += 1200                    # the trees give OrbitTarget 1200 s
        self.assertIsNone(self.engine.tick())
        result = None
        for _ in range(int(SEPARATION_HOLD_S) + 5):
            self.step(1.0); self.put(pitch=-45); self.swarm(safe=False)
            result = self.engine.tick()
            if result:
                break
            self.assertEqual(self.engine.active.get('swept', 0.0), 0.0)
        self.assertEqual(result['status'], 'FAILURE')
        self.assertIn('SEPARATION_HOLD_TIMEOUT', result['reason'])

    def test_single_drone_response_never_holds(self):
        self.put(pitch=-45)
        self.e.put('swarm', dict(stamp=self.clock(), required=False, peers_fresh=False, separation_safe=False))
        self.command('ApproachOverhead', speed=1.0, lost_timeout_s=3.0)
        self.assertIsNone(self.engine.tick())
        self.assertGreater([c for c in self.d.calls if c[0] == 'velocity'][-1][1][0], 0.5)

    def test_lost_person_fails_tracking_after_timeout(self):
        self.put()
        self.command('TrackTarget', lost_timeout_s=1.0)
        self.assertIsNone(self.engine.tick())
        result = None
        for _ in range(30):
            self.step(0.1)
            self.put(locked=False)
            result = self.engine.tick()
            if result:
                break
        self.assertEqual(result['status'], 'FAILURE')
        self.assertIn('TARGET_LOST', result['reason'])

    def test_geotag_needs_overhead_and_gps_and_excludes_person_afterwards(self):
        self.put(pitch=-60)
        self.command('GeotagOverhead')
        self.assertEqual(self.engine.tick()['status'], 'FAILURE')
        self.step()
        self.put(pitch=-90, gps=False)
        self.e.samples.pop('gps', None)
        self.command('GeotagOverhead')
        self.assertIn('NO_GPS_FIX', self.engine.tick()['reason'])
        self.step()
        self.put(pitch=-90)
        self.assertEqual(self.geotag(pitch=-90)['status'], 'SUCCESS')
        tag = self.engine.geotags()[-1]
        self.assertGreaterEqual(tag['samples'], 5)
        self.assertAlmostEqual(tag['latitude'], 47.0, places=4)
        self.command('ReleaseTargetLock')
        self.assertEqual(self.engine.tick()['status'], 'SUCCESS')
        # Straight back to searching (2026-11-01): a short pause, not the tree's 8 s.
        self.assertIn(('vision', dict(cmd='release', suppress_s=1.5)), self.d.calls)
        self.assertIn(('vision', dict(cmd='set_attitude', yaw=0.0, pitch=self.engine.vision.search_pitch)), self.d.calls)
        # Same label is handled; a new label right next to the geotag is excluded too.
        self.step()
        self.put(pitch=-90, label='L1')
        self.assertFalse(self.engine.flags()['HumanDetected'])
        self.put(pitch=-90, label='L2')
        self.assertFalse(self.engine.flags()['HumanDetected'])
        self.step()
        self.put(position=(60, 0, 10), pitch=-45, label='L3')
        self.assertTrue(self.engine.flags()['HumanDetected'])

    def test_a_relock_on_the_geotagged_person_is_dropped_before_it_is_centred(self):
        # SITL 2026-11-01: after the geotag the tracker re-selected the same person and the camera
        # slewed after them for seconds (the check waited for a centred image).
        self.put(pitch=-90)
        self.assertEqual(self.geotag(pitch=-90)['status'], 'SUCCESS')
        self.command('ReleaseTargetLock')
        self.assertEqual(self.engine.tick()['status'], 'SUCCESS')
        self.step(2.0)
        self.d.calls.clear()
        self.put(position=(2, 0, 10), pitch=-80, label='L2', err=(-0.5, 0.3))   # far from centred
        self.engine.vision.housekeeping(self.clock())
        self.assertIn(('vision', dict(cmd='release', suppress_s=1.0)), self.d.calls)
        # Someone new is kept.
        self.step(2.0)
        self.d.calls.clear()
        self.put(position=(60, 0, 10), pitch=-80, label='L3', err=(-0.5, 0.3))
        self.engine.vision.housekeeping(self.clock())
        self.assertFalse(any(c[0] == 'vision' and c[1].get('cmd') == 'release' for c in self.d.calls))

    def test_geotag_fuses_samples_and_drops_an_outlier(self):
        self.put(pitch=-90)
        self.command('GeotagOverhead')
        positions = [(0.2, 0, 10), (-0.2, 0, 10), (0, 0.2, 10), (0, -0.2, 10), (9, 9, 10), (0.1, 0.1, 10)]
        result = None
        for position in positions * 3:
            self.put(position=position, pitch=-90)
            result = self.engine.tick()
            if result:
                break
            self.step(0.1)
        self.assertEqual(result['status'], 'SUCCESS')
        tag = self.engine.geotags()[-1]
        self.assertGreaterEqual(tag['rejected_samples'], 1)          # the (9, 9) gust frame
        self.assertLess(math.hypot(*tag['local'][:2]), 0.3)
        self.assertLess(tag['sigma_m'], 2.0)

    def test_geotag_keeps_the_first_person_when_the_lock_changes(self):
        self.put(pitch=-90, label='L1')
        self.command('GeotagOverhead')
        self.assertIsNone(self.engine.tick())
        result = None
        for _ in range(40):                       # another person under the camera: no samples
            self.step(0.1)
            self.put(pitch=-90, label='L7')
            result = self.engine.tick()
            if result:
                break
        self.assertEqual(result['status'], 'SUCCESS')                  # after the timeout, 1 sample
        tag = self.engine.geotags()[-1]
        self.assertEqual((tag['label'], tag['samples']), ('L1', 1))

    def test_housekeeping_keeps_the_current_incident_locked(self):
        self.put(pitch=-90)
        self.engine.incident_label = 'L1'  # set by ApproachOverhead when it binds the person
        self.assertEqual(self.geotag(pitch=-90)['status'], 'SUCCESS')
        self.engine.vision.housekeeping(self.clock())
        self.assertFalse(any(c[0] == 'vision' for c in self.d.calls if isinstance(c, tuple)))
        self.engine.incident_label = None  # after CompleteRescue the same person is released
        self.engine.vision.housekeeping(self.clock())
        self.assertTrue(any(c[0] == 'vision' and c[1]['cmd'] == 'release' for c in self.d.calls if isinstance(c, tuple)))

    def test_orbit_completes_laps(self):
        self.put(pitch=-45)
        self.command('OrbitTarget', radius_m=10.0, laps=0.25, speed=2.0)
        result = None
        angle = 0.0
        for _ in range(400):
            self.step(0.1)
            angle += 0.02
            position = (10 + 10 * math.cos(angle), 10 * math.sin(angle), 10)
            # Keep the person (10, 0) centred: compute gimbal pitch/yaw for an east-facing aircraft.
            dx, dy = 10 - position[0], 0 - position[1]
            ground = math.hypot(dx, dy)
            pitch = -math.degrees(math.atan2(10, ground))
            gyaw = math.degrees(-math.atan2(dy, dx))
            self.put(position=position, pitch=pitch, gyaw=gyaw)
            result = self.engine.tick()
            if result:
                break
        self.assertEqual(result['status'], 'SUCCESS')

    def test_late_command_accepted_stale_command_refused(self):
        self.put()
        late = dict(schema_version=1, stamp=self.clock() - 2.0, mission_id='mission', run_id='run',
                    command_id='late', operation='SetSearchArea', area='0,0;1,0;1,1', timeout_s=5)
        self.assertIsNone(self.engine.submit(late))  # loaded computer: 2 s delivery is still valid
        stale = dict(late, command_id='stale', stamp=self.clock() - 5.0)
        with self.assertRaises(ValueError):
            self.engine.submit(stale)

    def test_search_area_ignores_people_outside_the_kml(self):
        # Aircraft at the origin, person 10 m east (pitch -45 at 10 m height).
        self.put(pitch=-45)
        self.assertTrue(self.engine.flags()['HumanDetected'])
        self.command('SetSearchArea', area='-5,-5;5,-5;5,5;-5,5')
        self.assertEqual(self.engine.tick()['status'], 'SUCCESS')
        self.put(pitch=-45)
        self.assertFalse(self.engine.flags()['HumanDetected'])  # person at x=10 is outside
        self.command('SetSearchArea', area='-5,-5;20,-5;20,5;-5,5')
        self.engine.tick()
        self.put(pitch=-45)
        self.assertTrue(self.engine.flags()['HumanDetected'])

    def test_search_area_rejects_bad_polygons(self):
        self.put()
        self.command('SetSearchArea', area='0,0;1,1')
        self.assertEqual(self.engine.tick()['status'], 'FAILURE')

    def test_scan_area_holds_station_and_sweeps_yaw(self):
        self.put(locked=False)
        self.command('ScanArea', duration_s=2.0, yaw_rate_dps=20.0)
        self.assertIsNone(self.engine.tick())  # station = where the aircraft is now (origin)
        yaws = []
        result = None
        for _ in range(60):
            self.step(0.1)
            yaw = yaws[-1] if yaws else 0.0
            self.put(position=(0.3, 0, 10), yaw=yaw, locked=False)
            result = self.engine.tick()
            velocity = [c for c in self.d.calls if c[0] == 'velocity']
            if velocity:
                yaws.append(velocity[-1][2])
            if result:
                break
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertIn('nobody detected', result['reason'])
        self.assertGreater(yaws[-1], 0.3)  # swept counter-clockwise
        self.assertLess(velocity[-1][1][0], 0)  # drifted 0.3 m east: pulled back to the station

    def test_detection_preempts_running_waypoint(self):
        self.put()
        self.command('SurveyWaypoint', segment_id='leg_0', pose='20;0;10', speed=1.0)
        self.engine.tick()
        self.assertEqual(self.engine.active['operation'], 'SurveyWaypoint')
        self.step()
        self.put()
        self.command('PauseCoverage')
        self.assertEqual(self.engine.active['operation'], 'PauseCoverage')
        self.assertEqual(self.engine.outbox[-1]['status'], 'FAILURE')
        self.assertIn('PREEMPTED', self.engine.outbox[-1]['reason'])

    def test_detection_preempts_running_watch(self):
        # A person seen during the watch itself: the pause takes over from ScanArea (it used to
        # be refused, so the response branch failed and the person was released as lost).
        self.put()
        self.command('ScanArea', duration_s=60.0, yaw_rate_dps=12.0)
        self.engine.tick()
        self.assertEqual(self.engine.active['operation'], 'ScanArea')
        self.step()
        self.put()
        self.command('PauseCoverage')
        self.assertEqual(self.engine.active['operation'], 'PauseCoverage')
        self.assertEqual(self.engine.outbox[-1]['status'], 'FAILURE')
        self.assertIn('PREEMPTED', self.engine.outbox[-1]['reason'])

    def test_overhead_drop_requires_verified_envelope_and_geotag(self):
        self.engine.safety.limits = Limits(overhead_envelope_verified=False, allow_operator_load=True,
                                           min_overhead_agl_m=5.0, max_overhead_agl_m=20.0)
        self.put(pitch=-90)
        self.e.put('payload', dict(operator_confirmed=True, consumed=False, payload_id='kit-1'))
        self.command('DropPayload')
        self.assertIn('envelope', self.engine.tick()['reason'])
        self.engine.safety.limits = Limits(overhead_envelope_verified=True, allow_operator_load=True,
                                           min_overhead_agl_m=5.0, max_overhead_agl_m=20.0)
        self.step()
        self.put(pitch=-90)
        self.e.put('payload', dict(operator_confirmed=True, consumed=False, payload_id='kit-1'))
        self.geotag(pitch=-90)
        self.e.put('payload', dict(operator_confirmed=True, consumed=False, payload_id='kit-1'))
        self.command('DropPayload')
        self.assertTrue(self.engine.flags()['OverheadDropReady'])
        self.assertIsNone(self.engine.tick())
        self.assertFalse(any(c[0] == 'release' for c in self.d.calls if isinstance(c, tuple)))  # not in the first tick
        self.step(0.2)
        self.put(pitch=-90)
        self.assertIsNone(self.engine.tick())
        self.assertTrue(any(c[0] == 'release' for c in self.d.calls if isinstance(c, tuple)))
        self.step()
        self.put(pitch=-90)
        self.e.put('payload', dict(transaction_id=self.engine.last_release, operator_only=True,
                                   command_sent=True, payload_id='kit-1'))
        result = self.engine.tick()
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertIn('UNVERIFIED', result['reason'])

    def test_return_to_launch_allowed_after_abort(self):
        self.put(position=(0, 0, 0.2))
        self.engine.set_home(self.e.get('vehicle'))
        self.engine.aborted = True
        self.step()
        self.put(position=(30, 5, 12))
        self.command('ReturnToLaunch', speed=2.0)
        self.assertIsNone(self.engine.tick())
        self.assertIn(('move', [0, 0, 12]), self.d.calls)

    def test_return_on_the_ground_at_launch_succeeds_without_flying(self):
        # Take-off aborted (SITL 2026-10-31): armed, still on the launch spot. Return must succeed
        # so the recovery's landing runs, instead of failing and leaving the drone armed.
        self.put(position=(0, 0, 0.2))
        self.engine.set_home(self.e.get('vehicle'))
        self.step()
        self.put(position=(0.3, -0.2, 0.25))
        vehicle = dict(self.e.get('vehicle'), armed=True); self.step(0.01); self.e.put('vehicle', vehicle)
        self.command('ReturnToLaunch', speed=2.0)
        result = self.engine.tick()
        self.assertEqual(result['status'], 'SUCCESS')
        self.assertIn('take-off aborted', result['reason'])
        self.assertFalse([c for c in self.d.calls if c[0] == 'move'])

    def test_return_from_the_air_near_launch_still_flies(self):
        self.put(position=(0, 0, 0.2))
        self.engine.set_home(self.e.get('vehicle'))
        self.step()
        self.put(position=(1, 1, 12))
        vehicle = dict(self.e.get('vehicle'), armed=True); self.step(0.01); self.e.put('vehicle', vehicle)
        self.command('ReturnToLaunch', speed=2.0)
        self.assertIsNone(self.engine.tick())
        self.assertTrue([c for c in self.d.calls if c[0] == 'move'])

    def test_return_climbs_to_own_height_then_flies_home_level(self):
        # Fleet return: every drone at its own height (rtl.py), climbing in place before the leg home.
        self.put(position=(0, 0, 0.2))
        self.engine.set_home(self.e.get('vehicle'))
        self.engine.vision.return_altitude_m = 35.0
        self.step()
        self.put(position=(30, 5, 12))
        self.command('ReturnToLaunch', speed=2.0)
        self.assertIsNone(self.engine.tick())
        self.assertEqual([c for c in self.d.calls if c[0] == 'move'], [('move', [30, 5, 35.2])])
        self.d.result, self.d.moving = 'SUCCESS', False      # reached the return height
        self.step(); self.put(position=(30, 5, 35.2))
        self.assertIsNone(self.engine.tick())                # climb done
        self.step(); self.put(position=(30, 5, 35.2))
        self.assertIsNone(self.engine.tick())                # next tick: the level leg home
        self.assertEqual([c for c in self.d.calls if c[0] == 'move'][-1], ('move', [0, 0, 35.2]))
        self.d.result, self.d.moving = 'SUCCESS', False
        self.step(); self.put(position=(0, 0, 35.2))
        self.assertEqual(self.engine.tick()['status'], 'SUCCESS')

    def test_return_altitude_rule(self):
        from as2_rescue_runtime import rtl
        self.assertEqual([rtl.return_altitude(rtl.slot_for(sid)) for sid in (1, 2, 3)], [16.0, 26.0, 36.0])
        self.assertEqual([rtl.return_altitude(s, step=3.0) for s in range(3)], [16.0, 19.0, 22.0])   # 3 m (RTK)
        self.assertEqual(rtl.slot_for(0, 'drone4'), 4)
        self.assertEqual(rtl.slot_for(0, 'alpha'), 0)
        with self.assertRaises(ValueError):
            rtl.return_altitude(11)                          # 126 m: refused, not silently capped

    def test_new_mission_takes_over_on_the_ground_after_normal_end(self):
        self.e.put('vehicle', dict(position=[0, 0, 0], yaw=0, frame_id='earth', position_valid=True, connected=True,
                                   speed=0.0, tilt_deg=0, body_rate_dps=0, vibration_rms=.1, battery=.8,
                                   hold_ack=True, motion_idle=True, platform_state=0, armed=False))
        value = dict(schema_version=1, stamp=self.clock(), mission_id='next', run_id='r2', command_id='n1',
                     operation='CheckpointCoverage', timeout_s=5)
        self.engine.submit(value)
        self.assertEqual(self.engine.owner, 'next')
        self.engine.aborted = True  # after an abort the explicit reconcile is required
        value = dict(value, mission_id='third', command_id='n2', stamp=self.clock())
        self.engine.active = None
        with self.assertRaises(ValueError):
            self.engine.submit(value)

    # Coverage lanes (2026-10-31) ------------------------------------------------------------
    def on_leg(self, start, end, entry=False):
        """The aircraft flies a planned SurveyWaypoint leg from start to end."""
        self.engine.active = dict(operation='SurveyWaypoint', command_id='leg', mission_id='mission', run_id='run')
        self.engine.leg = (list(start), list(end))
        self.engine.leg_is_entry = entry

    def test_no_person_response_on_the_way_to_the_first_lane(self):
        self.put(position=(0, 0, 10), pitch=-80.0)
        self.on_leg((0, 0, 10), (50, 0, 10), entry=True)
        self.assertFalse(self.engine.vision.human_detected())
        self.assertIn('first lane', self.engine.vision.deferred)
        self.on_leg((0, 0, 10), (50, 0, 10), entry=False)       # now on a survey lane
        self.assertTrue(self.engine.vision.human_detected())

    def test_person_ahead_on_the_lane_waits_until_abeam(self):
        self.on_leg((0, 0, 10), (50, 0, 10))
        self.put(position=(0, 0, 10), pitch=-50.0)               # person ~8 m ahead along the lane
        self.assertFalse(self.engine.vision.human_detected())
        self.assertIn('ahead on the lane', self.engine.vision.deferred)
        self.step()
        self.put(position=(6, 0, 10), pitch=-80.0)               # lane brought the aircraft abeam
        self.assertTrue(self.engine.vision.human_detected())
        self.assertIsNone(self.engine.vision.deferred)

    def test_person_beyond_the_lane_end_counts_once_the_drone_reaches_the_end(self):
        # A person just past the end of the lane (at the turn): abeam can never happen.
        self.on_leg((0, 0, 10), (20, 0, 10))
        self.put(position=(10, 0, 10), pitch=-45.0)              # person ~20 m on, 10 m past the end
        self.assertFalse(self.engine.vision.human_detected())
        self.step(); self.put(position=(19, 0, 10), pitch=-65.0)  # at the lane end, person ~4.7 m on
        self.assertTrue(self.engine.vision.human_detected())

    def test_person_beside_the_lane_is_a_detection_at_once(self):
        self.on_leg((0, 0, 10), (50, 0, 10))
        self.put(position=(10, 0, 10), pitch=-60.0, gyaw=90.0)    # looking sideways, abeam
        self.assertTrue(self.engine.vision.human_detected())

    def test_no_response_between_take_off_and_the_first_lane(self):
        # SITL 2026-11-01: no leg and no completed lane yet (the moment after take-off).
        self.engine.active = None; self.engine.leg = None; self.engine.last_goal = None
        self.engine.home = dict(position=[0, 0, 0], yaw=0.0, frame_id='earth')   # taken off
        self.put(position=(0, 0, 15), pitch=-60.0)                # a person 9 m away
        self.assertFalse(self.engine.vision.human_detected())
        self.assertIn('first lane', self.engine.vision.deferred)

    def test_watch_and_other_ops_are_not_lane_gated(self):
        self.put(position=(0, 0, 10), pitch=-50.0)
        self.engine.active = dict(operation='ScanArea', command_id='s', mission_id='mission', run_id='run')
        self.engine.leg = ((0, 0, 10), (50, 0, 10)); self.engine.leg_is_entry = True
        self.assertTrue(self.engine.vision.human_detected())

    def test_between_legs_a_far_person_is_not_a_detection(self):
        # SITL 2026-11-01: the entry leg ended with a person locked 53 m away; with no leg
        # running the old gate let that become a diagonal flight across the area.
        self.engine.active = None; self.engine.leg = None; self.engine.last_goal = [0, 0, 10]
        self.put(position=(0, 0, 10), pitch=-10.0)                # ~57 m away
        self.assertFalse(self.engine.vision.human_detected())
        self.assertIn('from the aircraft', self.engine.vision.deferred)
        self.step(); self.put(position=(0, 0, 10), pitch=-60.0)   # ~6 m away: fine
        self.assertTrue(self.engine.vision.human_detected())

    def test_far_or_transit_lock_is_released_but_a_lock_ahead_on_the_lane_is_kept(self):
        v = self.engine.vision
        self.on_leg((0, 0, 10), (50, 0, 10), entry=True)
        self.put(position=(0, 0, 10), pitch=-60.0)
        v.housekeeping(self.clock())
        self.assertIn(('vision', dict(cmd='release', suppress_s=3.0)), self.d.calls)
        self.d.calls.clear(); self.step(5.0)
        self.on_leg((0, 0, 10), (50, 0, 10), entry=False)
        self.put(position=(0, 0, 10), pitch=-50.0)                # ~8 m ahead on the lane
        v.housekeeping(self.clock())
        self.assertNotIn(('vision', dict(cmd='release', suppress_s=3.0)), self.d.calls)

    def test_nadir_search_sweeps_sideways_while_nobody_is_locked(self):
        v = self.engine.vision
        v.search_pitch, v.sweep_deg, v.sweep_period_s = -90.0, 30.0, 8.0
        self.on_leg((0, 0, 10), (50, 0, 10))
        self.put(locked=False)
        sent = []
        for _ in range(80):                                       # one 8 s period at 10 Hz
            self.step(0.1); self.put(locked=False)
            n = len(self.d.calls); v.sweep(self.clock())
            sent += [c[1] for c in self.d.calls[n:] if c[0] == 'vision' and c[1]['cmd'] == 'set_attitude']
        yaws = {c['yaw'] for c in sent}
        self.assertTrue({90.0, -90.0} <= yaws)                    # both sides of the lane
        self.assertGreaterEqual(min(c['pitch'] for c in sent), -90.0)
        self.assertLessEqual(min(c['pitch'] for c in sent), -89.0)  # through straight down
        self.assertGreaterEqual(min(abs(c['pitch']) for c in sent), 60.0 - 1.0)  # at most 30 deg off vertical
        self.assertLessEqual(len(sent), 4 * 8 + 1)                 # rate-limited (<= 4/s)

    def test_planned_sweep_from_the_mission_overrides_the_default(self):
        v = self.engine.vision
        self.put(pitch=-90.0, locked=False)
        self.command('SetGimbalSearch', pitch_deg=-90.0, sweep_deg=0.0, sweep_period_s=8.0)
        self.engine.tick()
        self.assertEqual(v.sweep_deg, 0.0)
        self.assertFalse(v.sweeping())                            # narrow lanes: camera stays still
        self.step(); self.put(pitch=-90.0, locked=False)
        self.command('SetGimbalSearch', pitch_deg=-90.0, sweep_deg=18.5, sweep_period_s=6.0)
        self.engine.tick()
        self.assertEqual((v.sweep_deg, v.sweep_period_s), (18.5, 6.0))

    def test_no_sweep_while_a_person_is_locked_or_with_a_forward_search(self):
        v = self.engine.vision
        v.search_pitch, v.sweep_deg = -90.0, 30.0
        self.on_leg((0, 0, 10), (50, 0, 10))
        self.put(locked=True); self.step(2.0); self.put(locked=True)
        v.sweep(self.clock())
        self.assertFalse([c for c in self.d.calls if c[0] == 'vision'])
        v.search_pitch = -45.0                                     # forward-looking search: no sweep
        self.put(locked=False); self.step(2.0); self.put(locked=False)
        v.sweep(self.clock())
        self.assertFalse([c for c in self.d.calls if c[0] == 'vision'])

    def test_overhead_predicate(self):
        vision = dict(fresh=True, locked=True, label='L1', suppressed=False, err_x=0.02, err_y=0.03,
                      gimbal=dict(yaw=0, pitch=-87, age_s=.1))
        self.assertTrue(overhead(vision))
        self.assertFalse(overhead(dict(vision, err_x=0.4)))
        self.assertFalse(overhead(dict(vision, gimbal=dict(yaw=0, pitch=-70, age_s=.1))))


if __name__ == '__main__':
    unittest.main()
