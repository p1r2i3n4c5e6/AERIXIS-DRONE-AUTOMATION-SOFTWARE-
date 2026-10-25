"""Tests for the route, surface and camera-transform publishers.

These are software checks with synthetic inputs. No aircraft was armed, no
motion was commanded and no payload was actuated. The A8 gimbal, range sensor
and mounting offsets must still be measured on real hardware.

ROS is initialised once for the whole module: rclpy permits a single context per
process, so per-test init/shutdown is not valid.
"""
import json
import math
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from std_msgs.msg import String

from as2_rescue_runtime.camera_tf_node import (CameraTransform, optical_rotation,
                                               quaternion_from_matrix)
from as2_rescue_runtime.route_node import RoutePublisher
from as2_rescue_runtime.surface_node import SurfaceEstimator


def setUpModule():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/plumbing_test'])


def tearDownModule():
    if rclpy.ok():
        rclpy.shutdown()


def spin_for(node, seconds, cadence=0.02):
    """Spin the node briefly so timers and subscriptions can run."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=cadence)


def capture(node, topic):
    """Subscribe to a topic on the node itself and collect parsed JSON."""
    received = []
    node.create_subscription(String, topic,
                             lambda m: received.append(json.loads(m.data)), 10)
    return received


def rotate_vector(quaternion, vector):
    """Rotate a vector by a quaternion, for physical direction assertions."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    cross = (y * vz - z * vy, z * vx - x * vz, x * vy - y * vx)
    return [vector[i] + 2 * w * cross[i]
            + 2 * (y * cross[2] - z * cross[1], z * cross[0] - x * cross[2],
                   x * cross[1] - y * cross[0])[i]
            for i in range(3)]


REGION = {'name': 'square',
          'shell': [[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
          'holes': [], 'exclusions': []}


class OpticalRotationTests(unittest.TestCase):
    """Pure geometry; these need no ROS context at all."""

    def test_orthonormal_and_right_handed_everywhere(self):
        for yaw in (-180, -90, 0, 45, 90, 180):
            for pitch in (-90, -60, -45, -20, 0, 20, 60, 90):
                with self.subTest((yaw, pitch)):
                    rotation = optical_rotation(yaw, pitch)
                    for i in range(3):
                        for j in range(3):
                            dot = sum(rotation[k][i] * rotation[k][j] for k in range(3))
                            self.assertAlmostEqual(dot, 1.0 if i == j else 0.0, places=9)
                    determinant = (rotation[0][0] * (rotation[1][1] * rotation[2][2]
                                                     - rotation[1][2] * rotation[2][1])
                                   - rotation[0][1] * (rotation[1][0] * rotation[2][2]
                                                       - rotation[1][2] * rotation[2][0])
                                   + rotation[0][2] * (rotation[1][0] * rotation[2][1]
                                                       - rotation[1][1] * rotation[2][0]))
                    self.assertAlmostEqual(determinant, 1.0, places=9)

    def test_nadir_axis_points_down(self):
        rotation = optical_rotation(0.0, -90.0)
        self.assertAlmostEqual(rotation[0][2], 0.0, places=9)
        self.assertAlmostEqual(rotation[1][2], 0.0, places=9)
        self.assertAlmostEqual(rotation[2][2], -1.0, places=9)

    def test_level_axis_points_forward(self):
        rotation = optical_rotation(0.0, 0.0)
        self.assertAlmostEqual(rotation[0][2], 1.0, places=9)
        self.assertAlmostEqual(rotation[2][2], 0.0, places=9)

    def test_quaternion_is_unit_length(self):
        for yaw, pitch in ((0, -90), (37, -41), (-120, 15), (180, 90)):
            with self.subTest((yaw, pitch)):
                quaternion = quaternion_from_matrix(optical_rotation(yaw, pitch))
                self.assertAlmostEqual(sum(v * v for v in quaternion), 1.0, places=9)


class RoutePublisherTests(unittest.TestCase):
    """A route approval must never authorise motion it cannot justify."""

    def _write_inputs(self, root, waypoints=None):
        bundle = root / 'bundle.json'
        value={
            'schema_version': 1, 'revision': 1, 'drone_id': 'drone0',
            'coverage': {'altitude_m': 15.0, 'waypoints': waypoints or [
                {'segment_id': 'leg_0_start', 'pose': [10.0, 10.0, 15.0]},
                {'segment_id': 'leg_0_end', 'pose': [90.0, 10.0, 15.0]},
            ]},
        }
        from as2_rescue_runtime.templates import mission,xml
        import hashlib
        points=[w['pose'] for w in value['coverage']['waypoints']]
        text=xml(mission('waypoints',points,'group',segment_ids=[w['segment_id'] for w in value['coverage']['waypoints']]))
        value.update(drone_id='route_test',region=REGION,origin=dict(latitude=0.,longitude=0.),xml_sha256=hashlib.sha256(text.encode()).hexdigest())
        value['coverage']['frame_id']='earth'
        from as2_rescue_planner.bundle import body_checksum
        value['bundle_sha256']=body_checksum(value)
        (root/('a'*32+'.xml')).write_text(text)
        (root/('a'*32+'.bundle.json')).write_text(json.dumps(dict(mission_id='a'*32,xml_sha256=value['xml_sha256'],bundle=value)))
        bundle.write_text(json.dumps(value))
        region = root / 'region.json'
        region.write_text(json.dumps(REGION))
        return str(bundle), str(region)

    def _publish(self, bundle, region, _clear=(), _omit_region=False, **overrides):
        """Build the route publisher and capture what it advertises.

        ``region_file`` is consumed in the constructor, so a test that needs the
        node to have no bound region must omit the parameter entirely rather
        than clear it afterwards. Empty ROS parameter overrides are rejected by
        rclpy, hence ``_clear`` for the post-construction cases.
        """
        arguments = ['--ros-args', '-r', '__ns:=/route_test',
                     '-p', 'bundle_directory:=' + str(Path(bundle).parent),
                     '-p', 'origin_verified:=true','-p','altitude_frame_verified:=true']
        for key, value in overrides.items():
            arguments += ['-p', '%s:=%s' % (key, value)]
        # The arguments must reach the node; building them and discarding them
        # would silently test the unconfigured defaults.
        node = RoutePublisher(args=arguments)
        if _omit_region:
            path=Path(bundle).parent/('a'*32+'.bundle.json')
            value=json.loads(path.read_text());value['bundle'].pop('region');path.write_text(json.dumps(value))
        received = capture(node, 'rescue/route')
        from geometry_msgs.msg import PoseStamped
        value=json.loads(Path(bundle).read_text());leg=value['coverage']['waypoints'][0]
        def request():
            pose=PoseStamped();pose.header.frame_id='earth';pose.header.stamp=node.get_clock().now().to_msg()
            pose.pose.position.x=10.;pose.pose.position.y=10.;pose.pose.position.z=15.;node.on_pose(pose)
            if not _clear:node.on_request(String(data=json.dumps(dict(stamp=node.now(),mission_id='a'*32,
                command_id='request-one',operation='SurveyWaypoint',segment_id=leg['segment_id'],target_pose=leg['pose'],frame_id='earth'))))
        node.create_timer(.05,request)
        spin_for(node, .45)
        return node, received

    def test_approves_inside_region_segment(self):
        with tempfile.TemporaryDirectory() as tmp:
            bundle, region = self._write_inputs(Path(tmp))
            node, messages = self._publish(bundle, region)
            try:
                self.assertTrue(messages, 'no route messages published')
                approved = [m for m in messages if m.get('approved')]
                self.assertTrue(approved, 'inside-region segment was not approved')
                self.assertEqual(approved[0]['mission_id'], 'a'*32)
                self.assertIn(approved[0]['segment_id'], ('leg_0_start', 'leg_0_end'))
                self.assertEqual({m['segment_id'] for m in approved},{'leg_0_start'})
                # Geometry validation is not obstacle clearance.
                self.assertFalse(approved[0]['path_clear'])
            finally:
                node.destroy_node()

    def _leg_then(self, position, avoidance, line_y=50.0):
        """Approve a leg from (10,y) to (90,y), then ask again from `position`; the last reply."""
        from geometry_msgs.msg import PoseStamped
        with tempfile.TemporaryDirectory() as tmp:
            bundle, _ = self._write_inputs(Path(tmp), [dict(segment_id='route_test_0', pose=[10.0, line_y, 15.0]),
                                                       dict(segment_id='route_test_1', pose=[90.0, line_y, 15.0])])
            node = RoutePublisher(args=['--ros-args', '-r', '__ns:=/route_test', '-p', 'bundle_directory:=' + tmp,
                                        '-p', 'origin_verified:=true', '-p', 'altitude_frame_verified:=true'])
            try:
                def request(at):
                    pose = PoseStamped()
                    pose.header.frame_id = 'earth'
                    pose.header.stamp = node.get_clock().now().to_msg()
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = at[0], at[1], 15.0
                    node.on_pose(pose)
                    node.on_request(String(data=json.dumps(dict(stamp=node.now(), mission_id='a' * 32,
                        command_id='c-leg', operation='SurveyWaypoint', segment_id='route_test_1',
                        target_pose=[90.0, line_y, 15.0], frame_id='earth', avoidance=avoidance))))
                    return node.approval()
                request((10.0, line_y))
                try:
                    return dict(request(position), reason='')
                except ValueError as exc:
                    return dict(approved=False, reason=str(exc))
            finally:
                node.destroy_node()

    def test_avoidance_legs_may_bend_inside_the_area_only(self):
        self.assertTrue(self._leg_then((50.0, 60.0), True)['approved'])        # 10 m off the line: avoiding
        refused = self._leg_then((50.0, 60.0), False)                          # same detour, normal leg
        self.assertFalse(refused['approved'])
        self.assertIn('corridor', refused['reason'])
        self.assertFalse(self._leg_then((50.0, 80.0), True)['approved'])      # 30 m off: beyond 25 m
        outside = self._leg_then((50.0, 105.0), True, line_y=95.0)             # 10 m off but outside the area
        self.assertFalse(outside['approved'])
        self.assertIn('planning area', outside['reason'])

    def test_refuses_without_mission_id(self):
        with tempfile.TemporaryDirectory() as tmp:
            bundle, region = self._write_inputs(Path(tmp))
            node, messages = self._publish(bundle, region, _clear=('mission_id',))
            try:
                self.assertTrue(messages)
                self.assertFalse(any(m.get('approved') for m in messages))
            finally:
                node.destroy_node()

    def test_refuses_segment_outside_region(self):
        with tempfile.TemporaryDirectory() as tmp:
            bundle, region = self._write_inputs(
                Path(tmp), waypoints=[{'segment_id': 'outside',
                                       'pose': [900.0, 900.0, 15.0]}])
            node, messages = self._publish(bundle, region)
            try:
                self.assertTrue(messages)
                self.assertFalse(any(m.get('approved') for m in messages),
                                 'an out-of-region segment was approved')
            finally:
                node.destroy_node()

    def test_refuses_without_region_binding(self):
        with tempfile.TemporaryDirectory() as tmp:
            bundle, region = self._write_inputs(Path(tmp))
            node, messages = self._publish(bundle, region, _omit_region=True)
            try:
                self.assertTrue(messages)
                self.assertFalse(any(m.get('approved') for m in messages),
                                 'a route was approved with no validated region')
            finally:
                node.destroy_node()

    def test_never_claims_path_clear_by_default(self):
        with tempfile.TemporaryDirectory() as tmp:
            bundle, region = self._write_inputs(Path(tmp))
            node, messages = self._publish(bundle, region)
            try:
                self.assertTrue(messages)
                self.assertTrue(all(m.get('path_clear') is False for m in messages),
                                'route claimed an unchecked corridor was clear')
            finally:
                node.destroy_node()


class RestationRouteTests(unittest.TestCase):
    """Mission 2 re-station legs: only to a station the staged tree lists, inside the region."""

    STATION = [50.0, 50.0, 15.0]
    LIST = '20,20;route_peer=80,80'

    def _restation(self, goal, start=(50.0, 50.0)):
        import hashlib
        from as2_rescue_planner.bundle import body_checksum
        from as2_rescue_planner.trees import vision_mission, xml
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            text = xml(vision_mission('human_orbit', [self.STATION], 'group', segment_ids=['route_test_station'],
                                      fleet_size=2, stations=self.LIST))
            value = {'schema_version': 1, 'revision': 1, 'drone_id': 'route_test', 'region': REGION,
                     'origin': dict(latitude=0., longitude=0.), 'xml_sha256': hashlib.sha256(text.encode()).hexdigest(),
                     'coverage': {'frame_id': 'earth', 'altitude_m': 15.0,
                                  'waypoints': [{'segment_id': 'route_test_station', 'pose': self.STATION}]}}
            value['bundle_sha256'] = body_checksum(value)
            (root / ('a' * 32 + '.xml')).write_text(text)
            (root / ('a' * 32 + '.bundle.json')).write_text(json.dumps(dict(mission_id='a' * 32, xml_sha256=value['xml_sha256'], bundle=value)))
            node = RoutePublisher(args=['--ros-args', '-r', '__ns:=/route_test', '-p', 'bundle_directory:=' + tmp,
                                        '-p', 'origin_verified:=true', '-p', 'altitude_frame_verified:=true',
                                        '-p', 'path_clear_verified:=true'])
            received = capture(node, 'rescue/route')
            from geometry_msgs.msg import PoseStamped

            def request():
                pose = PoseStamped()
                pose.header.frame_id = 'earth'
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = start[0], start[1], 15.0
                node.on_pose(pose)
                node.on_request(String(data=json.dumps(dict(stamp=node.now(), mission_id='a' * 32, command_id='c9:restation:1',
                    operation='Restation', segment_id='route_test_station', target_pose=goal, frame_id='earth'))))
            node.create_timer(.05, request)
            try:
                spin_for(node, .45)
            finally:
                node.destroy_node()
            return received

    def test_approves_a_listed_station_and_a_fleet_station(self):
        for goal in ([20.0, 20.0, 15.0], [80.0, 80.0, 15.0], self.STATION):
            approved = [m for m in self._restation(goal) if m.get('approved')]
            self.assertTrue(approved, goal)
            self.assertEqual((approved[0]['command_id'], approved[0]['target_pose']), ('c9:restation:1', goal))

    def test_refuses_any_other_point_or_height(self):
        for goal in ([30.0, 30.0, 15.0], [20.0, 20.0, 25.0]):
            messages = self._restation(goal)
            self.assertTrue(messages)
            self.assertFalse(any(m.get('approved') for m in messages), goal)
            self.assertIn('not a staged watch station', messages[-1]['reason'])

    def test_listed_station_still_needs_the_normal_region_or_entry_rule(self):
        # From outside the area: one short entry leg is the normal rule (max_entry_transit_m 500).
        messages = self._restation([20.0, 20.0, 15.0], start=(-40.0, -40.0))
        self.assertTrue(any(m.get('approved') for m in messages))
        messages = self._restation([20.0, 20.0, 15.0], start=(-600.0, -600.0))
        self.assertFalse(any(m.get('approved') for m in messages))
        self.assertIn('outside', messages[-1]['reason'])


class OriginRequestTimeoutTests(unittest.TestCase):
    """SITL 2026-10-23: a lost get_origin reply left drone0 without an earth frame for good."""

    def test_unanswered_origin_request_is_retried(self):
        from as2_rescue_runtime import route_node
        with tempfile.TemporaryDirectory() as tmp:
            node = RoutePublisher(args=['--ros-args', '-r', '__ns:=/origin_test', '-p', 'bundle_directory:=' + tmp,
                                        '-p', 'origin_source:=estimator'])
            try:
                clock = [100.0]
                node.now = lambda: clock[0]
                sent = []

                class Never:
                    def done(self): return False

                class Client:
                    def service_is_ready(self): return True
                    def call_async(self, request): sent.append(request); return Never()
                node.get_origin = Client()
                node.sync_origin()
                self.assertEqual(len(sent), 1)
                clock[0] += route_node.ORIGIN_REPLY_TIMEOUT_S - 1
                node.sync_origin()
                self.assertEqual(len(sent), 1)                       # still waiting
                clock[0] += 2
                node.sync_origin()                                   # gives up on the lost reply
                self.assertIn('did not answer', node.origin_error)
                node.sync_origin()                                   # and asks again
                self.assertEqual(len(sent), 2)
            finally:
                node.destroy_node()


class RuntimeWatchLogTests(unittest.TestCase):
    """Watch notes are logged at two severities; changing between them must not abort the runtime."""

    def test_watch_note_severity_change_keeps_runtime_running(self):
        from as2_rescue_runtime.runtime_node import RescueRuntime
        with tempfile.TemporaryDirectory() as tmp:
            node = RescueRuntime(args=['--ros-args', '-r', '__ns:=/watch_log_test', '-p', 'state_dir:=' + tmp])
            try:
                cmd = dict(station=[0.0, 20.0], planned=[0.0, 20.0, 12.0])
                # SITL 2026-10-23: 'stopped short' (warning) then 'choosing' (info) from one
                # call site raised in tick() and latched ABORT_LATCHED mid-flight.
                for state, note in [('stopped short', 'station leg stopped short (drone0 orbit nearby)'),
                                    ('choosing', 'moving watch to own alternate 35,5'),
                                    ('holding', 'no clear watch station: holding')]:
                    node.engine.vision.set_watch(cmd, state, note)
                    node.tick()
                    self.assertFalse(node.engine.aborted, state)
            finally:
                node.destroy_node()


class SurfaceEstimatorTests(unittest.TestCase):
    def _collect(self, seconds=0.6, **overrides):
        arguments = ['--ros-args', '-r', '__ns:=/surface_test']
        for key, value in overrides.items():
            arguments += ['-p', '%s:=%s' % (key, value)]
        node = SurfaceEstimator(args=arguments)
        received = capture(node, 'rescue/surface')
        spin_for(node, seconds)
        return node, received

    def test_fixed_source_is_unverified_by_default(self):
        node, received = self._collect()
        try:
            self.assertTrue(received)
            self.assertFalse(any(m.get('verified') for m in received),
                             'an unasserted fixed elevation was reported as verified')
            self.assertFalse(any(m.get('surface_z') is not None for m in received),
                             'an unverified surface published a usable elevation')
        finally:
            node.destroy_node()

    def test_operator_source_requires_verification(self):
        node, received = self._collect(source='operator', operator_verified='false')
        try:
            self.assertTrue(received)
            self.assertFalse(any(m.get('verified') for m in received))
        finally:
            node.destroy_node()

    def test_verified_operator_surface_is_published(self):
        node, received = self._collect(source='operator', operator_verified='true',
                                       fixed_surface_z='12.5', sigma_m='0.3')
        try:
            verified = [m for m in received if m.get('verified')]
            self.assertTrue(verified, 'a verified operator surface was not published')
            self.assertAlmostEqual(verified[0]['surface_z'], 12.5, places=6)
            self.assertAlmostEqual(verified[0]['sigma_m'], 0.3, places=6)
        finally:
            node.destroy_node()

    def test_range_source_without_measurement_stays_unverified(self):
        node, received = self._collect(source='range', range_topic='test/range')
        try:
            self.assertTrue(received)
            self.assertFalse(any(m.get('verified') for m in received),
                             'a range source with no sample claimed a verified surface')
        finally:
            node.destroy_node()

    def test_invalid_source_rejected(self):
        """An unknown surface source must fail closed at construction."""
        with self.assertRaises(ValueError):
            SurfaceEstimator(args=['--ros-args', '-r', '__ns:=/surface_bad',
                                   '-p', 'source:=telepathy'])


class CameraTransformTests(unittest.TestCase):
    """Transforms must be measured, timestamped and fresh, never guessed."""

    def _run(self, verified_gimbal, seconds=3.0):
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import Image
        node = CameraTransform(args=['--ros-args','-p','calibration_verified:=true'])
        published = []
        node.broadcaster.sendTransform = lambda t: published.append(t)
        pose_pub = node.create_publisher(PoseStamped, 'self_localization/pose', 10)
        gimbal_pub = node.create_publisher(String, 'gimbal/state', 10)
        image_pub = node.create_publisher(Image, 'camera/image_raw', 10)

        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'earth'
            pose.pose.orientation.w = 1.0
            pose.pose.position.z = 20.0
            pose_pub.publish(pose)
            gimbal_pub.publish(String(data=json.dumps({
                'stamp': node.get_clock().now().nanoseconds / 1e9,
                'verified': verified_gimbal, 'yaw': 0.0, 'pitch': -90.0, 'roll': 0.0})))
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = 'camera_optical_frame'
            image.height = image.width = 4
            image.encoding = 'bgr8'
            image.step = 12
            image.data = bytes(48)
            image_pub.publish(image)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        return node, published

    def test_requires_pose_before_publishing(self):
        node = CameraTransform()
        published = []
        node.broadcaster.sendTransform = lambda t: published.append(t)
        try:
            spin_for(node, 0.5)
            self.assertEqual(published, [],
                             'a camera transform was published with no aircraft pose')
        finally:
            node.destroy_node()

    def test_publishes_with_pose_gimbal_and_image(self):
        node, published = self._run(True)
        try:
            self.assertTrue(published, 'no camera transform published with valid inputs')
            transform = published[-1]
            self.assertEqual(transform.header.frame_id, 'earth')
            self.assertEqual(transform.child_frame_id, 'camera_optical_frame')
            quaternion = (transform.transform.rotation.x, transform.transform.rotation.y,
                          transform.transform.rotation.z, transform.transform.rotation.w)
            self.assertAlmostEqual(sum(v * v for v in quaternion), 1.0, places=6)
            # The meaningful check is physical, not a specific mounting
            # convention: a nadir gimbal must point the optical axis straight
            # down, i.e. rotate optical +Z onto world -Z.
            direction = rotate_vector(quaternion, (0.0, 0.0, 1.0))
            self.assertAlmostEqual(direction[0], 0.0, places=3)
            self.assertAlmostEqual(direction[1], 0.0, places=3)
            self.assertAlmostEqual(direction[2], -1.0, places=3)
        finally:
            node.destroy_node()

    def test_unverified_gimbal_does_not_publish_when_required(self):
        node, published = self._run(False, seconds=2.5)
        try:
            self.assertEqual(published, [],
                             'an unverified gimbal still produced a camera transform')
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()


class SteadyFixTests(unittest.TestCase):
    """The plan origin is written to the estimator only from a steady, plausible GNSS fix."""

    def fixes(self, altitudes, lat=47.3977, step=0.5, status=0):
        from types import SimpleNamespace
        return [(100.0 + k * step, SimpleNamespace(latitude=lat, longitude=8.5456, altitude=a,
                                                   status=SimpleNamespace(status=status)))
                for k, a in enumerate(altitudes)]

    def test_steady_plausible_fix_is_used(self):
        from as2_rescue_runtime.route_node import steady_fix
        fix, why = steady_fix(self.fixes([488.0, 488.2, 487.9, 488.1, 488.0, 488.1]), 102.6)
        self.assertEqual(why, '')
        self.assertAlmostEqual(fix.altitude, 488.1)

    def test_bad_fixes_are_refused(self):
        from as2_rescue_runtime.route_node import steady_fix
        cases = [(self.fixes([-232673.0] * 6), 'Implausible'),                       # seen in SITL
                 (self.fixes([488.0, 488.0, 470.0, 488.0, 488.0, 488.0]), 'not steady'),
                 (self.fixes([488.0, 488.0]), 'steady'),                              # too few fixes
                 (self.fixes([488.0] * 6, status=-1), 'Waiting')]
        for history, expected in cases:
            fix, why = steady_fix(history, history[-1][0] + 0.1)
            self.assertIsNone(fix)
            self.assertIn(expected, why)
