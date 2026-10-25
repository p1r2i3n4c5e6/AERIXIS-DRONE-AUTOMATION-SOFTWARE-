"""Surface elevation estimate for optical geolocation.

``GeotagTarget`` and the geolocator both refuse to work without a *verified*
surface elevation, and nothing in this workspace published ``rescue/surface``.
That refusal is correct: over flood water, a person is not standing on terrain
at zero altitude, and a downward camera ray intersected with a guessed ground
plane produces a confidently wrong location.

This node therefore never invents a surface. It accepts measurements from
configured sources and publishes the best verified estimate, or an explicitly
unverified value that geolocation will reject.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import String
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from .node_main import run as run_node


class SurfaceEstimator(Node):
    SOURCES = ('operator', 'range', 'fixed')

    def __init__(self, args=None):
        super().__init__('rescue_surface_estimator', cli_args=args)
        for name, value in [
            ('source', 'fixed'),          # 'fixed' | 'range' | 'operator'
            ('frame_id', 'earth'),
            ('fixed_surface_z', 0.0),     # Only usable with operator_verified
            ('sigma_m', 1.0),
            ('operator_verified', False), # Explicit human assertion of the datum
            ('max_measurement_age_s', 0.5),
            ('range_topic', 'sensor_measurements/range'),
            ('range_calibration_verified', False),
            ('max_range_tilt_deg', 15.0),
        ]:
            self.declare_parameter(name, value)

        source = self.get_parameter('source').value
        if source not in self.SOURCES:
            raise ValueError('Unsupported surface source: %s' % source)
        self.max_age = float(self.get_parameter('max_measurement_age_s').value)
        if not 0 < self.max_age <= 5:
            raise ValueError('Maximum measurement age must be in (0, 5] seconds')

        self.surface = None
        self.measurement = None       # Latest raw range sample: (metres, monotonic)
        self.measured_at = 0.0
        self.error = ''
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.publisher = self.create_publisher(String, 'rescue/surface', 10)
        self.health = self.create_publisher(String, 'rescue/surface_health', 10)
        if source == 'range':
            topic = self.get_parameter('range_topic').value
            if not topic:
                raise ValueError('A range topic is required for the range source')
            self.create_subscription(Range, topic, self.on_range, qos_profile_sensor_data)
        self.create_timer(0.2, self.publish)

    def on_range(self, msg):
        try:
            value = float(msg.range)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
            now = self.get_clock().now().nanoseconds/1e9
            if not 0 <= now-stamp <= self.max_age or stamp <= self.measured_at:
                raise ValueError('Range timestamp is stale, future or repeated')
            if not math.isfinite(value) or not 0 < msg.min_range < value < msg.max_range:
                raise ValueError('Range sample is not a usable distance')
            if not self.get_parameter('range_calibration_verified').value:
                raise ValueError('Range calibration and mounting transform are unverified')
            transform = self.buffer.lookup_transform(self.get_parameter('frame_id').value,
                msg.header.frame_id, Time.from_msg(msg.header.stamp))
            q = transform.transform.rotation
            if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .01:
                raise ValueError('Invalid range sensor transform')
            # sensor_msgs/Range measures along sensor +X, not world -Z.
            vertical = 2*(q.x*q.z-q.w*q.y)
            if vertical > -math.cos(math.radians(self.get_parameter('max_range_tilt_deg').value)):
                raise ValueError('Range beam is not within the calibrated nadir cone')
            self.measurement = transform.transform.translation.z + value*vertical
            if not math.isfinite(self.measurement):raise ValueError('Nonfinite surface geometry')
            self.measured_at = stamp
            self.error = ''
        except Exception as exc:
            self.measurement = None
            self.error = str(exc)

    def publish(self):
        now = self.get_clock().now().nanoseconds / 1e9
        frame_id = self.get_parameter('frame_id').value
        source = self.get_parameter('source').value
        sigma = float(self.get_parameter('sigma_m').value)
        try:
            if not frame_id:
                raise ValueError('A reference frame is required')
            if not math.isfinite(sigma) or sigma < 0:
                raise ValueError('Surface uncertainty must be nonnegative')

            if source == 'fixed':
                # A fixed datum is only a verified surface when a human asserts
                # it; otherwise it is an assumption and is reported as such.
                surface_z = float(self.get_parameter('fixed_surface_z').value)
                if not math.isfinite(surface_z):
                    raise ValueError('Fixed surface elevation must be finite')
                verified = bool(self.get_parameter('operator_verified').value)
                reason = ('' if verified else
                          'Fixed elevation has no operator verification; geolocation will refuse it')

            elif source == 'operator':
                surface_z = float(self.get_parameter('fixed_surface_z').value)
                if not math.isfinite(surface_z):
                    raise ValueError('Operator surface elevation must be finite')
                if not self.get_parameter('operator_verified').value:
                    raise ValueError('Operator source requires explicit verification')
                verified = True
                reason = ''

            else:  # range
                if self.measurement is None:
                    raise ValueError('No range measurement received yet')
                age = now - self.measured_at
                if not 0 <= age <= self.max_age:
                    raise ValueError('Range measurement is stale (%.2f s)' % age)
                surface_z = self.measurement
                verified = True
                reason = ''

            self.surface = dict(schema_version=1, stamp=now, frame_id=frame_id,
                                measurement_stamp=self.measured_at if source=='range' else now,
                                # An unverified elevation must not be published as
                                # a usable number, or a consumer that ignores the
                                # verified flag would geolocate against a guess.
                                surface_z=(surface_z if verified else None),
                                sigma_m=(sigma if verified else None), verified=verified,
                                source=source, reason=reason)
            self.error = ''
        except (ValueError, TypeError, KeyError) as exc:
            self.error = str(exc)
            # Publish an explicit unverified estimate. The engines treat this as
            # "no usable surface" rather than silently assuming ground level.
            self.surface = dict(schema_version=1, stamp=now, frame_id=frame_id or None,
                                surface_z=None, sigma_m=None, verified=False,
                                source=source, reason=self.error)

        self.publisher.publish(String(data=json.dumps(self.surface, allow_nan=False)))
        self.health.publish(String(data=json.dumps(dict(
            stamp=now, verified=bool(self.surface.get('verified')), source=source,
            measurement_age_s=(None if self.measurement is None else now - self.measured_at),
            error=self.error), allow_nan=False)))


def main(args=None):
    run_node(SurfaceEstimator, args=args)

