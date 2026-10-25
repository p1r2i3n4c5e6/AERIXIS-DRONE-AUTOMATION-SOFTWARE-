"""Flight log for gimbal aiming, person tracking and aircraft motion.

This node records what happened; it never commands anything. It exists so a
post-flight review can answer three questions that are otherwise guesswork:

* Was the gimbal actually pointing straight down (nadir) when a geotag was made?
* Was the track the same person, and was the camera really centred on them?
* Where was the aircraft, and how was it moving, at that moment?

Each record is one JSON line, written with a size cap so a long mission cannot
fill the disk. A compact rolling summary is also published on
``rescue/track_log`` for the GCS, and a human-readable line goes to the normal
ROS log so the same events are visible in ``ros2 topic echo`` and in the launcher.
"""
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .node_main import run as run_node

TARGET_DOWN_PITCH_DEG = -90.0


class TrackLogger(Node):
    def __init__(self, args=None):
        super().__init__('rescue_track_logger', cli_args=args)
        for name, value in [
            ('log_directory', str(Path.home() / '.local/share/as2-rescue-logs')),
            ('max_file_mb', 32.0),
            ('max_track_age_s', 0.75),
            ('gimbal_tolerance_deg', 5.0),
            ('centre_tolerance', 0.05),
            ('summary_hz', 2.0),
        ]:
            self.declare_parameter(name, value)

        self.max_file = float(self.get_parameter('max_file_mb').value) * 1024 * 1024
        if self.max_file <= 0:
            raise ValueError('Maximum log size must be positive')
        self.max_track_age = float(self.get_parameter('max_track_age_s').value)
        self.gimbal_tolerance = float(self.get_parameter('gimbal_tolerance_deg').value)
        self.centre_tolerance = float(self.get_parameter('centre_tolerance').value)

        directory = Path(self.get_parameter('log_directory').value)
        directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.directory = directory
        self.path = directory / time.strftime('track-%Y%m%d-%H%M%S.jsonl')
        self.log_file = open(self.path, 'a', encoding='utf-8')
        os.chmod(self.path, 0o600)

        self.gimbal = None
        self.tracks = None
        self.state = None
        self.fix = None
        self.last_track_id = None
        self.events = 0
        self.operation = None

        self.summary = self.create_publisher(String, 'rescue/track_log', 10)
        # gimbal/state + perception/tracks of one camera frame, in one message (vision/frame).
        self.create_subscription(String, 'vision/frame', self.on_frame, 10)
        self.create_subscription(String, 'rescue/state', self.on_state, 10)
        self.create_subscription(String, 'rescue/target_fix', self.on_fix, 10)
        # Gimbal-guided missions: geotags/payload reports, and the approach towards nadir.
        self.create_subscription(String, 'rescue/observations', self.on_observation, 10)
        self.declare_parameter('guidance_sample_hz', 2.0)
        self.create_timer(1.0 / max(0.2, float(self.get_parameter('guidance_sample_hz').value)),
                          self.guidance_sample)
        # Position and speed come from rescue/state (10 Hz), not from pose/twist at the flight
        # controller's odometry rate: a log needs no more, and every message costs Python CPU on the
        # Pi 5 that the person detector needs (2026-10-23).
        hz = float(self.get_parameter('summary_hz').value)
        if not 0 < hz <= 10:
            raise ValueError('Summary rate must be in (0, 10] Hz')
        self.create_timer(1.0 / hz, self.publish_summary)
        self.get_logger().info('Track log: %s' % self.path)

    # ------------------------------------------------------------ inputs
    def on_frame(self, msg):
        try:
            frame = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(frame, dict):
            return
        if isinstance(frame.get('gimbal'), dict):
            self.on_gimbal_value(frame['gimbal'])
        if isinstance(frame.get('tracks'), dict):
            self.on_tracks_value(frame['tracks'])

    def on_gimbal_value(self, value):
        previous = self.gimbal
        self.gimbal = value
        # Record aiming transitions rather than every sample, so the log stays
        # readable and a change in where the camera points is never lost.
        if previous is not None or value.get('verified'):
            before = self._aim(previous)
            after = self._aim(value)
            if before != after:
                self.record('gimbal_aim', aim_before=before, aim_after=after,
                            pitch=value.get('pitch'), yaw=value.get('yaw'))

    def on_tracks_value(self, value):
        self.tracks = value
        confirmed = [t for t in value.get('tracks', [])
                     if t.get('confirmed') and not t.get('predicted')]
        if not confirmed:
            if self.last_track_id is not None:
                self.record('track_lost', track_id=self.last_track_id)
                self.last_track_id = None
            return
        best = max(confirmed, key=lambda t: t.get('confidence', 0))
        track_id = best.get('track_id')
        if track_id != self.last_track_id:
            self.record('track_acquired', track_id=track_id,
                        confidence=best.get('confidence'),
                        bbox=best.get('bbox_pixels'))
            self.last_track_id = track_id

    def on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
        except ValueError:
            self.state = None
            return
        operation = self.state.get('active_operation')
        if operation != self.operation:
            self.record('operation', started=operation, finished=self.operation,
                        target_id=self.state.get('target_id'))
            self.operation = operation

    def on_observation(self, msg):
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        kind = value.pop('kind', 'observation')
        if kind == 'observation':
            value = dict(mission_id=value.get('mission_id'), target_id=value.get('target_id'))
        self.record(kind if kind != 'geotag' else 'geotag_overhead', nadir_at_event=self._is_nadir(),
                    centred=self._centred(), **{k: v for k, v in value.items() if k not in ('stamp',)})

    GUIDED = ('TrackTarget', 'ApproachOverhead', 'OrbitTarget', 'GeotagOverhead', 'DropPayload')

    def guidance_sample(self):
        """While the aircraft is steered by the gimbal: where it points and how it moves."""
        if self.operation not in self.GUIDED or not self.gimbal:
            return
        self.record('guidance', track_id=self.gimbal.get('track_id'), gimbal_yaw=self.gimbal.get('yaw'),
                    gimbal_pitch=self.gimbal.get('pitch'), centre_error=self.gimbal.get('center_error'),
                    nadir=self._is_nadir(), centred=self._centred())

    def on_fix(self, msg):
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        previous = self.fix
        self.fix = value
        if value.get('valid') and (previous is None or not previous.get('valid')):
            self.record('geotag', position=value.get('position'),
                        sigma_m=value.get('sigma_m'), track_id=value.get('track_id'),
                        latitude=value.get('latitude'), longitude=value.get('longitude'),
                        nadir_at_geotag=self._is_nadir())


    # ----------------------------------------------------------- helpers
    _CURRENT = object()

    def _aim(self, value=_CURRENT):
        if value is TrackLogger._CURRENT:
            value = self.gimbal
        if not value or not value.get('verified'):
            return 'unknown'
        if value.get('oscillating'):
            return 'oscillating'
        if self._is_nadir(value):
            return 'nadir'
        if value.get('tracking'):
            return 'tracking'
        return 'searching'

    def _is_nadir(self, value=None):
        value = value if value is not None else self.gimbal
        if not value or value.get('pitch') is None:
            return False
        try:
            pitch = float(value['pitch'])
        except (TypeError, ValueError):
            return False
        return abs(pitch - TARGET_DOWN_PITCH_DEG) <= self.gimbal_tolerance

    def _centred(self):
        value = self.gimbal or {}
        error = value.get('center_error')
        if not isinstance(error, list) or len(error) != 2:
            return None
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in error):
            return None
        return abs(error[0]) <= self.centre_tolerance and abs(error[1]) <= self.centre_tolerance

    def _aircraft(self):
        record = {}
        vehicle = (self.state or {}).get('vehicle') or {}
        if isinstance(vehicle.get('position'), list):
            record.update(position=vehicle['position'], frame_id='earth')
        if isinstance(vehicle.get('speed'), (int, float)):
            record['speed_mps'] = vehicle['speed']
        return record

    def record(self, event, **fields):
        entry = dict(schema_version=1, stamp=self.get_clock().now().nanoseconds / 1e9,
                     event=event, aircraft=self._aircraft())
        if self.state:
            entry['mission_id'] = self.state.get('mission_id')
            entry['operation'] = self.state.get('active_operation')
        entry.update(fields)
        line = json.dumps(entry, allow_nan=False)
        try:
            if self.log_file.tell() >= self.max_file:
                self._rotate()
            self.log_file.write(line + '\n')
            self.log_file.flush()
        except OSError as exc:
            self.get_logger().error('Track log write failed: %s' % exc)
        self.events += 1
        self.get_logger().info(line)

    def _rotate(self):
        """Start a new file before the size cap, keeping the old one intact."""
        self.log_file.close()
        self.path = self.directory / time.strftime('track-%Y%m%d-%H%M%S.jsonl')
        self.log_file = open(self.path, 'a', encoding='utf-8')
        os.chmod(self.path, 0o600)

    def publish_summary(self):
        value = dict(
            schema_version=1,
            stamp=self.get_clock().now().nanoseconds / 1e9,
            aim=self._aim(),
            nadir=self._is_nadir(),
            centred=self._centred(),
            track_id=self.last_track_id,
            events=self.events,
            log_file=str(self.path),
            aircraft=self._aircraft(),
        )
        if self.fix and self.fix.get('valid'):
            value['last_fix'] = dict(position=self.fix.get('position'),
                                     sigma_m=self.fix.get('sigma_m'))
        self.summary.publish(String(data=json.dumps(value, allow_nan=False)))


def main(args=None):
    run_node(TrackLogger, args=args, cleanup=lambda node: node.log_file.close())


if __name__ == '__main__':
    main()
