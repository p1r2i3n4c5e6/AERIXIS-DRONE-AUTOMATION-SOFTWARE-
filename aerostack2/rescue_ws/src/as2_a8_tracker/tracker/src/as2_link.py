"""AERIXIS link for the A8 tracker: measured gimbal attitude and onboard mission commands.

Kept separate from hybrid_tracker.py so the tracking pipeline itself is unchanged.

* AttitudeReader asks the SIYI camera for attitude (CMD 0x0D) on its own UDP socket,
  so it never shares a socket with the rate controller. It is read-only.
* ControlPort receives JSON commands from the ROS bridge on 127.0.0.1 only:
    {"cmd": "release", "suppress_s": 8}   drop the current selection, no new lock for N s
    {"cmd": "suppress", "seconds": 5}     report detections but never lock/steer for N s
    {"cmd": "resume"}                      end suppression now (selection starts fresh)
    {"cmd": "set_attitude", "yaw": 0, "pitch": -45}
                                           absolute gimbal angle (CMD 0x0E), only while
                                           no person is locked
* Angle commands go through AttitudeReader's socket. The rate controller is paused for
  the duration of the move so its zero-rate packets cannot cancel it.
"""
import json
import socket
import struct
import threading
import time

from siyi_startup import packet, replies

CONTROL_PORT = 9091
ATTITUDE_HZ = 10.0
ATTITUDE_MAX_AGE_S = 0.5


class AttitudeReader:
    def __init__(self, siyi_ip, siyi_port=37260, enabled=True, rate_hz=ATTITUDE_HZ):
        self.address = (siyi_ip, int(siyi_port))
        self.enabled = enabled
        self.period = 1.0 / max(1.0, float(rate_hz))
        self.lock = threading.Lock()
        self.sample = None
        self.seq = 0x4000
        self.pending = None       # send time of the one attitude query in flight
        self.quiet_until = 0.0    # after a lost reply: ignore replies until then
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.02)
        self.running = enabled
        self.thread = threading.Thread(target=self._loop, name='a8-attitude', daemon=True)
        if enabled:
            self.thread.start()

    def _next(self):
        self.seq = (self.seq + 1) & 0xffff
        return self.seq

    def _loop(self):
        next_query = 0.0
        while self.running:
            now = time.monotonic()
            if self.pending is not None and now - self.pending > 0.5:
                self.pending, self.quiet_until = None, now + 0.5
            if self.pending is None and now >= next_query and now >= self.quiet_until:
                seq = self._next()
                try:
                    self.sock.sendto(packet(0x0D, sequence=seq), self.address)
                    self.pending = now
                except OSError:
                    pass
                next_query = now + self.period
            try:
                data, sender = self.sock.recvfrom(4096)
            except (socket.timeout, BlockingIOError):
                continue
            except OSError:
                time.sleep(0.05)
                continue
            if sender != self.address:
                continue
            for command, seq, payload in replies(data):
                if command != 0x0D or len(payload) != 12:
                    continue
                # Not matched by sequence number: A8 mini firmware replies with its own counter
                # (2026-10-23), so a number match never produced a sample. One query in flight:
                # its reply is unambiguous; lost after 0.5 s, replies are ignored for 0.5 s more.
                if self.pending is None:
                    continue
                sent, self.pending = self.pending, None
                if time.monotonic() - sent > 0.5:
                    continue
                yaw, pitch, roll, yaw_rate, pitch_rate, roll_rate = [
                    v / 10.0 for v in struct.unpack('<6h', payload)]
                with self.lock:
                    self.sample = dict(yaw=(yaw + 180.0) % 360.0 - 180.0, pitch=pitch, roll=roll,
                                       yaw_rate=yaw_rate, pitch_rate=pitch_rate,
                                       roll_rate=roll_rate, t=time.monotonic(),
                                       round_trip_s=time.monotonic() - sent)

    def latest(self):
        with self.lock:
            sample = dict(self.sample) if self.sample else None
        if sample is None:
            return None
        sample['age_s'] = round(time.monotonic() - sample.pop('t'), 3)
        return sample

    def set_angle(self, yaw_deg, pitch_deg):
        yaw = max(-135.0, min(135.0, float(yaw_deg)))
        pitch = max(-90.0, min(25.0, float(pitch_deg)))
        data = struct.pack('<hh', int(round(yaw * 10)), int(round(pitch * 10)))
        self.sock.sendto(packet(0x0E, data, self._next()), self.address)
        return yaw, pitch

    def close(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.sock.close()


class ControlPort:
    def __init__(self, port=CONTROL_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('127.0.0.1', int(port)))
        self.sock.setblocking(False)

    def poll(self, limit=8):
        commands = []
        for _ in range(limit):
            try:
                data, _ = self.sock.recvfrom(2048)
            except (BlockingIOError, OSError):
                break
            try:
                value = json.loads(data.decode('utf-8'))
            except ValueError:
                continue
            if isinstance(value, dict) and isinstance(value.get('cmd'), str):
                commands.append(value)
        return commands

    def close(self):
        self.sock.close()


class MissionLink:
    """Selection suppression and angle moves requested by the onboard mission."""

    def __init__(self, siyi_ip, siyi_port, attitude_enabled, control_port=CONTROL_PORT,
                 log=None):
        self.attitude = AttitudeReader(siyi_ip, siyi_port, enabled=attitude_enabled)
        self.control = ControlPort(control_port)
        self.suppressed_until = 0.0
        self.reset_pending = False
        self.releases = 0
        self.last_command = None
        self.log = log

    def _info(self, text, *args):
        if self.log:
            self.log.info(text, *args)

    def handle(self, gimbal_controller, locked_track):
        """Apply queued commands. Returns True when the selection must be reset now."""
        now = time.monotonic()
        reset = False
        for command in self.control.poll():
            name = command['cmd']
            self.last_command = dict(cmd=name, t=round(time.time(), 3))
            if name == 'release':
                seconds = float(command.get('suppress_s', 8.0))
                self.suppressed_until = now + max(0.0, min(60.0, seconds))
                self.releases += 1
                reset = True
                self._info('AERIXIS release: selection dropped, locking suppressed %.1fs', seconds)
            elif name == 'suppress':
                seconds = float(command.get('seconds', 5.0))
                self.suppressed_until = now + max(0.0, min(60.0, seconds))
                self._info('AERIXIS suppress: locking suppressed %.1fs', seconds)
            elif name == 'resume':
                if self.suppressed_until > now:
                    self.suppressed_until = now
                    self.reset_pending = True
            elif name == 'set_attitude':
                if locked_track is not None and now >= self.suppressed_until:
                    self._info('AERIXIS set_attitude ignored: a person is locked')
                    continue
                if gimbal_controller.preview_only:
                    self._info('AERIXIS set_attitude ignored in preview mode')
                    continue
                seconds = float(command.get('settle_s', 2.5))
                gimbal_controller.rate_pause_until = now + max(0.2, min(6.0, seconds))
                try:
                    yaw, pitch = self.attitude.set_angle(command.get('yaw', 0.0),
                                                         command.get('pitch', 0.0))
                    self._info('AERIXIS set_attitude yaw=%.1f pitch=%.1f', yaw, pitch)
                except OSError as exc:
                    self._info('AERIXIS set_attitude send failed: %s', exc)
        if self.suppressed_until and now >= self.suppressed_until:
            self.suppressed_until = 0.0
            self.reset_pending = True
        if self.reset_pending:
            self.reset_pending = False
            reset = True
        return reset

    @property
    def suppressed(self):
        return time.monotonic() < self.suppressed_until

    def telemetry(self):
        return dict(gimbal_attitude=self.attitude.latest(),
                    suppressed=self.suppressed,
                    suppressed_remaining_s=round(max(0.0, self.suppressed_until - time.monotonic()), 2),
                    releases=self.releases,
                    last_command=self.last_command,
                    attitude_max_age_s=ATTITUDE_MAX_AGE_S)

    def close(self):
        self.attitude.close()
        self.control.close()
