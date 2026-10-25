"""Startup gimbal centring check against a simulated SIYI A8 mini (no network).

A8 mini firmware answers with its own sequence counter (Pi 5, 2026-10-23), so replies must be
matched by command and time. The check must still fail closed: no ACK, a rejected centre, an
attitude away from zero, a moving gimbal or only stale replies never verify.
"""
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from siyi_startup import center_and_verify, crc16  # noqa: E402

ADDRESS = ('192.168.144.25', 37260)


def reply(command, sequence, payload):
    body = b'\x55\x66\x02' + struct.pack('<HHB', len(payload), sequence & 0xffff, command) + payload
    return body + struct.pack('<H', crc16(body))


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


class Gimbal:
    """Fake UDP socket that answers like the camera: centre ACK and attitude replies."""

    def __init__(self, clock, echo=False, ack=1, attitude=(-2.9, 0.0, 0.0, 0.0), delay=0.02, send_ack=True):
        self.clock, self.echo, self.ack, self.attitude = clock, echo, ack, attitude
        self.delay, self.send_ack = delay, send_ack
        self.queue, self.counter, self.sent = [], 1400, []

    def sendto(self, data, address):
        command, sequence = data[7], struct.unpack_from('<H', data, 5)[0]
        self.sent.append(command)
        self.counter += 1
        answer_seq = sequence if self.echo else self.counter
        if command == 8 and self.send_ack:
            self.queue.append((self.clock() + self.delay, reply(8, answer_seq, bytes([self.ack]))))
        elif command == 13 and self.attitude is not None:
            yaw, pitch, yaw_rate, pitch_rate = self.attitude
            payload = struct.pack('<6h', int(yaw * 10), int(pitch * 10), 0, int(yaw_rate * 10),
                                  int(pitch_rate * 10), 0)
            self.queue.append((self.clock() + self.delay, reply(13, answer_seq, payload)))

    def recvfrom(self, size):
        ready = [item for item in self.queue if item[0] <= self.clock()]
        if not ready:
            raise BlockingIOError
        self.queue.remove(ready[0])
        return ready[0][1], ADDRESS


class StartupCentreTests(unittest.TestCase):
    def verify(self, **gimbal):
        clock = Clock()
        sock = Gimbal(clock, **gimbal)
        return center_and_verify(sock, ADDRESS, timeout=3.0, clock=clock, sleep=clock.sleep), sock

    def test_own_counter_firmware_verifies(self):
        result, sock = self.verify(echo=False)
        self.assertAlmostEqual(result['yaw'], -2.9)
        self.assertEqual(sock.sent[-1], 7)                 # stop-rotation sent at the end

    def test_echoing_firmware_still_verifies(self):
        result, _ = self.verify(echo=True)
        self.assertAlmostEqual(result['pitch'], 0.0)

    def test_rejected_centre_fails(self):
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            self.verify(ack=0)

    def test_attitude_away_from_zero_fails(self):
        with self.assertRaisesRegex(RuntimeError, 'not verified'):
            self.verify(attitude=(10.0, 0.0, 0.0, 0.0))

    def test_moving_gimbal_fails(self):
        with self.assertRaisesRegex(RuntimeError, 'not verified'):
            self.verify(attitude=(0.0, 0.0, 8.0, 0.0))

    def test_no_ack_fails(self):
        with self.assertRaisesRegex(RuntimeError, 'not verified'):
            self.verify(send_ack=False)

    def test_only_stale_replies_fail(self):
        with self.assertRaisesRegex(RuntimeError, 'not verified'):
            self.verify(delay=0.7)


if __name__ == '__main__':
    unittest.main()
