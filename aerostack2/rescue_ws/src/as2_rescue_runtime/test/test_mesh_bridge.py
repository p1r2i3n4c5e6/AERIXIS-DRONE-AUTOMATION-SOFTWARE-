"""Mesh bridge, real processes: base station (UDP) + LoRa fallback (a pseudo-terminal pair as the radio)."""
import json
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

import rclpy
from std_msgs.msg import String


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class Radio:
    """Two serial ports joined like a pair of LoRa modems; `cut` stops the air link."""

    def __init__(self):
        self.a, a_slave = os.openpty()
        self.b, b_slave = os.openpty()
        self.devices = (os.ttyname(a_slave), os.ttyname(b_slave))
        self.slaves = (a_slave, b_slave)
        self.cut = False
        self.running = True
        self.thread = threading.Thread(target=self.relay, daemon=True)
        self.thread.start()

    def relay(self):
        while self.running:
            ready, _, _ = select.select([self.a, self.b], [], [], 0.1)
            for fd in ready:
                try:
                    data = os.read(fd, 4096)
                except OSError:
                    continue
                if not self.cut:
                    os.write(self.b if fd == self.a else self.a, data)

    def close(self):
        self.running = False
        self.thread.join(1)
        for fd in (self.a, self.b, *self.slaves):
            os.close(fd)


class MeshBridgeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        key = 'k' * 64
        self.key_file = Path(self.tmp.name) / 'mesh.json'
        self.key_file.write_text(json.dumps(dict(send_key=key, receive_keys={'da': key, 'db': key, 'base': key})))
        self.radio = Radio()
        self.procs = []
        rclpy.init()
        self.node = rclpy.create_node('mesh_bridge_test')
        self.heard = []
        self.node.create_subscription(String, '/db/swarm/peer_rx', lambda m: self.heard.append(json.loads(m.data)), 10)
        self.tx = self.node.create_publisher(String, '/da/swarm/peer_tx', 10)
        self.health = {}
        self.node.create_subscription(String, '/db/swarm/radio_health', lambda m: self.health.update(json.loads(m.data)), 10)

    def tearDown(self):
        for p in self.procs:
            p.terminate()
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
        self.node.destroy_node()
        rclpy.try_shutdown()
        self.radio.close()
        self.tmp.cleanup()

    def bridge(self, ns, transport, port, peer_port, device):
        args = [sys.executable, '-c', 'from as2_rescue_runtime.mesh_bridge import main; main()', '--ros-args',
                '-r', '__ns:=/' + ns, '-p', 'transport:=' + transport, '-p', 'device:=' + device,
                '-p', 'baud:=57600', '-p', 'bind_port:=%d' % port, '-p', 'peers:=["127.0.0.1:%d"]' % peer_port,
                '-p', 'key_file:=' + str(self.key_file), '-p', 'frames_per_second:=4.0']
        self.procs.append(subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

    def exchange(self, seconds=6.0):
        """Send peer frames from da for `seconds`; returns frames db received."""
        self.heard.clear()
        end = time.time() + seconds
        while time.time() < end:
            now = time.time()
            self.tx.publish(String(data=json.dumps(dict(owner='da', position=[1.0, 2.0, 12.0], frame_id='earth',
                                                        stamp=now, sigma_m=0.1, speed=1.5, velocity=[1.5, 0.0]))))
            t = time.time() + 0.25
            while time.time() < t:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        return list(self.heard)

    def start_pair(self, a_peer_port=None, a_device=None):
        pa, pb = free_port(), free_port()
        self.bridge('da', 'both', pa, a_peer_port or pb, a_device or self.radio.devices[0])
        self.bridge('db', 'both', pb, pa, self.radio.devices[1])
        time.sleep(3.0)                       # processes up

    def test_both_links_carry_frames_and_lora_takes_over_without_the_base_station(self):
        dead = free_port()                    # da's UDP goes nowhere: only LoRa reaches db
        self.start_pair(a_peer_port=dead)
        frames = self.exchange()
        self.assertTrue(frames, 'nothing arrived over the LoRa fallback')
        self.assertEqual(frames[-1]['owner'], 'da')
        self.assertIn('da', self.health['links']['serial']['peers'])
        self.assertNotIn('da', self.health['links']['udp']['peers'])

    def test_base_station_carries_frames_at_full_rate_and_duplicates_are_dropped(self):
        self.start_pair()
        frames = self.exchange()
        stamps = [f['stamp'] for f in frames]
        self.assertGreater(len(frames), 10)
        self.assertEqual(len(stamps), len(set(stamps)), 'a frame was delivered twice (UDP + LoRa)')
        self.assertIn('da', self.health['links']['udp']['peers'])

    def test_no_lora_modem_keeps_the_base_station_link(self):
        self.start_pair(a_device='/dev/does-not-exist')
        frames = self.exchange()
        self.assertTrue(frames)
        time.sleep(0.5)


if __name__ == '__main__':
    unittest.main()
