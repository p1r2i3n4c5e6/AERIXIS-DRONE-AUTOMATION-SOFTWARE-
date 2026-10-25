"""node_main.run: on SIGINT/SIGTERM the node's shutdown step still has ROS (it can publish, like
the runtime's stop-and-hover), and the process exits 0 without a traceback (Pi log, 2026-10-25)."""
import os
import signal
import subprocess
import sys
import textwrap
import time
import unittest

import rclpy
from std_msgs.msg import String

CHILD = textwrap.dedent('''
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from as2_rescue_runtime.node_main import run

    class Toy(Node):
        def __init__(self):
            super().__init__('node_main_toy')
            self.pub = self.create_publisher(String, '/node_main_test/out', 10)
            self.create_timer(0.1, lambda: self.pub.publish(String(data='alive')))

    def last_words(node):
        for _ in range(20):
            node.pub.publish(String(data='stopping'))
            rclpy.spin_once(node, timeout_sec=0.05)

    run(Toy, graceful=last_words, cleanup=lambda node: print('cleanup-ran', flush=True))
''')


class NodeMainTests(unittest.TestCase):
    def check(self, signum):
        rclpy.init()
        probe = rclpy.create_node('node_main_probe')
        got = []
        probe.create_subscription(String, '/node_main_test/out', lambda m: got.append(m.data), 10)
        child = subprocess.Popen([sys.executable, '-c', CHILD], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 text=True, env=dict(os.environ))
        try:
            deadline = time.monotonic() + 10
            while 'alive' not in got and time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.05)
            self.assertIn('alive', got)
            child.send_signal(signum)
            deadline = time.monotonic() + 6
            while child.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.05)
            for _ in range(10):
                rclpy.spin_once(probe, timeout_sec=0.05)
            out, err = child.communicate(timeout=5)
        finally:
            if child.poll() is None:
                child.kill()
            probe.destroy_node()
            rclpy.shutdown()
        self.assertEqual(child.returncode, 0, err)
        self.assertNotIn('Traceback', err)
        self.assertIn('cleanup-ran', out)
        self.assertIn('stopping', got)            # published after the signal, ROS still up

    def test_sigint(self):
        self.check(signal.SIGINT)

    def test_sigterm(self):
        self.check(signal.SIGTERM)


if __name__ == '__main__':
    unittest.main()
