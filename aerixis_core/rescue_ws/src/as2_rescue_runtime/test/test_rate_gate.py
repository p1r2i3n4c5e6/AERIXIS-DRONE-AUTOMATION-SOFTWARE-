"""The C++ rate gate (as2_rescue_rate_gate) forwards 100 Hz input at the configured rate, and the
tf gate keeps every frame: a slow gimbal transform is not lost behind a fast odometry one.
Skipped when the gate is not built (it lives in its own package)."""
import os
import shutil
import subprocess
import time
import unittest

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy

try:
    from ament_index_python.packages import get_package_prefix
    GATE = os.path.join(get_package_prefix('as2_rescue_rate_gate'), 'lib', 'as2_rescue_rate_gate', 'rate_gate')
except Exception:                                   # not built / not sourced
    GATE = None


@unittest.skipUnless(GATE and os.access(GATE, os.X_OK), 'as2_rescue_rate_gate not built')
class RateGateTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('gate_probe', namespace='/gate_test')
        self.process = subprocess.Popen([GATE, '--ros-args', '-r', '__ns:=/gate_test',
            '-p', 'topics:=["self_localization/pose rescue/in/pose geometry_msgs/msg/PoseStamped 25"]',
            '-p', 'tf_in:=/gate_test/tf_raw', '-p', 'tf_out:=rescue/in/tf', '-p', 'tf_hz:=30.0'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def tearDown(self):
        self.process.terminate()
        self.process.wait(timeout=5)
        self.node.destroy_node()
        rclpy.shutdown()

    def test_rates(self):
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from tf2_msgs.msg import TFMessage
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        pose_pub = self.node.create_publisher(PoseStamped, 'self_localization/pose', qos_profile_sensor_data)
        tf_pub = self.node.create_publisher(TFMessage, 'tf_raw', reliable)
        got = {'pose': 0, 'odom': 0, 'gimbal': 0}
        self.node.create_subscription(PoseStamped, 'rescue/in/pose',
                                      lambda m: got.__setitem__('pose', got['pose'] + 1), qos_profile_sensor_data)

        def on_tf(message):
            for t in message.transforms:
                key = 'gimbal' if t.child_frame_id.endswith('gimbal') else 'odom'
                got[key] += 1
        self.node.create_subscription(TFMessage, 'rescue/in/tf', on_tf, reliable)

        def transform(child):
            t = TransformStamped()
            t.header.frame_id = 'drone0/odom'
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            return t
        deadline = time.monotonic() + 3.0           # gate start-up and discovery
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        for key in got:
            got[key] = 0
        start = time.monotonic()
        tick = 0
        while time.monotonic() - start < 4.0:      # 100 Hz input; gimbal every 10th tick = 10 Hz
            pose_pub.publish(PoseStamped())
            frames = [transform('drone0/base_link')] + ([transform('drone0/gimbal')] if tick % 10 == 0 else [])
            tf_pub.publish(TFMessage(transforms=frames))
            tick += 1
            end = start + tick * 0.01
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=max(0.0, end - time.monotonic()))
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.01)
        seconds = time.monotonic() - start
        self.assertTrue(18 <= got['pose'] / seconds <= 27, got)       # 25 Hz
        self.assertTrue(20 <= got['odom'] / seconds <= 32, got)       # 30 Hz per frame
        self.assertTrue(8 <= got['gimbal'] / seconds <= 11, got)      # all 10 Hz gimbal kept


if __name__ == '__main__':
    unittest.main()
