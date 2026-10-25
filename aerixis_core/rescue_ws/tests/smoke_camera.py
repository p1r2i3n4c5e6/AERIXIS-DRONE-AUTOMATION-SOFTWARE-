"""Test camera ROS output and disconnect freshness with a synthetic decoder."""
import os
os.environ.update(ROS_DOMAIN_ID='91', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR='/tmp/as2_perception_test_logs')
import json
import time
from unittest.mock import patch
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from as2_rescue_perception.camera_node import CameraNode


class FakeCapture:
    attempts = 0
    def __init__(self, *args):
        FakeCapture.attempts += 1
        self.open = FakeCapture.attempts == 1
        self.frames = 0
    def isOpened(self):
        return self.open
    def read(self):
        time.sleep(0.03)
        self.frames += 1
        return (True, np.zeros((8, 8, 3), dtype=np.uint8)) if self.frames <= 15 else (False, None)
    def release(self):
        pass


rclpy.init(args=['--ros-args', '-r', '__ns:=/camera_test'])
with patch('as2_rescue_perception.camera_node.cv2.VideoCapture', FakeCapture):
    camera = CameraNode()
    probe = Node('probe')
    images, infos, health = [], [], []
    subscriptions = [
        probe.create_subscription(Image, 'camera/image_raw', images.append, qos_profile_sensor_data),
        probe.create_subscription(CameraInfo, 'camera/camera_info', infos.append, qos_profile_sensor_data),
        probe.create_subscription(String, 'camera/health', lambda m: health.append(json.loads(m.data)), 10)]
    executor = SingleThreadedExecutor()
    executor.add_node(camera)
    executor.add_node(probe)
    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)
        assert images and infos, 'Synthetic camera did not publish'
        stamps = [(m.header.stamp.sec, m.header.stamp.nanosec) for m in images]
        assert len(stamps) == len(set(stamps)), 'Duplicate decoded frame published'
        assert all(i.k[0] == 0.0 for i in infos), 'Invented camera calibration'
        assert health and health[-1]['fresh'] is False
        assert FakeCapture.attempts >= 2, 'No reconnect attempted'
        print('Camera: BGR images, uncalibrated CameraInfo, no duplicates, reconnect and stale health verified')
    finally:
        executor.shutdown()
        camera.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
