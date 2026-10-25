"""Actual ROS transport + actual NCNN model, with synthetic images and camera IO."""
import os
os.environ.update(ROS_DOMAIN_ID='91', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR='/tmp/as2_perception_test_logs')
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from as2_rescue_perception.detector_node import PersonDetector

root = Path(__file__).resolve().parents[2]
rclpy.init(args=['--ros-args', '-r', '__ns:=/perception_test', '-p',
                 'deployment_dir:=' + str(root / 'RASPI5_DEPLOYMENT_PACKAGE')])
detector = PersonDetector()
probe = Node('probe')
executor = SingleThreadedExecutor()
executor.add_node(detector)
executor.add_node(probe)
output, tracks = [], []
sub = probe.create_subscription(Detection2DArray, 'perception/detections', output.append, qos_profile_sensor_data)
track_sub = probe.create_subscription(String, 'perception/tracks', lambda m: tracks.append(json.loads(m.data)), 10)
publisher = probe.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
frame = Image(height=512, width=512, encoding='bgr8', step=1536)
frame.header.frame_id = 'perception_test/camera_optical_frame'
frame.data = np.zeros((512,512,3),dtype=np.uint8).tobytes()
try:
    deadline = time.monotonic() + 8
    while not output and time.monotonic() < deadline:
        frame.header.stamp = probe.get_clock().now().to_msg()
        publisher.publish(frame)
        executor.spin_once(timeout_sec=0.05)
    assert output, 'Detector did not publish synthetic-image results'
    assert output[-1].header.frame_id == frame.header.frame_id
    assert len(output[-1].detections) == 0
    # Stop input and require explicit stale-track invalidation.
    deadline = time.monotonic() + 3
    while not any(t.get('stale') for t in tracks) and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert any(t.get('stale') and t['tracks'] == [] for t in tracks)
    before = len(output)
    frame.header.stamp.sec -= 10
    publisher.publish(frame)
    for _ in range(10):
        executor.spin_once(timeout_sec=0.02)
    assert len(output) == before, 'Stale image was processed'
    print('ROS detector: NCNN output delivered, stale tracks cleared, stale input rejected')
finally:
    executor.shutdown()
    detector.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()
