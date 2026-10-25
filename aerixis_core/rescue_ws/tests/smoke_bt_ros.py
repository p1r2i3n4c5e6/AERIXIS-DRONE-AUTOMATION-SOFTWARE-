"""Test detection freshness and cancel service against the real BT executable."""
import os
os.environ.update(ROS_DOMAIN_ID='91', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR='/tmp/as2_bt_phase1_test_logs')
from pathlib import Path
import sys
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'integrations/gcs'))
from bt_execution import LocalTreeRun

rclpy.init()
probe = Node('probe', namespace='perception_bt_test')
publisher = probe.create_publisher(Detection2DArray, 'perception/detections', qos_profile_sensor_data)
client = probe.create_client(Trigger, 'mission/cancel')
try:
    for case in ('fresh', 'stale', 'low_confidence', 'cancel_service'):
        body = ('<Sleep msec="10000"/>' if case == 'cancel_service' else
                '<Timeout msec="2000"><RetryUntilSuccessful num_attempts="-1">'
                '<Delay delay_msec="50"><IsPersonDetected/></Delay></RetryUntilSuccessful></Timeout>')
        run = LocalTreeRun('perception_bt_test',
                           f'<root BTCPP_format="4"><BehaviorTree ID="Test">{body}</BehaviorTree></root>')
        proc = run.start()
        lines = []
        def read():
            for line in proc.stdout:
                lines.append(line)
                run.state.accept(line.strip())
        reader = threading.Thread(target=read)
        reader.start()
        deadline = time.monotonic() + 8
        future = None
        while proc.poll() is None and time.monotonic() < deadline:
            msg = Detection2DArray()
            msg.header.frame_id = 'perception_bt_test/camera_optical_frame'
            msg.header.stamp = probe.get_clock().now().to_msg()
            if case == 'stale':
                msg.header.stamp.sec -= 5
            detection = Detection2D()
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'person'
            hypothesis.hypothesis.score = 0.1 if case == 'low_confidence' else 0.9
            detection.results = [hypothesis]
            msg.detections = [detection]
            publisher.publish(msg)
            if case == 'cancel_service' and future is None and client.service_is_ready():
                future = client.call_async(Trigger.Request())
            rclpy.spin_once(probe, timeout_sec=0.02)
        if proc.poll() is None:
            proc.kill()
            raise AssertionError('Synthetic BT timed out')
        reader.join(timeout=2)
        run.state.finish_process(proc.returncode)
        expected = {'fresh': 'SUCCESS', 'stale': 'FAILURE',
                    'low_confidence': 'FAILURE', 'cancel_service': 'CANCELLED'}[case]
        assert run.state.outcome == expected, ''.join(lines)
        if future is not None:
            assert future.done() and future.result().success
        print(f'{case}: {expected}')
finally:
    probe.destroy_node()
    rclpy.shutdown()
