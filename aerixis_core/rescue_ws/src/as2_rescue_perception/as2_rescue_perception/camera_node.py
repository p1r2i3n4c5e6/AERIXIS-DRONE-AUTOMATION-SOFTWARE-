"""Receive A8 RTSP frames with bounded caching, freshness and reconnect reporting."""
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import yaml

from .image_utils import LatestFrame
from .calibration import validate_calibration


class CameraNode(Node):
    def __init__(self):
        super().__init__('a8_camera')
        for name, value in [('rtsp_url', 'rtsp://192.168.144.25:8554/main.264'),
                            ('frame_id', 'camera_optical_frame'), ('calibration_file', ''),
                            ('publish_fps', 15.0), ('max_frame_age_s', 0.5),
                            ('reconnect_delay_s', 1.0), ('io_timeout_ms', 2000)]:
            self.declare_parameter(name, value)
        self.url = self.get_parameter('rtsp_url').value
        self.frame_id = self.get_parameter('frame_id').value
        self.max_age = self.get_parameter('max_frame_age_s').value
        self.retry = self.get_parameter('reconnect_delay_s').value
        self.timeout_ms = self.get_parameter('io_timeout_ms').value
        fps = self.get_parameter('publish_fps').value
        if fps <= 0 or self.max_age <= 0 or self.retry <= 0 or self.timeout_ms <= 0:
            raise ValueError('Camera rates and timeouts must be positive')
        self.calibration = None
        calibration_file = self.get_parameter('calibration_file').value
        if calibration_file:
            self.calibration = validate_calibration(yaml.safe_load(Path(calibration_file).read_text()))
        self.cache = LatestFrame()
        self.last_sequence = 0
        self.state = 'CONNECTING'
        self.calibrated = False
        self.last_error = ''
        self.stop = threading.Event()
        self.images = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
        self.infos = self.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_sensor_data)
        self.health = self.create_publisher(String, 'camera/health', 10)
        self.create_timer(1.0 / fps, self.publish_frame)
        self.create_timer(0.5, self.publish_health)
        cv2.setNumThreads(1)
        self.worker = threading.Thread(target=self.capture, daemon=True)
        self.worker.start()

    def capture(self):
        while not self.stop.is_set():
            cap = None
            try:
                self.state = 'CONNECTING'
                cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, [
                    cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self.timeout_ms,
                    cv2.CAP_PROP_READ_TIMEOUT_MSEC, self.timeout_ms])
                if not cap.isOpened():
                    raise RuntimeError('RTSP connection failed')
                while not self.stop.is_set():
                    ok, frame = cap.read()
                    if not ok or frame is None:
                        raise RuntimeError('RTSP frame read failed')
                    # This is decoder receipt time, not a hardware exposure timestamp.
                    self.cache.put(frame, time.monotonic(), self.get_clock().now().to_msg())
                    self.state = 'STREAMING'
                    self.last_error = ''
            except Exception as exc:
                self.cache.clear()
                self.state = 'DISCONNECTED'
                self.last_error = str(exc)
            finally:
                if cap is not None:
                    cap.release()
            self.stop.wait(self.retry)

    def publish_frame(self):
        item = self.cache.take(self.last_sequence, time.monotonic(), self.max_age)
        if item is None:
            return
        self.last_sequence, frame, stamp = item
        frame = np.ascontiguousarray(frame)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = msg.width, msg.height
        self.calibrated = bool(self.calibration and
                               self.calibration['image_width'] == msg.width and
                               self.calibration['image_height'] == msg.height)
        if self.calibrated:
            c = self.calibration
            info.distortion_model = c['distortion_model']
            info.d = [float(v) for v in c['distortion_coefficients']['data']]
            info.k = [float(v) for v in c['camera_matrix']['data']]
            info.r = [float(v) for v in c['rectification_matrix']['data']]
            info.p = [float(v) for v in c['projection_matrix']['data']]
        # Uncalibrated CameraInfo has K[0] == 0; never invent camera intrinsics.
        self.images.publish(msg)
        self.infos.publish(info)

    def publish_health(self):
        age = self.cache.age(time.monotonic())
        fresh = age is not None and age <= self.max_age
        self.health.publish(String(data=json.dumps(dict(
            state=self.state if fresh else 'STALE', connection_state=self.state,
            frame_age_s=age, fresh=fresh,
            calibrated=self.calibrated, timestamp_source='decoder_receipt',
            end_to_end_latency_verified=False, error=self.last_error))))

    def destroy_node(self):
        self.stop.set()
        self.worker.join(timeout=self.timeout_ms / 1000.0 + self.retry + 1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
