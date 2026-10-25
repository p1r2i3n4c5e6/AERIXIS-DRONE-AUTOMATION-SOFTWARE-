"""Person detections/tracks only: no flight commands, distance inference or release."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .image_utils import decode_bgr8
from .model_adapter import ModelAdapter


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        for name, value in [('deployment_dir', ''), ('ncnn_threads', 2),
                            ('confidence', 0.35), ('max_image_age_s', 0.5),
                            ('max_track_age_s', 0.75)]:
            self.declare_parameter(name, value)
        self.max_age = self.get_parameter('max_image_age_s').value
        self.max_track_age = self.get_parameter('max_track_age_s').value
        if self.max_age <= 0 or self.max_track_age <= 0:
            raise ValueError('Freshness limits must be positive')
        self.model = ModelAdapter(self.get_parameter('deployment_dir').value,
                                  self.get_parameter('ncnn_threads').value,
                                  self.get_parameter('confidence').value)
        self.last_frame_ns = None
        self.last_valid = None
        self.last_seen = {}
        self.inference_ms = None
        self.error = ''
        self.tracks_invalidated = True
        self.detections = self.create_publisher(
            Detection2DArray, 'perception/detections', qos_profile_sensor_data)
        self.tracks = self.create_publisher(String, 'perception/tracks', 10)
        self.health = self.create_publisher(String, 'perception/health', 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_timer(0.1, self.watchdog)

    def on_image(self, msg):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if not msg.header.frame_id or (self.last_frame_ns is not None and stamp_ns <= self.last_frame_ns) or not 0 <= age <= self.max_age:
            return
        self.last_frame_ns = stamp_ns
        try:
            frame = decode_bgr8(msg)
            boxes, self.inference_ms = self.model.infer(frame)
            # Inference may have made the result too old to use.
            age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            if not 0 <= age <= self.max_age:
                self.error = 'Inference output exceeded image age limit'
                return
            boxes = [b for b in boxes if len(b) == 5 and all(math.isfinite(v) for v in b)
                     and b[2] > b[0] and b[3] > b[1] and 0 <= b[4] <= 1]
            result = Detection2DArray()
            result.header = msg.header
            for x1, y1, x2, y2, confidence in boxes:
                detection = Detection2D()
                detection.header = msg.header
                detection.bbox.center.position.x = (x1 + x2) / 2.0
                detection.bbox.center.position.y = (y1 + y2) / 2.0
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = 'person'
                hypothesis.hypothesis.score = confidence
                detection.results = [hypothesis]
                result.detections.append(detection)
            now = time.monotonic()
            # Do not match new people against tracks from a disconnected camera.
            if self.last_valid is not None and now - self.last_valid > self.max_track_age:
                self.model.tracker.trackers.clear()
                self.last_seen.clear()
            tracks = self.model.tracker.update(boxes)
            hits = {t.id: t.hits for t in self.model.tracker.trackers}
            active = []
            for track in tracks:
                track_id = track['id']
                if not track['tracking']:
                    self.last_seen[track_id] = now
                observation_age = now - self.last_seen.get(track_id, float('-inf'))
                if observation_age <= self.max_track_age:
                    active.append(dict(track_id=track_id, bbox_pixels=track['box'],
                                       confidence=track['conf'], predicted=track['tracking'],
                                       confirmed=hits.get(track_id, 0) >= 2,
                                       observation_age_s=observation_age))
            self.last_seen = {i: t for i, t in self.last_seen.items() if now - t <= self.max_track_age}
            self.detections.publish(result)  # Publish empty detections too; absence is meaningful.
            self.tracks.publish(String(data=json.dumps(dict(
                schema_version=1, frame_id=msg.header.frame_id, image_stamp_ns=stamp_ns,
                tracks=active, world_position_valid=False))))
            self.last_valid = now
            self.tracks_invalidated = False
            self.error = ''
        except Exception as exc:
            self.error = str(exc)
            self.get_logger().error('Detection failed: ' + self.error)

    def watchdog(self):
        age = None if self.last_valid is None else time.monotonic() - self.last_valid
        fresh = age is not None and age <= self.max_track_age
        if not fresh and not self.tracks_invalidated:
            self.tracks.publish(String(data=json.dumps(dict(
                schema_version=1, tracks=[], stale=True, world_position_valid=False))))
            self.tracks_invalidated = True
        self.health.publish(String(data=json.dumps(dict(
            fresh=fresh, observation_age_s=age, inference_ms=self.inference_ms,
            world_position_valid=False, error=self.error))))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PersonDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
