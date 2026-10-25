"""vision/frame (one message per camera frame) must give the runtime exactly the evidence the three
separate topics gave (vision/target, gimbal/state, perception/tracks)."""
import json
from types import SimpleNamespace
import unittest

from as2_rescue_runtime.runtime_node import RescueRuntime
from as2_rescue_runtime.safety import Evidence


class Clock:
    def __init__(self):
        self.value = 1000.0

    def __call__(self):
        return self.value


def runtime():
    node = RescueRuntime.__new__(RescueRuntime)
    node.clock = Clock()
    node.evidence = Evidence(node.clock)
    node.engine = SimpleNamespace(active=None)
    return node


FRAME = dict(
    schema_version=1,
    vision=dict(stamp=999.95, fresh=True, locked=True, label='7', err_x=0.1, err_y=-0.05, gimbal=None),
    gimbal=dict(stamp=999.95, verified=True, fresh=True, yaw=2.0, pitch=-30.0, track_id='7'),
    tracks=dict(image_stamp_ns=int(999.95e9), tracks=[
        dict(track_id='7', confirmed=True, predicted=False, confidence=0.8),
        dict(track_id='9', confirmed=True, predicted=False, confidence=0.9),
        dict(track_id='3', confirmed=False, predicted=False, confidence=0.99)]))


def as_msg(value):
    return SimpleNamespace(data=json.dumps(value))


class CameraFrameTests(unittest.TestCase):
    def test_frame_equals_the_three_topics(self):
        separate, combined = runtime(), runtime()
        separate.json_sample('vision', as_msg(FRAME['vision']))
        separate.json_sample('gimbal', as_msg(FRAME['gimbal']))
        separate.tracks(as_msg(FRAME['tracks']))
        combined.vision_frame(as_msg(FRAME))
        for key in ('vision', 'gimbal', 'track'):
            self.assertEqual(separate.evidence.get(key), combined.evidence.get(key), key)
        self.assertEqual(combined.evidence.get('track')['track_id'], '9')   # most confident confirmed

    def test_partial_or_bad_frames_are_ignored_safely(self):
        node = runtime()
        node.vision_frame(SimpleNamespace(data='not json'))
        node.vision_frame(as_msg([1, 2]))
        node.vision_frame(as_msg(dict(vision='x', gimbal=None)))
        self.assertIsNone(node.evidence.get('vision'))
        self.assertIsNone(node.evidence.get('gimbal'))
        node.vision_frame(as_msg(dict(gimbal=FRAME['gimbal'])))          # gimbal only
        self.assertEqual(node.evidence.get('gimbal')['yaw'], 2.0)
        self.assertIsNone(node.evidence.get('vision'))


if __name__ == '__main__':
    unittest.main()
