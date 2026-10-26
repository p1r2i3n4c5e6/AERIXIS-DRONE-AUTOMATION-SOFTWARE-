"""Hailo HEF output (on-chip NMS, per-class boxes normalised to the model input) goes through the same
decode as every other format: boxes back in frame pixels, class filter, person size rules (2026-10-26)."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import detector_models as dm  # noqa: E402


class _Probe(dm.Detector):
    def __init__(self, names, size, classes=None):
        super().__init__(dict(input=size, names=names), classes=classes)


class HailoDecodeTests(unittest.TestCase):
    def test_nms_boxes_land_in_frame_pixels(self):
        names = {0: 'person', 1: 'dog'}
        w, h = 640, 640
        # person box centred, 100 x 300 px in the 640x640 input; a dog box; one low-score person
        person = [[(320 - 150) / h, (320 - 50) / w, (320 + 150) / h, (320 + 50) / w, 0.9],
                  [0.1, 0.1, 0.2, 0.15, 0.2]]
        dog = [[0.70, 0.10, 0.80, 0.25, 0.8]]
        out = dm.hailo_nms_to_yolo([person, dog], w, h, len(names))
        self.assertEqual(out.shape, (6, 3))
        probe = _Probe(names, (w, h))
        # a 1280x720 frame letterboxed into 640x640: r = 0.5, dw = 0, dh = 140
        dets, labels, _ = probe.decode(out, 0.5, 0.0, 140.0, 1280, 720, 0.5)
        self.assertEqual(sorted(labels), [0, 1])
        p = dets[labels.index(0)]
        np.testing.assert_allclose(p[:4], [540, 60, 740, 660], atol=1.0)   # input y 170..470 -> (y - 140) / 0.5
        self.assertAlmostEqual(p[4], 0.9, places=5)

    def test_class_filter_and_empty_output(self):
        names = {0: 'person', 1: 'dog'}
        out = dm.hailo_nms_to_yolo([[], [[0.7, 0.1, 0.8, 0.25, 0.8]]], 640, 640, 2)
        dets, labels, _ = _Probe(names, (640, 640), classes=[0]).decode(out, 1.0, 0, 0, 640, 640, 0.5)
        self.assertEqual(dets, [])
        empty = dm.hailo_nms_to_yolo([[], []], 640, 640, 2)
        self.assertEqual(empty.shape, (6, 0))
        self.assertEqual(_Probe(names, (640, 640)).decode(empty, 1.0, 0, 0, 640, 640, 0.5)[0], [])

    def test_hef_is_an_implemented_format(self):
        self.assertIn('hef', dm.IMPLEMENTED)


if __name__ == '__main__':
    unittest.main()
