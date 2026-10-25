"""dualtrack2 + AERIXIS layer (2026-10-24): 16:9 model input, onboard options, telemetry, preview."""
import json
import shlex
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import hybrid_tracker as h  # noqa: E402
import detector_models as dm  # noqa: E402


class InputSizeTests(unittest.TestCase):
    def test_camera_frame_fills_the_16_9_input(self):
        frame = np.zeros((360, 640, 3), np.uint8)
        padded, r, dw, dh = h._letterbox(frame)
        self.assertEqual(padded.shape, (384, 640, 3))
        self.assertEqual((r, dw, dh), (1.0, 0, 12))         # 640x360 fits 640x384: 12 px bars only

    def test_square_size_still_works(self):
        padded, r, dw, dh = h._letterbox(np.zeros((360, 640, 3), np.uint8), 512)
        self.assertEqual(padded.shape, (512, 512, 3))

    def test_shipped_model_matches_the_input(self):
        info = dm.describe(ROOT / "model")
        self.assertEqual(info["input"], tuple(h.INPUT_SIZE))
        self.assertEqual((info["format"], info["names"]), ("ncnn", {0: "person"}))

    def test_model_without_size_or_names_is_refused(self):
        with tempfile.TemporaryDirectory() as d:
            Path(d, "model.ncnn.param").write_text("x")
            Path(d, "model.ncnn.bin").write_text("x")
            with self.assertRaisesRegex(ValueError, "Input size unknown"):
                dm.describe(d)
            Path(d, "metadata.yaml").write_text("imgsz:\n- 512\n- 512\n")
            with self.assertRaisesRegex(ValueError, "Class names unknown"):
                dm.describe(d)

    def test_boxes_map_back_to_the_frame(self):
        # A person box in model coordinates (centre 320,212 in the 640x384 input = 320,200 in
        # the 640x360 frame after removing the 12 px bar) must come back in frame pixels.
        out = np.zeros((5, 5040), np.float32)
        out[:, 0] = [320, 212, 20, 60, 0.9]
        det = FakeDetector({0: "person"}, out)
        dets, labels, _ = det.detect(np.zeros((360, 640, 3), np.uint8))
        self.assertEqual((len(dets), labels), (1, [0]))
        x1, y1, x2, y2, conf = dets[0]
        self.assertAlmostEqual((x1 + x2) / 2, 320, places=3)
        self.assertAlmostEqual((y1 + y2) / 2, 200, places=3)
        self.assertAlmostEqual(y2 - y1, 60, places=3)
        self.assertEqual(det.last_profile["height_rejected"], 0)
        det.min_person_height = 80.0                           # too small for this limit: rejected
        dets, _, _ = det.detect(np.zeros((360, 640, 3), np.uint8))
        self.assertEqual(dets, [])
        self.assertEqual(det.last_profile["height_rejected"], 1)


class FakeDetector(dm.Detector):
    def __init__(self, names, out, **kw):
        super().__init__(dict(input=(640, 384), names=names), **kw)
        self.out = out

    def run(self, img):
        return self.out


class AnyClassTests(unittest.TestCase):
    NAMES = {0: "person", 1: "dog", 2: "cat"}

    def output(self, *boxes):
        out = np.zeros((7, 5040), np.float32)
        for i, (cx, cy, w, hh, cls, score) in enumerate(boxes):
            out[:4, i] = [cx, cy, w, hh]
            out[4 + cls, i] = score
        return out

    def test_every_class_is_decoded_with_its_label(self):
        out = self.output((100, 112, 20, 60, 0, .9), (300, 212, 60, 30, 1, .8), (500, 212, 30, 30, 2, .7))
        dets, labels, _ = FakeDetector(self.NAMES, out).detect(np.zeros((360, 640, 3), np.uint8))
        self.assertEqual(labels, [0, 1, 2])

    def test_class_filter_and_person_only_shape_rules(self):
        # A wide dog box is fine; the same shape as a "person" fails the person aspect rule.
        out = self.output((300, 212, 60, 30, 1, .8), (100, 112, 60, 30, 0, .9))
        _, labels, _ = FakeDetector(self.NAMES, out).detect(np.zeros((360, 640, 3), np.uint8))
        self.assertEqual(labels, [1])
        _, labels, _ = FakeDetector(self.NAMES, out, classes=[0]).detect(np.zeros((360, 640, 3), np.uint8))
        self.assertEqual(labels, [])

    def test_overlapping_boxes_of_different_classes_both_survive(self):
        out = self.output((300, 212, 40, 80, 0, .9), (302, 214, 40, 80, 1, .85), (304, 212, 40, 80, 0, .6))
        _, labels, _ = FakeDetector(self.NAMES, out).detect(np.zeros((360, 640, 3), np.uint8))
        self.assertEqual(sorted(labels), [0, 1])                # same-class duplicate suppressed

    def test_unknown_classes_are_refused(self):
        with self.assertRaisesRegex(ValueError, "not in this model"):
            FakeDetector(self.NAMES, None, classes=[7])
        info = dict(names=self.NAMES)
        self.assertEqual(dm.class_ids(info, ["Dog", "2"]), [1, 2])
        self.assertIsNone(dm.class_ids(info, []))
        with self.assertRaisesRegex(ValueError, "horse"):
            dm.class_ids(info, ["horse"])

    def test_distance_only_for_classes_with_a_known_height(self):
        person = dict(box=[0, 0, 10, 100], label="person")
        self.assertIsNotNone(h.distance_m(person, 360))
        self.assertIsNone(h.distance_m(dict(person, label="frisbee"), 360))


class AerixisInterfaceTests(unittest.TestCase):
    def test_every_option_the_onboard_wrapper_passes_is_accepted(self):
        sys.path.insert(0, str(ROOT.parent))
        from as2_a8_tracker import tracker_process
        wrapper_args = tracker_process.main.__code__  # noqa: F841 (import check)
        args = SimpleNamespace(rtsp="rtsp://192.168.144.25:8554/main.264", camera_ip="192.168.144.25",
                               threads=2, command_cap=35, min_person_height=24, telemetry_port=9090,
                               control_port=9091, log_dir="/tmp/a8logs", cores=[0, 1], detector_cores=[2, 3],
                               stream=8080, mode="track")
        with patch.object(tracker_process, "tracker_home", return_value=ROOT):
            command = tracker_process.build_command(args, [])
        seen = {}
        with patch.object(h, "run", side_effect=lambda **kw: seen.update(kw)), \
                patch.object(h, "_setup_logging", create=True), \
                patch.object(sys, "argv", ["hybrid_tracker.py"] + command[2:]):
            try:
                h.main()
            except SystemExit as exc:
                self.fail("tracker refused the onboard options: %s (%s)" % (exc, shlex.join(command[2:])))
        self.assertEqual(seen["detector_cores"], (2, 3))
        self.assertEqual(seen["min_person_height"], 24.0)
        self.assertEqual((seen["telemetry_port"], seen["control_port"]), (9090, 9091))
        self.assertTrue(seen["as2_link"])

    def test_telemetry_carries_the_aerixis_fields(self):
        locked = {"id": 7, "box": [300, 150, 340, 250], "conf": 0.8}
        packet = json.loads(h.build_telemetry_packet(
            1.0, 25.0, [locked], locked, "LOCKED", 640, 360, "",
            extra={"build": h.BUILD_ID, "selected_label": 7, "preview_only": False,
                   "gimbal_attitude": {"yaw": 1.0}, "suppressed": False}))
        for key in ("locked_target", "targets_count", "build", "selected_label", "preview_only",
                    "gimbal_attitude", "suppressed"):
            self.assertIn(key, packet)
        self.assertIn("dualtrack2", packet["build"])


class PreviewTests(unittest.TestCase):
    def test_viewer_is_counted_while_streaming(self):
        handler = h._MJPEGHandler.__new__(h._MJPEGHandler)
        handler.path = "/stream.mjpg"
        handler.server = SimpleNamespace(latest_jpeg=b"jpeg", viewers=0, viewer_lock=threading.Lock())
        handler.send_response = handler.send_header = handler.end_headers = lambda *a: None
        watching = []

        def write(data):
            watching.append(handler.server.viewers)
            if data == b"\r\n":
                raise BrokenPipeError("viewer left")
        handler.wfile = SimpleNamespace(write=write)
        handler.do_GET()
        self.assertEqual(set(watching), {1})
        self.assertEqual(handler.server.viewers, 0)


class AnyClassLockTests(unittest.TestCase):
    """Real OC-SORT per class; NanoTrack replaced by a box-following fake."""

    def setUp(self):
        from dual_tracking import DualTracker, CLASS_ID_STRIDE
        self.stride = CLASS_ID_STRIDE
        self.now = 100.0
        nano = SimpleNamespace(box=None)
        nano.init = lambda frame, box: setattr(nano, "box", box)
        nano.update = lambda frame: (True, nano.box)
        nano.getTrackingScore = lambda: 0.9
        self.frame = np.zeros((360, 640, 3), np.uint8)
        self.make = lambda lock: DualTracker(".", factory=lambda: nano, clock=lambda: self.now,
                                             names={0: "person", 1: "dog"}, lock_classes=lock)
        self.fid = 0

    def feed(self, tracker, dets, labels):
        self.now += 0.05
        self.fid += 1
        return tracker.process(self.frame, self.fid, self.now,
                               (self.fid, self.now, dets, self.frame, labels), ())

    def test_lock_class_limits_the_selection(self):
        tracker = self.make({1})
        person, dog = [300, 120, 340, 240, 0.9], [100, 200, 180, 250, 0.8]   # person nearer the centre
        for _ in range(4):
            tracks, target, _ = self.feed(tracker, [person, dog], [0, 1])
        self.assertEqual(tracker.selected_cls, 1)
        self.assertGreaterEqual(tracker.selected_id, self.stride)
        self.assertEqual(target["label"], "dog")
        self.assertEqual({t["label"] for t in tracks}, {"person", "dog"})

    def test_another_class_never_continues_the_selection(self):
        tracker = self.make(None)
        person = [300, 120, 340, 240, 0.9]
        for _ in range(4):
            self.feed(tracker, [person], [0])
        self.assertEqual((tracker.selected_cls, tracker.selected_id < self.stride), (0, True))
        selected = tracker.selected_id
        # Same box, now reported as a dog: not the person, so no fresh confirmation for the lock.
        before = tracker.last_detection_t
        self.feed(tracker, [person], [1])
        self.assertEqual(tracker.last_detection_t, before)
        self.assertEqual(tracker.selected_id, selected)


if __name__ == "__main__":
    unittest.main()
