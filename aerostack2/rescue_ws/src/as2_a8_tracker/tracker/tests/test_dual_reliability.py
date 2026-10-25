"""Offline lock regression tests: no sockets, video source, or actuator."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dual_tracking import DualTracker


class Nano:
    def __init__(self):
        self.ok = True
        self.xywh = (100, 60, 80, 160)
        self.score = .9
        self.calls = self.inits = 0
        self.on_update = lambda: None

    def init(self, frame, box):
        self.inits += 1
        self.xywh = box

    def update(self, frame):
        self.calls += 1
        self.on_update()
        return self.ok, self.xywh

    def getTrackingScore(self):
        return self.score


class OC:
    def __init__(self, ids):
        self.ids = ids
        self.trackers = [SimpleNamespace(id=i-1, hits=2) for i in ids]

    def update(self, arr, *args):
        return np.array([list(d[:4]) + [i] for d, i in zip(arr, self.ids)])


class LockTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.nano = Nano()
        self.factory_calls = 0
        def factory():
            self.factory_calls += 1
            return self.nano
        self.t = DualTracker(".", factory=factory, clock=lambda: self.now)
        self.frame = np.zeros((360, 640, 3), np.uint8)
        self.box = [100, 60, 180, 220]
        self.t._seed(self.frame, 1, self.now, self.box, ())
        self.t.selected_id = self.t.oc_id = 1
        self.t.selected_cls = 0
        self.t.selected_generation = 0
        self.t.last_detection_t = self.now
        self.t.last_detection_box = self.box
        self.fid = 1

    def step(self, result=None, advance=.04):
        self.now += advance
        self.fid += 1
        return self.t.process(self.frame, self.fid, self.now, result)[1]

    def result(self, ids, boxes):
        self.t.ocs = {0: OC(ids)}
        return (self.fid, self.now, [b+[.9] for b in boxes], self.frame)

    def test_failed_outside_box_retains_identity_and_stops(self):
        self.nano.ok = False
        self.nano.xywh = (800, 0, 80, 160)
        self.assertIsNone(self.step())
        self.assertEqual(self.t.selected_id, 1)

    def test_brief_score_dip_requires_two_good_fresh_frames(self):
        self.nano.score = .1
        self.assertIsNone(self.step())
        self.nano.score = .9
        self.assertIsNone(self.step())
        self.assertEqual(self.step()["id"], 1)
        self.assertEqual(self.factory_calls, 1)

    def test_duplicate_frame_does_not_count_as_recovery(self):
        self.nano.score = .1
        self.step()
        self.nano.score = .9
        self.step()
        calls = self.nano.calls
        self.assertIsNone(self.t.process(self.frame, self.fid, self.now)[1])
        self.assertEqual(calls, self.nano.calls)
        self.assertIsNotNone(self.step())

    def test_uncertain_recovery_has_deadline(self):
        self.nano.score = .1
        self.step()
        self.nano.score = .9
        self.assertIsNone(self.step(advance=.3))
        self.assertFalse(self.t.nano_initialized)
        self.assertEqual(self.t.selected_id, 1)

    def test_ambiguity_blocks_motion_despite_good_nano(self):
        result = self.result([1, 2], [self.box, [110, 60, 190, 220]])
        self.assertIsNone(self.step(result))
        self.assertIsNone(self.step())
        self.assertEqual(self.t.selected_id, 1)
        self.assertIsNotNone(self.step(self.result([1], [self.box])))

    def test_id_change_needs_three_distinct_detections(self):
        for _ in range(2):
            self.assertIsNone(self.step(self.result([2], [self.box])))
        self.assertIsNotNone(self.step(self.result([2], [self.box])))
        self.assertEqual(self.t.oc_id, 2)
        self.assertEqual(self.t.selected_id, 1)

    def test_stale_during_nano_update_cannot_drive(self):
        self.nano.on_update = lambda: setattr(self, "now", self.now+.5)
        self.assertIsNone(self.step())

    def hold(self, seconds, advance=.04):
        target = None
        for _ in range(int(seconds / advance)):
            target = self.step(advance=advance)
        return target

    def test_confident_nano_holds_the_lock_while_the_detector_is_quiet(self):
        # Small target at altitude: no detections, but NanoTrack is sure and the box is well inside.
        self.assertIsNotNone(self.hold(3.5))
        self.assertIn("holding", self.t.reason)
        self.assertIsNone(self.hold(1.0))                     # beyond nano_hold_s (4 s): movement stops
        self.assertEqual(self.t.selected_id, 1)               # but the selection is still held

    def test_detector_expiry_stops_a_doubtful_nano(self):
        self.nano.score = .5                                  # valid for Nano, below hold_score
        self.assertIsNone(self.hold(2.2))
        self.assertEqual(self.t.selected_id, 1)

    def test_detection_elsewhere_ends_the_long_hold(self):
        # Others seen, ours missed: only the normal 2 s grace, no 4 s NanoTrack hold.
        self.hold(2.2)
        self.assertIsNone(self.step(self.result([5], [[400, 60, 480, 220]])))
        self.assertIn("expired", self.t.reason)
        self.assertEqual(self.t.selected_id, 1)

    def test_a_miss_in_a_crowd_does_not_stop_the_gimbal(self):
        self.assertIsNotNone(self.step(self.result([5], [[400, 60, 480, 220]])))   # ours missed once
        self.assertIsNone(self.t.motion_block)
        self.assertIsNotNone(self.step(self.result([1], [self.box])))               # ours seen again

    def test_overlapping_other_person_still_stops(self):
        self.assertIsNone(self.step(self.result([1, 2], [self.box, [104, 60, 184, 220]])))
        self.assertIn("ambiguous", self.t.reason)

    def test_target_leaving_the_frame_releases_the_selection(self):
        for x in range(85, 0, -15):                           # walks to the left edge
            self.nano.xywh = (x, 60, 80, 160)
            self.step()
        self.nano.ok = False                                  # gone out of the picture
        self.hold(1.2)
        self.assertIsNone(self.t.selected_id)
        self.assertEqual(self.t.reason, "target left the frame")
        self.assertEqual(self.t.state, "SEARCHING")

    def test_target_lost_in_the_middle_is_released_after_five_seconds(self):
        self.nano.ok = False
        self.hold(4.5)
        self.assertEqual(self.t.selected_id, 1)               # short occlusion: kept
        self.hold(0.7)
        self.assertIsNone(self.t.selected_id)
        self.assertIn("not seen", self.t.reason)

    def test_next_target_is_selected_after_a_release(self):
        self.nano.ok = False
        self.hold(5.2)
        self.nano.ok = True
        self.t.ocs = {}
        for _ in range(4):
            self.fid += 1
            self.now += .04
            target = self.t.process(self.frame, self.fid, self.now,
                                    (self.fid, self.now, [[300, 60, 380, 220, .9]], self.frame))[1]
        self.assertIsNotNone(self.t.selected_id)
        self.assertEqual(self.t.releases, 1)

    def test_reseed_keeps_loaded_networks(self):
        self.assertTrue(self.t._seed(self.frame, 2, self.now, self.box, ()))
        self.assertEqual(self.factory_calls, 1)
        self.assertEqual(self.nano.inits, 2)

    def test_replay_budget_fails_closed(self):
        history = [(2, self.now, self.frame), (3, self.now, self.frame)]
        with patch("dual_tracking.time.perf_counter", side_effect=[0., .1, .11]):
            self.assertFalse(self.t._seed(self.frame, 1, self.now, self.box, history))
        self.assertFalse(self.t.nano_initialized)

    def test_actual_ocsort_acquires_after_confirmation(self):
        self.t.reset()
        for _ in range(3):
            self.fid += 1
            self.now += .04
            result = (self.fid, self.now, [self.box+[.9]], self.frame)
            target = self.t.process(self.frame, self.fid, self.now, result)[1]
            if self.t.selected_id is not None:
                break
        self.assertIsNotNone(target)


if __name__ == "__main__":
    unittest.main()
