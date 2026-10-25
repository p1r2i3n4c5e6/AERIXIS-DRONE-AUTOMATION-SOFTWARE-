"""A stalled RTSP stream must time out and reconnect, not block the tracker forever (2026-10-25)."""
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
import hybrid_tracker as h  # noqa: E402


class FakeCap:
    """Gives `frames` frames, then fails reads like a timed-out stream."""
    def __init__(self, frames):
        self.frames = frames

    def set(self, *_):
        return True

    def isOpened(self):
        return True

    def read(self):
        if self.frames > 0:
            self.frames -= 1
            return True, np.zeros((720, 1280, 3), np.uint8)
        time.sleep(0.01)
        return False, None

    def release(self):
        pass


class RtspTimeoutTests(unittest.TestCase):
    def test_open_passes_open_and_read_timeouts(self):
        calls = []
        with patch.object(h.cv2, 'VideoCapture', side_effect=lambda *a: calls.append(a) or FakeCap(0)):
            h.open_rtsp('rtsp://camera/main.264')
        params = calls[0][2]
        self.assertIn(h.cv2.CAP_PROP_READ_TIMEOUT_MSEC, params)
        self.assertEqual(params[params.index(h.cv2.CAP_PROP_READ_TIMEOUT_MSEC) + 1], h.RTSP_READ_TIMEOUT_MS)
        self.assertIn(h.cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, params)

    def test_stalled_stream_reconnects_and_frames_resume(self):
        caps = [FakeCap(3), FakeCap(50)]          # first session stalls after 3 frames
        opened = []
        with patch.object(h, 'open_rtsp', side_effect=lambda url: opened.append(url) or caps.pop(0)):
            grabber = h.LowLatencyA8Grabber('rtsp://camera/main.264')
            try:
                frames, end = 0, time.time() + 5
                while time.time() < end and (grabber.reconnects < 1 or frames < 8):
                    ok, frame = grabber.read(timeout=0.1)
                    if ok:
                        frames += 1
                        self.assertEqual(frame.shape, (360, 640, 3))   # resized for processing
            finally:
                grabber.release()
        self.assertEqual(grabber.reconnects, 1)
        self.assertEqual(len(opened), 2)
        self.assertGreaterEqual(frames, 8)        # frames resumed on the new session


if __name__ == '__main__':
    unittest.main()
