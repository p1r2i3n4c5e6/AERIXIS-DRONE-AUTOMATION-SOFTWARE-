"""ncnn NanoTrack port gives the same boxes and scores as OpenCV's TrackerNano (2026-10-26)."""
import sys
from pathlib import Path
import numpy as np
import cv2
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import nano_ncnn   # noqa: E402
import dual_tracking as dt   # noqa: E402

MODEL = ROOT / 'model'


def walking_scene(n=60):
    """Textured background with a person-like block walking right and a decoy standing still."""
    rng = np.random.default_rng(7)
    base = cv2.GaussianBlur((rng.random((360, 640, 3)) * 255).astype(np.uint8), (9, 9), 0)
    frames = []
    for k in range(n):
        f = base.copy()
        x = 200 + 3 * k
        cv2.rectangle(f, (x, 120), (x + 30, 200), (40, 60, 200), -1)
        cv2.circle(f, (x + 15, 110), 10, (60, 160, 220), -1)
        cv2.rectangle(f, (480, 130), (505, 205), (200, 200, 40), -1)
        frames.append(f)
    return frames, (200, 100, 31, 101)


def run(tracker, frames, box):
    tracker.init(frames[0], box)
    out = []
    for f in frames[1:]:
        ok, b = tracker.update(f)
        out.append((tuple(int(v) for v in b), float(tracker.getTrackingScore())))
    return out


def test_available_and_opt_in():
    import os
    assert nano_ncnn.available(MODEL)
    assert not isinstance(dt.nano_factory(MODEL), nano_ncnn.NanoTrackNcnn)   # default: OpenCV
    os.environ['A8_NANO_BACKEND'] = 'ncnn'
    try:
        assert isinstance(dt.nano_factory(MODEL), nano_ncnn.NanoTrackNcnn)
    finally:
        os.environ.pop('A8_NANO_BACKEND')


def test_matches_opencv_tracker_nano():
    frames, box = walking_scene()
    ours = run(nano_ncnn.NanoTrackNcnn(MODEL), frames, box)
    ref = run(dt.opencv_nano(MODEL), frames, box)
    worst = max(max(abs(a - b) for a, b in zip(o[0], r[0])) for o, r in zip(ours, ref))
    score = max(abs(o[1] - r[1]) for o, r in zip(ours, ref))
    assert worst <= 1 and score < 1e-3, (worst, score)
    assert abs(ours[-1][0][0] - (200 + 3 * 59)) < 15     # it followed the walker, not the decoy


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn(); print('PASS', name)
