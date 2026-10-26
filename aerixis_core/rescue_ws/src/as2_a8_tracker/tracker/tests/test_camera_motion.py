"""Camera-motion compensation for OC-SORT (Fast gimbal profile lost IDs while panning, 2026-10-26)."""
import sys
from pathlib import Path
import numpy as np
import cv2
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import dual_tracking as dt


def bare_tracker(camera_motion=True):
    t = dt.DualTracker.__new__(dt.DualTracker)
    t.ocs = {}; t.cmc_prev = None; t.last_shift = (0., 0.); t.camera_motion = camera_motion
    t.names = {0: 'person'}
    return t


def scene(shift_x, shift_y=0):
    rng = np.random.default_rng(3)
    base = cv2.GaussianBlur((rng.random((420, 1200)) * 255).astype(np.uint8), (5, 5), 0)
    x0, y0 = 400 - int(shift_x), 30 - int(shift_y)
    return cv2.cvtColor(base[y0:y0 + 360, x0:x0 + 640], cv2.COLOR_GRAY2BGR)


def test_shift_estimate():
    t = bare_tracker()
    t._camera_shift(scene(0))
    dx, dy = t._camera_shift(scene(24, -8))
    assert abs(dx - 24) < 2 and abs(dy + 8) < 2, (dx, dy)


def run_pan(camera_motion):
    """Small person, still in the world; the camera starts panning so the image moves 25 px/frame."""
    t = bare_tracker(camera_motion)
    ids = []; pan = 0
    for k in range(12):
        if k >= 4: pan += 25
        frame = scene(pan)
        if camera_motion:
            t._shift_tracks(*t._camera_shift(frame))
        x = 300 + pan
        rows = t._observe([[x, 150, x + 15, 190, 0.9]], [0], float(k), (360, 640))
        ids += [r['id'] for r in rows]
    return ids


def test_pan_keeps_id_with_compensation():
    ids = run_pan(True)
    assert len(ids) >= 9 and len(set(ids)) == 1, ids


def test_pan_without_compensation_changes_id():
    ids = run_pan(False)
    # The failure the compensation fixes: once the pan starts the target is lost or re-labelled.
    assert len(ids) <= 5 or len(set(ids)) > 1, ids


def test_shared_arrays_moved_once():
    t = bare_tracker()
    t._observe([[100, 100, 120, 150, 0.9]], [0], 0., (360, 640))
    t._observe([[100, 100, 120, 150, 0.9]], [0], 1., (360, 640))
    trk = t.ocs[0].trackers[0]
    before = trk.last_observation.copy(); x_before = float(trk.kf.x[0, 0])
    t._shift_tracks(10., -5.)
    assert np.allclose(trk.last_observation[:4], before[:4] + [10, -5, 10, -5])
    assert abs(float(trk.kf.x[0, 0]) - x_before - 10) < 1e-9
    for box in trk.observations.values():
        assert np.allclose(box[:4], before[:4] + [10, -5, 10, -5])


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn(); print('PASS', name)
