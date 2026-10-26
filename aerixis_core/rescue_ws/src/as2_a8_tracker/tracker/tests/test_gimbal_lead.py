"""Gimbal speed profiles and look-ahead (2026-10-26)."""
import sys, threading, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import hybrid_tracker as h
from as2_a8_tracker import tracker_process as tp


def controller(lead_s):
    c = h.UnifiedGimbalController.__new__(h.UnifiedGimbalController)
    c.state_lock = threading.Lock(); c.scan_active = False; c.last_target = None
    c.lead_s = lead_s; c._vel_prev = None; c.est_vx = c.est_vy = 0.0
    return c


def track(ident, cx, cy, t):
    return {'id': ident, 'box': (cx - 10, cy - 20, cx + 10, cy + 20), 'velocity': (0.0, 0.0),
            'state_time': t, 'observed_t': t}


def test_profiles_and_override():
    normal = tp.gimbal_arguments('normal')
    assert normal == ['--siyi-command-cap', '35', '--kp-yaw', '20.0', '--kp-pitch', '16.0', '--lead-s', '0.0']
    fast = tp.gimbal_arguments('fast')
    assert fast[1] == '70' and fast[-1] == '0.15'
    assert tp.gimbal_arguments('max', command_cap=50)[1] == '50'
    try:
        tp.gimbal_arguments('turbo'); raise AssertionError('unknown profile accepted')
    except ValueError:
        pass


def test_no_lead_keeps_original_behaviour():
    c = controller(0.0)
    for i in range(5):
        c.update(track(1, 100 + 20 * i, 200, 10 + 0.04 * i), 640, 360)
    assert c.last_target['vx'] == 0.0 and c.last_target['vy'] == 0.0


def test_lead_measures_speed_and_resets():
    c = controller(0.15)
    for i in range(8):                       # 20 px per 40 ms = 500 px/s to the right
        c.update(track(1, 100 + 20 * i, 200, 10 + 0.04 * i), 640, 360)
    assert 450 < c.last_target['vx'] <= 500 and abs(c.last_target['vy']) < 1e-6
    c.update(track(1, 260, 200, 10 + 0.04 * 7), 640, 360)    # same observation again: unchanged
    assert 450 < c.last_target['vx'] <= 500
    c.update(track(2, 600, 50, 10.4), 640, 360)                # a new target never inherits a speed
    assert c.last_target['vx'] == 0.0
    c.update(track(2, 610, 50, 11.0), 640, 360)                # gap > 0.3 s: start again
    assert c.last_target['vx'] == 0.0
    c.update(None, 640, 360)
    assert c._vel_prev is None and c.est_vx == 0.0


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn(); print('PASS', name)
