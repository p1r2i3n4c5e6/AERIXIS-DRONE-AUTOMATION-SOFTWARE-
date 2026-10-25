import json
import math
from pathlib import Path
import socket
import sys
import time

from as2_a8_tracker.sim_vision import image_error, pointing_angles
from as2_a8_tracker.vision_schema import gimbal_state, target_message, tracks

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tracker' / 'src'))
from as2_link import MissionLink  # noqa: E402


def test_person_ahead_and_right_maps_to_positive_error():
    aircraft = [0.0, 0.0, 20.0]
    # Aircraft faces east (yaw 0); person 30 m east, 3 m south = slightly right.
    yaw, pitch = pointing_angles(aircraft, 0.0, [30.0, 0.0, 0.0], 0.0)
    error = image_error(aircraft, 0.0, yaw, pitch, [30.0, -3.0, 0.0])
    assert error is not None and error[0] > 0.05 and abs(error[1]) < 0.1


def test_person_below_centre_has_positive_vertical_error():
    aircraft = [0.0, 0.0, 20.0]
    error = image_error(aircraft, 0.0, 0.0, -30.0, [20.0, 0.0, 0.0])
    assert error is not None and error[1] > 0


def test_pointing_centres_the_person_and_reaches_nadir_overhead():
    aircraft = [5.0, 5.0, 15.0]
    person = [12.0, -4.0, 0.0]
    yaw, pitch = pointing_angles(aircraft, math.radians(30), person, 0.0)
    error = image_error(aircraft, math.radians(30), yaw, pitch, person)
    assert abs(error[0]) < 1e-6 and abs(error[1]) < 1e-6
    _, nadir = pointing_angles([12.0, -4.0, 15.0], 0.0, person, yaw)
    assert nadir == -90.0


def test_out_of_view_returns_none():
    assert image_error([0, 0, 20], 0.0, 0.0, 0.0, [-30.0, 0.0, 0.0]) is None


def test_schema_only_reports_locked_selection_as_track():
    target = target_message(1.0, fresh=True, locked=True, label='L2', err_x=.1, err_y=-.2,
                            dist_m=12.0, conf=.8, gimbal=dict(yaw=5, pitch=-40, roll=0,
                            yaw_rate=0, pitch_rate=0, roll_rate=0, age_s=.05))
    state = gimbal_state(target)
    assert state['verified'] and state['tracking'] and state['track_id'] == 'L2'
    assert state['pitch'] == -40.0
    assert tracks(target, 5)['tracks'][0]['track_id'] == 'L2'
    lost = target_message(1.0, fresh=False, locked=True, label='L2')
    assert not lost['locked'] and tracks(lost, 5)['tracks'] == []


class FakeController:
    preview_only = False


def test_release_resets_and_suppresses_then_selection_restarts():
    port = 19391
    link = MissionLink('127.0.0.1', 9, False, control_port=port)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(json.dumps(dict(cmd='release', suppress_s=0.3)).encode(), ('127.0.0.1', port))
        time.sleep(0.05)
        assert link.handle(FakeController(), object()) is True
        assert link.suppressed and link.releases == 1
        time.sleep(0.35)
        assert link.handle(FakeController(), None) is True  # fresh selection after suppression
        assert not link.suppressed
    finally:
        sender.close()
        link.close()


def test_angle_command_refused_while_person_locked():
    port = 19392
    link = MissionLink('127.0.0.1', 9, False, control_port=port)
    controller = FakeController()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(json.dumps(dict(cmd='set_attitude', pitch=-45)).encode(), ('127.0.0.1', port))
        time.sleep(0.05)
        link.handle(controller, object())
        assert not hasattr(controller, 'rate_pause_until')
        sender.sendto(json.dumps(dict(cmd='set_attitude', pitch=-45)).encode(), ('127.0.0.1', port))
        time.sleep(0.05)
        link.handle(controller, None)
        assert controller.rate_pause_until > time.monotonic()
    finally:
        sender.close()
        link.close()


class _Param:
    def __init__(self, value):
        self.value = value


def _bridge(published, rescue='person'):
    from types import SimpleNamespace
    from as2_a8_tracker.bridge_node import TrackerBridge
    node = TrackerBridge.__new__(TrackerBridge)
    params = dict(gimbal_yaw_sign=1.0, gimbal_pitch_sign=1.0, gimbal_max_age_s=0.5, rescue_classes=rescue,
                  aux_topics_hz=10.0)
    node.get_parameter = lambda name: _Param(params[name])
    node.last_aux, node.last_aux_lock = 0.0, None
    node.now = lambda: 10.0
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10 * 10**9))
    pub = lambda key: SimpleNamespace(publish=lambda msg: published.setdefault(key, json.loads(msg.data)))
    node.target_pub, node.gimbal_pub, node.tracks_pub, node.frame_pub = (pub(k) for k in ('target', 'gimbal', 'tracks', 'frame'))
    return node


def test_lock_on_a_non_rescue_class_is_not_a_lock_for_missions():
    packet = dict(locked_target=dict(id=10003, err_x=0.1, err_y=0.0, conf=0.8, label='dog'),
                  selected_label=10003, lock_state='LOCKED', model='objects80-n-640x384')
    published = {}
    _bridge(published).publish(packet)
    assert published['target']['locked'] is False
    assert published['target']['lock_state'] == 'LOCKED_OTHER'
    assert published['target']['target_class'] == 'dog'
    assert published['tracks']['tracks'] == []
    published = {}
    _bridge(published, rescue='person,dog').publish(packet)
    assert published['target']['locked'] is True and published['target']['label'] == '10003'


def test_person_lock_is_unchanged():
    published = {}
    _bridge(published).publish(dict(locked_target=dict(id=3, err_x=0.0, err_y=0.0, conf=0.9),
                                    selected_label=3, lock_state='LOCKED'))
    assert published['target']['locked'] is True and published['target']['target_class'] == 'person'


def test_side_topics_are_limited_but_frames_and_lock_changes_are_not():
    """2026-10-25: vision/frame every packet; target/gimbal/tracks at most 10 Hz, lock changes at once."""
    from types import SimpleNamespace
    counts = {}
    node = _bridge({})
    pub = lambda key: SimpleNamespace(publish=lambda msg: counts.__setitem__(key, counts.get(key, 0) + 1))
    node.target_pub, node.gimbal_pub, node.tracks_pub, node.frame_pub = (pub(k) for k in ('target', 'gimbal', 'tracks', 'frame'))
    clock = [10.0]
    node.now = lambda: clock[0]
    searching = dict(lock_state='SEARCHING')
    for i in range(25):                      # one second at 25 fps, no lock
        clock[0] = 10.0 + i * 0.04
        node.publish(searching)
    assert counts['frame'] == 25
    assert 9 <= counts['target'] <= 11 and counts['tracks'] == counts['target'] == counts['gimbal']
    before = counts['target']
    clock[0] += 0.01                          # a lock 10 ms later still goes out immediately
    node.publish(dict(locked_target=dict(id=7, err_x=0.0, err_y=0.0, conf=0.9, label='person'),
                      selected_label=7, lock_state='LOCKED'))
    assert counts['target'] == before + 1
