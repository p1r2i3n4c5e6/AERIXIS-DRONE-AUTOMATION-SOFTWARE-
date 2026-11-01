"""One schema for real and simulated vision, so the rescue runtime cannot tell them apart.

Every publisher of person tracking (the A8 tracker bridge on the aircraft, the simulated
camera in SITL) builds its ROS messages here.
"""
import json

from std_msgs.msg import String


def target_message(stamp, *, fresh, locked, label=None, err_x=0.0, err_y=0.0, dist_m=None,
                   conf=0.0, lock_state='SEARCHING', suppressed=False, targets_count=0,
                   fps=0.0, gimbal=None, preview_only=False, source='a8_tracker', build='',
                   target_class=None, model=None, hfov_deg=None, vfov_deg=None):
    """Compact per-frame summary consumed by the rescue runtime and the GCS link. hfov/vfov: the
    camera's field of view, so the GCS can plan the search sweep for this camera (2026-11-01)."""
    return dict(schema_version=1, stamp=stamp, source=source, build=build, fresh=bool(fresh),
                locked=bool(locked and fresh), label=label if locked else None,
                err_x=float(err_x), err_y=float(err_y), dist_m=dist_m, conf=float(conf),
                lock_state=lock_state, suppressed=bool(suppressed),
                targets_count=int(targets_count), fps=float(fps), gimbal=gimbal,
                preview_only=bool(preview_only), target_class=target_class, model=model,
                hfov_deg=hfov_deg, vfov_deg=vfov_deg)


def gimbal_state(target, gimbal_max_age_s=0.5):
    """gimbal/state in the schema the runtime safety model and the track logger expect."""
    g = target.get('gimbal') or {}
    fresh = bool(g) and g.get('age_s', 99.0) <= gimbal_max_age_s
    value = dict(schema_version=1, stamp=target['stamp'], verified=fresh, fresh=fresh,
                 oscillating=False, tracking=bool(target['locked']),
                 track_id=target['label'], center_error=[target['err_x'], target['err_y']],
                 control_enabled=not target.get('preview_only', False),
                 feedback_age_s=g.get('age_s'), source=target.get('source'))
    for key in ('yaw', 'pitch', 'roll', 'yaw_rate', 'pitch_rate', 'roll_rate'):
        if key in g:
            value[key] = float(g[key])
    return value


def tracks(target, stamp_ns):
    """perception/tracks: only the locked selection is a confirmed, unpredicted track."""
    items = []
    if target['locked']:
        items.append(dict(track_id=target['label'], confirmed=True, predicted=False,
                          confidence=target['conf'], center_error=[target['err_x'], target['err_y']],
                          dist_m=target['dist_m'], label=target['label']))
    return dict(image_stamp_ns=stamp_ns, source=target.get('source'), tracks=items,
                detections=target['targets_count'])


def as_string(value):
    return String(data=json.dumps(value, allow_nan=False))


def frame_message(target, gimbal, tracks_value):
    """vision/frame: the three per-frame messages of one camera frame in one message, for nodes
    that need all of them (rescue runtime, track logger). Same content and rate as the separate
    topics, a third of the per-message cost in Python (Pi 5, 2026-10-23)."""
    return dict(schema_version=1, vision=target, gimbal=gimbal, tracks=tracks_value)
