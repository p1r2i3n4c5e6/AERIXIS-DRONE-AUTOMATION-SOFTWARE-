"""Per-drone mission bundle emission.

A bundle is the unit the operator previews and the aircraft receives. The same
deterministic plan produces the same bundles, so a preview cannot disagree with
what is uploaded. Emission is pure: it writes files and touches no aircraft.
"""
import hashlib
import json
import os
from pathlib import Path

from .coverage import manifest

BUNDLE_SCHEMA = 1
MAX_BUNDLES = 64


def _checksum(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def _canonical(value):
    """Whole-number floats as ints: a browser's JSON round trip turns 15.0 into 15."""
    if isinstance(value, float) and value.is_integer() and abs(value) < 2 ** 53:
        return int(value)
    if isinstance(value, dict):
        return {key: _canonical(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_canonical(item) for item in value]
    return value


def body_checksum(body):
    """Checksum of a bundle body that survives JSON transport through JavaScript."""
    return _checksum(json.dumps(_canonical(body), sort_keys=True, allow_nan=False))


def waypoints_for(assignment, altitude_m, frame_id='earth'):
    """Convert a partition's lanes into coverage waypoints for one aircraft.

    Each lane contributes its start and end, which is what an AS2
    ``SurveyWaypoint`` sequence consumes. Lane order is preserved so the
    executed route matches the previewed route.
    """
    if not isinstance(assignment, dict) or 'lanes' not in assignment:
        raise ValueError('An assignment mapping with lanes is required')
    if not isinstance(altitude_m, (int, float)) or isinstance(altitude_m, bool):
        raise ValueError('Coverage altitude must be a number')
    if not 1.0 <= altitude_m <= 50.0:
        raise ValueError('Coverage altitude must be between 1 and 50 m')
    if not isinstance(frame_id, str) or not frame_id:
        raise ValueError('A reference frame is required')

    waypoints = []
    for index, lane in enumerate(assignment['lanes']):
        for endpoint, label in ((lane['start'], 'start'), (lane['end'], 'end')):
            waypoints.append({
                'segment_id': 'leg_%d_%s' % (index, label),
                'frame_id': frame_id,
                'pose': [round(float(endpoint[0]), 3), round(float(endpoint[1]), 3), float(altitude_m)],
            })
    if not waypoints:
        raise ValueError('Assignment produced no waypoints')
    return waypoints


def build_bundles(plan, mission_id, revision, altitude_m, drone_capabilities=None):
    """Build one bundle per assigned aircraft, plus a shared fleet manifest.

    ``drone_capabilities`` optionally carries per-aircraft payload and sensor
    facts so optional mission subtrees can be selected from real capability
    rather than from a template assumption.
    """
    if not isinstance(plan, dict) or 'assignments' not in plan:
        raise ValueError('A partition plan is required')
    capabilities = {item['drone_id']: item for item in (drone_capabilities or [])}

    fleet_manifest = manifest(plan, mission_id, revision)
    bundles = []
    for assignment in plan['assignments']:
        drone_id = assignment['drone_id']
        capability = capabilities.get(drone_id, {})
        waypoints = waypoints_for(assignment, altitude_m)
        body = {
            'schema_version': BUNDLE_SCHEMA,
            'mission_id': mission_id,
            'revision': revision,
            'drone_id': drone_id,
            'assignment': {
                'lane_count': assignment['lane_count'],
                'distance_m': assignment['distance_m'],
                'estimated_duration_s': assignment['estimated_duration_s'],
                'lane_range': assignment['lane_range'],
            },
            'coverage': {
                'frame_id': 'earth',
                'altitude_m': float(altitude_m),
                'altitude_verified': False,
                'waypoints': waypoints,
            },
            'capabilities': {
                'coverage': True,
                'detection': bool(capability.get('detection', False)),
                'payload': bool(capability.get('payload', False)),
            },
            'plan_sha256': fleet_manifest['plan_sha256'],
        }
        body['bundle_sha256'] = body_checksum(body)
        bundles.append(body)

    if len(bundles) > MAX_BUNDLES:
        raise ValueError('Fleet exceeds the %d-aircraft bundle limit' % MAX_BUNDLES)
    return {'schema_version': BUNDLE_SCHEMA, 'manifest': fleet_manifest, 'bundles': bundles}


def write_bundles(directory, result):
    """Write bundles atomically to a private directory, one file per aircraft.

    Existing files are never overwritten in place: the write is staged and then
    moved, so a reader never observes a partially written bundle.
    """
    if not isinstance(result, dict) or 'bundles' not in result:
        raise ValueError('A bundle result is required')
    target = Path(directory)
    target.mkdir(parents=True, exist_ok=True, mode=0o700)

    written = []
    manifest_path = target / 'fleet_manifest.json'
    _atomic_write(manifest_path, result['manifest'])
    written.append(str(manifest_path))
    for bundle in result['bundles']:
        path = target / ('%s.rev%d.json' % (bundle['drone_id'], bundle['revision']))
        _atomic_write(path, bundle)
        written.append(str(path))
    return written


def _atomic_write(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    with open(temporary, 'w', encoding='utf-8') as handle:
        os.chmod(temporary, 0o600)
        json.dump(value, handle, indent=2, allow_nan=False, sort_keys=True)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def verify_bundle(bundle):
    """Recompute a bundle's checksum. A mismatch means the file was altered."""
    if not isinstance(bundle, dict):
        raise ValueError('A bundle mapping is required')
    claimed = bundle.get('bundle_sha256')
    if not isinstance(claimed, str) or len(claimed) != 64:
        raise ValueError('Bundle has no valid checksum')
    body = {key: value for key, value in bundle.items() if key != 'bundle_sha256'}
    expected = body_checksum(body)
    if expected != claimed:
        raise ValueError('Bundle checksum mismatch; the file has been altered')
    return True
