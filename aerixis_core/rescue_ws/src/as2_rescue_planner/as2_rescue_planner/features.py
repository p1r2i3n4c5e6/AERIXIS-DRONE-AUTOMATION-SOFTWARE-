"""Custom missions from simple operator choices (no behaviour-tree nodes shown to the operator).

The GCS mission builder sends a *feature spec*: what the aircraft carries (camera, payload), the
mission type (coverage lanes, watch stations, track) and the features wanted when a person is
found (geotag, orbit, payload drop, resume searching) plus how the flight ends (RTL or LAND) and
what the recovery branch does on an emergency (RTL or LAND).

``custom_mission`` composes the tree from that spec, always inside the same mandatory safety shell
as the built-in missions (``trees.vision_mission``): wait VehicleHealthy, SafetyGate(VehicleHealthy)
around the whole lifecycle, PeersFresh wait plus SeparationSafe/PeersFresh gates around the search
in a fleet, RecoverThenFail with a return/land recovery, and an explicit return/land and disarm.
The operator cannot switch any of these off. Only node types the built-in missions already use
are emitted, so every tree passes the onboard registry validation without new C++ nodes.

``BUILTIN_FEATURES`` expresses the five built-in missions as specs; for those specs
``custom_mission`` reproduces the built-in XML exactly (tested), so "edit a copy of mission N" in
the GCS starts from the real mission.
"""
from .trees import (_sweep_ports, VISION_DEFAULTS, VISION_MODES, END_ACTIONS, _coverage, _end_steps, _search_area,
                    _takeoff_steps, _wait_for, _wrap_flight, node)

MISSION_TYPES = ('coverage', 'watch', 'track')
SPEC_DEFAULTS = dict(camera=True, payload=False, type='coverage', detect=True, geotag=True,
                     orbit=False, drop=False, resume=True, finish='RTL', emergency='RTL')

BUILTIN_FEATURES = {
    'human_track_rtl': dict(camera=True, payload=False, type='track', detect=True, geotag=False,
                            orbit=False, drop=False, resume=False, finish='RTL', emergency='RTL'),
    'human_orbit': dict(camera=True, payload=False, type='watch', detect=True, geotag=False,
                        orbit=True, drop=False, resume=False, finish='RTL', emergency='RTL'),
    'human_geotag_resume': dict(camera=True, payload=False, type='coverage', detect=True, geotag=True,
                                orbit=False, drop=False, resume=True, finish='RTL', emergency='RTL'),
    'human_geotag_drop_resume': dict(camera=True, payload=True, type='coverage', detect=True, geotag=True,
                                     orbit=False, drop=True, resume=True, finish='RTL', emergency='RTL'),
    'swarm_coverage': dict(camera=False, payload=False, type='coverage', detect=False, geotag=False,
                           orbit=False, drop=False, resume=True, finish='RTL', emergency='RTL'),
}
assert set(BUILTIN_FEATURES) == set(VISION_MODES)


def feature_spec(spec):
    """Validate and complete a feature spec. Raises ValueError with an operator-readable reason."""
    if not isinstance(spec, dict):
        raise ValueError('Mission features must be an object')
    unknown = set(spec) - set(SPEC_DEFAULTS)
    if unknown:
        raise ValueError('Unknown mission feature: ' + ', '.join(sorted(unknown)))
    s = dict(SPEC_DEFAULTS, **spec)
    for key, default in SPEC_DEFAULTS.items():
        if isinstance(default, bool) and not isinstance(s[key], bool):
            raise ValueError('Feature %s must be true or false' % key)
    if s['type'] not in MISSION_TYPES:
        raise ValueError('Mission type must be coverage, watch or track')
    for key in ('finish', 'emergency'):
        if s[key] not in END_ACTIONS:
            raise ValueError('%s action must be RTL or LAND' % key.capitalize())
    if s['detect'] and not s['camera']:
        raise ValueError('Human detection needs the camera')
    if s['type'] in ('watch', 'track') and not s['detect']:
        raise ValueError('A %s mission needs the camera and human detection' % s['type'])
    for key in ('geotag', 'orbit', 'drop'):
        if s[key] and not s['detect']:
            raise ValueError('%s needs human detection' % key.capitalize())
    if s['type'] == 'track' and (s['geotag'] or s['orbit'] or s['drop']):
        raise ValueError('Track follows the person; geotag, orbit and drop are not combined with it')
    if s['drop'] and not s['payload']:
        raise ValueError('Payload drop needs a payload servo on the aircraft')
    if s['drop'] and not s['geotag']:
        raise ValueError('Payload drop happens overhead after the geotag; enable geotag')
    if s['detect'] and s['type'] != 'track' and not (s['geotag'] or s['orbit']):
        raise ValueError('Choose what to do when a person is found: geotag, orbit or both')
    return s


def requirements(spec):
    """Pre-flight items this mission needs onboard (the GCS readiness page uses these keys)."""
    s = feature_spec(spec)
    need = []
    if s['detect']:
        need += ['camera', 'gimbal']
    if s['geotag']:
        need.append('gnss')
    if s['drop']:
        need += ['payload', 'overhead_envelope']
    return need


def describe(spec, fleet_size=1):
    """Plain sentences for the GCS Conditions card: what the aircraft will do."""
    s = feature_spec(spec)
    out = []
    if s['type'] == 'watch':
        out.append('Flies to one watch station inside its share of the area, holds it and sweeps yaw.')
    else:
        out.append('Flies survey lanes over its share of the KML area.')
    if not s['detect']:
        out.append('No detection: the camera is not used.')
    else:
        out.append('Only a person standing inside the KML area counts as a detection.')
        act = []
        if s['type'] == 'track':
            act.append('holds position and tracks the person')
        if s['geotag']:
            act.append('flies overhead until the gimbal looks straight down and records the position')
        if s['drop']:
            act.append('releases the payload once if loaded, steady and inside the drop height window')
        if s['orbit']:
            act.append('circles the person at the set radius for the set laps')
        out.append('When a person is found it ' + ', then '.join(act) + '.')
        out.append('Then it releases the person and continues searching.' if s['resume']
                   else 'After the first person the mission ends.')
    out.append('At the end it %s.' % ('returns to launch and lands' if s['finish'] == 'RTL'
                                      else 'lands where it is'))
    out.append('On an onboard emergency (unhealthy vehicle, low battery, abort) it %s.'
               % ('returns to launch and lands' if s['emergency'] == 'RTL' else 'lands immediately where it is'))
    if fleet_size > 1:
        out.append('Fleet: waits for the mesh, and the search stops if another aircraft comes too close.')
    return out


def custom_mission(spec, points, swarm_id, segment_ids=None, speed=.5, fleet_size=1,
                   area=None, stations=None, **options):
    """Compose a mission tree from a feature spec (see the module docstring)."""
    s = feature_spec(spec)
    if not points:
        raise ValueError('At least one waypoint is required')
    if not swarm_id:
        raise ValueError('Common mission group ID is required')
    if segment_ids is not None and len(segment_ids) != len(points):
        raise ValueError('Segment IDs must match all waypoints')
    o = dict(VISION_DEFAULTS)
    o.update({k: v for k, v in options.items() if v is not None})
    swarm = fleet_size > 1
    watch = s['type'] == 'watch'

    coverage = _coverage(points, swarm_id, segment_ids, speed)
    if swarm:  # mandatory in a fleet
        coverage = node('SafetyGate', flag='SeparationSafe', children=[
            node('SafetyGate', flag='PeersFresh', children=[coverage])])
    if watch:
        scan = dict(duration_s=o['search_time_s'], yaw_rate_dps=o['scan_rate_dps'],
                    timeout_s=min(1800, int(o['search_time_s']) + 60),
                    pose=';'.join(map(str, points[-1])), speed=speed,
                    segment_id=segment_ids[-1] if segment_ids else 'leg_%d' % (len(points) - 1))
        if swarm and stations:
            scan['area'] = stations
        coverage = node('Sequence', name='WatchArea', children=[coverage, node('ScanArea', **scan)])

    body = coverage
    if s['detect']:
        body = _person_branch(s, o, coverage)

    steps = _takeoff_steps(points[0][2])
    if swarm:
        steps.append(_wait_for('PeersFresh', o['mesh_wait_s'] * 1000))
    if s['detect']:
        steps += [_search_pose(o), _wait_for('VisionHealthy', 10000)]
        if area:
            steps.append(_search_area(area))
    steps += [body, *_end_steps(s['finish'], o['rtl_speed']),
              node('Disarm', service_name='set_arming_state')]
    return _wrap_flight(steps, s['emergency'], o['rtl_speed'])


def _search_pose(o):
    return node('ForceSuccess', children=[
        node('SetGimbalSearch', timeout_s=6, **_sweep_ports(o))])


def _release(o):
    return node('ReleaseTargetLock', suppress_s=o['suppress_s'], timeout_s=5)


def _person_actions(s, o):
    actions = []
    if s['type'] == 'track':
        actions.append(node('TrackTarget', lost_timeout_s=o['lost_timeout_s'],
                            duration_s=o['track_duration_s'], timeout_s=1800))
    if s['geotag']:
        actions += [node('ApproachOverhead', speed=o['approach_speed'],
                         lost_timeout_s=o['lost_timeout_s'], timeout_s=240),
                    node('GeotagOverhead', timeout_s=5)]
    if s['drop']:
        # Skipped (not failed) when no loaded payload is confirmed; never repeated automatically.
        actions.append(node('ForceSuccess', name='DropIfLoaded', children=[
            node('Sequence', children=[
                node('PayloadAvailable'),
                node('Timeout', msec=10000, children=[
                    node('RetryUntilSuccessful', num_attempts=-1, children=[
                        node('Delay', delay_msec=100, children=[
                            node('StableFor', seconds=1, children=[node('OverheadTarget')])])])]),
                node('DropPayload', timeout_s=5),
                node('ReportPayloadOutcome', timeout_s=5)])]))
    if s['orbit']:
        actions.append(node('OrbitTarget', radius_m=o['orbit_radius_m'], laps=o['orbit_laps'],
                            speed=o['orbit_speed'], lost_timeout_s=o['lost_timeout_s'], timeout_s=1200))
    return actions


def _person_branch(s, o, coverage):
    """ReactiveFallback[person incident, search]: a detection inside the area pre-empts the search."""
    actions = _person_actions(s, o)
    word = 'Track' if s['type'] == 'track' else 'Orbit' if s['orbit'] and not s['geotag'] else 'Respond'
    if not s['resume']:
        # One person, then the mission ends (return/land follows the body).
        if s['type'] != 'track':
            actions.append(_release(o))
        inner = actions[0] if len(actions) == 1 else node('Sequence', children=actions)
        return node('ReactiveFallback', name='SearchThen' + word, children=[
            node('Sequence', name=word + 'Person', children=[
                node('HumanDetected'), node('PauseCoverage', timeout_s=15),
                node('ForceSuccess', children=[inner])]),
            coverage])
    # Resume: handle the person, release the lock (it is then excluded), look for the next one.
    # Lanes resume from the checkpoint; a watch goes back to its station when ScanArea restarts.
    lanes = s['type'] != 'watch'
    head = [node('CheckpointCoverage')] if lanes else []
    tail = [node('ResumeCoverage', timeout_s=300)] if lanes else []
    response = head + [node('PauseCoverage', timeout_s=15)] + actions + [
        _release(o), _search_pose(o)] + tail + [node('CompleteRescue')]
    recovery = [node('PauseCoverage', timeout_s=15), _release(o), _search_pose(o),
                node('ReportObservation')] + tail + [node('CompleteRescue')]
    geotag = s['geotag'] and s['type'] != 'track'
    incident = node('Sequence', name='PersonIncident', children=[
        node('HumanDetected'), node('ForceFailure', children=[node('Fallback', children=[
            node('Sequence', name='GeotagResponse' if geotag else 'PersonResponse', children=response),
            node('Sequence', name='LostPersonRecovery', children=recovery)])])])
    return node('ReactiveFallback', name='SearchAndGeotag' if geotag else 'SearchAnd' + word,
                children=[incident, coverage])
