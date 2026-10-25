"""Mission tree construction, owned by the planner package.

This lives in ``as2_rescue_planner`` rather than the runtime so the dependency
graph stays acyclic: the runtime and the mission manager both depend on the
planner (they validate route bundles and emit XML), but the planner must not
depend on either of them.

The trees describe control flow only. Every physical operation stays guarded by
the runtime's own interlocks; generating a tree never arms, moves or releases
anything.
"""
import xml.etree.ElementTree as ET

MODES = ('waypoints', 'surveillance', 'search_geotag', 'rescue_delivery',
         'waypoint_delivery', 'swarm_search', 'swarm_delivery')

# A8-tracker missions (gimbal-guided). Shown first in the GCS; human_geotag_resume is the
# default. Each one works for a single aircraft or a fleet partition from the same KML.
VISION_MODES = {
    'human_track_rtl': 'Mission 1 - Track & target a person; return to launch if lost',
    'human_orbit': 'Mission 2 - Orbit the detected person, then return to launch',
    'human_geotag_resume': 'Mission 3 - Geotag every person and resume the search',
    'human_geotag_drop_resume': 'Mission 4 - Geotag, drop payload over the person, resume',
    'swarm_coverage': 'Mission 5 - Swarm area coverage only',
}
VISION_DEFAULTS = dict(search_pitch_deg=-45.0, lost_timeout_s=3.0, track_duration_s=0.0,
                       orbit_radius_m=10.0, orbit_laps=2.0, orbit_speed=1.0,
                       approach_speed=1.0, rtl_speed=2.0, suppress_s=8.0, mesh_wait_s=30.0,
                       search_time_s=600.0, scan_rate_dps=12.0)


def node(kind, children=None, **ports):
    return dict(type=kind, ports={k: str(v) for k, v in ports.items()},
                children=children or [])


def mission(mode, points, swarm_id, target_id='delivery-zone-1', segment_ids=None,
            speed=.5, payload_confirmation='sensors'):
    if mode not in MODES:
        raise ValueError('Unknown mission template')
    if payload_confirmation not in ('sensors', 'operator'):
        raise ValueError('Unknown payload confirmation mode')
    # Operator-confirmed payloads have no physical release sensor, so the tree
    # reports the command outcome instead of claiming a verified release.
    outcome = 'ReportPayloadOutcome' if payload_confirmation == 'operator' else 'VerifyPayloadRelease'
    if not points:
        raise ValueError('At least one waypoint is required')
    if not swarm_id:
        raise ValueError('Common mission group ID is required')
    if segment_ids is not None and len(segment_ids) != len(points):
        raise ValueError('Segment IDs must match all waypoints')

    coverage = node('Sequence', name='CheckpointedCoverage', children=[
        node('SurveyWaypoint',
             segment_id=segment_ids[i] if segment_ids else 'leg_%d' % i,
             pose=';'.join(map(str, p)), speed=speed, timeout_s=120, swarm_id=swarm_id)
        for i, p in enumerate(points)])
    detection = mode in ('search_geotag', 'rescue_delivery', 'swarm_search', 'swarm_delivery')
    delivery = mode in ('rescue_delivery', 'swarm_delivery', 'waypoint_delivery')
    swarm = mode.startswith('swarm_')

    if swarm:
        coverage = node('SafetyGate', flag='SeparationSafe', children=[
            node('SafetyGate', flag='PeersFresh', children=[coverage])])

    if detection:
        response = [node('CheckpointCoverage'), node('PauseCoverage'), node('ConfirmHold'),
                    node('Timeout', msec=10000, children=[node('TrackPerson', timeout_s=9)]),
                    node('GeotagTarget')]
        if delivery:
            response += [
                node('AcquireAssignment', swarm_id=swarm_id),
                node('ApproachDeliveryZone', swarm_id=swarm_id, timeout_s=60),
                node('PauseCoverage'), node('ConfirmHold'), node('VerifyDeliveryZone'),
                node('Timeout', msec=8000, children=[
                    node('RetryUntilSuccessful', num_attempts=-1, children=[
                        node('Delay', delay_msec=100, children=[
                            node('StableFor', seconds=2, children=[node('SafeToRelease')])])])]),
                node('ReleasePayload', timeout_s=5), node(outcome),
                node('ReleaseAssignment', swarm_id=swarm_id)]
        response += [node('ReportObservation'), node('ResumeCoverage', timeout_s=60),
                     node('CompleteRescue')]
        incident = node('Sequence', children=[
            node('RescuePending'), node('ForceFailure', children=[node('Fallback', children=[
                node('Sequence', name='RescueResponse', children=response),
                node('Sequence', name='ObservationRecovery', children=[
                    node('PauseCoverage'), node('ConfirmHold'), node('ReportObservation'),
                    node('ResumeCoverage', timeout_s=60), node('CompleteRescue')])])])])
        coverage = node('ReactiveFallback', children=[incident, coverage])
    elif mode == 'waypoint_delivery':
        coverage = node('Sequence', children=[
            coverage, node('PauseCoverage'), node('ConfirmHold'),
            node('AcquireAssignment', target_id=target_id, swarm_id=swarm_id),
            node('VerifyDeliveryZone', target_id=target_id),
            node('Timeout', msec=8000, children=[
                node('RetryUntilSuccessful', num_attempts=-1, children=[
                    node('Delay', delay_msec=100, children=[
                        node('StableFor', seconds=2, children=[node('SafeToRelease')])])])]),
            node('ReleasePayload', target_id=target_id, timeout_s=5), node(outcome),
            node('ReleaseAssignment', target_id=target_id, swarm_id=swarm_id)])
    if mode == 'surveillance':
        coverage = node('Sequence', children=[coverage, node('ReportObservation')])

    flight = node('SafetyGate', flag='VehicleHealthy', children=[
        node('Sequence', name='FlightLifecycle', children=[
            node('Offboard', service_name='set_offboard_mode'),
            node('Arm', service_name='set_arming_state'),
            node('ManagedTakeoff', height=points[0][2], speed=.5),
            coverage,
            node('ManagedLand', speed=.5),
            node('Disarm', service_name='set_arming_state')])])
    ready = node('Timeout', msec=10000, children=[
        node('RetryUntilSuccessful', num_attempts=-1, children=[
            node('Delay', delay_msec=100, children=[node('VehicleHealthy')])])])
    return node('RecoverThenFail', children=[
        node('Sequence', children=[ready, flight]), node('RequestRecovery', timeout_s=15)])


def xml(tree, pretty=False):
    root = ET.Element('root', BTCPP_format='4', main_tree_to_execute='Mission')
    main = ET.SubElement(root, 'BehaviorTree', ID='Mission')

    def add(parent, entry):
        element = ET.SubElement(parent, entry['type'], entry.get('ports', {}))
        for child in entry.get('children', []):
            add(element, child)

    add(main, tree)
    # Compact on purpose: the tree travels over a low-rate telemetry radio and the route bundle
    # binds this exact text by checksum. Viewers lay it out themselves.
    if pretty:
        ET.indent(root)
    return ET.tostring(root, encoding='unicode')


def _coverage(points, swarm_id, segment_ids, speed):
    return node('Sequence', name='CheckpointedCoverage', children=[
        node('SurveyWaypoint',
             segment_id=segment_ids[i] if segment_ids else 'leg_%d' % i,
             pose=';'.join(map(str, p)), speed=speed, timeout_s=600, swarm_id=swarm_id)
        for i, p in enumerate(points)])


def _wait_for(flag, msec, period=200):
    return node('Timeout', name='WaitFor' + flag, msec=int(msec), children=[
        node('RetryUntilSuccessful', num_attempts=-1, children=[
            node('Delay', delay_msec=period, children=[node(flag)])])])


END_ACTIONS = ('RTL', 'LAND')


def _end_steps(action, rtl_speed):
    """How the flight ends: fly home and land (RTL) or land where the aircraft is (LAND)."""
    if action not in END_ACTIONS:
        raise ValueError('End action must be RTL or LAND')
    land = node('ManagedLand', speed=.5, timeout_s=120)
    if action == 'LAND':
        return [land]
    return [node('ReturnToLaunch', speed=rtl_speed, timeout_s=900), land]


def _recovery(action, rtl_speed):
    """Mandatory recovery branch of RecoverThenFail; the operator only picks RTL or LAND."""
    steps = _end_steps(action, rtl_speed)
    return node('Fallback', name='Recovery', children=[
        node('Sequence', name='ReturnAndLand' if action == 'RTL' else 'LandInPlace', children=steps),
        node('RequestRecovery', timeout_s=15)])


def _takeoff_steps(height):
    # A freshly started runner can lose the first service response during DDS discovery;
    # offboard and arm requests are idempotent, so retry them a few times.
    return [node('RetryUntilSuccessful', num_attempts=3, children=[
                node('Offboard', service_name='set_offboard_mode')]),
            node('RetryUntilSuccessful', num_attempts=3, children=[
                node('Arm', service_name='set_arming_state')]),
            node('ManagedTakeoff', height=height, speed=.5, timeout_s=90)]


def _search_area(area):
    return node('SetSearchArea', area=';'.join('%g,%g' % (round(x, 1), round(y, 1)) for x, y in area),
                timeout_s=5)


def _wrap_flight(steps, emergency, rtl_speed):
    """Mandatory safety shell: wait healthy -> SafetyGate(VehicleHealthy)[lifecycle], recovery."""
    flight = node('SafetyGate', flag='VehicleHealthy', children=[
        node('Sequence', name='FlightLifecycle', children=steps)])
    return node('RecoverThenFail', children=[
        node('Sequence', children=[_wait_for('VehicleHealthy', 10000, 100), flight]),
        _recovery(emergency, rtl_speed)])


def vision_mission(mode, points, swarm_id, segment_ids=None, speed=.5, fleet_size=1,
                   area=None, stations=None, finish='RTL', emergency='RTL', **options):
    """Behaviour trees for the five A8-tracker missions.

    Structure shared by all of them:
      RecoverThenFail
        Sequence: wait VehicleHealthy -> SafetyGate(VehicleHealthy)[ lifecycle ]
        recovery: Fallback[ ReturnToLaunch -> ManagedLand , RequestRecovery (hold) ]
      lifecycle: Offboard, Arm, ManagedTakeoff, [mesh check], [gimbal search pose,
                 vision check], <mission body>, ReturnToLaunch, ManagedLand, Disarm
    """
    if mode not in VISION_MODES:
        raise ValueError('Unknown mission template')
    if not points:
        raise ValueError('At least one waypoint is required')
    if not swarm_id:
        raise ValueError('Common mission group ID is required')
    if segment_ids is not None and len(segment_ids) != len(points):
        raise ValueError('Segment IDs must match all waypoints')
    o = dict(VISION_DEFAULTS)
    o.update({k: v for k, v in options.items() if v is not None})
    swarm = fleet_size > 1
    coverage = _coverage(points, swarm_id, segment_ids, speed)
    if swarm:
        coverage = node('SafetyGate', flag='SeparationSafe', children=[
            node('SafetyGate', flag='PeersFresh', children=[coverage])])
    search_pose = node('ForceSuccess', children=[
        node('SetGimbalSearch', pitch_deg=o['search_pitch_deg'], timeout_s=6)])
    release = node('ReleaseTargetLock', suppress_s=o['suppress_s'], timeout_s=5)

    if mode == 'human_orbit':
        # Watch, not survey: fly to this aircraft's station inside the area, hold and sweep
        # yaw for search_time_s. A person inside the area pre-empts the watch. ScanArea knows
        # the planned station (pose) and, in a fleet, the stations it may move the watch to
        # when a peer's orbit covers it (area; see watch.py).
        scan = dict(duration_s=o['search_time_s'], yaw_rate_dps=o['scan_rate_dps'],
                    timeout_s=min(1800, int(o['search_time_s']) + 60),
                    pose=';'.join(map(str, points[-1])), speed=speed,
                    segment_id=segment_ids[-1] if segment_ids else 'leg_%d' % (len(points) - 1))
        if swarm and stations:
            scan['area'] = stations
        coverage = node('Sequence', name='WatchArea', children=[coverage, node('ScanArea', **scan)])

    if mode == 'human_track_rtl':
        body = node('ReactiveFallback', name='SearchThenTrack', children=[
            node('Sequence', name='TrackPerson', children=[
                node('HumanDetected'), node('PauseCoverage', timeout_s=15),
                node('ForceSuccess', children=[
                    node('TrackTarget', lost_timeout_s=o['lost_timeout_s'],
                         duration_s=o['track_duration_s'], timeout_s=1800)])]),
            coverage])
    elif mode == 'human_orbit':
        body = node('ReactiveFallback', name='SearchThenOrbit', children=[
            node('Sequence', name='OrbitPerson', children=[
                node('HumanDetected'), node('PauseCoverage', timeout_s=15),
                node('ForceSuccess', children=[node('Sequence', children=[
                    node('OrbitTarget', radius_m=o['orbit_radius_m'], laps=o['orbit_laps'],
                         speed=o['orbit_speed'], lost_timeout_s=o['lost_timeout_s'], timeout_s=1200),
                    release])])]),
            coverage])
    elif mode in ('human_geotag_resume', 'human_geotag_drop_resume'):
        response = [node('CheckpointCoverage'), node('PauseCoverage', timeout_s=15),
                    node('ApproachOverhead', speed=o['approach_speed'],
                         lost_timeout_s=o['lost_timeout_s'], timeout_s=240),
                    node('GeotagOverhead', timeout_s=5)]
        if mode == 'human_geotag_drop_resume':
            response.append(node('ForceSuccess', name='DropIfLoaded', children=[
                node('Sequence', children=[
                    node('PayloadAvailable'),
                    node('Timeout', msec=10000, children=[
                        node('RetryUntilSuccessful', num_attempts=-1, children=[
                            node('Delay', delay_msec=100, children=[
                                node('StableFor', seconds=1, children=[node('OverheadTarget')])])])]),
                    node('DropPayload', timeout_s=5),
                    node('ReportPayloadOutcome', timeout_s=5)])]))
        response += [release, search_pose, node('ResumeCoverage', timeout_s=300),
                     node('CompleteRescue')]
        incident = node('Sequence', name='PersonIncident', children=[
            node('HumanDetected'), node('ForceFailure', children=[node('Fallback', children=[
                node('Sequence', name='GeotagResponse', children=response),
                node('Sequence', name='LostPersonRecovery', children=[
                    node('PauseCoverage', timeout_s=15), release, search_pose,
                    node('ReportObservation'), node('ResumeCoverage', timeout_s=300),
                    node('CompleteRescue')])])])])
        body = node('ReactiveFallback', name='SearchAndGeotag', children=[incident, coverage])
    else:  # swarm_coverage
        body = coverage

    steps = _takeoff_steps(points[0][2])
    if swarm:
        steps.append(_wait_for('PeersFresh', o['mesh_wait_s'] * 1000))
    if mode != 'swarm_coverage':
        steps += [search_pose, _wait_for('VisionHealthy', 10000)]
        if area:
            # Only people standing inside the KML area count as detections.
            steps.append(_search_area(area))
    steps += [body, *_end_steps(finish, o['rtl_speed']),
              node('Disarm', service_name='set_arming_state')]
    return _wrap_flight(steps, emergency, o['rtl_speed'])

