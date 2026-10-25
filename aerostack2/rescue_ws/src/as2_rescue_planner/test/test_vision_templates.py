import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from as2_rescue_planner.mission import DEFAULT_TEMPLATE, plan_mission
from as2_rescue_planner.trees import VISION_MODES, vision_mission, xml

KML = Path(__file__).resolve().parents[4] / 'sample_field_mission.kml'
POINTS = [[0, 0, 15], [20, 0, 15], [20, 10, 15]]


def tags(mode, **kw):
    root = ET.fromstring(xml(vision_mission(mode, POINTS, 'group', **kw)))
    return [e.tag for e in root.iter()]


def test_every_mission_returns_home_and_lands_with_recovery():
    for mode in VISION_MODES:
        found = tags(mode)
        assert found.count('ReturnToLaunch') == 2, mode  # normal end + recovery branch
        assert 'RecoverThenFail' in found and 'ManagedLand' in found and 'Disarm' in found
        assert found.count('SurveyWaypoint') == len(POINTS)


def test_mission_specific_nodes():
    assert 'TrackTarget' in tags('human_track_rtl') and 'ApproachOverhead' not in tags('human_track_rtl')
    assert 'OrbitTarget' in tags('human_orbit')
    m3 = tags('human_geotag_resume')
    assert {'ApproachOverhead', 'GeotagOverhead', 'ReleaseTargetLock', 'ResumeCoverage'} <= set(m3)
    assert 'DropPayload' not in m3
    m4 = tags('human_geotag_drop_resume')
    assert {'DropPayload', 'PayloadAvailable', 'OverheadTarget'} <= set(m4)
    m5 = tags('swarm_coverage')
    assert 'HumanDetected' not in m5 and 'SetGimbalSearch' not in m5


def test_fleet_adds_mesh_check_and_separation_gates():
    single, fleet = tags('swarm_coverage'), tags('swarm_coverage', fleet_size=3)
    assert 'PeersFresh' not in single
    assert 'PeersFresh' in fleet and fleet.count('SafetyGate') == 3


def test_options_reach_ports():
    root = ET.fromstring(xml(vision_mission('human_orbit', POINTS, 'g', orbit_radius_m=15, orbit_laps=3)))
    orbit = next(root.iter('OrbitTarget'))
    assert orbit.get('radius_m') == '15' and orbit.get('laps') == '3'


@pytest.mark.skipif(not KML.exists(), reason='sample KML not present')
def test_plan_defaults_to_mission_three_and_generates_per_drone_trees():
    fleet = [dict(drone_id='drone0', endurance_s=1200, reserve_fraction=.25),
             dict(drone_id='drone1', endurance_s=1200, reserve_fraction=.25)]
    result = plan_mission(dict(kml=KML.read_text(), fleet=fleet, mission_id=uuid.uuid4().hex,
                               altitude_m=15, speed_mps=1.5, spacing_m=15))
    assert result['template'] == DEFAULT_TEMPLATE == 'human_geotag_resume'
    assert set(result['xml']) == {'drone0', 'drone1'}
    assert all('ApproachOverhead' in x for x in result['xml'].values())


def test_entry_transit_from_outside_launch_point():
    from as2_rescue_planner.geometry import PlannerRegion
    shell = [[0, 0], [20, 0], [20, 20], [0, 20]]
    region = PlannerRegion(shell)
    assert region.contains_entry_segment([-10, 5], [5, 5])          # straight in
    assert not region.contains_entry_segment([5, 5], [6, 5])        # starts inside
    assert not region.contains_entry_segment([-10, 5], [30, 5])     # ends outside
    blocked = PlannerRegion(shell, exclusions=[[[3, 3], [7, 3], [7, 7], [3, 7]]])
    assert not blocked.contains_entry_segment([-10, 5], [10, 5])    # crosses a no-fly zone
    assert blocked.contains_entry_segment([-10, 15], [10, 15])


@pytest.mark.skipif(not KML.exists(), reason='sample KML not present')
def test_bundle_checksum_survives_browser_json_round_trip():
    import json
    from as2_rescue_planner.bundle import verify_bundle
    result = plan_mission(dict(kml=KML.read_text(), fleet=[dict(drone_id='drone0', endurance_s=1200, reserve_fraction=.25)],
                               mission_id=uuid.uuid4().hex, altitude_m=12, speed_mps=1.5, spacing_m=8))
    bundle = result['bundles'][0]
    # JavaScript JSON.stringify writes 12.0 as 12; emulate that round trip.
    text = json.dumps(bundle).replace('.0,', ',').replace('.0]', ']').replace('.0}', '}')
    assert verify_bundle(json.loads(text))


def test_orbit_mission_watches_one_station_per_drone_without_survey_lanes():
    from as2_rescue_planner.mission import validate_route_bundle
    fleet = [dict(drone_id='drone0', endurance_s=1200), dict(drone_id='drone1', endurance_s=1200)]
    result = plan_mission(dict(kml=KML.read_text(), fleet=fleet, mission_id=uuid.uuid4().hex, template='human_orbit',
                               altitude_m=15, speed_mps=1.5, search_time_s=300, orbit_radius_m=12))
    assert result['complete'], result['reason']
    for bundle in result['bundles']:
        assert len(bundle['coverage']['waypoints']) == 1 and bundle['assignment']['pattern'] == 'station'
        text = result['xml'][bundle['drone_id']]
        validate_route_bundle(bundle, bundle['drone_id'], text)
        root = ET.fromstring(text)
        assert len(list(root.iter('SurveyWaypoint'))) == 1
        assert root.find('.//ScanArea').get('duration_s') == '300.0'
        assert root.find('.//OrbitTarget').get('radius_m') == '12.0'
        assert root.find('.//SetSearchArea') is not None
    assert set(result['stations']) == {'drone0', 'drone1'}


def test_generated_xml_is_compact_for_the_radio():
    text = xml(vision_mission('human_geotag_resume', POINTS, 'group'))
    assert '\n' not in text and '>  <' not in text


def _orbit_plan(fleet=('drone0', 'drone1')):
    return plan_mission(dict(kml=KML.read_text(), fleet=[dict(drone_id=d, endurance_s=1200) for d in fleet],
                             mission_id=uuid.uuid4().hex, template='human_orbit', altitude_m=12))


def test_orbit_fleet_tree_lists_its_watch_stations_and_route_node_goals():
    from as2_rescue_planner.watch import parse, watch_goals
    result = _orbit_plan()
    for bundle in result['bundles']:
        own = bundle['drone_id']
        scan = ET.fromstring(result['xml'][own]).find('.//ScanArea')
        station = bundle['coverage']['waypoints'][0]
        assert scan.get('pose') == ';'.join(map(str, station['pose'])) and scan.get('segment_id') == own + '_station'
        stations = parse(scan.get('area'))
        fleet = {s['owner']: s['xy'] for s in stations if s['owner']}
        other = 'drone1' if own == 'drone0' else 'drone0'
        assert list(fleet) == [other]
        assert fleet[other] == [round(v, 1) for v in result['stations'][other][:2]]
        goals = watch_goals(result['xml'][own])
        assert goals[0] == station['pose'] and all(g[2] == station['pose'][2] for g in goals)
        assert len(goals) == 1 + len(stations)


def test_alternates_stay_inside_the_area_and_in_the_own_share():
    import math
    from as2_rescue_planner import coverage
    from as2_rescue_planner.geometry import PlannerRegion
    from as2_rescue_planner.watch import alternates
    big = PlannerRegion([[-50, -40], [50, -40], [50, 40], [-50, 40]])  # the 100 x 80 m SITL field
    plan = coverage.partition_region(big, [dict(drone_id='a', endurance_s=1200), dict(drone_id='b', endurance_s=1200)], 10, 0, 1.0, 10)
    for assignment in plan['assignments']:
        lanes = assignment['lanes']
        waypoints = [dict(pose=list(p) + [12.0]) for lane in lanes for p in (lane['start'], lane['end'])]
        ys = [p['pose'][1] for p in waypoints]
        station = [0.0, sum(ys) / len(ys)]
        points = alternates(big, waypoints, station)
        assert 3 <= len(points) <= 8
        assert all(big.contains_circle(p, 5.0) and min(ys) <= p[1] <= max(ys) for p in points)
        assert all(math.dist(p, station) >= 8.0 for p in points)
        assert len({tuple(p) for p in points}) == len(points)


def test_watch_station_list_round_trip_and_rejects_bad_text():
    from as2_rescue_planner.watch import encode, parse
    text = encode([[1.04, -2.0], [35.0, 25.0]], {'drone1': [0.0, 20.036, 12.0], 'drone0': [0.0, -19.96]})
    assert text == '1,-2;35,25;drone0=0,-20;drone1=0,20'
    assert parse(text)[2] == dict(owner='drone0', xy=[0.0, -20.0])
    for bad in ('1,2,3', 'x,1', 'drone 0=1,2', 'nan,1', '1e9,0'):
        with pytest.raises(ValueError):
            parse(bad)
    with pytest.raises(ValueError):
        encode([], {'bad id': [0, 0]})


def test_route_bundle_refuses_a_tampered_watch_station():
    import hashlib
    from as2_rescue_planner.bundle import body_checksum
    from as2_rescue_planner.mission import validate_route_bundle
    result = _orbit_plan()
    bundle = result['bundles'][1]
    text = result['xml']['drone1']
    scan = ET.fromstring(text).find('.//ScanArea')
    area, pose = scan.get('area'), scan.get('pose')
    moved = ';'.join(['1.0'] + pose.split(';')[1:])
    for old, new in (('area="%s"' % area, 'area="%s;500,500"' % area), ('pose="%s"' % pose, 'pose="%s"' % moved)):
        tampered = text.replace(old, new)
        assert tampered != text
        copy = dict(bundle, xml_sha256=hashlib.sha256(tampered.encode()).hexdigest())
        copy['bundle_sha256'] = body_checksum({k: v for k, v in copy.items() if k != 'bundle_sha256'})
        with pytest.raises(ValueError):
            validate_route_bundle(copy, 'drone1', tampered)
