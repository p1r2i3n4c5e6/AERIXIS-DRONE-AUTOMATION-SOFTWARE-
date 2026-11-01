"""Custom missions composed from GCS feature specs (features.py)."""
import itertools
import os
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from as2_rescue_planner.features import (BUILTIN_FEATURES, SPEC_DEFAULTS, custom_mission, describe,
                                         feature_spec, requirements)
from as2_rescue_planner.mission import plan_mission, validate_route_bundle
from as2_rescue_planner.trees import vision_mission, xml

KML = Path(__file__).resolve().parents[4] / 'sample_field_mission.kml'
POINTS = [[0, 0, 15], [20, 0, 15], [20, 10, 15]]
AREA = [[0, 0], [30, 0], [30, 30]]
CAMERA_NODES = {'SetGimbalSearch', 'VisionHealthy', 'SetSearchArea', 'HumanDetected', 'ScanArea',
                'TrackTarget', 'OrbitTarget', 'ApproachOverhead', 'GeotagOverhead', 'ReleaseTargetLock'}
RUNNER = Path(os.environ.get('AS2_TEST_RUNNER', '/home/prince/aerostack2/install/as2_behavior_tree/lib/as2_behavior_tree/as2_behavior_tree_node'))


def valid_specs():
    """Every combination the builder can send that the rules accept."""
    keys = ('camera', 'payload', 'detect', 'geotag', 'orbit', 'drop', 'resume')
    for values in itertools.product((False, True), repeat=len(keys)):
        for kind in ('coverage', 'watch', 'track'):
            for finish, emergency in (('RTL', 'RTL'), ('LAND', 'LAND'), ('RTL', 'LAND')):
                spec = dict(zip(keys, values), type=kind, finish=finish, emergency=emergency)
                try:
                    yield feature_spec(spec)
                except ValueError:
                    pass


def root_of(spec, **kw):
    return ET.fromstring(xml(custom_mission(spec, POINTS, 'group', area=AREA, **kw)))


def test_builtin_specs_reproduce_the_builtin_missions_exactly():
    for mode, spec in BUILTIN_FEATURES.items():
        for fleet in (1, 3):
            kw = dict(fleet_size=fleet, area=AREA, stations='1,2;drone1=3,4' if fleet > 1 else None,
                      orbit_radius_m=12, search_time_s=300)
            assert xml(custom_mission(spec, POINTS, 'g', **kw)) == xml(vision_mission(mode, POINTS, 'g', **kw)), (mode, fleet)


def test_rules_reject_impossible_combinations():
    bad = [dict(camera=False, detect=True), dict(type='watch', detect=False, geotag=False),
           dict(type='track', geotag=True), dict(drop=True, payload=False), dict(drop=True, payload=True, geotag=False, orbit=True),
           dict(geotag=False, orbit=False), dict(camera=False, detect=False, geotag=True), dict(finish='HOVER'),
           dict(emergency='rtl'), dict(type='survey'), dict(camera='yes'), dict(nodes=['Arm'])]
    for spec in bad:
        with pytest.raises(ValueError):
            feature_spec(spec)
    with pytest.raises(ValueError):
        feature_spec(['camera'])
    assert feature_spec({}) == SPEC_DEFAULTS


def test_without_camera_no_camera_nodes_are_built():
    spec = feature_spec(dict(camera=False, detect=False, geotag=False))
    tags = {e.tag for e in root_of(spec).iter()}
    assert not tags & CAMERA_NODES
    assert requirements(spec) == []
    assert 'SurveyWaypoint' in tags
    # Camera carried but detection off: still no camera nodes.
    assert not {e.tag for e in root_of(feature_spec(dict(detect=False, geotag=False))).iter()} & CAMERA_NODES


def test_mandatory_safety_is_always_present():
    count = 0
    for spec in valid_specs():
        for fleet in (1, 2):
            count += 1
            root = root_of(spec, fleet_size=fleet, stations='1,2;drone1=3,4' if fleet > 1 else None)
            top = root.find('BehaviorTree')[0]
            assert top.tag == 'RecoverThenFail' and len(top) == 2
            task, recovery = top
            assert task[0].get('name') == 'WaitForVehicleHealthy'
            assert task[1].tag == 'SafetyGate' and task[1].get('flag') == 'VehicleHealthy'
            life = list(task[1][0])
            assert [life[-2].tag, life[-1].tag] == ['ManagedLand', 'Disarm']
            assert (life[-3].tag == 'ReturnToLaunch') == (spec['finish'] == 'RTL')
            assert recovery.tag == 'Fallback' and recovery[-1].tag == 'RequestRecovery'
            assert [e.tag for e in recovery[0]] == (['ReturnToLaunch', 'ManagedLand'] if spec['emergency'] == 'RTL' else ['ManagedLand'])
            flags = [e.get('flag') for e in root.iter('SafetyGate')]
            if fleet > 1:
                assert root.find(".//*[@name='WaitForPeersFresh']") is not None
                assert {'SeparationSafe', 'PeersFresh'} <= set(flags)
                gate = next(e for e in root.iter('SafetyGate') if e.get('flag') == 'SeparationSafe')
                assert len(list(gate.iter('SurveyWaypoint'))) == len(POINTS)
            else:
                assert flags == ['VehicleHealthy']
            tags = {e.tag for e in root.iter()}
            assert ('DropPayload' in tags) == spec['drop'] and ('OrbitTarget' in tags) == spec['orbit']
            assert ('GeotagOverhead' in tags) == spec['geotag'] and ('ScanArea' in tags) == (spec['type'] == 'watch')
            assert bool(tags & CAMERA_NODES) == spec['detect']
            assert ('ResumeCoverage' in tags) == (spec['detect'] and spec['resume'] and spec['type'] != 'watch')
    assert count > 40


def test_geotag_orbit_drop_and_resume_order():
    spec = feature_spec(dict(payload=True, geotag=True, drop=True, orbit=True, resume=True))
    response = root_of(spec).find(".//*[@name='GeotagResponse']")
    assert [e.tag for e in response] == ['CheckpointCoverage', 'PauseCoverage', 'ApproachOverhead', 'GeotagOverhead',
                                         'ForceSuccess', 'OrbitTarget', 'ReleaseTargetLock', 'ForceSuccess',
                                         'ResumeCoverage', 'CompleteRescue']
    watch = root_of(feature_spec(dict(type='watch', geotag=False, orbit=True, resume=True)))
    response = watch.find(".//*[@name='PersonResponse']")
    assert [e.tag for e in response] == ['PauseCoverage', 'OrbitTarget', 'ReleaseTargetLock', 'ForceSuccess', 'CompleteRescue']


def test_describe_and_requirements_follow_the_features():
    spec = dict(payload=True, drop=True, orbit=True, finish='LAND', emergency='LAND')
    text = ' '.join(describe(spec, fleet_size=2))
    assert 'payload' in text and 'circles' in text and 'lands where it is' in text and 'Fleet' in text
    assert requirements(spec) == ['camera', 'gimbal', 'gnss', 'payload', 'overhead_envelope']


@pytest.mark.skipif(not RUNNER.exists(), reason='as2_behavior_tree runner not built')
def test_every_custom_tree_passes_the_onboard_registry_validation():
    validation = pytest.importorskip('as2_mission_manager.validation')
    manager = pytest.importorskip('as2_mission_manager.manager')
    registry = manager.load_registry(RUNNER)
    for spec in valid_specs():
        for fleet in (1, 2):
            text = xml(custom_mission(spec, POINTS, 'group', fleet_size=fleet, area=AREA,
                                      stations='1,2;drone1=3,4' if fleet > 1 else None))
            report = validation.validate_xml(text, registry)
            assert report['requires_flight'], spec


def _plan(features, fleet=('drone0',), **extra):
    return plan_mission(dict(kml=KML.read_text(), template='custom', features=features, mission_id=uuid.uuid4().hex,
                             fleet=[dict(drone_id=d, endurance_s=1200, reserve_fraction=.25) for d in fleet],
                             altitude_m=12, speed_mps=1.5, **extra))


@pytest.mark.skipif(not KML.exists(), reason='sample KML not present')
def test_custom_plans_validate_against_their_route_bundles():
    cases = [dict(camera=False, detect=False, geotag=False),
             dict(payload=True, drop=True, orbit=True, finish='LAND'),
             dict(type='track', geotag=False, resume=True),
             dict(type='watch', geotag=True, orbit=True, emergency='LAND')]
    for features in cases:
        # The sample field is too small for two lane bands at 10 m separation; two watch stations fit.
        fleet = ('drone0', 'drone1') if features.get('type') == 'watch' else ('drone0',)
        result = _plan(features, fleet=fleet, search_time_s=120)
        assert result['complete'], (features, result['reason'])
        assert result['template'] == 'custom' and result['features'] == feature_spec(features)
        for bundle in result['bundles']:
            validate_route_bundle(bundle, bundle['drone_id'], result['xml'][bundle['drone_id']])
            assert bundle['delivery_policy']['overhead_authorized'] == bool(features.get('drop'))
        if features.get('type') == 'watch':
            assert set(result['stations']) == {'drone0', 'drone1'}
            assert all(len(b['coverage']['waypoints']) == 1 for b in result['bundles'])
    with pytest.raises(ValueError):
        _plan(dict(camera=False, detect=True))


@pytest.mark.skipif(not KML.exists(), reason='sample KML not present')
def test_emergency_action_setting_changes_only_the_recovery_branch():
    base = dict(template='human_geotag_resume', mission_id='a' * 32)
    rtl = plan_mission(dict(kml=KML.read_text(), fleet=[dict(drone_id='drone0', endurance_s=1200)], altitude_m=12, **base))
    land = plan_mission(dict(kml=KML.read_text(), fleet=[dict(drone_id='drone0', endurance_s=1200)], altitude_m=12,
                             emergency_action='LAND', **base))
    a, b = rtl['xml']['drone0'], land['xml']['drone0']
    assert a != b and land['emergency_action'] == 'LAND'
    assert a.replace('<Sequence name="ReturnAndLand"><ReturnToLaunch speed="2.0" timeout_s="900" />', '<Sequence name="LandInPlace">') == b
    with pytest.raises(ValueError):
        plan_mission(dict(kml=KML.read_text(), fleet=[dict(drone_id='drone0', endurance_s=1200)], altitude_m=12,
                          emergency_action='HOVER', **base))


# Per-aircraft behaviour (2026-11-01): "Search & orbit" — only the drones the operator chose
# orbit a person; the others geotag and keep searching.
SEARCH = dict(type='coverage', detect=True, geotag=True, orbit=False, resume=True)
ORBIT = dict(SEARCH, orbit=True)


def plan_two(own=None, **kw):
    fleet = [dict(drone_id='drone0', endurance_s=1800), dict(drone_id='drone1', endurance_s=1800, **({'features': own} if own is not None else {}))]
    return plan_mission(dict(kml=KML.read_text(), fleet=fleet, mission_id=uuid.uuid4().hex, template='custom',
                             features=SEARCH, altitude_m=15, spacing_m=15, speed_mps=1.5, **kw))


def test_only_the_chosen_drone_orbits_and_both_keep_searching():
    result = plan_two(ORBIT, orbit_radius_m=8, orbit_laps=1)
    tags = {d: [e.tag for e in ET.fromstring(result['xml'][d]).iter()] for d in ('drone0', 'drone1')}
    assert 'OrbitTarget' in tags['drone1'] and 'OrbitTarget' not in tags['drone0']
    for d in ('drone0', 'drone1'):
        assert 'GeotagOverhead' in tags[d] and 'ResumeCoverage' in tags[d] and 'SurveyWaypoint' in tags[d]
    orbit = next(ET.fromstring(result['xml']['drone1']).iter('OrbitTarget'))
    assert float(orbit.get('radius_m')) == 8 and float(orbit.get('laps')) == 1
    assert result['aircraft_features']['drone1']['orbit'] is True


def test_per_aircraft_features_may_not_change_the_mission_type_or_payload():
    with pytest.raises(ValueError, match='may not change type'):
        plan_two(dict(ORBIT, type='watch', geotag=False))
    with pytest.raises(ValueError, match='may not change payload'):
        plan_two(dict(SEARCH, payload=True, drop=True))


def test_per_aircraft_features_need_a_custom_mission():
    fleet = [dict(drone_id='drone0', endurance_s=1800, features=ORBIT)]
    with pytest.raises(ValueError, match='custom mission'):
        plan_mission(dict(kml=KML.read_text(), fleet=fleet, mission_id=uuid.uuid4().hex,
                          template='human_geotag_resume', altitude_m=15, spacing_m=15, speed_mps=1.5))


# Mission 7 (2026-11-01): 2 search drones + 2 delivery drones.
def plan_roles(roles, **kw):
    fleet = [dict(drone_id='drone%d' % i, endurance_s=1800, **({'role': r} if r else {})) for i, r in enumerate(roles)]
    return plan_mission(dict(kml=KML.read_text(), fleet=fleet, mission_id=uuid.uuid4().hex, template='custom',
                             features=SEARCH, altitude_m=15, spacing_m=15, speed_mps=1.5, **kw))


def test_delivery_drones_get_no_lanes_and_a_delivery_tree():
    result = plan_roles(['search', 'search', 'delivery', 'delivery'], delivery_wait_s=900, delivery_speed=2, airborne_wait_s=60)
    assert result['complete'], result['reason']
    assert result['delivery'] == ['drone2', 'drone3']
    by_id = {b['drone_id']: b for b in result['bundles']}
    assert set(by_id) == {'drone0', 'drone1', 'drone2', 'drone3'}
    for d in ('drone2', 'drone3'):
        tags = [e.tag for e in ET.fromstring(result['xml'][d]).iter()]
        assert by_id[d]['coverage']['waypoints'] == []
        assert tags.index('AcquireTarget') < tags.index('Arm') < tags.index('FlyToTarget') < tags.index('DropAtTarget')
        assert 'SurveyWaypoint' not in tags and 'HumanDetected' not in tags
        assert 'PeersFresh' in tags and 'RecoverThenFail' in tags and 'ReturnToLaunch' in tags
        acquire = [e for e in ET.fromstring(result['xml'][d]).iter('AcquireTarget')]
        assert [float(e.get('timeout_s')) for e in acquire] == [900, 60]      # ground wait, in-air wait
        assert 'KeepRunningUntilFailure' in tags and 'PayloadAvailable' in tags  # until packages run out
        # The ground wait is outside the flight safety gate (a parked data gap must not cancel it).
        root = ET.fromstring(result['xml'][d])
        gate = next(root.iter('SafetyGate'))
        assert len(list(root.iter('AcquireTarget'))) == 2 and len(list(gate.iter('AcquireTarget'))) == 1
        # Nobody to serve on the ground: no take-off, a normal end (Inverter makes it a success).
        top = root.find('.//Fallback[@name="DeliverIfSomeoneIsLeft"]')
        assert top is not None and top[0].tag == 'Inverter' and top[0][0].tag == 'AcquireTarget'
        assert tags.index('DropAtTarget') < tags.index('PayloadAvailable') < tags.index('ReturnToLaunch')
        validate_route_bundle(by_id[d], d, result['xml'][d])      # what the GCS checks at staging
    for d in ('drone0', 'drone1'):
        tags = [e.tag for e in ET.fromstring(result['xml'][d]).iter()]
        assert 'SurveyWaypoint' in tags and 'GeotagOverhead' in tags and 'AcquireTarget' not in tags
    # Lanes are split between the two search drones only.
    assert {a['drone_id'] for a in result['plan']['assignments']} == {'drone0', 'drone1'}
    assert len(result['manifest']['artifacts']) == 4


def test_delivery_needs_geotagging_search_drones():
    with pytest.raises(ValueError, match='at least one search drone'):
        plan_roles(['delivery', 'delivery'])
    with pytest.raises(ValueError, match='role must be'):
        plan_roles(['search', 'courier'])
    fleet = [dict(drone_id='drone0', endurance_s=1800), dict(drone_id='drone1', endurance_s=1800, role='delivery')]
    with pytest.raises(ValueError, match='geotag'):
        plan_mission(dict(kml=KML.read_text(), fleet=fleet, mission_id=uuid.uuid4().hex, template='custom',
                          features=dict(type='coverage', detect=True, geotag=False, orbit=True, resume=True),
                          altitude_m=15, spacing_m=15, speed_mps=1.5))
