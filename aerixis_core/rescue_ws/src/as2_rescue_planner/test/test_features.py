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
