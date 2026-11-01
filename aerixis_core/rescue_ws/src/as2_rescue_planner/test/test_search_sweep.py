"""Camera-aware search sweep (2026-11-01): each lane sees exactly its own strip of the area."""
import math
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from as2_rescue_planner.mission import plan_mission
from as2_rescue_planner.trees import search_sweep

KML = Path(__file__).resolve().parents[4] / 'sample_field_mission.kml'


def test_narrow_lanes_need_no_sweep():
    # 15 m up, 65 deg camera: ~19 m across straight down; 5 m lanes need +/-2.75 m.
    s = search_sweep(15, 5, 1.5)
    assert s['sweep_deg'] == 0 and s['covered'] and s['swath_m'] > 19


def test_wide_lanes_sweep_just_far_enough():
    s = search_sweep(15, 30, 1.5)
    need = 30 / 2 * 1.1
    edge = 15 * math.tan(math.radians(s['sweep_deg'] + 42 / 2))
    assert s['covered'] and abs(edge - need) < 0.2 and s['swath_m'] >= 2 * need - 0.5
    # passed over within one footprint length along the lane
    assert s['period_s'] * 1.5 <= 2 * 15 * math.tan(math.radians(32.5))


def test_a_narrower_camera_sweeps_more_and_too_wide_lanes_are_flagged():
    assert search_sweep(15, 30, 1.5, hfov_deg=40, vfov_deg=25)['sweep_deg'] > search_sweep(15, 30, 1.5)['sweep_deg']
    assert not search_sweep(5, 200, 1.5)['covered']


def plan(**kw):
    fleet = [dict(drone_id='drone0', endurance_s=3000, **kw.pop('entry', {}))]
    return plan_mission(dict(kml=KML.read_text(), fleet=fleet, mission_id=uuid.uuid4().hex,
                             template='human_geotag_resume', altitude_m=15, speed_mps=1.5,
                             **dict(dict(search_pitch_deg=-90), **kw)))


def test_planned_tree_carries_the_sweep_for_each_drones_camera():
    result = plan(spacing_m=5)
    gimbal = next(ET.fromstring(result['xml']['drone0']).iter('SetGimbalSearch'))
    assert float(gimbal.get('sweep_deg')) == 0 and result['cameras']['drone0']['hfov_deg'] == 65
    narrow = plan(spacing_m=30, entry=dict(camera_hfov_deg=40, camera_vfov_deg=25))
    gimbal = next(ET.fromstring(narrow['xml']['drone0']).iter('SetGimbalSearch'))
    assert float(gimbal.get('sweep_deg')) > 0 and narrow['cameras']['drone0']['hfov_deg'] == 40


def test_forward_looking_search_has_no_planned_sweep():
    result = plan(spacing_m=15, search_pitch_deg=-45)
    gimbal = next(ET.fromstring(result['xml']['drone0']).iter('SetGimbalSearch'))
    assert gimbal.get('sweep_deg') is None and not result['cameras']
