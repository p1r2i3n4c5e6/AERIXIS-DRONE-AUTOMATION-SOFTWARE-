#!/usr/bin/env python3
"""Check the delivered arena, clearances, ground truth and URDF/Xacro geometry."""
import csv
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from generate_arena import ROOT, dist


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    data = json.loads((ROOT/'ground_truth/layout.json').read_text())
    cfg, humans, trees = data['config'], data['humans'], data['trees']
    world_file = ROOT/'worlds'/f'{cfg["world_name"]}.sdf'
    world = ET.parse(world_file).getroot().find('world')
    models = {m.attrib['name']: m for m in world.findall('model')}
    check(len(models) == len(world.findall('model')), 'Duplicate SDF model names')
    check(len(humans) == cfg['human_count'] == len([n for n in models if n.startswith('human_')]), 'Human count')
    check(len(trees) == cfg['tree_count'] == len([n for n in models if n.startswith('tree_')]), 'Tree count')
    check(world.findtext("model[@name='ground']/link/collision/geometry/box/size") == f'{cfg["size_m"]:g} {cfg["size_m"]:g} 0.5', 'Ground dimensions')
    check(not world.findall('.//uri'), 'Unexpected external resource dependency')
    check(world.findtext('spherical_coordinates/world_frame_orientation') == 'ENU', 'World frame')
    origin = cfg['origin']
    for field, key in [('latitude_deg', 'latitude_deg'), ('longitude_deg', 'longitude_deg'), ('elevation', 'altitude_m')]:
        check(float(world.findtext('spherical_coordinates/'+field)) == origin[key], 'World origin mismatch')
    for model in models.values():
        check(model.findtext('static') == 'true', 'Environment model is not static')
    for i, h in enumerate(humans):
        check(abs(h['x']) < cfg['size_m']/2 and abs(h['y']) < cfg['size_m']/2, 'Human outside arena')
        check(math.hypot(h['x'], h['y']) >= cfg['launch_clearance_radius_m'] + 2, 'Person in launch area')
        for other in humans[i+1:]:
            check(dist(h, other) >= cfg['human_min_spacing_m'], 'People too close')
    check(len({h['sector'] for h in humans}) == len(humans), 'Coverage cells duplicated')
    for i, t in enumerate(trees):
        check(t['height'] - 2.2*t['radius'] >= 1.299, 'Foliage too close to the ground')
        check(max(abs(t['x']), abs(t['y']))+t['radius'] < cfg['size_m']/2, 'Canopy outside arena')
        check(math.hypot(t['x'], t['y'])-t['radius'] >= cfg['launch_clearance_radius_m'], 'Tree in launch area')
        check(all(dist(t, h)-t['radius'] >= cfg['human_canopy_clearance_m'] for h in humans), 'Tree obscures a target')
        check(all(dist(t, b) >= t['radius']+b['radius']+1.5 for b in trees[i+1:]), 'Overlapping tree envelopes')
    # Independent inverse: verify CSV latitude/longitude convert back to ENU feet.
    a, e2 = 6378137.0, 6.6943799901413165e-3
    def ecef(lat, lon, alt):
        lat, lon = math.radians(lat), math.radians(lon)
        n = a/math.sqrt(1-e2*math.sin(lat)**2)
        return ((n+alt)*math.cos(lat)*math.cos(lon), (n+alt)*math.cos(lat)*math.sin(lon),
                (n*(1-e2)+alt)*math.sin(lat))
    zero = ecef(origin['latitude_deg'], origin['longitude_deg'], origin['altitude_m'])
    phi, lam = math.radians(origin['latitude_deg']), math.radians(origin['longitude_deg'])
    with (ROOT/'ground_truth/humans.csv').open() as f:
        records = list(csv.DictReader(f))
    check(len(records) == len(humans), 'CSV row count')
    for h in records:
        xyz = ecef(float(h['latitude_deg']), float(h['longitude_deg']), float(h['altitude_m']))
        dx, dy, dz = [v-z for v, z in zip(xyz, zero)]
        enu = (-math.sin(lam)*dx+math.cos(lam)*dy,
               -math.sin(phi)*math.cos(lam)*dx-math.sin(phi)*math.sin(lam)*dy+math.cos(phi)*dz,
               math.cos(phi)*math.cos(lam)*dx+math.cos(phi)*math.sin(lam)*dy+math.sin(phi)*dz)
        check(max(abs(v-float(h[k])) for v, k in zip(enu, ['x', 'y', 'z'])) < 1e-5, 'GPS/ENU mismatch')
        pose = list(map(float, models[h['name']].findtext('pose').split()))
        check(abs(pose[0]-float(h['x'])) < 1e-5 and abs(pose[1]-float(h['y'])) < 1e-5, 'SDF/CSV mismatch')
    urdf = ROOT/'urdf'/f'{cfg["world_name"]}.urdf'
    def geometry_signature(path):
        root = ET.parse(path).getroot()
        result = []
        for link in root.findall('link'):
            for kind in ['visual', 'collision']:
                for p in link.findall(kind):
                    shape = list(p.find('geometry'))[0]
                    color = p.find('material/color')
                    result.append((link.attrib['name'], kind, tuple(sorted(p.find('origin').attrib.items())),
                                   shape.tag, tuple(sorted(shape.attrib.items())),
                                   color.get('rgba') if color is not None else None))
        return result
    check(len(ET.parse(urdf).getroot().findall('link')) == len(models)+1, 'URDF link count')
    if shutil.which('xacro'):
        with tempfile.TemporaryDirectory(prefix='arena_validation_') as tmp:
            expanded = Path(tmp)/'arena.urdf'
            subprocess.run(['xacro', str(urdf)+'.xacro', '-o', str(expanded)], check=True)
            check(geometry_signature(urdf) == geometry_signature(expanded), 'Xacro/URDF geometry differs')
            if shutil.which('check_urdf'):
                subprocess.run(['check_urdf', str(expanded)], stdout=subprocess.DEVNULL, check=True)
    else:
        print('SKIP Xacro expansion: xacro is not installed.')
    if shutil.which('gz'):
        subprocess.run(['gz', 'sdf', '-k', str(world_file)], check=True)
    print(f'PASS: {len(humans)} humans, {len(trees)} trees, spacing {data["actual_min_human_spacing_m"]:.2f} m; clearances, GPS, SDF and URDF/Xacro checked.')


if __name__ == '__main__':
    main()
