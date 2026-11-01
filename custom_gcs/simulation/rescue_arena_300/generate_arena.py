#!/usr/bin/env python3
"""Generate an offline Gazebo world, matching Xacro/URDF and target truth.

Only the Python standard library is required. All geometry is authored here;
no external meshes, Fuel downloads or model repositories are needed.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import random
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
XACRO = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", XACRO)


def el(parent, tag, content=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if content is not None:
        node.text = str(content)
    return node


def vec(values):
    return " ".join(f"{x:.9g}" for x in values)


def save_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def enu_to_geodetic(east, north, up, origin):
    """WGS84 ENU tangent plane -> ECEF -> geodetic (not degree scaling)."""
    a, e2 = 6378137.0, 6.6943799901413165e-3
    lat, lon = map(math.radians, (origin['latitude_deg'], origin['longitude_deg']))
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    n = a / math.sqrt(1 - e2 * sl * sl)
    h = origin['altitude_m']
    x = (n + h) * cl * co - so * east - sl * co * north + cl * co * up
    y = (n + h) * cl * so + co * east - sl * so * north + cl * so * up
    z = (n * (1 - e2) + h) * sl + cl * north + sl * up
    p = math.hypot(x, y)
    phi = math.atan2(z, p * (1 - e2))
    for _ in range(12):
        n = a / math.sqrt(1 - e2 * math.sin(phi) ** 2)
        phi = math.atan2(z + e2 * n * math.sin(phi), p)
    n = a / math.sqrt(1 - e2 * math.sin(phi) ** 2)
    return math.degrees(phi), math.degrees(math.atan2(y, x)), p / math.cos(phi) - n


def dist(a, b):
    return math.hypot(a['x'] - b['x'], a['y'] - b['y'])


def layout(cfg):
    rng = random.Random(cfg['seed'])
    size = cfg['size_m']
    # Exactly one person per cell: 10 columns x 5 rows at the default count.
    cols = math.ceil(math.sqrt(cfg['human_count'] * 2))
    rows = math.ceil(cfg['human_count'] / cols)
    cells = [(col, row) for row in range(rows) for col in range(cols)]
    if len(cells) != cfg['human_count']:
        cells = sorted(rng.sample(cells, cfg['human_count']), key=lambda xy: (xy[1], xy[0]))
    humans = []
    for col, row in cells:
        for _ in range(10000):
            p = dict(x=round(-size / 2 + (col + rng.uniform(.24, .76)) * size / cols, 4),
                     y=round(-size / 2 + (row + rng.uniform(.24, .76)) * size / rows, 4))
            if math.hypot(p['x'], p['y']) < cfg['launch_clearance_radius_m'] + 2:
                continue
            if all(dist(p, h) >= cfg['human_min_spacing_m'] for h in humans):
                break
        else:
            raise ValueError('Human spacing cannot fit this configuration; reduce count or spacing.')
        p.update(name=f'human_{len(humans)+1:03}', yaw=round(rng.uniform(-math.pi, math.pi), 6),
                 height=round(rng.uniform(1.60, 1.88), 3), sector=f'R{row+1}C{col+1}')
        humans.append(p)
    trees = []
    for _ in range(100000):
        if len(trees) == cfg['tree_count']:
            break
        radius = round(rng.uniform(2.0, 3.2), 3)
        margin = radius + 3
        p = dict(x=round(rng.uniform(-size / 2 + margin, size / 2 - margin), 4),
                 y=round(rng.uniform(-size / 2 + margin, size / 2 - margin), 4),
                 radius=radius, height=round(rng.uniform(6.5, 10.5), 3))
        p['height'] = round(max(p['height'], 2.2 * radius + 1.3), 3)
        if math.hypot(p['x'], p['y']) < cfg['launch_clearance_radius_m'] + radius:
            continue
        if any(dist(p, h) < radius + cfg['human_canopy_clearance_m'] for h in humans):
            continue
        if any(dist(p, t) < radius + t['radius'] + 1.5 for t in trees):
            continue
        p.update(name=f'tree_{len(trees)+1:03}', yaw=round(rng.uniform(-math.pi, math.pi), 6))
        trees.append(p)
    if len(trees) != cfg['tree_count']:
        raise ValueError('Tree count cannot fit the requested clearances.')
    return humans, trees


def part(name, shape, dims, xyz, color, collision=True, rpy=(0, 0, 0)):
    return dict(name=name, shape=shape, dims=dims, xyz=xyz, color=color,
                collision=collision, rpy=rpy)


def human_parts(h, index):
    scale = h['height'] / 1.8
    shirts = [(0.8, .16, .12, 1), (.1, .3, .8, 1), (.95, .63, .06, 1),
              (.68, .15, .57, 1), (.85, .86, .8, 1), (.06, .55, .58, 1)]
    skins = [(.65, .40, .25, 1), (.9, .68, .48, 1), (.38, .22, .14, 1)]
    shirt, skin, pants, boots = shirts[index % 6], skins[index % 3], (.12, .16, .23, 1), (.06, .05, .04, 1)
    parts = [part('torso', 'box', (.25, .43, .49), (0, 0, 1.215), shirt),
             part('hips', 'box', (.23, .33, .20), (0, 0, .90), pants),
             part('neck', 'cylinder', (.065, .10), (0, 0, 1.50), skin),
             part('head', 'sphere', (.14,), (0, 0, 1.66), skin),
             part('hair', 'sphere', (.117,), (-.025, 0, 1.683), (.09, .055, .035, 1), False),
             part('nose', 'sphere', (.033,), (.137, 0, 1.66), skin, False)]
    for side, sign in [('left', 1), ('right', -1)]:
        parts += [part(f'{side}_leg', 'cylinder', (.078, .75), (0, sign*.105, .445), pants),
                  part(f'{side}_shoe', 'box', (.28, .14, .10), (.055, sign*.105, .05), boots),
                  part(f'{side}_sleeve', 'cylinder', (.072, .29), (0, sign*.275, 1.275), shirt,
                       rpy=(sign*.18, 0, 0)),
                  part(f'{side}_forearm', 'cylinder', (.052, .25), (.02, sign*.30, 1.01), skin),
                  part(f'{side}_hand', 'sphere', (.06,), (.02, sign*.30, .855), skin)]
    for p in parts:
        p['dims'] = tuple(d*scale for d in p['dims'])
        p['xyz'] = tuple(d*scale for d in p['xyz'])
    return parts


def tree_parts(t, index):
    r, h = t['radius'], t['height']
    foliage = [(.16, .32, .10, 1), (.23, .40, .12, 1), (.12, .29, .18, 1), (.31, .42, .13, 1)][index % 4]
    # Three overlapping spheres remain within radius r, including their offsets.
    return [part('trunk', 'cylinder', (.20 + .015*h, h-r), (0, 0, (h-r)/2), (.30, .18, .085, 1)),
            part('crown', 'sphere', (r,), (0, 0, h-r), foliage),
            part('crown_left', 'sphere', (.70*r,), (.28*r, .10*r, h-r-.5*r), foliage),
            part('crown_right', 'sphere', (.68*r,), (-.28*r, -.1*r, h-r-.45*r), foliage)]


def scene_objects(cfg, humans, trees):
    size = cfg['size_m']
    objects = [dict(name='ground', x=0, y=0, yaw=0,
                    parts=[part('surface', 'box', (size, size, .5), (0, 0, -.25), (.40, .47, .29, 1))])]
    white = (.88, .87, .73, 1)
    markings = []
    for axis in range(2):
        for sign in [-1, 1]:
            xyz = (0, sign*(size/2-.3), .012) if axis == 0 else (sign*(size/2-.3), 0, .012)
            dims = (size, .35, .02) if axis == 0 else (.35, size, .02)
            markings.append(part(f'edge_{axis}_{sign}', 'box', dims, xyz, white, False))
    markings.append(part('pad', 'cylinder', (7, .02), (0, 0, .012), (.17, .21, .23, 1), False))
    for name, xyz, dims in [('h_left', (-1.7, 0, .03), (.6, 5, .02)),
                            ('h_right', (1.7, 0, .03), (.6, 5, .02)),
                            ('h_bar', (0, 0, .03), (3.4, .6, .02))]:
        markings.append(part(name, 'box', dims, xyz, (1, .83, .15, 1), False))
    objects.append(dict(name='arena_markings', x=0, y=0, yaw=0, parts=markings))
    objects.extend(dict(**h, parts=human_parts(h, i)) for i, h in enumerate(humans))
    objects.extend(dict(**t, parts=tree_parts(t, i)) for i, t in enumerate(trees))
    return objects


def geometry(parent, p, urdf=False):
    g = el(parent, 'geometry')
    if p['shape'] == 'box':
        if urdf:
            el(g, 'box', size=vec(p['dims']))
        else:
            el(el(g, 'box'), 'size', vec(p['dims']))
    else:
        attrs = dict(radius=f"{p['dims'][0]:.9g}")
        if p['shape'] == 'cylinder':
            attrs['length'] = f"{p['dims'][1]:.9g}"
        shape = el(g, p['shape'], **attrs) if urdf else el(g, p['shape'])
        if not urdf:
            for key, val in attrs.items():
                el(shape, key, val)


def sdf_world(cfg, objects):
    root = ET.Element('sdf', version='1.9')
    root.append(ET.Comment('Generated by generate_arena.py. Metres; +X east, +Y north, +Z up.'))
    w = el(root, 'world', name=cfg['world_name'])
    physics = el(w, 'physics', name='px4_250hz', type='ignored')
    for key, val in [('max_step_size', .004), ('real_time_factor', 1), ('real_time_update_rate', 250)]:
        el(physics, key, val)
    systems = [('physics', 'Physics'), ('user-commands', 'UserCommands'),
               ('scene-broadcaster', 'SceneBroadcaster'), ('contact', 'Contact'),
               ('imu', 'Imu'), ('air-pressure', 'AirPressure'),
               ('navsat', 'NavSat'), ('magnetometer', 'Magnetometer'), ('sensors', 'Sensors')]
    for filename, name in systems:
        plugin = el(w, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            el(plugin, 'render_engine', 'ogre2')
    el(w, 'gravity', '0 0 -9.80665')
    el(w, 'magnetic_field', '6e-06 2.3e-05 -4.2e-05')
    el(w, 'atmosphere', type='adiabatic')
    geo = el(w, 'spherical_coordinates')
    for k, v in [('surface_model', 'EARTH_WGS84'), ('world_frame_orientation', 'ENU'),
                 ('latitude_deg', cfg['origin']['latitude_deg']),
                 ('longitude_deg', cfg['origin']['longitude_deg']),
                 ('elevation', cfg['origin']['altitude_m']), ('heading_deg', 0)]:
        el(geo, k, v)
    scene = el(w, 'scene')
    for k, v in [('ambient', '.65 .65 .65 1'), ('background', '.68 .81 .91 1'),
                 ('shadows', 'true'), ('grid', 'false')]:
        el(scene, k, v)
    light = el(w, 'light', name='sun', type='directional')
    for k, v in [('pose', '0 0 200 0 0 0'), ('diffuse', '.95 .93 .86 1'),
                 ('specular', '.15 .15 .15 1'), ('direction', '-.4 .2 -.9'), ('cast_shadows', 'true')]:
        el(light, k, v)
    attenuation = el(light, 'attenuation')
    for k, v in [('range', 2000), ('constant', 1), ('linear', 0), ('quadratic', 0)]:
        el(attenuation, k, v)
    gui = el(w, 'gui', fullscreen='false')
    view = el(gui, 'plugin', filename='MinimalScene', name='3D View')
    options = el(view, 'gz-gui')
    el(options, 'property', 'docked', type='string', key='state')
    el(view, 'engine', 'ogre2')
    el(view, 'scene', 'scene')
    el(view, 'camera_pose', '-220 -260 250 0 .63 .87')
    clip = el(view, 'camera_clip')
    el(clip, 'near', .1)
    el(clip, 'far', 2000)
    for filename in ['GzSceneManager', 'InteractiveViewControl', 'WorldControl', 'WorldStats', 'EntityTree']:
        el(gui, 'plugin', filename=filename, name=filename)
    for obj in objects:
        model = el(w, 'model', name=obj['name'])
        el(model, 'static', 'true')
        el(model, 'pose', vec((obj['x'], obj['y'], 0, 0, 0, obj['yaw'])))
        link = el(model, 'link', name='body')
        for p in obj['parts']:
            v = el(link, 'visual', name=p['name'])
            el(v, 'pose', vec((*p['xyz'], *p['rpy'])))
            geometry(v, p)
            material = el(v, 'material')
            el(material, 'ambient', vec(p['color']))
            el(material, 'diffuse', vec(p['color']))
            el(material, 'specular', '0.05 0.05 0.05 1')
            if p['collision']:
                c = el(link, 'collision', name=p['name'] + '_collision')
                el(c, 'pose', vec((*p['xyz'], *p['rpy'])))
                geometry(c, p)
    return root


def urdf_robot(objects):
    robot = ET.Element('robot', name='rescue_arena_300')
    el(robot, 'link', name='world')
    el(el(robot, 'gazebo'), 'static', 'true')
    for obj in objects:
        link = el(robot, 'link', name=obj['name'])
        inertial = el(link, 'inertial')
        el(inertial, 'mass', value='1')
        el(inertial, 'inertia', ixx='1', iyy='1', izz='1', ixy='0', ixz='0', iyz='0')
        joint = el(robot, 'joint', name=obj['name']+'_fixed', type='fixed')
        el(joint, 'parent', link='world')
        el(joint, 'child', link=obj['name'])
        el(joint, 'origin', xyz=vec((obj['x'], obj['y'], 0)), rpy=vec((0, 0, obj['yaw'])))
        for p in obj['parts']:
            v = el(link, 'visual', name=p['name'])
            el(v, 'origin', xyz=vec(p['xyz']), rpy=vec(p['rpy']))
            geometry(v, p, urdf=True)
            el(el(v, 'material', name=obj['name']+'_'+p['name']), 'color', rgba=vec(p['color']))
            if p['collision']:
                c = el(link, 'collision', name=p['name']+'_collision')
                el(c, 'origin', xyz=vec(p['xyz']), rpy=vec(p['rpy']))
                geometry(c, p, urdf=True)
    return robot


def xacro_robot(objects):
    robot = ET.Element('robot', name='rescue_arena_300')
    robot.append(ET.Comment('Rebuild placements from config/arena.json with generate_arena.py.'))
    # One macro per primitive supports editing dimensions/materials without meshes.
    for shape in ['box', 'sphere', 'cylinder']:
        extra = {'box': 'size', 'sphere': 'radius', 'cylinder': 'radius length'}[shape]
        macro = el(robot, f'{{{XACRO}}}macro', name='arena_'+shape,
                   params=f'name xyz rpy rgba collision {extra}')
        v = el(macro, 'visual', name='${name}')
        el(v, 'origin', xyz='${xyz}', rpy='${rpy}')
        g = el(v, 'geometry')
        el(g, shape, **{k: '${'+k+'}' for k in extra.split()})
        el(el(v, 'material', name='${name}_material'), 'color', rgba='${rgba}')
        cond = el(macro, f'{{{XACRO}}}if', value='${collision}')
        c = el(cond, 'collision', name='${name}_collision')
        el(c, 'origin', xyz='${xyz}', rpy='${rpy}')
        el(el(c, 'geometry'), shape, **{k: '${'+k+'}' for k in extra.split()})
    el(robot, 'link', name='world')
    el(el(robot, 'gazebo'), 'static', 'true')
    for obj in objects:
        link = el(robot, 'link', name=obj['name'])
        inertial = el(link, 'inertial')
        el(inertial, 'mass', value='1')
        el(inertial, 'inertia', ixx='1', iyy='1', izz='1', ixy='0', ixz='0', iyz='0')
        joint = el(robot, 'joint', name=obj['name']+'_fixed', type='fixed')
        el(joint, 'parent', link='world')
        el(joint, 'child', link=obj['name'])
        el(joint, 'origin', xyz=vec((obj['x'], obj['y'], 0)), rpy=vec((0, 0, obj['yaw'])))
        for p in obj['parts']:
            attrs = dict(name=obj['name']+'_'+p['name'], xyz=vec(p['xyz']), rpy=vec(p['rpy']),
                         rgba=vec(p['color']), collision=str(p['collision']).lower())
            if p['shape'] == 'box':
                attrs['size'] = vec(p['dims'])
            else:
                attrs['radius'] = f"{p['dims'][0]:.9g}"
                if p['shape'] == 'cylinder':
                    attrs['length'] = f"{p['dims'][1]:.9g}"
            el(link, f'{{{XACRO}}}arena_{p["shape"]}', **attrs)
    return robot


def validate_config(cfg):
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', cfg['world_name']):
        raise ValueError('Use an alphanumeric world name beginning with a letter.')
    for field in ['size_m', 'human_min_spacing_m', 'human_canopy_clearance_m', 'launch_clearance_radius_m']:
        if not math.isfinite(cfg[field]) or cfg[field] <= 0:
            raise ValueError(f'{field} must be finite and positive.')
    if not isinstance(cfg['human_count'], int) or cfg['human_count'] < 1:
        raise ValueError('human_count must be a positive integer.')
    if not isinstance(cfg['tree_count'], int) or cfg['tree_count'] < 0:
        raise ValueError('tree_count must be a nonnegative integer.')
    origin = cfg['origin']
    if not -89 < origin['latitude_deg'] < 89 or not -180 <= origin['longitude_deg'] <= 180:
        raise ValueError('Invalid origin; polar locations are not supported.')
    if not math.isfinite(origin['altitude_m']):
        raise ValueError('Origin altitude must be finite.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/arena.json')
    parser.add_argument('--output', type=Path, default=ROOT)
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    validate_config(cfg)
    humans, trees = layout(cfg)
    objects = scene_objects(cfg, humans, trees)
    out = args.output.resolve()
    name = cfg['world_name']
    save_xml(out/'worlds'/f'{name}.sdf', sdf_world(cfg, objects))
    save_xml(out/'urdf'/f'{name}.urdf', urdf_robot(objects))
    save_xml(out/'urdf'/f'{name}.urdf.xacro', xacro_robot(objects))
    truth_dir = out/'ground_truth'
    truth_dir.mkdir(parents=True, exist_ok=True)
    for h in humans:
        lat, lon, alt = enu_to_geodetic(h['x'], h['y'], 0, cfg['origin'])
        h.update(latitude_deg=lat, longitude_deg=lon, altitude_m=alt, z=0)
    with (truth_dir/'humans.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(humans[0]))
        writer.writeheader()
        writer.writerows(humans)
    nearest = min((dist(a, b) for i, a in enumerate(humans) for b in humans[i+1:]), default=None)
    manifest = dict(config=cfg, frame='ENU', ground_truth_reference='feet at z=0 in the world tangent plane',
                    actual_min_human_spacing_m=nearest, humans=humans, trees=trees)
    (truth_dir/'layout.json').write_text(json.dumps(manifest, indent=2)+'\n')
    # Intended for AS2's world launcher only; PX4 spawns its own drone separately.
    (out/'config').mkdir(exist_ok=True)
    (out/'config/as2_world.yaml').write_text(
        '# World only. PX4 SITL spawns and controls the aircraft.\n'
        f'world_name: {name}\nuse_sim_time: true\ndrones: []\nobjects: []\n')
    print(f'Created {name}: {cfg["size_m"]:g} m square, {len(humans)} humans, {len(trees)} trees.')
    if nearest is not None:
        print(f'Closest human pair: {nearest:.2f} m. Output: {out}')


if __name__ == '__main__':
    main()
