#!/usr/bin/env python3
"""Prepare the rescue arena for one simulation run, placed at the chosen map location.

The arena package (../rescue_arena_300) is generated from config/arena.json. This writes a copy
with its GPS origin at --home, so the Gazebo world, PX4's simulated GPS (SIM_GZ_HOME_*), the
ground-truth people (humans.csv, latitude/longitude) and the GCS map all agree. The original
arena files are never changed.

  prepare_arena.py --home LAT,LON[,ALT] --drones N [--out DIR]

Writes DIR/worlds/rescue_arena_300.sdf, DIR/ground_truth/{humans.csv,layout.json} and
DIR/sim_info.json (home, spawn points, people in each drone's local frame). Prints DIR.
"""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ARENA = HERE.parent / 'rescue_arena_300'
STATE = Path.home() / '.local/share/aerixis/arena'
SPACING_M = 20.0          # swarm gate: 10 m + both GNSS uncertainties (SITL 2026-10-22)
TREE_MARGIN_M = 4.0       # take-off spot to the nearest canopy edge


# Fixed take-off spots (arena metres, east/north): drone i always takes slot i, so drones can be
# added to a running arena one by one without moving the others. 20 m apart (swarm gate), inside
# the 19 m central clearing.
SLOTS = [(-10.0, -10.0), (10.0, -10.0), (-10.0, 10.0), (10.0, 10.0)]


def spawn_points(n, trees):
    """Take-off spots of the first n drones (fixed slots), checked against the trees."""
    if not 1 <= n <= len(SLOTS):
        raise SystemExit(f'The arena clearing fits 1 to {len(SLOTS)} drones (20 m apart).')
    points = SLOTS[:n]
    for x, y in points:
        edge = min(math.hypot(x - t['x'], y - t['y']) - t['radius'] for t in trees)
        if edge < TREE_MARGIN_M:
            raise SystemExit(f'Spawn ({x}, {y}) is {edge:.1f} m from a tree canopy (need {TREE_MARGIN_M} m).')
    return points


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--home', required=True, help='LAT,LON[,ALT] of the arena centre')
    parser.add_argument('--drones', type=int, default=1)
    parser.add_argument('--out', type=Path, default=None)
    args = parser.parse_args()
    parts = [float(v) for v in args.home.split(',')]
    if len(parts) < 2 or not (-90 <= parts[0] <= 90 and -180 <= parts[1] <= 180):
        raise SystemExit('--home must be LAT,LON[,ALT]')
    lat, lon, alt = parts[0], parts[1], parts[2] if len(parts) > 2 else 0.0

    config = json.loads((ARENA / 'config/arena.json').read_text())
    config['origin'] = dict(latitude_deg=lat, longitude_deg=lon, altitude_m=alt)
    out = (args.out or STATE / f'{lat:.6f}_{lon:.6f}').resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / 'arena.json').write_text(json.dumps(config, indent=2) + '\n')
    subprocess.run([sys.executable, str(ARENA / 'generate_arena.py'), '--config', str(out / 'arena.json'),
                    '--output', str(out)], check=True, stdout=subprocess.DEVNULL)

    layout = json.loads((out / 'ground_truth/layout.json').read_text())
    spawns = spawn_points(args.drones, layout['trees'])
    humans = [dict(name=h['name'], x=h['x'], y=h['y'], latitude_deg=h['latitude_deg'],
                   longitude_deg=h['longitude_deg'], height=h['height']) for h in layout['humans']]
    half = config['size_m'] / 2
    info = dict(
        world=config['world_name'], world_dir=str(out / 'worlds'), size_m=config['size_m'],
        home=dict(latitude_deg=lat, longitude_deg=lon, altitude_m=alt),
        # Arena corners (world ENU metres, SW NE) for the GCS map outline.
        bounds_enu=[[-half, -half], [half, half]],
        spawns=[dict(drone=f'drone{i}', x=x, y=y) for i, (x, y) in enumerate(spawns)],
        humans=humans,
        # People in the arena frame for the simulated tracker (persons_frame:=arena). Each tracker
        # converts them to AS2's earth frame from its first pose on the ground and its spawn point:
        # the earth origin is chosen by the estimator, not at the spawn point (SITL 2026-10-31).
        persons_arena=';'.join(f'{h["x"]:.2f},{h["y"]:.2f}' for h in humans))
    (out / 'sim_info.json').write_text(json.dumps(info, indent=2) + '\n')
    print(out)


if __name__ == '__main__':
    main()
