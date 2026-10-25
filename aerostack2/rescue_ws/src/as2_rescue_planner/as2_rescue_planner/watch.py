"""Watch stations for mission 2 (human_orbit) when several aircraft share one KML area.

Each aircraft flies to one planned station in its share of the area and watches from there.
A peer orbiting a person can cover that station (the planner does not know where people are),
so the planner also lists a few other stations the aircraft may move its watch to:

  alternates  points inside this aircraft's own share (its band of coverage lanes)
  fleet       the other aircraft's planned stations, by drone ID (a peer that is busy with a
              person elsewhere leaves its own share unwatched)

The list travels in the tree (ScanArea ``area`` port), so it is covered by the XML checksum
that binds the route bundle. route_node approves a re-station leg only to one of these points
(``watch_goals``) and only if the leg passes the normal region/corridor checks.

Encoding: ``x,y`` is an alternate, ``drone0=x,y`` a fleet station; entries are separated by
``;``; plan-frame metres with one decimal.
"""
import math
import re
import xml.etree.ElementTree as ET

STATION_SUFFIX = '_station'
MAX_ALTERNATES = 8
MAX_ENTRIES = 72
_OWNER = re.compile(r'[A-Za-z0-9_-]{1,32}')


def station_segment(segment_id):
    """True for the single entry leg of a mission-2 watch station."""
    return isinstance(segment_id, str) and segment_id.endswith(STATION_SUFFIX)


def alternates(region, waypoints, station, count=MAX_ALTERNATES, inset_m=5.0, spacing_m=8.0):
    """Spread-out points along this aircraft's lanes, kept inset from the area boundary.

    ``waypoints`` are the lane endpoints of the aircraft's share in lane order (start, end,
    start, end, ...). Farthest-point sampling from the planned station keeps the choice
    deterministic and the points apart.
    """
    pool = []
    for inset in (inset_m, inset_m / 2):  # a narrow share keeps whatever fits at half the inset
        for start, end in zip(waypoints[0::2], waypoints[1::2]):
            a, b = start['pose'][:2], end['pose'][:2]
            for t in (0.15, 0.5, 0.85):
                point = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                if region.contains_circle(point, inset):
                    pool.append([round(point[0], 1), round(point[1], 1)])
        if pool:
            break
    chosen = []
    anchors = [list(station[:2])]
    while pool and len(chosen) < count:
        best = max(pool, key=lambda p: min(math.dist(p, q) for q in anchors))
        if min(math.dist(best, q) for q in anchors) < spacing_m:
            break
        chosen.append(best)
        anchors.append(best)
        pool.remove(best)
    return chosen


def encode(alternate_points, fleet_stations):
    items = ['%g,%g' % (round(x, 1), round(y, 1)) for x, y in alternate_points]
    for owner in sorted(fleet_stations):
        if not _OWNER.fullmatch(owner):
            raise ValueError('Drone ID unsuitable for a watch station list: %r' % owner)
        x, y = fleet_stations[owner][:2]
        items.append('%s=%g,%g' % (owner, round(x, 1), round(y, 1)))
    return ';'.join(items)


def parse(text):
    """[dict(owner=None or drone ID, xy=[x, y]), ...]; raises ValueError on malformed text."""
    out = []
    for item in str(text or '').split(';'):
        item = item.strip()
        if not item:
            continue
        owner = None
        if '=' in item:
            owner, item = item.split('=', 1)
            if not _OWNER.fullmatch(owner):
                raise ValueError('Invalid drone ID in watch station list')
        values = [float(v) for v in item.split(',')]
        if len(values) != 2 or not all(math.isfinite(v) and abs(v) < 1e6 for v in values):
            raise ValueError('Watch stations must be finite x,y metres')
        out.append(dict(owner=owner, xy=values))
    if len(out) > MAX_ENTRIES:
        raise ValueError('Too many watch stations')
    return out


def watch_goals(xml_text):
    """Every re-station goal [x, y, z] a tree allows: each ScanArea's planned station (pose
    port) and the stations listed in its area port, all at the planned station's height."""
    goals = []
    for node in ET.fromstring(xml_text).iter('ScanArea'):
        pose = node.get('pose') or ''
        if not pose:
            continue
        planned = [float(v) for v in pose.split(';')]
        if len(planned) != 3 or not all(math.isfinite(v) for v in planned):
            raise ValueError('ScanArea station pose must be x;y;z metres')
        goals.append(planned)
        goals.extend([entry['xy'][0], entry['xy'][1], planned[2]] for entry in parse(node.get('area')))
    return goals
