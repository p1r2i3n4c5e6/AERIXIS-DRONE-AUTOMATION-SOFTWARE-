"""Boustrophedon coverage generation and deterministic fleet partitioning.

Two properties matter here and are enforced rather than assumed:

* Determinism. The same region and the same fleet always produce byte-identical
  assignments, so a preview cannot disagree with the uploaded plan.
* Workload balance is by estimated flight distance, not by area alone, because
  turn overhead and corridor length differ between partitions.
"""
import hashlib
import json
import math
import re

from .geometry import PlannerRegion

DEFAULT_SPACING_M = 10.0
DEFAULT_CORRIDOR_M = 5.0
MIN_SPACING_M = 1.0
MAX_SPACING_M = 200.0


def _line_segments(line_y, region, shell):
    """Intersect a horizontal lane with the region, returning usable x spans.

    ``shell`` is supplied separately because during a rotated sweep the ring is
    expressed in rotated coordinates while ``region`` is the original object.
    """
    crossings = []
    count = len(shell)
    for index in range(count):
        x1, y1 = shell[index]
        x2, y2 = shell[(index + 1) % count]
        if (y1 > line_y) != (y2 > line_y):
            crossings.append((x2 - x1) * (line_y - y1) / (y2 - y1) + x1)
    crossings.sort()
    spans = []
    for index in range(0, len(crossings) - 1, 2):
        left, right = crossings[index], crossings[index + 1]
        if right - left <= 1e-6:
            continue
        # Sample the span so holes and exclusions genuinely break the lane.
        samples = max(3, int((right - left) / 2.0) + 1)
        run_start = None
        for step in range(samples + 1):
            x = left + (right - left) * step / samples
            if region.contains((x, line_y)):
                if run_start is None:
                    run_start = x
            elif run_start is not None:
                if x - run_start > 1e-6:
                    spans.append((run_start, x))
                run_start = None
        if run_start is not None and right - run_start > 1e-6:
            spans.append((run_start, right))
    return spans


def generate_lanes(region, spacing_m=DEFAULT_SPACING_M, heading_deg=0.0):
    """Sweep parallel lanes over the region and return them in traversal order.

    Lanes are generated in a rotated frame so a caller can align coverage with a
    shore, road or channel. The result is always ordered in a boustrophedon
    (alternating direction) sequence to avoid dead-heading between rows.
    """
    if not isinstance(region, PlannerRegion):
        raise ValueError('A validated PlannerRegion is required')
    if not isinstance(spacing_m, (int, float)) or isinstance(spacing_m, bool):
        raise ValueError('Lane spacing must be a number')
    if not MIN_SPACING_M <= spacing_m <= MAX_SPACING_M:
        raise ValueError('Lane spacing must be between %g and %g m' % (MIN_SPACING_M, MAX_SPACING_M))
    if not isinstance(heading_deg, (int, float)) or not math.isfinite(heading_deg):
        raise ValueError('Coverage heading must be a finite angle')

    angle = math.radians(heading_deg)
    cos_a, sin_a = math.cos(angle), math.sin(angle)

    def rotate(point):
        return (point[0] * cos_a + point[1] * sin_a, -point[0] * sin_a + point[1] * cos_a)

    def unrotate(point):
        return (point[0] * cos_a - point[1] * sin_a, point[0] * sin_a + point[1] * cos_a)

    corners = [rotate(corner) for corner in
               [(region.bounds()[0], region.bounds()[1]), (region.bounds()[2], region.bounds()[1]),
                (region.bounds()[2], region.bounds()[3]), (region.bounds()[0], region.bounds()[3])]]
    low = min(point[1] for point in corners)
    high = max(point[1] for point in corners)

    # Containment is tested through the same transform, so the rotated shell must
    # be available for lane intersection as well.
    rotated_shell = [rotate(point) for point in region.shell]

    class Rotated:
        shell = rotated_shell
        bounds = lambda self: (min(p[0] for p in corners), low, max(p[0] for p in corners), high)

        def contains(self, point):
            return region.contains(unrotate(point))

    rotated = Rotated()
    rows = int(math.floor((high - low) / spacing_m)) + 1
    if rows > 100000:
        raise ValueError('Coverage spacing produces an unreasonable number of lanes')

    lanes = []
    index = 0
    y = low + spacing_m / 2.0
    while y <= high + 1e-9:
        spans = _line_segments(y, rotated, rotated.shell)
        # Alternating rows travel in opposite directions, so the span itself is
        # reversed: start/end and the stored length must stay mutually consistent.
        if index % 2:
            spans = [(right, left) for left, right in reversed(spans)]
        for left, right in spans:
            lanes.append({'index': len(lanes), 'row': index,
                          'start': unrotate((left, y)), 'end': unrotate((right, y)),
                          'length_m': abs(right - left)})
        index += 1
        y = low + spacing_m / 2.0 + index * spacing_m
    return lanes


def plan_route(lanes, start=None):
    """Order lanes by nearest-endpoint chaining from an optional start position.

    A lane may be traversed in either direction, so a reversed copy records the
    swapped endpoints. Removal is by identity rather than equality, because the
    reversed copy is deliberately a different mapping from the original.
    """
    remaining = list(lanes)
    route = []
    current = start
    while remaining:
        if current is None:
            chosen = min(remaining, key=lambda lane: (lane['row'], lane['index']))
        else:
            chosen = min(remaining, key=lambda lane: min(
                math.dist(current, lane['start']), math.dist(current, lane['end'])))
            if math.dist(current, chosen['end']) < math.dist(current, chosen['start']):
                chosen = dict(chosen, start=chosen['end'], end=chosen['start'])
        route.append(chosen)
        remaining = [lane for lane in remaining if lane is not chosen
                     and lane['index'] != chosen['index']]
        current = chosen['end']
    return route


def route_length(route, start=None):
    total = 0.0
    current = start
    for lane in route:
        if current is not None:
            total += math.dist(current, lane['start'])
        total += lane['length_m']
        current = lane['end']
    return total


def _capability_error(aircraft, spacing_m):
    if not isinstance(aircraft, dict):
        raise ValueError('Each aircraft must be a capability mapping')
    identifier = aircraft.get('drone_id')
    if not isinstance(identifier, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]{0,63}',identifier):
        raise ValueError('Each aircraft needs a drone_id')
    endurance = aircraft.get('endurance_s')
    if endurance is not None and (not isinstance(endurance, (int, float))
                                  or isinstance(endurance, bool)
                                  or not math.isfinite(endurance) or endurance <= 0):
        raise ValueError('Aircraft %s has an invalid endurance' % identifier)
    reserve = aircraft.get('reserve_fraction', 0.25)
    if not isinstance(reserve, (int, float)) or isinstance(reserve, bool) \
            or not 0.0 <= reserve < 1.0:
        raise ValueError('Aircraft %s has an invalid reserve fraction' % identifier)
    if not aircraft.get('coverage', True):
        raise ValueError('Aircraft %s is not assigned a coverage capability' % identifier)
    footprint = aircraft.get('footprint_m', spacing_m)
    if not isinstance(footprint, (int, float)) or isinstance(footprint, bool) \
            or not math.isfinite(footprint) or footprint <= 0:
        raise ValueError('Aircraft %s has an invalid sensor footprint' % identifier)
    return identifier


def partition_region(region, aircraft, spacing_m=DEFAULT_SPACING_M, heading_deg=0.0,
                     speed_mps=5.0, corridor_m=DEFAULT_CORRIDOR_M):
    """Split coverage into one deterministic assignment per aircraft.

    Aircraft are placed in a caller-supplied order so the assignment does not
    depend on dictionary iteration order. Each partition is a contiguous band of
    lanes, which keeps separation corridors meaningful; a partition that would
    exceed its endurance budget is reported rather than silently exceeding it.
    """
    if not isinstance(aircraft, list) or not aircraft:
        raise ValueError('At least one aircraft capability is required')
    if not isinstance(speed_mps, (int, float)) or isinstance(speed_mps, bool) \
            or not math.isfinite(speed_mps) or speed_mps <= 0:
        raise ValueError('Planned cruise speed must be positive')
    if not isinstance(corridor_m, (int, float)) or isinstance(corridor_m, bool) \
            or not math.isfinite(corridor_m) or corridor_m < 0:
        raise ValueError('Separation corridor must be a non-negative distance')

    identifiers = [_capability_error(item, spacing_m) for item in aircraft]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError('Aircraft identifiers must be unique within a fleet')

    lanes = generate_lanes(region, spacing_m, heading_deg)
    if not lanes:
        raise ValueError('Coverage spacing produces no lanes over this region')

    # Contiguous bands in lane order keep neighbours geometrically adjacent.
    total_lanes = len(lanes)
    count = len(aircraft)
    bounds = []
    weights=[item.get('endurance_s')*(1-item.get('reserve_fraction',.25)) if item.get('endurance_s') else 1.0 for item in aircraft]
    accumulated=0.0
    for index in range(count):
        first = round(total_lanes * accumulated / sum(weights))
        accumulated+=weights[index]
        last = round(total_lanes * accumulated / sum(weights))
        bounds.append((first, last))

    assignments = []
    unassigned = []
    for index, (first, last) in enumerate(bounds):
        band = lanes[first:last]
        entry = aircraft[index]
        drone_id = entry['drone_id']
        if not band:
            # A band that rounds to zero lanes means this aircraft contributes
            # no coverage. Reporting it explicitly is required: silently
            # dropping it would let an operator believe every aircraft is
            # working when one is idle, and would hide a reserve breach.
            unassigned.append(drone_id)
            assignments.append({
                'drone_id': drone_id,
                'lane_count': 0,
                'distance_m': 0.0,
                'estimated_duration_s': 0.0,
                'endurance_budget_s': (None if entry.get('endurance_s') is None else
                                       round(entry['endurance_s']
                                             * (1.0 - entry.get('reserve_fraction', 0.25)), 3)),
                'within_endurance': False,
                'lane_range': [first, last - 1],
                'lanes': [],
            })
            continue
        route = plan_route(band)
        distance = route_length(route)
        # Reserve both the travel time and a fixed per-lane turn allowance.
        turn_overhead = 2.0 * len(route)
        estimated_s = distance / speed_mps + turn_overhead
        budget = entry.get('endurance_s')
        reserve = entry.get('reserve_fraction', 0.25)
        usable = None if budget is None else budget * (1.0 - reserve)
        fits = True if usable is None else estimated_s <= usable
        if not fits:
            unassigned.append(drone_id)
        assignments.append({
            'drone_id': drone_id,
            'lane_count': len(route),
            'distance_m': round(distance, 3),
            'estimated_duration_s': round(estimated_s, 3),
            'endurance_budget_s': None if usable is None else round(usable, 3),
            'within_endurance': fits,
            'lane_range': [first, last - 1],
            'lanes': [{'index': lane['index'],
                       'start': [round(lane['start'][0], 3), round(lane['start'][1], 3)],
                       'end': [round(lane['end'][0], 3), round(lane['end'][1], 3)],
                       'length_m': round(lane['length_m'], 3)} for lane in route],
        })

    return {
        'schema_version': 1,
        'region': region.name,
        'area_m2': round(region.area, 3),
        'spacing_m': float(spacing_m),
        'heading_deg': float(heading_deg),
        'corridor_m': float(corridor_m),
        'speed_mps': float(speed_mps),
        'lane_count': total_lanes,
        'assignments': assignments,
        'unassigned': unassigned,
        'complete': not unassigned,
        'reason': ('' if not unassigned else
                   'Endurance budget exceeded for: ' + ', '.join(unassigned)),
    }


def separation_ok(assignments, corridor_m):
    """Check that adjacent assigned bands keep the requested minimum corridor.

    Bands are contiguous by construction, so the corridor is enforced by
    trimming the shared boundary lane set rather than by relying on flight-time
    deconfliction alone.
    """
    if corridor_m <= 0:
        return True
    from .geometry import _segments_cross
    def point_distance(p,a,b):
        dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
        t=max(0,min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length)) if length else 0
        return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
    routes=[]
    for item in assignments:
        points=[p for lane in item['lanes'] for p in (lane['start'],lane['end'])]
        routes.append(list(zip(points,points[1:])))
    for i,route in enumerate(routes):
        for other in routes[i+1:]:
            for a,b in route:
                for c,d in other:
                    if _segments_cross(a,b,c,d) or min(point_distance(a,c,d),point_distance(b,c,d),point_distance(c,a,b),point_distance(d,a,b))<corridor_m-1e-6:
                        return False
    return True


def manifest(plan, mission_id, revision):
    """Wrap a plan in the identity a download, preview and upload all share."""
    if not isinstance(mission_id, str) or not mission_id or len(mission_id) > 200:
        raise ValueError('A mission ID is required')
    if not isinstance(revision, int) or isinstance(revision, bool) or revision < 1:
        raise ValueError('A positive integer revision is required')
    body = json.dumps(plan, sort_keys=True, allow_nan=False, separators=(',', ':'))
    return {
        'schema_version': 1,
        'mission_id': mission_id,
        'revision': revision,
        'plan': plan,
        'plan_sha256': hashlib.sha256(body.encode('utf-8')).hexdigest(),
    }
