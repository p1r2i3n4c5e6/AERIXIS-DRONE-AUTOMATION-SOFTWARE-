"""Mission 2 watch geometry: keep out of peers' orbits, choose where to watch from.

swarm_agent publishes keep-out zones in swarm/status (``keep_out``, see swarm.keep_out_zones):
circles around each peer and around each peer's claimed person (orbit radius included), each as
wide as the separation gate's margin plus a buffer. A watching drone never plans a leg into a
zone; if it is already inside one (a peer came to it), a leg may only take it farther out.

Stations it may watch from (in this order of preference):
  1. its planned station
  2. the planned station of a peer that is busy with a person (that share is unwatched)
  3. alternates inside its own share, nearest to the planned station first
"""
import math

# A new station must clear every zone by this much more than the zone itself, so a station is
# not left again at once because a claim estimate or a peer sigma moved by a metre.
STATION_CLEARANCE_M = 3.0
ARRIVED_M = 1.0


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def zones_from(swarm):
    """Valid keep-out zones from a swarm/status sample; None when it carries none (unknown)."""
    if not isinstance(swarm, dict) or not isinstance(swarm.get('keep_out'), list):
        return None
    out = []
    for zone in swarm['keep_out']:
        xy = zone.get('xy') if isinstance(zone, dict) else None
        if (isinstance(xy, list) and len(xy) == 2 and all(_finite(v) for v in xy)
                and _finite(zone.get('radius_m')) and zone['radius_m'] > 0):
            out.append(zone)
    return out


def distance_to_segment(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = dx * dx + dy * dy
    t = 0.0 if length == 0 else max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / length))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def covering(point, zones, extra=0.0, kinds=None):
    """The first zone (of the given kinds) that point lies in, widened by extra; else None."""
    for zone in zones:
        if kinds and zone.get('kind') not in kinds:
            continue
        if math.dist(point[:2], zone['xy']) < zone['radius_m'] + extra:
            return zone
    return None


def leg_blocker(start, goal, zones, kinds=None):
    """The zone a straight leg start -> goal would enter (or get deeper into), else None.

    From outside a zone the whole leg must stay outside it; from inside, the leg may only
    move away from its centre (a peer that came close is never approached further).
    """
    for zone in zones:
        if kinds and zone.get('kind') not in kinds:
            continue
        here = math.dist(start[:2], zone['xy'])
        closest = distance_to_segment(zone['xy'], start[:2], goal[:2])
        if closest < min(zone['radius_m'], here - 1e-3):
            return zone
    return None


def choose_station(position, planned, options, zones, busy=(), rejected=(), current=None, patient=False):
    """Where to watch from next: (xy, why) to go there, (None, why) to wait, or None.

    options: [dict(owner=None or drone ID, xy)] from the tree; busy: drone IDs that are
    responding to a person (their own stations are unwatched); rejected: stations route_node
    refused; current: the station held now (not a new choice). With patient=True a preferred
    station whose leg is crossed only by a peer aircraft (not an orbit) is waited for instead
    of falling back to a worse one: the peer is usually just passing.
    """
    candidates = []
    if planned:
        candidates.append((list(planned[:2]), 'planned station'))
    for owner in busy:
        candidates += [(list(o['xy']), 'station of %s (busy with a person)' % owner)
                       for o in options if o['owner'] == owner]
    own = [list(o['xy']) for o in options if o['owner'] is None]
    if planned:
        own.sort(key=lambda xy: math.dist(xy, planned[:2]))
    candidates += [(xy, 'alternate station in own share') for xy in own]
    for xy, why in candidates:
        if current and math.dist(xy, current) < ARRIVED_M:
            continue
        if any(math.dist(xy, r) < 0.05 for r in rejected):
            continue
        if covering(xy, zones, STATION_CLEARANCE_M):
            continue
        zone = leg_blocker(position, xy, zones)
        if zone is None:
            return xy, why
        if patient and leg_blocker(position, xy, zones, kinds=('orbit',)) is None:
            return None, 'waiting for %s to pass before moving to %s' % (zone.get('owner', 'a peer'), why)
    return None
