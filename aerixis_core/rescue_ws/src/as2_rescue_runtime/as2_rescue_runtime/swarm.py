"""Single-authority durable responder ownership. Expiry never silently reallocates."""
import json
from pathlib import Path
from as2_mission_manager.manager import atomic_json

class AssignmentAuthority:
    def __init__(self,path,clock):
        self.path=Path(path);self.path.parent.mkdir(parents=True,exist_ok=True,mode=0o700);self.clock=clock
        self.items=json.loads(self.path.read_text()) if self.path.exists() else {}
        # Wall/monotonic clock and old owners are uncertain after a restart.
        for item in self.items.values():
            if item['state']=='OWNED':item['state']='UNRESOLVED'
        self.save()
    def save(self):atomic_json(self.path,self.items)
    def claim(self,mission,target,owner,ttl=5.0):
        if not all(isinstance(v,str) and v and len(v)<=100 for v in (mission,target,owner)):raise ValueError('Invalid assignment identity')
        if not 1<=ttl<=30:raise ValueError('Lease duration outside bounds')
        self.expire();key=json.dumps([mission,target]);item=self.items.get(key)
        if item and (item['state']!='OWNED' or item['owner']!=owner):return False
        self.items[key]=dict(mission=mission,target=target,owner=owner,state='OWNED',expires=self.clock()+ttl)
        self.save();return True
    def expire(self):
        changed=False
        for item in self.items.values():
            if item['state']=='OWNED' and item['expires']<=self.clock():item['state']='UNRESOLVED';changed=True
        if changed:self.save()
    def finish(self,mission,target,owner,stopped):
        key=json.dumps([mission,target]);item=self.items.get(key)
        if not item or item['owner']!=owner or not stopped:return False
        item['state']='COMPLETED';self.save();return True

def separation_ok(own,peers,now,frame_id,min_distance=10,max_age=.5):
    import math
    if not peers:return False
    for p in peers:
        if p.get('frame_id')!=frame_id or not 0<=now-p.get('stamp',float('-inf'))<=max_age:return False
        position=p.get('position',[])
        if len(position)!=3 or not all(math.isfinite(x) for x in position):return False
        sigma=p.get('sigma_m')
        if not isinstance(sigma,(int,float)) or not math.isfinite(sigma) or sigma<0:return False
        if math.dist(own,position)-sigma<min_distance:return False
    return True


# Message delivery + processing time added to every peer's age in the pair margin.
LINK_LATENCY_S = 0.2


def pair_margin(min_separation, own_sigma, own_speed, peer_sigma, peer_speed, age):
    """Distance two drones must keep: the separation, both position uncertainties, and how far
    both may have moved since the peer's message was sent (its real age + link latency)."""
    return min_separation + own_sigma + peer_sigma + (own_speed + peer_speed) * (max(0.0, age) + LINK_LATENCY_S)


def separation_safe(own, peers, now, min_separation, max_age):
    """Every peer fresh (<= max_age), in the same frame, and farther (3-D) than its pair margin.
    Uses each message's real age: with a base-station link (~0.1 s) the margin is close to the
    separation; over LoRa (~1 s) it grows with the speeds."""
    import math
    if not peers:
        return False
    for p in peers:
        age = now - p.get('stamp', float('-inf'))
        if p.get('frame_id') != own.get('frame_id') or not 0 <= age <= max_age:
            return False
        position = p.get('position', [])
        sigma = p.get('sigma_m')
        if len(position) != 3 or not all(math.isfinite(x) for x in position):
            return False
        if not isinstance(sigma, (int, float)) or not math.isfinite(sigma) or sigma < 0:
            return False
        margin = pair_margin(min_separation, own['sigma_m'], own.get('speed', 0.0), sigma, p.get('speed', 0.0), age)
        if math.dist(own['position'], position) < margin:
            return False
    return True


def supervisor_allows_renewal(supervisor, now):
    """A surviving radio process cannot renew authority for a dead flight supervisor."""
    try:
        flags = supervisor['flags']
        return (0 <= now - supervisor['stamp'] <= .5
                and bool(supervisor.get('mission_id'))
                and flags.get('VehicleHealthy') is True
                and flags.get('AbortRequested') is False)
    except (KeyError, TypeError, ValueError):
        return False


# Person de-confliction ---------------------------------------------------------------------
# A drone that starts responding to a person (orbit, track, approach) puts a small claim on
# that person in every peer frame it sends; peers do not respond to a person next to a live
# claim, and treat a peer's geotags like their own. Frames stay small for LoRa: the label,
# the claim start time and the person's plan-frame x,y rounded to 0.1 m.

def _xy(value):
    import math
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return None
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(x) and math.isfinite(y) and abs(x) < 1e6 and abs(y) < 1e6):
        return None
    return [round(x, 1), round(y, 1)]


def person_claim(supervisor, now, max_age=.5):
    """Mesh form of this drone's live person claim, taken from its own rescue/state."""
    try:
        claim = supervisor.get('claim')
        if not claim or not 0 <= now - supervisor['stamp'] <= max_age:
            return None
        if not supervisor.get('mission_id') or claim.get('mission_id') != supervisor['mission_id']:
            return None
        since = float(claim['since'])
        value = dict(label=str(claim['label'])[:16], since=round(since, 2))
        xy = _xy(claim.get('position'))
        if xy:
            value['xy'] = xy
        radius = float(claim.get('radius_m') or 0.0)
        if 0 < radius <= 100:
            value['r'] = int(-(-radius // 1))  # orbit radius, whole metres rounded up
        return value
    except (AttributeError, KeyError, TypeError, ValueError):
        return None


def recent_served(supervisor, count=3):
    """The last few people this drone delivered a package to, as plan-frame x,y (mission 7):
    other delivery drones never serve them again."""
    try:
        served = [_xy(point) for point in list(supervisor.get('served') or [])[-count:]]
    except (AttributeError, TypeError):
        return []
    return [xy for xy in served if xy]


def recent_tags(supervisor, count=4):
    """The last few people this drone's mission has dealt with, as plan-frame x,y (peers
    exclude them): geotags (missions 3/4) and people orbited/tracked to the end (``handled``)."""
    try:
        points = [tag.get('local') for tag in (supervisor.get('geotags') or [])]
        points += list(supervisor.get('handled') or [])
        tags = [_xy(point) for point in points[-count:]]
    except (AttributeError, TypeError):
        return []
    return [xy for xy in tags if xy]


class PeerPeople:
    """Peers' person claims and geotags heard over the mesh.

    A claim lives as long as the peer keeps repeating it: it expires claim_ttl seconds after
    the last peer frame that carried it (frames repeat at 5 Hz over UDP, ~1 Hz over LoRa), and
    is gone at once when a newer frame from that peer comes without it. Geotags are kept for
    tag_ttl seconds after they were last heard, because a peer only repeats its newest few.
    """

    def __init__(self, claim_ttl=3.0, tag_ttl=1800.0, max_tags=50):
        self.claim_ttl = claim_ttl
        self.tag_ttl = tag_ttl
        self.max_tags = max_tags
        self.latest = {}  # owner -> (stamp, claim or None)
        self.tag_points = {}  # owner -> [[x, y, heard], ...]
        self.served_points = {}  # owner -> [[x, y, heard], ...] people that peer delivered to

    def update(self, owner, value, now):
        import math
        try:
            stamp = float(value['stamp'])
        except (KeyError, TypeError, ValueError):
            return
        if not math.isfinite(stamp) or (owner in self.latest and stamp <= self.latest[owner][0]):
            return
        claim = value.get('claim')
        if isinstance(claim, dict):
            try:
                since = float(claim['since'])
                radius = claim.get('r')
                radius = float(radius) if isinstance(radius, (int, float)) and 0 <= radius <= 100 else None
                claim = dict(label=str(claim.get('label', ''))[:16], since=since, xy=_xy(claim.get('xy')), r=radius)
                if not math.isfinite(since):
                    claim = None
            except (KeyError, TypeError, ValueError):
                claim = None
        else:
            claim = None
        self.latest[owner] = (stamp, claim)
        for key, store in (('tags', self.tag_points), ('served', self.served_points)):
            points = store.setdefault(owner, [])
            for tag in (value.get(key) or [])[:8]:
                xy = _xy(tag)
                if not xy:
                    continue
                same = next((p for p in points if math.hypot(p[0] - xy[0], p[1] - xy[1]) < 1.0), None)
                if same:
                    same[2] = now
                else:
                    points.append([xy[0], xy[1], now])
            del points[:-self.max_tags]

    def claims(self, now):
        out = []
        for owner, (stamp, claim) in sorted(self.latest.items()):
            if claim and 0 <= now - stamp <= self.claim_ttl:
                out.append(dict(owner=owner, label=claim['label'], since=claim['since'],
                                xy=claim['xy'], r=claim['r'], age_s=round(now - stamp, 2)))
        return out

    def served(self, now):
        out = []
        for owner, points in sorted(self.served_points.items()):
            points[:] = [p for p in points if now - p[2] <= self.tag_ttl]
            out.extend(dict(owner=owner, xy=[p[0], p[1]]) for p in points)
        return out

    def tags(self, now):
        out = []
        for owner, points in sorted(self.tag_points.items()):
            points[:] = [p for p in points if now - p[2] <= self.tag_ttl]
            out.extend(dict(owner=owner, xy=[p[0], p[1]]) for p in points)
        return out


# ---------------------------------------------------------------- mesh wire form of peer frames
# LoRa channels are slow: a peer frame must fit in ONE radio frame (180 B after zlib), otherwise the
# serial bridge (2 frames/s) can only deliver 1 update/s and peers go stale (max age 1 s).
# velocity: horizontal earth-frame velocity [vx, vy] for in-flight avoidance (as2_rescue_avoidance).
_PEER_KEYS = dict(owner='o', position='p', frame_id='f', stamp='t', sigma_m='s', speed='v', claim='c', tags='g',
                  velocity='w', served='d')
_CLAIM_KEYS = dict(label='l', since='n', xy='x')


def compact_peer(frame):
    """Short-key form sent over the mesh (frame_id 'earth' is implied)."""
    out = {}
    for key, value in frame.items():
        if key == 'frame_id' and value == 'earth':
            continue
        if key == 'claim' and isinstance(value, dict):
            value = {_CLAIM_KEYS.get(k, k): v for k, v in value.items()}
        out[_PEER_KEYS.get(key, key)] = value
    return out


def expand_peer(frame):
    """Inverse of compact_peer; frames already in long form pass through unchanged."""
    if 'o' not in frame:
        return frame
    long_peer = {v: k for k, v in _PEER_KEYS.items()}
    long_claim = {v: k for k, v in _CLAIM_KEYS.items()}
    out = {'frame_id': 'earth'}
    for key, value in frame.items():
        key = long_peer.get(key, key)
        if key == 'claim' and isinstance(value, dict):
            value = {long_claim.get(k, k): v for k, v in value.items()}
        out[key] = value
    return out



# Keep-out zones (mission 2 watch) -----------------------------------------------------------
# A watching drone must not walk into the separation gate. Around each peer, and around each
# peer's claimed person plus its orbit radius, this drone keeps out of a horizontal circle as
# wide as the gate's own margin (minimum separation + both position sigmas + the peer's speed
# over the peer-frame age) plus a buffer. Horizontal distance never exceeds the 3-D distance
# the gate uses, so a point outside a zone is outside the gate's margin.

def keep_out_zones(own, peers, claims, now, minimum_separation, max_age, buffer_m=3.0,
                   default_radius_m=10.0, default_sigma_m=2.5, max_peer_age=3.0):
    """[dict(owner, kind='peer'|'orbit', xy, radius_m[, label]), ...] in own's frame.

    ``peers`` are the last frames heard from each peer, ``claims`` PeerPeople.claims(now).
    A claim without an estimated position is centred on its peer, widened by one orbit
    diameter (the person is within one radius of the orbiting peer).
    """
    import math
    zones = []
    if not own or not math.isfinite(own.get('sigma_m', float('nan'))):
        return zones
    by_owner = {}
    for peer in peers:
        try:
            age = now - float(peer['stamp'])
            xy = _xy(peer['position'])
            sigma = float(peer.get('sigma_m', default_sigma_m))
            speed = float(peer.get('speed', 0.0))
        except (KeyError, TypeError, ValueError):
            continue
        if (peer.get('frame_id') != own.get('frame_id') or not xy or not 0 <= age <= max_peer_age
                or not math.isfinite(sigma) or not math.isfinite(speed)):
            continue
        # The peer may have moved since its last frame: its speed over that age, plus one period.
        margin = minimum_separation + own['sigma_m'] + max(0.0, sigma) + abs(speed) * (age + max_age) + buffer_m
        by_owner[peer['owner']] = (xy, margin)
        zones.append(dict(owner=peer['owner'], kind='peer', xy=xy, radius_m=round(margin, 1)))
    for claim in claims:
        owner = claim.get('owner')
        peer = by_owner.get(owner)
        margin = peer[1] if peer else minimum_separation + own['sigma_m'] + default_sigma_m + buffer_m
        radius = claim.get('r')
        radius = default_radius_m if radius is None else radius
        if claim.get('xy'):
            zones.append(dict(owner=owner, kind='orbit', label=claim.get('label'), xy=list(claim['xy']),
                              radius_m=round(radius + margin, 1)))
        elif peer:
            zones.append(dict(owner=owner, kind='orbit', label=claim.get('label'), xy=list(peer[0]),
                              radius_m=round(2 * radius + margin, 1)))
    return zones
