"""Gimbal-guided rescue manoeuvres driven by the A8 tracker (or its SITL simulator).

The tracker keeps the selected person centred with the gimbal. These operations read the
measured gimbal attitude and move the aircraft:

  TrackTarget       hold position, face the person, report lost (mission 1)
  ApproachOverhead  fly towards the person until the gimbal looks straight down (3, 4)
  OrbitTarget       circle the person at a fixed radius for N laps (mission 2)
  GeotagOverhead    record the person's latitude/longitude while overhead (3, 4)
  DropPayload       one-shot overhead release through the PX4 AUX adapter (4)
  ReleaseTargetLock drop the selection so the search can continue (3, 4)
  SetGimbalSearch   point the gimbal for searching (all)
  ReturnToLaunch    fly back over the take-off point (all)
  SetSearchArea     only people inside the KML area count as detections (1-4)
  ScanArea          hold a station inside the area and sweep yaw slowly (mission 2 watch); in a
                    fleet, move the watch away from a peer's orbit (see watch.py)

Fleet de-confliction: binding a person puts a claim on them (label, start time, estimated
plan-frame x,y), which swarm_agent repeats in this drone's mesh peer frames until the claim is
released (ReleaseTargetLock, CompleteRescue, return/land/recovery, mission change). A person
next to a peer's live claim or a peer's geotag is not a detection here. If two drones claim the
same person within CLAIM_TIE_WINDOW_S of each other, the lower drone ID keeps the person;
otherwise the earlier claim does. The drone that gives way leaves the person: ApproachOverhead
fails (missions 3/4 resume coverage), TrackTarget/OrbitTarget move clear of the person and look
for somebody nobody has claimed (missions 1/2).

Ground is assumed flat at the take-off height. The range to the person comes from
height-above-take-off and gimbal pitch, so slopes, buildings or water level differences
bias the estimate; the overhead check itself (gimbal at nadir, person centred) does not
depend on that estimate.
"""
import math
import time

from as2_rescue_planner.watch import parse as parse_stations
from .safety import finite
from .watch import ARRIVED_M, choose_station, covering, leg_blocker, zones_from

HFOV_DEG = 65.0
VFOV_DEG = 42.0
MAX_RANGE_M = 150.0


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def gimbal_fresh(vision, max_age=0.5):
    gimbal = (vision or {}).get('gimbal') or {}
    return bool(gimbal) and finite(gimbal.get('pitch')) and finite(gimbal.get('yaw')) and \
        finite(gimbal.get('age_s')) and gimbal['age_s'] <= max_age


def locked(vision):
    return bool(vision and vision.get('fresh') and vision.get('locked') and
                not vision.get('suppressed') and vision.get('label'))


def target_estimate(position, aircraft_yaw, gimbal, error, ground_z, max_range=MAX_RANGE_M):
    """Ground position of the centred person: flat ground at ground_z, ENU frame.

    Returns (point, horizontal_distance, height_above_ground) or None when the gimbal
    looks at or above the horizon (range unobservable) or the result is out of range.
    """
    agl = position[2] - ground_z
    if not finite(agl) or agl < 1.0:
        return None
    depression = -float(gimbal['pitch'])
    if depression < 3.0:
        return None
    bearing = wrap(aircraft_yaw - math.radians(float(gimbal['yaw'])))
    horizontal = 0.0 if depression >= 89.5 else agl / math.tan(math.radians(depression))
    if horizontal > max_range:
        return None
    x = position[0] + horizontal * math.cos(bearing)
    y = position[1] + horizontal * math.sin(bearing)
    if depression > 60.0 and error:
        # Near nadir the image error is a ground offset in the camera frame:
        # image top points along the camera bearing, image right is clockwise of it.
        right = float(error[0]) * agl * math.tan(math.radians(HFOV_DEG / 2))
        top = -float(error[1]) * agl * math.tan(math.radians(VFOV_DEG / 2))
        x += right * math.sin(bearing) + top * math.cos(bearing)
        y += -right * math.cos(bearing) + top * math.sin(bearing)
    return [x, y, ground_z], math.hypot(x - position[0], y - position[1]), agl


def overhead(vision, pitch_tolerance_deg=6.0, centre_tolerance=0.1):
    """Gimbal looking straight down with the locked person centred."""
    if not locked(vision) or not gimbal_fresh(vision):
        return False
    gimbal = vision['gimbal']
    return (gimbal['pitch'] <= -(90.0 - pitch_tolerance_deg) and
            abs(vision.get('err_x', 1)) <= centre_tolerance and
            abs(vision.get('err_y', 1)) <= centre_tolerance)


def approach_velocity(position, target, max_speed, gain=0.45, min_speed=0.3):
    dx, dy = target[0] - position[0], target[1] - position[1]
    distance = math.hypot(dx, dy)
    if distance < 0.15:
        return 0.0, 0.0
    speed = min(max_speed, gain * distance)
    if distance > 1.0:
        speed = max(min(min_speed, max_speed), speed)
    return speed * dx / distance, speed * dy / distance


def orbit_velocity(position, centre, radius, speed, gain=0.6, direction=1.0):
    """Counter-clockwise (direction=1) circle with radial correction."""
    rx, ry = position[0] - centre[0], position[1] - centre[1]
    distance = math.hypot(rx, ry)
    if distance < 0.5:
        rx, ry, distance = 1.0, 0.0, 1.0  # Directly above: move out along +x first.
    ux, uy = rx / distance, ry / distance
    error = radius - distance
    radial = max(-speed, min(speed, gain * error))
    tangential = direction * speed * (1.0 if abs(error) < 0.25 * radius + 1.0 else 0.3)
    vx = radial * ux - tangential * uy
    vy = radial * uy + tangential * ux
    norm = math.hypot(vx, vy)
    limit = 1.2 * speed
    if norm > limit:
        vx, vy = vx * limit / norm, vy * limit / norm
    return vx, vy, math.atan2(ry, rx), abs(error)


def parse_area(text, max_vertices=64):
    """Plan-frame polygon 'x,y;x,y;...' (metres) from a tree port."""
    points = []
    for item in str(text or '').split(';'):
        if not item.strip():
            continue
        values = [float(v) for v in item.split(',')]
        if len(values) != 2 or not all(finite(v) and abs(v) < 1e6 for v in values):
            raise ValueError('Search area vertices must be finite x,y metres')
        points.append(values)
    if not 3 <= len(points) <= max_vertices:
        raise ValueError('Search area needs 3..%d vertices' % max_vertices)
    return points


def in_polygon(point, ring):
    inside = False
    x, y = point[0], point[1]
    for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def limit_yaw(current, goal, max_lead=0.35):
    """Yaw reference at most max_lead rad ahead of the aircraft, so turns stay gentle."""
    return wrap(current + max(-max_lead, min(max_lead, wrap(goal - current))))


def hold_altitude(z, reference, gain=0.8, limit=0.5):
    return max(-limit, min(limit, gain * (reference - z)))


def parse_station_pose(text):
    """Planned watch station 'x;y;z' from the ScanArea pose port, or None."""
    try:
        pose = [float(v) for v in str(text or '').split(';')]
    except ValueError:
        return None
    return pose if len(pose) == 3 and all(finite(v) for v in pose) and 1 <= pose[2] <= 50 else None


def fuse_geotag_samples(samples, angle_sigma_deg=None):
    """Robust mean of overhead geotag samples: drop samples far from the median, average the rest.

    samples: dicts with local [x, y, z], latitude, longitude, agl, gps_sigma. Returns the fused
    sample with sigma_m (GNSS sigma, centring/angle error shrinking with sqrt(n), and the spread).
    """
    if not samples:
        raise ValueError('No geotag sample')
    angle_sigma_deg = GEOTAG_ANGLE_SIGMA_DEG if angle_sigma_deg is None else angle_sigma_deg
    xs = sorted(p['local'][0] for p in samples)
    ys = sorted(p['local'][1] for p in samples)
    mx, my = xs[len(xs) // 2], ys[len(ys) // 2]
    residual = [math.hypot(p['local'][0] - mx, p['local'][1] - my) for p in samples]
    typical = sorted(residual)[len(residual) // 2]
    limit = max(1.0, 3.0 * typical)
    kept = [p for p, r in zip(samples, residual) if r <= limit]
    n = len(kept)
    mean = lambda key: sum(p[key] for p in kept) / n
    x = sum(p['local'][0] for p in kept) / n
    y = sum(p['local'][1] for p in kept) / n
    spread = math.sqrt(sum((p['local'][0] - x) ** 2 + (p['local'][1] - y) ** 2 for p in kept) / n)
    agl = mean('agl')
    sigma = math.sqrt(mean('gps_sigma') ** 2 + (agl * math.tan(math.radians(angle_sigma_deg))) ** 2 / n
                      + spread ** 2)
    return dict(local=[x, y, kept[0]['local'][2]], latitude=mean('latitude'), longitude=mean('longitude'),
                agl=agl, sigma_m=sigma, samples=n, rejected=len(samples) - n, spread_m=spread)


def geodetic_offset(latitude, longitude, east, north):
    """Small local ENU offset (metres) applied to a WGS84 position."""
    lat = latitude + north / 111320.0
    lon = longitude + east / (111320.0 * max(0.01, math.cos(math.radians(latitude))))
    return lat, lon


VISION_OPS = ('TrackTarget', 'ApproachOverhead', 'OrbitTarget', 'GeotagOverhead', 'DropPayload',
              'ReleaseTargetLock', 'SetGimbalSearch', 'ReturnToLaunch', 'SetSearchArea', 'ScanArea')
FLIGHT_RECOVERY_OPS = ('ReturnToLaunch', 'ManagedLand')
# Running while the drone searches: a locked person who is excluded is released even then.
SEARCH_OPS = ('ScanArea', 'SurveyWaypoint', 'ResumeCoverage')
# Two claims started this close in time (about one LoRa peer-frame period plus clock skew)
# count as simultaneous; the lower drone ID then keeps the person.
CLAIM_TIE_WINDOW_S = 2.0
# After giving way, look this long for another person before failing the operation.
REACQUIRE_S = 120.0
# Geotag from several overhead samples, not one (2026-10-24): every camera frame while the person
# stays centred at nadir gives an independent estimate (aircraft drift, centring error, gimbal
# jitter); a robust mean of GEOTAG_MIN_SAMPLES over GEOTAG_WINDOW_S averages that noise out, and
# outliers (a frame mid-gust) are dropped. GNSS bias is common to all samples and is not reduced.
# After GEOTAG_TIMEOUT_S the tag is made from whatever samples were collected (at least one).
GEOTAG_WINDOW_S = 1.0
GEOTAG_MIN_SAMPLES = 5
GEOTAG_TIMEOUT_S = 3.0
GEOTAG_ANGLE_SIGMA_DEG = 8.0
# Decisions about who a person is (search area, geotags, peer claims) use the range estimate
# only once the tracker has the person centred: right after a new selection the gimbal still
# points where it searched, and the estimate is where it points, not where the person is.
CENTRED = 0.2
# A person just accepted as a detection is not released by housekeeping for this long, so a
# noisy estimate cannot drop the lock between the detection and the response binding it.
DETECTION_HOLD_S = 5.0
# Mission 2 watch: without a clear station to watch from (holding, re-stationing) for this long
# in total, ScanArea fails and the mission returns home. A peer's 2-lap orbit takes ~2-3 min.
WATCH_HOLD_S = 240.0
# Person response in a fleet (approach, orbit): while a peer is closer than the separation margin
# (or the peers' positions are stale) the drone hovers where it is instead of moving; after this
# long in one hold the response fails and the tree's lost-person branch releases the person.
SEPARATION_HOLD_S = 60.0
# Farther than this from the planned station when the watch starts: the station leg stopped short.
STATION_REACHED_M = 3.0
# route_node must approve a re-station leg within this time, else that station is skipped.
ROUTE_REPLY_S = 3.0
# A preferred station whose leg a peer aircraft is crossing is waited for this long (in total
# since the watch lost its station) before a less preferred one is taken.
PASSING_WAIT_S = 20.0


def claim_wins(peer, mine, me):
    """True when a peer's claim on a person beats this drone's claim on the same person.

    Both drones evaluate the same pair of claim start times (each in its sender's clock), so
    exactly one of them keeps the person even if their clocks disagree.
    """
    if mine is None:
        return True
    dt = float(peer['since']) - float(mine['since'])
    if dt < -CLAIM_TIE_WINDOW_S:
        return True
    if dt > CLAIM_TIE_WINDOW_S:
        return False
    return str(peer['owner']) < str(me)
LONG_OPS = ('SurveyWaypoint', 'ResumeCoverage', 'ReturnToLaunch', 'TrackTarget', 'OrbitTarget',
            'ApproachOverhead', 'ScanArea')


class VisionOps:
    """Tick functions for VISION_OPS. Raise ValueError to fail an operation."""

    def __init__(self, engine):
        self.engine = engine
        # Detections this close to an earlier geotag are treated as the same person. Kept
        # small: people in a flood are often close together, and missing one is worse than
        # re-checking one (GeotagOverhead still refuses a duplicate within 3 m).
        self.exclusion_radius_m = 6.0
        self.release_cooldown = 0.0
        self.search_pitch = -45.0
        # Set by SetSearchArea for this mission: people outside it are not detections.
        self.search_area = None
        # A person within this distance of a peer's live claim is the peer's person.
        self.claim_radius_m = 8.0
        # This drone's claim: dict(label, position [x, y] or None, since, mission_id).
        self.claim = None
        # Last person given way to a peer: dict(label, owner, peer_label, stamp).
        self.yielded = None
        self.detected = None  # (label, time) of the last accepted detection
        # People this mission orbited/tracked to the end, plan-frame [x, y] (peers exclude them).
        self.handled_points = []
        # Return height above home for ReturnToLaunch (rtl.py); 0 = return at the current height.
        self.return_altitude_m = 0.0
        # Mission 2 watch state for rescue/state: dict(state, note, station, planned, stamp).
        self.watch = None
        # Person response held for separation: {op, label, since, note} or None (rescue/state.hold).
        self.hold = None

    # Shared evidence ---------------------------------------------------------------------
    @property
    def ev(self):
        return self.engine.evidence

    def vehicle(self):
        # Same age limit as the readiness check (safety.py): a check that passes never meets a
        # read that finds nothing (SITL 2026-10-23: 'NoneType' object is not subscriptable).
        vehicle = self.ev.get('vehicle', self.engine.safety.limits.vehicle_max_age_s)
        if not vehicle or not vehicle.get('position_valid'):
            raise ValueError('Fresh valid aircraft position required')
        return vehicle

    def vision(self):
        return self.ev.get('vision')

    def ground_z(self):
        home = self.engine.home
        return home['position'][2] if home else 0.0

    def estimate(self, vehicle, vision, centred=False):
        if not locked(vision) or not gimbal_fresh(vision) or not finite(vehicle.get('yaw')):
            return None
        if centred and not (abs(vision.get('err_x', 1.0)) <= CENTRED and abs(vision.get('err_y', 1.0)) <= CENTRED):
            return None
        return target_estimate(vehicle['position'], vehicle['yaw'], vision['gimbal'],
                               (vision.get('err_x', 0.0), vision.get('err_y', 0.0)), self.ground_z())

    def excluded(self, point):
        """Next to an earlier geotag of this drone or of a peer (heard over the mesh)."""
        tags = [g['local'] for g in self.engine.geotags()] + [t['xy'] for t in self.peer_tags()] + self.handled_points
        return any(math.hypot(point[0] - xy[0], point[1] - xy[1]) < self.exclusion_radius_m for xy in tags)

    # Fleet de-confliction ---------------------------------------------------------------
    def swarm_list(self, key):
        swarm = self.ev.get('swarm', 1.0) or {}
        items = swarm.get(key)
        out = []
        for item in items if isinstance(items, list) else []:
            xy = item.get('xy') if isinstance(item, dict) else None
            if isinstance(xy, list) and len(xy) == 2 and all(finite(v) for v in xy):
                out.append(item)
        return out

    def peer_tags(self):
        return self.swarm_list('peer_tags')

    def peer_claims(self):
        return [c for c in self.swarm_list('peer_claims') if finite(c.get('since')) and c.get('owner')]

    def peer_claim_at(self, point, mine=None):
        """The peer claim on the person at point that this drone must give way to, or None."""
        me = getattr(self.engine, 'identity', '')
        for claim in self.peer_claims():
            if math.hypot(point[0] - claim['xy'][0], point[1] - claim['xy'][1]) < self.claim_radius_m \
                    and claim_wins(claim, mine, me):
                return claim
        return None

    def own_claim_for(self, label):
        return self.claim if self.claim and self.claim['label'] == label else None

    def release_claim(self):
        self.claim = None
        self.detected = None

    def claim_state(self):
        """This drone's claim as published in rescue/state (swarm_agent sends it to peers)."""
        if not self.claim or self.claim['mission_id'] != self.engine.owner:
            return None
        claim = dict(self.claim)
        if claim['position'] is not None:
            claim['position'] = [round(v, 1) for v in claim['position']]
        return claim

    def deconflict(self, now):
        """Keep the claim's position current and give the person way if a peer's claim wins."""
        claim = self.claim
        if claim is None:
            return
        if claim['mission_id'] != self.engine.owner:
            self.release_claim()
            return
        vision = self.vision()
        vehicle = self.ev.get('vehicle')
        if locked(vision) and vision['label'] == claim['label'] and vehicle and vehicle.get('position_valid'):
            estimate = self.estimate(vehicle, vision, centred=True)
            if estimate:
                claim['position'] = list(estimate[0][:2])
        if claim['position'] is None:
            return
        winner = self.peer_claim_at(claim['position'], claim)
        if winner:
            self.yielded = dict(label=claim['label'], owner=winner['owner'], peer_label=winner.get('label'), stamp=now)
            if self.engine.incident_label == claim['label']:
                self.engine.incident_label = None
            self.release_claim()

    def gave_way(self, cmd):
        return bool(self.yielded and cmd.get('label') == self.yielded['label'] and not self.own_claim_for(cmd['label']))

    def look_away(self, suppress_s=3.0):
        self.engine.driver.vision_command(dict(cmd='release', suppress_s=suppress_s))
        self.engine.driver.vision_command(dict(cmd='set_attitude', yaw=0.0, pitch=self.search_pitch))

    def reacquire(self, cmd, flags, now):
        """After giving a person way: move clear of them, sweep yaw and take the next person that
        nobody has claimed. Returns True while still searching (this tick is handled)."""
        vehicle = self.vehicle()
        if 'yielded_at' not in cmd:
            winner = self.yielded or {}
            peer = next((c for c in self.peer_claims() if c['owner'] == winner.get('owner')), None)
            cmd['retreat'] = self.retreat_point(cmd, vehicle, peer)
            for key in ('label', 'centre', 'swept', 'last_angle', 'lost_since', 'since', 'last_estimate', 'yaw'):
                cmd.pop(key, None)
            cmd['yielded_at'] = now
            cmd['yaw_ref'] = vehicle['yaw']
            cmd['last'] = now
            self.look_away()
        if self.human_detected():
            self.bind_label(cmd, self.vision())
            for key in ('yielded_at', 'retreat', 'yaw_ref', 'last'):
                cmd.pop(key, None)
            return False
        if now - cmd['yielded_at'] > REACQUIRE_S:
            self.engine.driver.stop()
            raise ValueError('YIELDED: person handled by %s; nobody else found in %.0fs' %
                             ((self.yielded or {}).get('owner', 'a peer'), REACQUIRE_S))
        if abs(wrap(cmd['yaw_ref'] - vehicle['yaw'])) < 0.5:
            cmd['yaw_ref'] = wrap(cmd['yaw_ref'] + math.radians(12.0) * (now - cmd['last']))
        cmd['last'] = now
        vx, vy = approach_velocity(vehicle['position'], cmd['retreat'], min(cmd.get('speed', 1.0), 2.0), min_speed=0.0)
        self.engine.driver.velocity([vx, vy, hold_altitude(vehicle['position'][2], cmd['z_ref'])],
                                    limit_yaw(vehicle['yaw'], cmd['yaw_ref']), vehicle['frame_id'])
        return True

    def retreat_point(self, cmd, vehicle, peer):
        """Where to wait after giving way: where this response started, pushed out of the
        peer's orbit (same radius as ours) plus a separation margin."""
        start = cmd.get('origin') or vehicle['position'][:2]
        if not peer:
            return list(start)
        cx, cy = peer['xy']
        dx, dy = start[0] - cx, start[1] - cy
        distance = math.hypot(dx, dy)
        clear = (cmd.get('radius_m') or 0.0) + 10.0
        if distance >= clear:
            return list(start)
        if distance < 0.5:
            dx, dy = vehicle['position'][0] - cx, vehicle['position'][1] - cy
            distance = math.hypot(dx, dy)
            if distance < 0.5:
                dx, dy, distance = 1.0, 0.0, 1.0
        return [cx + clear * dx / distance, cy + clear * dy / distance]

    def outside(self, point):
        return self.search_area is not None and not in_polygon(point, self.search_area)

    def human_detected(self):
        """New person locked: not already handled, not next to an earlier geotag and, when a
        search area is set, estimated to stand inside it."""
        vision = self.vision()
        if not locked(vision) or vision['label'] in self.engine.handled_labels:
            return False
        vehicle = self.ev.get('vehicle')
        estimate = self.estimate(vehicle, vision, centred=True) if vehicle and vehicle.get('position_valid') else None
        if estimate is None and (self.search_area is not None or self.peer_claims() or self.peer_tags()):
            return False  # cannot tell yet whether this person is inside the area / somebody else's
        if estimate and self.excluded(estimate[0]):
            return False
        if estimate and self.peer_claim_at(estimate[0], self.own_claim_for(vision['label'])):
            return False
        if self.search_area is not None and self.outside(estimate[0]):
            return False
        self.detected = (vision['label'], self.engine.clock())
        return True

    def housekeeping(self, now):
        """De-conflict with peers, and release a lock on a person who is already geotagged or
        claimed by a peer so the search continues."""
        self.deconflict(now)
        active = self.engine.active
        if (active is not None and active['operation'] not in SEARCH_OPS) or now < self.release_cooldown:
            return
        vision = self.vision()
        vehicle = self.ev.get('vehicle')
        if not locked(vision) or not vehicle or not vehicle.get('position_valid'):
            return
        if vision['label'] == self.engine.incident_label:
            return  # the person being geotagged/served right now (drop still to come)
        if self.detected and self.detected[0] == vision['label'] and now - self.detected[1] < DETECTION_HOLD_S:
            return  # just accepted: the response is about to bind this person
        estimate = self.estimate(vehicle, vision, centred=True)
        if vision['label'] in self.engine.handled_labels or (estimate and (
                self.excluded(estimate[0]) or self.outside(estimate[0]) or self.peer_claim_at(estimate[0]))):
            # Drop it and look away: otherwise the gimbal keeps staring at the same person and
            # the tracker re-selects them instead of anyone new below the aircraft.
            self.look_away()
            self.release_cooldown = now + 4.0

    def lost(self, cmd, vision, now):
        """True once the selection has been missing for longer than lost_timeout_s."""
        if locked(vision) and (cmd.get('label') is None or vision['label'] == cmd['label']):
            cmd.pop('lost_since', None)
            return False
        cmd.setdefault('lost_since', now)
        limit = cmd.get('lost_timeout_s') or 3.0
        return now - cmd['lost_since'] > limit

    def bind_label(self, cmd, vision):
        if 'label' not in cmd:
            if not locked(vision):
                raise ValueError('No locked person to work on')
            cmd['label'] = vision['label']
            self.engine.incident_label = vision['label']
            self.detected = None  # the incident label protects the lock from here on
            vehicle = self.ev.get('vehicle')
            valid = bool(vehicle and vehicle.get('position_valid'))
            if valid:
                cmd.setdefault('origin', list(vehicle['position'][:2]))
            estimate = self.estimate(vehicle, vision, centred=True) if valid else None
            # Claim the person for the fleet (renewed by swarm_agent while it is held). Peers keep
            # out of the orbit radius around them plus the separation margin.
            radius = float(cmd.get('radius_m') or 10.0) if cmd.get('operation') == 'OrbitTarget' else 0.0
            self.claim = dict(label=vision['label'], position=list(estimate[0][:2]) if estimate else None,
                              since=self.engine.clock(), mission_id=self.engine.owner, radius_m=radius)
            self.yielded = None

    def separation_hold(self, cmd, flags, now):
        """True while this response must hover because peer separation is not assured.

        Fleet missions only (same rule as the station legs): SeparationSafe and PeersFresh must
        both hold. The drone keeps its height and keeps facing the person; the response's own
        progress (approach, orbit laps) pauses and resumes from where it was."""
        swarm = self.ev.get('swarm', 1.0)
        required = bool(cmd.get('swarm_required') or (swarm and swarm.get('required')))
        if not required or (flags['PeersFresh'] and flags['SeparationSafe']):
            if cmd.pop('sep_hold_since', None) is not None:
                self.hold = None
                cmd.pop('last_angle', None)   # do not count the hold as orbit progress
            return False
        cmd.setdefault('sep_hold_since', now)
        why = 'peer positions stale' if not flags['PeersFresh'] else 'peer closer than the separation margin'
        if now - cmd['sep_hold_since'] > SEPARATION_HOLD_S:
            self.engine.driver.stop()
            self.hold = None
            raise ValueError('SEPARATION_HOLD_TIMEOUT: %s for %.0fs during %s' % (why, SEPARATION_HOLD_S, cmd['operation']))
        self.hold = dict(op=cmd['operation'], label=cmd.get('label'), since=cmd['sep_hold_since'],
                         note='holding %s: %s' % (cmd['operation'], why))
        vehicle = self.vehicle()
        self.engine.driver.velocity([0.0, 0.0, hold_altitude(vehicle['position'][2], cmd['z_ref'])],
                                    vehicle['yaw'], vehicle['frame_id'])
        cmd.pop('overhead_since', None)
        return True

    # Operations -------------------------------------------------------------------------
    def tick(self, cmd, flags, now):
        handler = getattr(self, 'op_' + cmd['operation'])
        return handler(cmd, flags, now)

    def start_motion(self, cmd, flags):
        if not cmd['started']:
            if not flags['VehicleHealthy']:
                raise ValueError('Vehicle readiness lost')
            if not self.engine.motion_ready():
                self.engine.driver.stop()
                return False  # previous motion still winding down; try again next tick
            self.engine.journal.mark_flight_authority(self.engine.owner)
            cmd['started'] = True
            cmd['z_ref'] = self.vehicle()['position'][2]

    def op_TrackTarget(self, cmd, flags, now):
        vision = self.vision()
        if 'yielded_at' not in cmd:
            self.bind_label(cmd, vision)
        if self.start_motion(cmd, flags) is False:
            return None
        if not flags['VehicleHealthy']:
            raise ValueError('Vehicle readiness lost')
        if ('yielded_at' in cmd or self.gave_way(cmd)) and self.reacquire(cmd, flags, now):
            return None
        vision = self.vision()
        if self.lost(cmd, vision, now):
            self.engine.driver.stop()
            raise ValueError('TARGET_LOST: %s not seen for %.1fs' % (cmd['label'], cmd.get('lost_timeout_s') or 3.0))
        vehicle = self.vehicle()
        estimate = self.estimate(vehicle, vision) if locked(vision) else None
        yaw = vehicle['yaw']
        if estimate and estimate[1] > 2.0:
            yaw = math.atan2(estimate[0][1] - vehicle['position'][1], estimate[0][0] - vehicle['position'][0])
        if estimate:
            cmd['last_estimate'] = estimate[0]
        self.engine.driver.velocity([0.0, 0.0, hold_altitude(vehicle['position'][2], cmd['z_ref'])],
                                    limit_yaw(vehicle['yaw'], yaw), vehicle['frame_id'])
        duration = cmd.get('duration_s') or 0.0
        cmd.setdefault('since', now)
        if duration > 0 and now - cmd['since'] >= duration:
            self.engine.driver.stop()
            return self.engine.finish('SUCCESS', 'Tracked %s for %.0fs' % (cmd['label'], duration))
        return None

    def op_ApproachOverhead(self, cmd, flags, now):
        vision = self.vision()
        self.bind_label(cmd, vision)
        if self.start_motion(cmd, flags) is False:
            return None
        if not flags['VehicleHealthy']:
            raise ValueError('Vehicle readiness lost')
        if self.gave_way(cmd):
            self.engine.driver.stop()
            self.look_away()
            raise ValueError('YIELDED: %s is handled by %s' % (cmd['label'], self.yielded['owner']))
        if self.lost(cmd, vision, now):
            self.engine.driver.stop()
            raise ValueError('TARGET_LOST during approach')
        if self.separation_hold(cmd, flags, now):
            return None
        vehicle = self.vehicle()
        frame = vehicle['frame_id']
        vz = hold_altitude(vehicle['position'][2], cmd['z_ref'])
        if not locked(vision) or not gimbal_fresh(vision):
            self.engine.driver.velocity([0.0, 0.0, vz], vehicle['yaw'], frame)
            cmd.pop('overhead_since', None)
            return None
        if overhead(vision):
            self.engine.driver.velocity([0.0, 0.0, vz], cmd.get('yaw', vehicle['yaw']), frame)
            cmd.setdefault('overhead_since', now)
            if now - cmd['overhead_since'] >= (cmd.get('settle_s') or 1.5) and vehicle['speed'] <= 0.35:
                self.engine.driver.stop()
                return self.engine.finish('SUCCESS', 'Overhead %s: gimbal %.1f deg' % (cmd['label'], vision['gimbal']['pitch']))
            return None
        cmd.pop('overhead_since', None)
        estimate = self.estimate(vehicle, vision)
        if estimate is None:
            # Gimbal near the horizon: range is unobservable; turn towards the person and
            # creep forward until the gimbal pitches down enough to estimate it.
            bearing = wrap(vehicle['yaw'] - math.radians(vision['gimbal']['yaw']))
            speed = min(0.5, cmd.get('speed', 1.0))
            self.engine.driver.velocity([speed * math.cos(bearing), speed * math.sin(bearing), vz],
                                        limit_yaw(vehicle['yaw'], bearing), frame)
            return None
        point, distance, _ = estimate
        vx, vy = approach_velocity(vehicle['position'], point, cmd.get('speed', 1.0))
        if distance > 3.0:
            cmd['yaw'] = math.atan2(point[1] - vehicle['position'][1], point[0] - vehicle['position'][0])
        self.engine.driver.velocity([vx, vy, vz], limit_yaw(vehicle['yaw'], cmd.get('yaw', vehicle['yaw'])), frame)
        return None

    def op_OrbitTarget(self, cmd, flags, now):
        vision = self.vision()
        if 'yielded_at' not in cmd:
            self.bind_label(cmd, vision)
        if self.start_motion(cmd, flags) is False:
            return None
        if not flags['VehicleHealthy']:
            raise ValueError('Vehicle readiness lost')
        if ('yielded_at' in cmd or self.gave_way(cmd)) and self.reacquire(cmd, flags, now):
            return None
        vision = self.vision()
        if self.lost(cmd, vision, now):
            self.engine.driver.stop()
            raise ValueError('TARGET_LOST during orbit')
        if self.separation_hold(cmd, flags, now):
            return None
        vehicle = self.vehicle()
        radius = cmd.get('radius_m') or 10.0
        if not 3.0 <= radius <= 100.0:
            raise ValueError('Orbit radius must be 3..100 m')
        estimate = self.estimate(vehicle, vision)
        if estimate:
            point = estimate[0]
            old = cmd.get('centre')
            # Smooth the centre: a person moves slowly; gimbal noise should not.
            cmd['centre'] = point if old is None else [old[0] + 0.15 * (point[0] - old[0]),
                                                       old[1] + 0.15 * (point[1] - old[1]), point[2]]
        if 'centre' not in cmd:
            self.engine.driver.velocity([0.0, 0.0, 0.0], vehicle['yaw'], vehicle['frame_id'])
            return None
        centre = cmd['centre']
        vx, vy, angle, radial_error = orbit_velocity(vehicle['position'], centre, radius,
                                                     min(cmd.get('speed', 1.0), 3.0))
        if radial_error < 0.25 * radius + 1.0:
            if 'last_angle' in cmd:
                cmd['swept'] = cmd.get('swept', 0.0) + abs(wrap(angle - cmd['last_angle']))
            cmd['last_angle'] = angle
        yaw = math.atan2(centre[1] - vehicle['position'][1], centre[0] - vehicle['position'][0])
        self.engine.driver.velocity([vx, vy, hold_altitude(vehicle['position'][2], cmd['z_ref'])],
                                    limit_yaw(vehicle['yaw'], yaw), vehicle['frame_id'])
        laps = cmd.get('laps') or 1.0
        if cmd.get('swept', 0.0) >= laps * 2 * math.pi:
            self.engine.driver.stop()
            return self.engine.finish('SUCCESS', 'Orbited %s %.1f laps at %.0f m' % (cmd['label'], laps, radius))
        return None

    def op_GeotagOverhead(self, cmd, flags, now):
        vision = self.vision()
        good = overhead(vision, pitch_tolerance_deg=8.0, centre_tolerance=0.15)
        samples = cmd.setdefault('geotag_samples', [])
        if 'geotag_t0' not in cmd:
            if not good:
                raise ValueError('Geotag needs the gimbal at nadir with the person centred')
            gps = self.ev.get('gps', 1.0)
            if not gps or not gps.get('valid'):
                raise ValueError('NO_GPS_FIX: geotag needs a fresh GNSS fix')
            cmd.update(geotag_t0=now, geotag_label=vision['label'])
        if good and vision['label'] == cmd['geotag_label']:
            sample = self.geotag_sample(vision)
            if sample:
                samples.append(sample)
        elapsed = now - cmd['geotag_t0']
        if not ((elapsed >= GEOTAG_WINDOW_S and len(samples) >= GEOTAG_MIN_SAMPLES) or
                (elapsed >= GEOTAG_TIMEOUT_S and samples)):
            if elapsed >= GEOTAG_TIMEOUT_S:
                raise ValueError('NO_GPS_FIX: no valid geotag sample in %.0f s' % GEOTAG_TIMEOUT_S)
            return None
        fused = fuse_geotag_samples(samples)
        point, agl, last = fused['local'], fused['agl'], samples[-1]
        duplicate = next((g for g in self.engine.geotags() if math.hypot(point[0] - g['local'][0], point[1] - g['local'][1]) < 3.0), None)
        if duplicate:
            self.engine.target = duplicate['id']
            return self.engine.finish('SUCCESS', 'Same person as geotag %s; not tagged twice' % duplicate['label'])
        label = cmd['geotag_label']
        tag = dict(id='%s-%s-%d' % (self.engine.owner or 'mission', label, len(self.engine.geotags()) + 1),
                   label=label, latitude=fused['latitude'], longitude=fused['longitude'],
                   altitude_msl=last['altitude'], local=point, aircraft=last['aircraft'],
                   height_above_takeoff_m=agl, gimbal_pitch_deg=last['pitch'],
                   centre_error=last['centre_error'], sigma_m=round(fused['sigma_m'], 2),
                   samples=fused['samples'], rejected_samples=fused['rejected'],
                   spread_m=round(fused['spread_m'], 2), window_s=round(elapsed, 2),
                   stamp=now, wall_time=time.time())
        self.engine.add_geotag(tag)
        self.detected = None
        self.engine.target = tag['id']
        self.engine.driver.report(dict(kind='geotag', mission_id=self.engine.owner, **tag))
        return self.engine.finish('SUCCESS', 'Geotag %s at %.7f, %.7f (+-%.1f m, %d samples)' % (
            tag['label'], tag['latitude'], tag['longitude'], tag['sigma_m'], tag['samples']))

    def geotag_sample(self, vision):
        """One overhead estimate: ground point and its WGS84 position from this frame's GNSS fix."""
        gps = self.ev.get('gps', 1.0)
        vehicle = self.vehicle()
        if not gps or not gps.get('valid') or not vehicle.get('position'):
            return None
        estimate = self.estimate(vehicle, vision)
        point, _, agl = estimate if estimate else ([*vehicle['position'][:2], self.ground_z()], 0.0,
                                                   vehicle['position'][2] - self.ground_z())
        east, north = point[0] - vehicle['position'][0], point[1] - vehicle['position'][1]
        latitude, longitude = geodetic_offset(gps['latitude'], gps['longitude'], east, north)
        return dict(local=point, latitude=latitude, longitude=longitude, agl=agl,
                    gps_sigma=gps.get('horizontal_sigma_m', 3.0), altitude=gps.get('altitude'),
                    aircraft=list(vehicle['position']), pitch=vision['gimbal']['pitch'],
                    centre_error=[vision.get('err_x'), vision.get('err_y')])

    def op_DropPayload(self, cmd, flags, now):
        engine = self.engine
        limits = engine.safety.limits
        if not cmd['started']:
            if not limits.overhead_envelope_verified:
                raise ValueError('Overhead drop envelope not verified in the aircraft profile')
            vision = self.vision()
            if not overhead(vision):
                raise ValueError('Drop needs the gimbal at nadir with the person centred')
            vehicle = self.vehicle()
            agl = vehicle['position'][2] - self.ground_z()
            if not limits.min_overhead_agl_m <= agl <= limits.max_overhead_agl_m:
                raise ValueError('Drop height %.1f m outside %.1f..%.1f m' % (agl, limits.min_overhead_agl_m, limits.max_overhead_agl_m))
            if vehicle['speed'] > 0.4 or not flags['VehicleHealthy']:
                raise ValueError('Aircraft not steady enough to release')
            if not flags['PayloadAvailable']:
                raise ValueError('No confirmed loaded payload')
            payload = self.ev.get('payload') or {}
            payload_id = str(payload.get('payload_id') or '')
            tag = engine.geotags()[-1] if engine.geotags() else None
            if not tag or tag['label'] != vision['label']:
                raise ValueError('Geotag this person before dropping')
            engine.target = tag['id']
            transaction = engine.owner + ':' + engine.target + ':' + payload_id
            engine.journal.mark_flight_authority(engine.owner)
            fresh, prior = engine.journal.reserve_release(engine.owner, engine.target, payload_id, transaction)
            engine.last_release = transaction
            cmd['started'] = True
            if not fresh:
                return engine.finish('SUCCESS' if prior['state'] == 'CONFIRMED' else 'FAILURE',
                                     'Existing payload transaction; never re-actuated')
            # Release one beat later: the payload adapter must first see a published supervisor
            # state that authorises this overhead drop (OverheadDropReady).
            cmd['release_at'] = now + 0.15
            cmd['payload_id'] = payload_id
            return None
        if 'release_at' in cmd:
            if now < cmd['release_at']:
                return None
            if not overhead(self.vision()):
                raise ValueError('Lost nadir alignment before release; nothing was actuated')
            self.engine.driver.release(engine.last_release, cmd.pop('payload_id'))
            del cmd['release_at']
        payload = self.ev.get('payload') or {}
        if payload.get('transaction_id') == engine.last_release:
            if payload.get('released') is True:
                engine.journal.release_result(engine.last_release, True)
                return engine.finish('SUCCESS', 'Physical release confirmed')
            if payload.get('operator_only') is True and payload.get('command_sent') is True:
                engine.journal.release_result(engine.last_release, False)
                return engine.finish('SUCCESS', 'COMMAND_SENT_UNVERIFIED: servo commanded, delivery not sensed')
        return None

    def op_ReleaseTargetLock(self, cmd, flags, now):
        label = self.engine.incident_label or (self.vision() or {}).get('label')
        if label:
            self.engine.handled_labels.add(label)
        point = (self.claim or {}).get('position')
        if point and not any(math.hypot(point[0] - g['local'][0], point[1] - g['local'][1]) < 3.0
                             for g in self.engine.geotags()):
            # Orbited/tracked to the end without a geotag (missions 1/2): peers must not take
            # this person as new once the claim is gone.
            self.handled_points = (self.handled_points + [[round(point[0], 1), round(point[1], 1)]])[-20:]
        self.release_claim()
        self.engine.driver.vision_command(dict(cmd='release', suppress_s=cmd.get('suppress_s') or 8.0))
        self.release_cooldown = now + 2.0
        return self.engine.finish('SUCCESS', 'Released %s' % (label or 'selection'))

    def op_SetGimbalSearch(self, cmd, flags, now):
        pitch = cmd.get('pitch_deg')
        pitch = -45.0 if pitch is None else pitch
        if not -90.0 <= pitch <= 25.0:
            raise ValueError('Gimbal pitch must be -90..25 deg')
        self.search_pitch = pitch
        vision = self.vision()
        if not cmd['started']:
            self.engine.driver.vision_command(dict(cmd='set_attitude', yaw=0.0, pitch=pitch))
            cmd['started'] = True
            cmd['since'] = now
        if gimbal_fresh(vision) and abs(vision['gimbal']['pitch'] - pitch) <= 5.0 and \
                abs(vision['gimbal']['yaw']) <= 8.0:
            return self.engine.finish('SUCCESS', 'Gimbal search attitude %.0f deg' % pitch)
        if locked(vision):
            return self.engine.finish('SUCCESS', 'Person already locked; search attitude skipped')
        if now - cmd['since'] > 4.0:
            raise ValueError('Gimbal search attitude not confirmed (tracker in preview or no attitude feedback)')
        return None

    def op_SetSearchArea(self, cmd, flags, now):
        self.search_area = parse_area(cmd.get('area'))
        return self.engine.finish('SUCCESS', 'Detections limited to the %d-vertex search area' % len(self.search_area))

    # Mission 2 watch -----------------------------------------------------------------------
    def keep_out(self):
        """Keep-out zones around peers and their orbits; None when they are not known."""
        swarm = self.ev.get('swarm', 1.0)
        zones = zones_from(swarm)
        if zones is None and not (swarm and swarm.get('required')):
            return []  # single aircraft: nobody to keep out of
        return zones

    def set_watch(self, cmd, state, note):
        self.watch = dict(state=state, note=note, station=[round(v, 1) for v in cmd['station']],
                          planned=[round(v, 1) for v in cmd['planned'][:2]] if cmd.get('planned') else None,
                          stamp=self.engine.clock())

    def station_leg_blocker(self, position, goal):
        """A peer orbit zone covering the planned watch station or the way to it, else None.

        Only orbit zones: the station legs were separated by the planner, and the separation
        gate still guards them against a peer that is just flying past."""
        zones = self.keep_out() or []
        return covering(goal, zones, kinds=('orbit',)) or leg_blocker(position, goal, zones, kinds=('orbit',))

    def station_blocked(self, zone, position, goal):
        """The station leg stopped short of a peer's orbit; ScanArea moves the watch."""
        self.watch = dict(state='stopped short',
                          note='station leg stopped short (%s orbit nearby)' % zone.get('owner', 'peer'),
                          station=[round(v, 1) for v in position[:2]], planned=[round(v, 1) for v in goal[:2]],
                          stamp=self.engine.clock())
        return ('STATION_BLOCKED: %s orbit around %s covers the way to the watch station %.0f,%.0f; '
                'stopped at %.0f,%.0f, the watch moves to a clear station' %
                (zone.get('owner', 'peer'), zone.get('label') or 'a person', goal[0], goal[1], position[0], position[1]))

    def op_ScanArea(self, cmd, flags, now):
        """Hold a station and sweep yaw; the tree pre-empts this when a person is detected.

        In a fleet the tree also gives the planned station (pose) and the stations the watch may
        move to (area). When a peer's orbit covers the station (or the station leg stopped short
        of it) the watch moves to the best clear station over a route-approved leg, or holds
        where it is, sweeping, until one is clear (at most WATCH_HOLD_S in total)."""
        if self.start_motion(cmd, flags) is False:
            return None
        if not flags['VehicleHealthy']:
            raise ValueError('Vehicle readiness lost')
        vehicle = self.vehicle()
        here = vehicle['position'][:2]
        if 'station' not in cmd:
            cmd['station'] = list(here)
            cmd['yaw_ref'] = vehicle['yaw']
            cmd['last'] = now
            cmd['since'] = now
            self.engine.driver.vision_command(dict(cmd='set_attitude', yaw=0.0, pitch=self.search_pitch))
            cmd['planned'] = parse_station_pose(cmd.get('pose'))
            cmd['options'] = parse_stations(cmd.get('area')) if cmd['planned'] else []
            cmd['rejected'] = []
            cmd['legs'] = 0
            if cmd['planned'] and math.dist(here, cmd['planned'][:2]) > STATION_REACHED_M:
                cmd['mode'] = 'choose'
                cmd['unsettled_since'] = now
                self.set_watch(cmd, 'choosing', (self.watch or {}).get('note') or 'not at the planned station')
            else:
                cmd['mode'] = 'watching'
                self.set_watch(cmd, 'watching', 'watching planned station')
        swarm = self.ev.get('swarm', 1.0)
        required = bool(cmd.get('swarm_required') or (swarm and swarm.get('required')))
        zones = self.keep_out()
        known = zones is not None and (not required or flags['PeersFresh'])
        if cmd['mode'] == 'watching' and cmd['planned'] and known:
            zone = covering(cmd['station'], zones, kinds=('orbit',))
            if zone:
                cmd['mode'] = 'choose'
                cmd['unsettled_since'] = now
                self.set_watch(cmd, 'choosing', 'peer orbit nearby (%s)' % zone.get('owner', 'peer'))
        if cmd['mode'] in ('choose', 'holding', 'waiting') and now >= cmd.get('next_choice', 0.0):
            cmd['next_choice'] = now + 1.0
            busy = sorted({c['owner'] for c in self.peer_claims()})
            choice = choose_station(here, cmd['planned'], cmd['options'], zones, busy, cmd['rejected'],
                                    current=cmd['station'] if cmd['mode'] == 'holding' else None,
                                    patient=now - cmd['unsettled_since'] < PASSING_WAIT_S) if known else None
            if choice and choice[0] is None:
                if (self.watch or {}).get('note') != choice[1]:
                    self.set_watch(cmd, 'waiting', choice[1])
                if cmd['mode'] != 'waiting':
                    cmd['station'] = list(here)
                cmd['mode'] = 'waiting'
            elif choice:
                cmd['legs'] += 1
                cmd['leg'] = dict(id='%s:restation:%d' % (cmd['command_id'], cmd['legs']), why=choice[1],
                                  goal=[choice[0][0], choice[0][1], cmd['planned'][2]], since=now)
                cmd['mode'] = 'transit'
                cmd['station'] = list(here)  # hover here until route_node approves the leg
                self.set_watch(cmd, 're-stationing', 'moving watch to %s %.0f,%.0f' % (choice[1], *choice[0]))
            elif cmd['mode'] in ('choose', 'waiting'):
                cmd['mode'] = 'holding'
                cmd['station'] = list(here)
                self.set_watch(cmd, 'holding', 'no clear watch station: holding clear of peer orbit, still searching'
                               if known else 'peer keep-out zones unknown: holding, still searching')
        if cmd['mode'] != 'watching' and now - cmd['unsettled_since'] > WATCH_HOLD_S:
            self.engine.driver.stop()
            raise ValueError('WATCH_BLOCKED: no clear watch station for %.0fs (peer orbit or separation)' % WATCH_HOLD_S)
        rate = math.radians(max(0.0, min(30.0, cmd.get('yaw_rate_dps') or 12.0)))
        # Advance the reference only while the aircraft keeps up, so the sweep never runs away.
        if abs(wrap(cmd['yaw_ref'] - vehicle['yaw'])) < 0.5:
            cmd['yaw_ref'] = wrap(cmd['yaw_ref'] + rate * (now - cmd['last']))
        cmd['last'] = now
        if cmd['mode'] == 'transit' and self.restation_leg(cmd, vehicle, flags, zones, required, now):
            return None
        vx, vy = approach_velocity(vehicle['position'], cmd['station'], 0.5, min_speed=0.0)
        self.engine.driver.velocity([vx, vy, hold_altitude(vehicle['position'][2], cmd['z_ref'])],
                                    limit_yaw(vehicle['yaw'], cmd['yaw_ref']), vehicle['frame_id'])
        duration = cmd.get('duration_s') or 0.0
        if duration > 0 and now - cmd['since'] >= duration:
            self.engine.driver.stop()
            moved = ' (watch re-stationed %d time%s)' % (cmd['legs'], 's' if cmd['legs'] != 1 else '') if cmd['legs'] else ''
            return self.engine.finish('SUCCESS', 'Watched the area for %.0fs; nobody detected%s' % (duration, moved))
        return None

    def restation_leg(self, cmd, vehicle, flags, zones, required, now):
        """Fly the approved leg to a new watch station. True when it commanded motion this tick;
        False to hover at cmd['station'] (waiting for approval, stopped short, or arrived)."""
        leg = cmd['leg']
        goal = leg['goal']
        here = vehicle['position'][:2]
        if required and not (flags['PeersFresh'] and flags['SeparationSafe']):
            # Same rule as every other leg: no motion without fresh peers and separation. A leg
            # already flying fails like SurveyWaypoint; one not yet started is not started.
            if leg.get('moving'):
                self.engine.driver.stop()
                raise ValueError('Peer freshness/separation unavailable')
            zone = dict(owner='peer', kind='separation')
        else:
            zone = leg_blocker(here, goal, zones) if zones is not None else dict(owner='keep-out zones unknown')
        if zone:
            cmd['mode'] = 'choose'
            cmd['station'] = list(here)
            self.set_watch(cmd, 'choosing', 're-station leg stopped short (%s %s nearby)' %
                           (zone.get('owner', 'peer'), 'orbit' if zone.get('kind') == 'orbit' else 'aircraft'))
            return False
        frame = vehicle['frame_id']
        request = dict(mission_id=self.engine.owner, command_id=leg['id'], operation='Restation',
                       segment_id=cmd.get('segment_id') or '')
        self.engine.driver.request_route(request, goal, frame)
        route = self.ev.get('route')
        approved = bool(route and route.get('approved') is True and route.get('path_clear') is True
                        and route.get('mission_id') == self.engine.owner and route.get('command_id') == leg['id']
                        and route.get('target_pose') == goal and route.get('frame_id') == frame)
        if not approved:
            if leg.get('moving'):
                leg.setdefault('unapproved_since', now)
                if now - leg['unapproved_since'] > 1.0:
                    self.engine.driver.stop()
                    raise ValueError('Route approval lost during re-station: ' + str((route or {}).get('reason', 'not approved')))
                return False
            if now - leg['since'] > ROUTE_REPLY_S:
                cmd['rejected'].append(goal[:2])
                cmd['mode'] = 'choose'
                self.set_watch(cmd, 'choosing', 'route not approved to %.0f,%.0f: %s' %
                               (goal[0], goal[1], (route or {}).get('reason', 'no reply')))
            return False
        leg.pop('unapproved_since', None)
        leg['moving'] = True
        if math.dist(here, goal[:2]) < ARRIVED_M:
            cmd['mode'] = 'watching'
            cmd['station'] = list(goal[:2])
            cmd['yaw_ref'] = vehicle['yaw']
            cmd.pop('unsettled_since', None)
            note = 'watching planned station' if leg['why'] == 'planned station' else \
                'watch re-stationed (peer orbit nearby): %s' % leg['why']
            self.set_watch(cmd, 'watching', note)
            return False
        speed = max(0.3, min(cmd.get('speed') or 1.0, 2.0))
        vx, vy = approach_velocity(vehicle['position'], goal, speed)
        heading = math.atan2(goal[1] - here[1], goal[0] - here[0])
        self.engine.driver.velocity([vx, vy, hold_altitude(vehicle['position'][2], goal[2])],
                                    limit_yaw(vehicle['yaw'], heading), frame)
        return True

    def op_ReturnToLaunch(self, cmd, flags, now):
        engine = self.engine
        vehicle = self.vehicle()
        if not cmd['started'] and vehicle and vehicle.get('armed') is False and vehicle.get('speed', 1.0) < 0.5:
            # PX4 says disarmed (e.g. it never took off): there is nothing to fly home. SITL
            # 2026-10-23: with AI on, a return started on the ground waited for ever to "arrive".
            engine.driver.stop()
            return engine.finish('SUCCESS', 'Already on the ground (FC disarmed): no return flight')
        if not flags['FlightAvailable']:
            raise ValueError('Position/connection unavailable for return')
        avoidance = engine.avoidance
        if 'legs' not in cmd and avoidance is not None and avoidance.enabled:
            # AI avoidance on: fly home at the mission height through the avoidance layer (it bends
            # around other drones like on a leg); no climb to a return height.
            home = engine.home or dict(position=[0.0, 0.0, 0.0])
            here = vehicle['position']
            altitude = max(here[2], home['position'][2] + (cmd.get('height') or 0.0))
            cmd['legs'] = [[home['position'][0], home['position'][1], altitude]]
            cmd['pose'] = cmd['legs'][-1]
            cmd['leg_start'] = list(here)
            avoidance.start_leg(cmd)
        if cmd.get('avoid'):
            if not avoidance.enabled:
                # Switched off on the way home: the usual return (own height, then home) from here.
                for key in ('avoid', 'avoid_policy', 'legs', 'leg_start'):
                    cmd.pop(key, None)
                cmd['started'] = False
                engine.driver.stop()
                return None
            if not cmd['started']:
                if not engine.motion_ready():
                    engine.driver.stop()
                    return None
                engine.driver.track(None)
                engine.journal.mark_flight_authority(engine.owner)
                cmd['started'] = True
            gps = engine.evidence.get('gps', 2.0) or {}
            velocity, heading, reached = avoidance.leg_command(
                cmd, cmd['leg_start'], cmd['legs'][-1], min(cmd.get('speed') or 1.0, 2.0), vehicle,
                engine.evidence.get('swarm', 1.5), float(gps.get('horizontal_sigma_m', 2.5)), now, keep_heading=True)
            if reached:
                engine.driver.stop()
                return engine.finish('SUCCESS', 'Over launch position (AI avoidance, mission height)')
            engine.driver.velocity(velocity, limit_yaw(vehicle['yaw'], heading), vehicle['frame_id'])
            return None
        if 'legs' not in cmd:
            home = engine.home or dict(position=[0.0, 0.0, 0.0])
            # This drone's own return height (rtl.py), so drones returning together never share
            # a height; climb in place first, then fly home level. Never descend to return.
            above_home = max(cmd.get('height') or 0.0, self.return_altitude_m)
            altitude = max(vehicle['position'][2], home['position'][2] + above_home)
            here = vehicle['position']
            cmd['legs'] = ([[here[0], here[1], altitude]] if altitude - here[2] > 0.5 else []) + \
                [[home['position'][0], home['position'][1], altitude]]
            cmd['pose'] = cmd['legs'][-1]
        if not cmd['started']:
            if not engine.motion_ready():
                engine.driver.stop()
                return None  # Wait for the previous motion / hover switch to finish.
            engine.driver.track(None)
            engine.journal.mark_flight_authority(engine.owner)
            engine.driver.move(cmd['legs'][0], vehicle['frame_id'], min(cmd.get('speed') or 1.0, 3.0), facing=False)
            cmd['started'] = True
        result = engine.driver.motion_result()
        if result == 'FAILURE':
            raise ValueError('AERIXIS core GoTo to launch position failed')
        if result == 'SUCCESS':
            cmd['legs'].pop(0)
            if cmd['legs']:
                cmd['started'] = False   # climbed: now fly home level
                return None
            return engine.finish('SUCCESS', 'Over launch position')
        return None
