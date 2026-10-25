"""Multi-drone encounter simulator for training and evaluating avoidance (laptop only).

Horizontal plane, all drones at the same height (the worst case: avoidance never climbs). The
separation rule and GNSS accuracy are `Rules`: GNSS (10 m) or RTK (3 m). Per drone: velocity command
tracked with an acceleration limit, a speed limit, and a GNSS position estimate that drifts slowly
(Gauss-Markov) around the truth. Drones follow their planned route with a look-ahead along the
current leg, so after any detour they rejoin the planned line. Peers are known only through the
mesh (links.py): late, noisy, sometimes missing.

A policy maps one drone's view to a velocity command:
  view = dict(t, pos, vel, pref, speed, max_speed, sigma, min_separation,
              neighbours=[dict(pos, vel, sigma, age)])
pos/vel are the drone's own estimates; neighbours are the newest mesh messages it received.
"""
from dataclasses import dataclass
import math
import random

from .links import Mesh
from . import orca

DT = 0.1                 # control step (s): the runtime decides at 10 Hz
ACCEL = 2.5              # m/s^2, x500-like
GNSS_TAU = 30.0          # s, correlation time of the GNSS error (it drifts, it does not jump)


@dataclass(frozen=True)
class Rules:
    """The fleet's separation rule and the position accuracy it rests on."""
    name: str
    separation: float      # m, the fleet's minimum separation (swarm gate)
    gnss_sigma: float      # m, 1-sigma horizontal error of each drone's own estimate
    reported_sigma: float  # m, what each drone reports with its position
    collision: float       # m between centres counted as a collision


# Ordinary GNSS: 10 m. RTK on every drone (experts only): 3 m; x500 V2 is ~0.85 m across with
# props, so centres under 1 m count as a collision.
GNSS = Rules('gnss', 10.0, 0.8, 1.0, 2.0)
RTK = Rules('rtk', 3.0, 0.05, 0.1, 1.0)
RULES = {r.name: r for r in (GNSS, RTK)}
GNSS_SIGMA, REPORTED_SIGMA, MIN_SEPARATION, COLLISION = (GNSS.gnss_sigma, GNSS.reported_sigma,
                                                          GNSS.separation, GNSS.collision)
REACHED = 2.0            # m from a waypoint
LOOKAHEAD = 6.0          # m along the leg: rejoins the planned line after a detour


class Drone:
    def __init__(self, route, speed, rng, gnss_sigma=GNSS_SIGMA):
        self.gnss_sigma = gnss_sigma
        self.route = [tuple(p) for p in route]
        self.speed = speed
        self.pos = list(self.route[0])
        self.vel = [0.0, 0.0]
        self.leg = 1 if len(self.route) > 1 else 0
        self.err = [rng.gauss(0.0, gnss_sigma), rng.gauss(0.0, gnss_sigma)]
        self.done = len(self.route) == 1   # a single point: hover there (a blocker)
        self.stopped = False               # stop_gate aborted this drone's mission
        self.path_error = 0.0              # largest distance from the planned line
        self.loop = False

    def estimate(self):
        return (self.pos[0] + self.err[0], self.pos[1] + self.err[1])

    def drift(self, rng):
        a = math.exp(-DT / GNSS_TAU)
        s = self.gnss_sigma * math.sqrt(1.0 - a * a)
        self.err = [a * e + rng.gauss(0.0, s) for e in self.err]

    def preferred(self):
        """Velocity along the route: carrot LOOKAHEAD m ahead on the current leg (own estimate)."""
        if self.done or self.stopped:
            hold = self.route[-1] if self.done else self.estimate()
            p = self.estimate()
            dx, dy = hold[0] - p[0], hold[1] - p[1]
            d = math.hypot(dx, dy)
            k = min(self.speed, d) / d if d > 0.3 else 0.0
            return (dx * k, dy * k)
        p = self.estimate()
        a, b = self.route[self.leg - 1], self.route[self.leg]
        lx, ly = b[0] - a[0], b[1] - a[1]
        length = math.hypot(lx, ly) or 1e-9
        ux, uy = lx / length, ly / length
        along = (p[0] - a[0]) * ux + (p[1] - a[1]) * uy
        target_along = min(length, max(0.0, along) + LOOKAHEAD)
        tx, ty = a[0] + ux * target_along, a[1] + uy * target_along
        dx, dy = tx - p[0], ty - p[1]
        d = math.hypot(dx, dy)
        remaining = math.hypot(b[0] - p[0], b[1] - p[1]) + sum(
            math.dist(self.route[i], self.route[i + 1]) for i in range(self.leg, len(self.route) - 1))
        speed = min(self.speed, max(0.3, 0.6 * remaining)) if not self.loop else self.speed
        return (dx / d * speed, dy / d * speed) if d > 1e-6 else (0.0, 0.0)

    def route_frame(self):
        """(unit direction of the current leg, signed distance from its line (+ = left of it),
        remaining route length), from the drone's own estimate."""
        p = self.estimate()
        if len(self.route) < 2:
            return (1.0, 0.0), 0.0, 0.0
        leg = min(max(self.leg, 1), len(self.route) - 1)
        a, b = self.route[leg - 1], self.route[leg]
        lx, ly = b[0] - a[0], b[1] - a[1]
        length = math.hypot(lx, ly) or 1e-9
        ux, uy = lx / length, ly / length
        cross = ux * (p[1] - a[1]) - uy * (p[0] - a[0])
        remaining = 0.0 if self.done else math.hypot(b[0] - p[0], b[1] - p[1]) + sum(
            math.dist(self.route[i], self.route[i + 1]) for i in range(leg, len(self.route) - 1))
        return (ux, uy), cross, remaining

    def advance_route(self):
        if self.done or self.stopped:
            return
        p = self.estimate()
        if math.dist(p, self.route[self.leg]) < REACHED:
            if self.leg + 1 < len(self.route):
                self.leg += 1
            elif self.loop:
                self.leg = 1
            else:
                self.done = True

    def track_true_path_error(self):
        best = float('inf')
        for i in range(len(self.route) - 1):
            best = min(best, _segment_distance(self.pos, self.route[i], self.route[i + 1]))
        if best < float('inf'):
            self.path_error = max(self.path_error, best)

    def step(self, command, max_speed):
        cx, cy = command
        s = math.hypot(cx, cy)
        if s > max_speed:
            cx, cy = cx / s * max_speed, cy / s * max_speed
        dvx, dvy = cx - self.vel[0], cy - self.vel[1]
        dv = math.hypot(dvx, dvy)
        if dv > ACCEL * DT:
            dvx, dvy = dvx / dv * ACCEL * DT, dvy / dv * ACCEL * DT
        self.vel = [self.vel[0] + dvx, self.vel[1] + dvy]
        self.pos = [self.pos[0] + self.vel[0] * DT, self.pos[1] + self.vel[1] * DT]


def _side(me, other):
    """+1 if `other` is on my left (I pass it on my right), -1 on my right, 0 if I am not moving."""
    speed = math.hypot(*me.vel)
    if speed < 0.3:
        return 0
    cross = me.vel[0] * (other.pos[1] - me.pos[1]) - me.vel[1] * (other.pos[0] - me.pos[0])
    return 1 if cross > 0 else -1


def _segment_distance(p, a, b):
    lx, ly = b[0] - a[0], b[1] - a[1]
    l2 = lx * lx + ly * ly
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * lx + (p[1] - a[1]) * ly) / l2))
    return math.hypot(p[0] - a[0] - t * lx, p[1] - a[1] - t * ly)


# ----------------------------------------------------------------------------- policies
def policy_straight(view):
    """No avoidance: fly the route (the danger baseline)."""
    return view['pref']


def policy_stop_gate(view):
    """Today's logic: a peer inside the separation gate's margin stops this drone's mission (it then
    returns home; here it hovers and counts as an aborted mission)."""
    for n in view['neighbours']:
        margin = orca.gate_margin(view['min_separation'], view['sigma'], math.hypot(*view['vel']), n['sigma'],
                                  math.hypot(*n['vel']), n['age'])
        if math.dist(view['pos'], n['pos']) < margin:
            return None   # abort
    return view['pref']


def gate_neighbours(view):
    """Peers as the onboard layer avoids them (orca.neighbour_for_gate)."""
    own_speed = math.hypot(*view['vel'])
    return [orca.neighbour_for_gate(n['pos'], n['vel'], n['age'], view['sigma'], own_speed, n['sigma'],
                                    view['min_separation']) for n in view['neighbours']]


def adaptive_horizon(neighbours):
    """Look further ahead when peer data is old (LoRa): 6 s at fresh data, up to 12 s."""
    age = max((n['age'] for n in neighbours), default=0.0)
    return min(12.0, 6.0 + 3.0 * age)


def policy_orca(view):
    """ORCA shield with right-hand passing, on the route's preferred velocity."""
    neighbours = gate_neighbours(view)
    if not neighbours:
        return view['pref']
    horizon = adaptive_horizon(view['neighbours'])
    pref = orca.right_hand_preference(view['pos'], view['vel'], view['pref'], neighbours, horizon)
    walls = orca.area_walls(view['pos'], view.get('area'), view['max_speed'])
    return orca.orca_velocity(view['pos'], view['vel'], pref, neighbours, view['max_speed'], horizon, DT, walls=walls)


POLICIES = {'straight': policy_straight, 'stop_gate': policy_stop_gate, 'orca': policy_orca}


# ----------------------------------------------------------------------------- episode
class Episode:
    """One scenario flown step by step: `views()` gives every drone's view of the world for this
    step, `step(commands)` applies one velocity command per drone (None = abort the mission, as the
    stop gate does) and advances DT. `run_episode` drives it with a policy; training drives it too."""

    def __init__(self, scenario, profile, seed, time_limit=None, trace=False, rules=GNSS):
        self.rng = rng = random.Random(seed)
        self.scenario, self.rules = scenario, rules
        self.drones = [Drone(d['route'], d['speed'], rng, rules.gnss_sigma) for d in scenario['drones']]
        for spec, drone in zip(scenario['drones'], self.drones):
            drone.loop = bool(spec.get('loop'))
        self.mesh = Mesh(profile, len(self.drones), rng)
        self.limit = time_limit or scenario.get('time_limit', 300.0)
        self.t = 0.0
        self.min_dist, self.violation_s, self.collisions = float('inf'), 0.0, 0
        self.in_collision = set()
        self.tracks = [[] for _ in self.drones] if trace else None
        self.movers = [d for d in self.drones if not d.loop and len(d.route) > 1]
        self.closest = {}        # pair -> (distance, side of j seen from i, side of i seen from j)
        self.pair_dist = {}      # pair -> true distance after the last step

    @property
    def finished(self):
        # Blockers (orbiting / hovering drones) fly on; the episode ends with the movers.
        return self.t >= self.limit or all(d.done or d.stopped for d in self.movers)

    def views(self):
        t, mesh = self.t, self.mesh
        for i, d in enumerate(self.drones):
            if mesh.due(i, t):
                mesh.send(i, dict(stamp=t, pos=d.estimate(), vel=tuple(d.vel), sigma=self.rules.reported_sigma), t)
        mesh.deliver(t)
        views = []
        for i, d in enumerate(self.drones):
            d.advance_route()
            neighbours = [dict(pos=m['pos'], vel=m['vel'], sigma=m['sigma'], age=t - m['stamp'])
                          for m in mesh.latest[i].values() if t - m['stamp'] < 5.0]
            direction, cross, remaining = d.route_frame()
            views.append(dict(id=i, t=t, pos=d.estimate(), vel=tuple(d.vel), pref=d.preferred(), speed=d.speed,
                              max_speed=min(3.0, max(1.0, 1.5 * d.speed)), sigma=self.rules.reported_sigma,
                              min_separation=self.rules.separation, area=self.scenario.get('area'),
                              neighbours=neighbours, route_dir=direction, cross_track=cross,
                              remaining=remaining, loop=d.loop, done=d.done, stopped=d.stopped))
        return views

    def step(self, commands):
        drones, rng = self.drones, self.rng
        for d, command in zip(drones, commands):
            if d.stopped:
                command = (0.0, 0.0)
            elif command is None:
                d.stopped, command = True, (0.0, 0.0)
            d.step(command, min(3.0, max(1.0, 1.5 * d.speed)))
            d.drift(rng)
            d.track_true_path_error()
        worst = float('inf')
        for i in range(len(drones)):
            for j in range(i + 1, len(drones)):
                dist = math.dist(drones[i].pos, drones[j].pos)
                self.pair_dist[(i, j)] = dist
                worst = min(worst, dist)
                if dist < self.closest.get((i, j), (float('inf'),))[0]:
                    self.closest[(i, j)] = (dist, _side(drones[i], drones[j]), _side(drones[j], drones[i]))
                pair = (i, j)
                if dist < self.rules.collision and pair not in self.in_collision:
                    self.collisions += 1
                    self.in_collision.add(pair)
                elif dist >= self.rules.collision + 1.0:
                    self.in_collision.discard(pair)
        self.min_dist = min(self.min_dist, worst)
        if worst < self.rules.separation:
            self.violation_s += DT
        if self.tracks is not None:
            for k, d in enumerate(drones):
                self.tracks[k].append((round(self.t, 2), round(d.pos[0], 2), round(d.pos[1], 2)))
        self.t += DT

    def result(self):
        # Passing side at closest approach, for pairs that actually met (both moving, within 30 m):
        # +1 = the other drone was on my left, i.e. I passed on the right.
        sides = [side for (dist, a, b) in self.closest.values() if dist < 30.0 for side in (a, b) if side]
        movers = self.movers
        result = dict(collisions=self.collisions, right_passes=sum(1 for x in sides if x > 0), passes=len(sides),
                      min_distance=round(self.min_dist, 2), violation_s=round(self.violation_s, 1),
                      completed=all(d.done for d in movers), aborted=sum(d.stopped for d in self.drones),
                      max_path_error=round(max((d.path_error for d in movers), default=0.0), 2),
                      time=round(self.t, 1), ideal_time=round(self.scenario.get('ideal_time', 0.0), 1))
        if self.tracks is not None:
            result['tracks'] = self.tracks
        return result


def run_episode(scenario, policy, profile, seed, time_limit=None, trace=False, rules=GNSS):
    """Fly one scenario with `policy` for every drone; returns metrics (and true tracks with trace)."""
    episode = Episode(scenario, profile, seed, time_limit, trace, rules)
    while not episode.finished:
        episode.step([None if v['stopped'] else policy(v) for v in episode.views()])
    return episode.result()
