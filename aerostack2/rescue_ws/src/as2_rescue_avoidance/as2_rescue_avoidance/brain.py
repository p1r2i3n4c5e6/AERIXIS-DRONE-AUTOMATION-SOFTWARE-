"""The learned avoidance policy ("brain"): what it sees, what it decides, and the ORCA shield.

Every drone runs the same small network on its own mesh view, decides every DECISION_S seconds and
the ORCA shield (orca.py) checks the result every control step, so whatever the network says, the
executed velocity keeps the separation ORCA guarantees. Inference is NumPy only (no PyTorch on the
Pi 5): two hidden layers of 64, well under a millisecond.

Observation (in the drone's route frame: x along the current leg, y to its left), OBS_SIZE floats:
  own  velocity, preferred velocity, distance from the planned line, remaining route, mission speed
  K nearest peers (newest mesh message, position extrapolated by its age): present flag, relative
       position, relative velocity, radius to keep (separation gate margin + buffer), message age,
       distance; distances are also given in units of that radius, so one network serves 3 m and 10 m
Action (2 numbers in [-1, 1]):
  a0  speed along the route: -1 -> 0.1x, 0 -> 1.0x, +1 -> 1.2x the route's preferred velocity
      (an untrained network, output ~0, flies plain ORCA: training starts from the baseline)
  a1  sideways: up to 0.8 x mission speed, + = to the right
"""
import math

import numpy as np

from . import orca

DECISION_S = 0.5
NEIGHBOURS = 4
OWN_SIZE = 7
PEER_SIZE = 9
OBS_SIZE = OWN_SIZE + NEIGHBOURS * PEER_SIZE
SEE_M = 80.0
MIN_SEPARATION = 10.0       # when a view does not say (GNSS fleet); RTK fleets use 3 m


def neighbours_of(view):
    """ORCA neighbours from the view's mesh messages, as the onboard layer avoids them: extrapolated
    by the message age, radius = the separation gate's margin + buffer (orca.neighbour_for_gate)."""
    own_speed = math.hypot(*view['vel'])
    separation = view.get('min_separation', MIN_SEPARATION)
    return [orca.neighbour_for_gate(n['pos'], n['vel'], n['age'], view['sigma'], own_speed, n['sigma'], separation)
            for n in view['neighbours']]


def adaptive_horizon(view):
    age = max((n['age'] for n in view['neighbours']), default=0.0)
    return min(12.0, 6.0 + 3.0 * age)


def shield(view, desired, neighbours=None):
    """ORCA with right-hand passing on `desired`: the velocity actually flown. With view['area']
    (the planning area's outline) the area's edges are hard limits: near an edge the pass goes to
    the side that stays inside."""
    neighbours = neighbours_of(view) if neighbours is None else neighbours
    if not neighbours:
        return desired
    horizon = adaptive_horizon(view)
    walls = orca.area_walls(view['pos'], view.get('area'), view['max_speed'])
    pref = orca.right_hand_preference(view['pos'], view['vel'], desired, neighbours, horizon)
    return orca.orca_velocity(view['pos'], view['vel'], pref, neighbours, view['max_speed'], horizon, 0.1,
                              walls=walls)


def observe(view, neighbours=None):
    """The network's input for one drone (float32 array of OBS_SIZE)."""
    ux, uy = view['route_dir']
    lx, ly = -uy, ux                              # left of travel
    along = lambda x, y: x * ux + y * uy
    left = lambda x, y: x * lx + y * ly
    vx, vy = view['vel']
    px, py = view['pref']
    obs = np.zeros(OBS_SIZE, dtype=np.float32)
    obs[:OWN_SIZE] = (along(vx, vy) / 3.0, left(vx, vy) / 3.0, along(px, py) / 3.0, left(px, py) / 3.0,
                      max(-2.0, min(2.0, view['cross_track'] / 20.0)), min(view['remaining'], 100.0) / 100.0,
                      view['speed'] / 2.0)
    neighbours = neighbours_of(view) if neighbours is None else neighbours
    own = view['pos']
    rows = []
    for n, raw in zip(neighbours, view['neighbours']):
        rx, ry = n['pos'][0] - own[0], n['pos'][1] - own[1]
        d = math.hypot(rx, ry)
        if d > SEE_M:
            continue
        rvx, rvy = n['vel'][0] - vx, n['vel'][1] - vy
        clearance = max(-1.0, min(4.0, (d - n['radius']) / n['radius']))
        rows.append((d, (1.0, along(rx, ry) / 50.0, left(rx, ry) / 50.0, along(rvx, rvy) / 3.0,
                         left(rvx, rvy) / 3.0, n['radius'] / 20.0, min(raw['age'], 5.0) / 2.0, d / 50.0,
                         clearance)))
    rows.sort(key=lambda r: r[0])
    for k, (_, row) in enumerate(rows[:NEIGHBOURS]):
        obs[OWN_SIZE + k * PEER_SIZE: OWN_SIZE + (k + 1) * PEER_SIZE] = row
    return obs


def desired_velocity(view, action):
    """Velocity the network asks for (before the shield)."""
    a0 = max(-1.0, min(1.0, float(action[0])))
    a1 = max(-1.0, min(1.0, float(action[1])))
    ux, uy = view['route_dir']
    scale = 1.0 + (0.2 * a0 if a0 > 0 else 0.9 * a0)
    side = 0.8 * view['speed'] * a1
    return (view['pref'][0] * scale + uy * side, view['pref'][1] * scale - ux * side)


class Network:
    """Actor mean: tanh MLP with the trained weights (NumPy)."""

    def __init__(self, weights):
        self.layers = [(np.asarray(weights['w%d' % i], np.float32), np.asarray(weights['b%d' % i], np.float32))
                       for i in range(3)]
        if self.layers[0][0].shape[0] != OBS_SIZE or self.layers[-1][0].shape[1] != 2:
            raise ValueError('model expects %d inputs, this brain gives %d (trained for another version)'
                             % (self.layers[0][0].shape[0], OBS_SIZE))

    def __call__(self, obs):
        x = obs
        for k, (w, b) in enumerate(self.layers):
            x = x @ w + b
            x = np.tanh(x)            # hidden layers and the output squash to [-1, 1]
        return x


class LearnedPolicy:
    """Policy callable for sim.run_episode / the onboard layer: decides every DECISION_S per drone,
    the shield runs every call. One instance per episode (it remembers each drone's last decision)."""

    def __init__(self, weights):
        self.net = weights if isinstance(weights, Network) else Network(weights)
        self.memory = {}

    def __call__(self, view):
        neighbours = neighbours_of(view)
        last = self.memory.get(view['id'])
        if last is None or view['t'] - last[0] >= DECISION_S - 1e-6:
            last = (view['t'], self.net(observe(view, neighbours)))
            self.memory[view['id']] = last
        return shield(view, desired_velocity(view, last[1]), neighbours)


def load(path):
    with np.load(path) as data:
        return {k: data[k] for k in data.files}
