"""Encounter generators for training and evaluation (metres, ENU, all drones at one height).

Every scenario is timed so the drones meet: without avoidance they would come together.
  head_on     two drones on exactly the same path in opposite directions (the operator's case)
  crossing    two paths crossing at 30..150 deg, both reaching the crossing together
  overtaking  same path, same direction, the faster drone catches the slower one
  blocker     a drone orbits a person (10 m circle) right on another drone's path
  random      3..5 drones, random paths that all pass through the same central area
Small random offsets (start, speed, timing) make every seed a different encounter.
"""
import math
import random

FAMILIES = ('head_on', 'crossing', 'overtaking', 'blocker', 'random')


def _line(start, end, speed):
    return dict(route=[start, end], speed=speed)


def _ideal(drones):
    return max(math.dist(d['route'][0], d['route'][-1]) / d['speed'] for d in drones if len(d['route']) > 1)


def head_on(rng):
    half = rng.uniform(50.0, 90.0)
    lateral = rng.uniform(-1.0, 1.0)            # "exactly the same path" up to GNSS
    sa, sb = rng.uniform(1.0, 2.0), rng.uniform(1.0, 2.0)
    # Start so that both reach the middle at the same moment.
    ta, tb = half / sa, half / sb
    lead = max(ta, tb)
    da = _line((-sa * lead, 0.0), (half + 20.0, 0.0), sa)
    db = _line((sb * lead, lateral), (-half - 20.0, lateral), sb)
    drones = [da, db]
    return dict(family='head_on', drones=drones, ideal_time=_ideal(drones))


def crossing(rng):
    angle = math.radians(rng.uniform(30.0, 150.0))
    meet = rng.uniform(50.0, 80.0)
    sa, sb = rng.uniform(1.0, 2.0), rng.uniform(1.0, 2.0)
    t = meet / min(sa, sb)
    a0 = (-sa * t, 0.0)
    b0 = (-sb * t * math.cos(angle), -sb * t * math.sin(angle))
    da = _line(a0, (-a0[0] * 0.8 + 20.0, 0.0), sa)
    db = _line(b0, (-b0[0] * 0.8 + 20.0 * math.cos(angle), -b0[1] * 0.8 + 20.0 * math.sin(angle)), sb)
    drones = [da, db]
    return dict(family='crossing', drones=drones, ideal_time=_ideal(drones))


def overtaking(rng):
    slow, fast = rng.uniform(0.6, 1.0), rng.uniform(1.6, 2.0)
    gap = rng.uniform(25.0, 40.0)
    length = gap * fast / (fast - slow) + 40.0
    drones = [_line((gap, 0.0), (gap + length * slow / fast + 30.0, 0.0), slow),
              _line((0.0, rng.uniform(-1.0, 1.0)), (length + gap, 0.0), fast)]
    return dict(family='overtaking', drones=drones, ideal_time=_ideal(drones))


def blocker(rng):
    radius = 10.0
    centre = (rng.uniform(-5.0, 5.0), rng.uniform(-6.0, 6.0))
    circle = [(centre[0] + radius * math.cos(k * math.pi / 6), centre[1] + radius * math.sin(k * math.pi / 6))
              for k in range(13)]
    orbit = dict(route=circle, speed=1.0, loop=True)
    speed = rng.uniform(1.0, 2.0)
    mover = _line((-80.0, 0.0), (80.0, 0.0), speed)
    drones = [mover, orbit]
    return dict(family='blocker', drones=drones, ideal_time=_ideal(drones))


def random_fleet(rng):
    count = rng.randint(3, 5)
    drones = []
    for k in range(count):
        heading = rng.uniform(0.0, 2 * math.pi)
        speed = rng.uniform(1.0, 2.0)
        through = (rng.uniform(-15.0, 15.0), rng.uniform(-15.0, 15.0))
        before = rng.uniform(55.0, 75.0)
        start = (through[0] - before * math.cos(heading), through[1] - before * math.sin(heading))
        end = (through[0] + 60.0 * math.cos(heading), through[1] + 60.0 * math.sin(heading))
        drones.append(_line(start, end, speed))
    # Launch points at least 15 m apart (the swarm gate's rule before take-off).
    for i in range(count):
        for j in range(i):
            if math.dist(drones[i]['route'][0], drones[j]['route'][0]) < 15.0:
                return random_fleet(rng)
    return dict(family='random', drones=drones, ideal_time=_ideal(drones))


GENERATORS = {'head_on': head_on, 'crossing': crossing, 'overtaking': overtaking,
              'blocker': blocker, 'random': random_fleet}


def make(family, seed):
    scenario = GENERATORS[family](random.Random(seed))
    scenario['seed'] = seed
    scenario['time_limit'] = scenario['ideal_time'] * 2.5 + 60.0
    return scenario
