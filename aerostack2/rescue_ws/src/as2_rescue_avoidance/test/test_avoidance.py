"""ORCA shield, mesh link model and simulator checks (no ROS needed)."""
import math
import random
import unittest

from as2_rescue_avoidance import links, orca, scenarios, sim

MODEL = __import__('os').path.join(__import__('os').path.dirname(__file__), '..', 'models', 'brain.npz')


def peer(pos, vel, radius=12.0):
    return dict(pos=pos, vel=vel, radius=radius)


class OrcaTests(unittest.TestCase):
    def test_no_neighbour_keeps_the_preferred_velocity(self):
        self.assertEqual(orca.orca_velocity((0, 0), (1, 0), (1.5, 0), [], 3.0, 6.0, 0.1), (1.5, 0))

    def test_speed_limit(self):
        vx, vy = orca.orca_velocity((0, 0), (0, 0), (10.0, 0.0), [], 3.0, 6.0, 0.1)
        self.assertAlmostEqual(math.hypot(vx, vy), 3.0, places=6)

    def test_head_on_turns_right_and_leaves_the_collision_course(self):
        own, other = ((0.0, 0.0), (1.5, 0.0)), ((25.0, 0.0), (-1.5, 0.0))   # 8 s apart
        n = [peer(*other)]
        pref = orca.right_hand_preference(own[0], own[1], (1.5, 0.0), n, 6.0)
        self.assertLess(pref[1], 0.0)                                   # right of +x is -y
        vx, vy = orca.orca_velocity(own[0], own[1], pref, n, 3.0, 6.0, 0.1)
        self.assertLess(vy, 0.0)
        # The other drone, mirrored, also goes to its own right (+y): they pass on the right.
        pref_b = orca.right_hand_preference(other[0], other[1], (-1.5, 0.0), [peer(*own)], 6.0)
        self.assertGreater(orca.orca_velocity(other[0], other[1], pref_b, [peer(*own)], 3.0, 6.0, 0.1)[1], 0.0)
        # With the new velocity the predicted miss distance grows (each drone takes its half).
        rel_vel = (vx - (-1.5), vy - 0.0)
        before = orca.time_to_closest((25.0, 0.0), (3.0, 0.0))[1]
        after = orca.time_to_closest((25.0, 0.0), rel_vel)[1]
        self.assertGreater(after, before)

    def test_far_or_diverging_neighbour_changes_nothing(self):
        self.assertEqual(orca.orca_velocity((0, 0), (1, 0), (1, 0), [peer((-30, 0), (-1, 0))], 3.0, 6.0, 0.1), (1, 0))
        self.assertEqual(orca.right_hand_preference((0, 0), (1, 0), (1, 0), [peer((-30, 0), (-1, 0))], 6.0), (1, 0))

    def test_inside_the_radius_moves_away(self):
        vx, vy = orca.orca_velocity((0, 0), (0, 0), (0, 0), [peer((5.0, 0.0), (0.0, 0.0))], 3.0, 6.0, 0.1)
        self.assertLess(vx, 0.0)

    def test_old_message_is_extrapolated_and_widened(self):
        n = orca.neighbour_from_message((10.0, 0.0), (2.0, 0.0), 1.0, 1.0, 1.0, 10.0, buffer_m=2.0)
        self.assertEqual(n['pos'], (12.0, 0.0))
        self.assertAlmostEqual(n['radius'], 10.0 + 1.0 + 1.0 + 0.5 * 2.0 * 1.0 + 2.0)


    def test_gate_neighbour_uses_the_separation_gate_margin(self):
        # RTK 3 m, both sigmas 0.1, 1.5 m/s each, 0.1 s old: 3 + 0.2 + 3 x (0.1 + 0.2) + 1 = 5.1 m.
        n = orca.neighbour_for_gate((20.0, 0.0), (-1.5, 0.0), 0.1, 0.1, 1.5, 0.1, 3.0)
        self.assertEqual(n['pos'], (19.85, 0.0))
        self.assertAlmostEqual(n['radius'], 3.0 + 0.2 + 3.0 * 0.3 + orca.GATE_BUFFER_M)


class LinkTests(unittest.TestCase):
    def test_newest_message_wins_and_loss_rate(self):
        rng = random.Random(1)
        mesh = links.Mesh(links.LinkProfile('t', 10.0, 0.1, 0.5, 0.25), 2, rng)
        for k in range(2000):
            mesh.send(0, dict(stamp=k * 0.1), k * 0.1)
        mesh.deliver(1e9)
        self.assertEqual(mesh.latest[1][0]['stamp'], max(m['stamp'] for m in [mesh.latest[1][0]]))
        self.assertGreater(mesh.latest[1][0]['stamp'], 199.0)
        sent, delivered = 0, 0
        mesh = links.Mesh(links.LinkProfile('t', 10.0, 0.0, 0.0, 0.25), 2, random.Random(2))
        for k in range(4000):
            mesh.send(0, dict(stamp=k), k)
            sent += 1
            before = len(mesh.queue)
            delivered += before
            mesh.queue.clear()
        self.assertAlmostEqual(delivered / sent, 0.75, delta=0.03)


class SimulatorTests(unittest.TestCase):
    def test_same_seed_same_episode(self):
        sc = scenarios.make('random', 7)
        a = sim.run_episode(sc, sim.policy_orca, links.LORA, 7)
        b = sim.run_episode(sc, sim.policy_orca, links.LORA, 7)
        self.assertEqual(a, b)

    def test_without_avoidance_the_scenarios_really_meet(self):
        for family in scenarios.FAMILIES:
            met = sum(sim.run_episode(scenarios.make(family, s), sim.policy_straight, links.BASE_STATION, s)
                      ['min_distance'] < 10.0 for s in range(10))
            # Paths that cross always meet; a random fleet nearly always (91% in stage 1) and a drone
            # passing an orbit only when the orbiting drone is on its side (81%).
            self.assertGreaterEqual(met, {'random': 7, 'blocker': 6}.get(family, 10), family)

    def test_orca_head_on_same_path_passes_right_and_completes(self):
        for profile in (links.BASE_STATION, links.LORA):
            for seed in range(5):
                r = sim.run_episode(scenarios.make('head_on', seed), sim.policy_orca, profile, seed)
                self.assertEqual(r['collisions'], 0)
                self.assertTrue(r['completed'])
                self.assertGreater(r['min_distance'], 10.0)
                self.assertEqual(r['right_passes'], r['passes'])

    def test_stop_gate_is_todays_behaviour(self):
        r = sim.run_episode(scenarios.make('head_on', 1), sim.policy_stop_gate, links.BASE_STATION, 1)
        self.assertEqual(r['aborted'], 2)
        self.assertFalse(r['completed'])

    def test_head_on_along_the_area_edge_passes_inside_the_area(self):
        # SITL 2026-10-23 (26 x 28 m area): a lane 2.8 m inside the edge, passing on the right
        # meant leaving the area; the route check then failed the leg. With the area as hard ORCA
        # lines the drone whose right is outside slows / goes left and both stay inside.
        from as2_rescue_avoidance import brain
        area = [(-30.0, -10.0), (30.0, -10.0), (30.0, 10.0), (-30.0, 10.0)]
        for seed in range(4):
            scenario = dict(family='edge', drones=[dict(route=[(-28.0, -7.2), (28.0, -7.2)], speed=1.5),
                                                   dict(route=[(28.0, -7.0), (-28.0, -7.0)], speed=1.5)],
                            ideal_time=56.0 / 1.5, time_limit=200.0, area=area)
            for policy in (sim.policy_orca, brain.LearnedPolicy(brain.Network(brain.load(MODEL)))):
                r = sim.run_episode(scenario, policy, links.BASE_STATION, seed, trace=True, rules=sim.RTK)
                self.assertEqual(r['collisions'], 0)
                self.assertTrue(r['completed'], (seed, policy))
                self.assertGreater(r['min_distance'], sim.RTK.separation)
                lowest = min(y for track in r['tracks'] for (_, _, y) in track)
                self.assertGreater(lowest, -10.0 + 0.3, (seed, policy))     # never outside the area

    def test_area_walls(self):
        square = [(0, 0), (20, 0), (20, 20), (0, 20)]
        self.assertEqual(orca.area_walls((30, 5), square, 2.0), [])         # outside: entering, no walls
        walls = orca.area_walls((10, 2.5), square, 2.0)
        self.assertEqual(len(walls), 1)                                     # only the near (south) edge
        # Heading straight out at 2 m/s is cut to (2.5 - 1.5) / 2 = 0.5 m/s towards the edge.
        vx, vy = orca.orca_velocity((10, 2.5), (0, -2), (0, -2), [], 3.0, 6.0, 0.1, walls=walls)
        self.assertAlmostEqual(vy, -0.5, places=6)

    def test_rtk_rules_three_metres(self):
        # Head-on on the same line with RTK: ORCA alone passes right, outside 3 m plus margins, and
        # the detour (width) stays small; today's gate (stop_gate) aborts both missions instead.
        for seed in range(3):
            r = sim.run_episode(scenarios.make('head_on', seed), sim.policy_orca, links.BASE_STATION, seed, rules=sim.RTK)
            self.assertTrue(r['completed'])
            self.assertEqual(r['collisions'], 0)
            self.assertGreater(r['min_distance'], sim.RTK.separation + 1.0)
            self.assertLess(r['max_path_error'], 4.0)
            self.assertEqual(r['right_passes'], r['passes'])
            g = sim.run_episode(scenarios.make('head_on', seed), sim.policy_stop_gate, links.BASE_STATION, seed,
                                rules=sim.RTK)
            self.assertEqual(g['aborted'], 2)
            self.assertGreater(g['min_distance'], sim.RTK.separation)


class BrainTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        from as2_rescue_avoidance import brain
        self.np, self.brain = np, brain
        rng = np.random.default_rng(0)
        self.tiny = {'w0': rng.normal(0, 1e-4, (brain.OBS_SIZE, 64)), 'b0': np.zeros(64),
                     'w1': rng.normal(0, 1e-4, (64, 64)), 'b1': np.zeros(64),
                     'w2': rng.normal(0, 1e-4, (64, 2)), 'b2': np.zeros(2)}

    def view(self):
        return sim.Episode(scenarios.make('head_on', 1), links.BASE_STATION, 1).views()[0]

    def test_observation_is_in_the_route_frame(self):
        v = self.view()
        obs = self.brain.observe(v)
        self.assertEqual(obs.shape, (self.brain.OBS_SIZE,))
        # Turning the whole world (route, peers) does not change what the drone sees.
        def turn(x, y, a=1.0):
            return (x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a))
        w = dict(v, pos=turn(*v['pos']), vel=turn(*v['vel']), pref=turn(*v['pref']), route_dir=turn(*v['route_dir']),
                 neighbours=[dict(n, pos=turn(*n['pos']), vel=turn(*n['vel'])) for n in v['neighbours']])
        self.assertTrue(self.np.allclose(obs, self.brain.observe(w), atol=1e-5))

    def test_action_mapping(self):
        v = dict(self.view(), pref=(1.5, 0.0), route_dir=(1.0, 0.0), speed=1.5)
        self.assertEqual(self.brain.desired_velocity(v, (0.0, 0.0)), (1.5, 0.0))      # neutral = route
        self.assertAlmostEqual(self.brain.desired_velocity(v, (-1.0, 0.0))[0], 0.15)
        self.assertLess(self.brain.desired_velocity(v, (0.0, 1.0))[1], 0.0)           # +a1 = to the right

    def test_untrained_brain_flies_plain_orca(self):
        for family in scenarios.FAMILIES:
            sc = scenarios.make(family, 2)
            a = sim.run_episode(sc, self.brain.LearnedPolicy(self.tiny), links.LORA, 2)
            b = sim.run_episode(sc, sim.policy_orca, links.LORA, 2)
            self.assertEqual(a, b, family)


class TrainedBrainTests(unittest.TestCase):
    """The shipped model (models/brain.npz): safe and right-handed on encounters it never saw."""

    def test_trained_brain_on_unseen_head_on_and_overtaking(self):
        import os
        from as2_rescue_avoidance import brain
        path = os.path.join(os.path.dirname(__file__), '..', 'models', 'brain.npz')
        if not os.path.exists(path):
            self.skipTest('no trained model')
        net = brain.Network(brain.load(path))
        for family in ('head_on', 'overtaking'):
            for seed in range(900_000, 900_005):
                for profile in (links.BASE_STATION, links.LORA):
                    for rules in (sim.RTK, sim.GNSS):              # one brain for 3 m and 10 m
                        r = sim.run_episode(scenarios.make(family, seed), brain.LearnedPolicy(net), profile, seed,
                                            rules=rules)
                        self.assertEqual(r['collisions'], 0)
                        self.assertTrue(r['completed'], (family, seed, profile.name, rules.name))
                        self.assertGreater(r['min_distance'], rules.separation)
                        if family == 'head_on':
                            self.assertEqual(r['right_passes'], r['passes'])


if __name__ == '__main__':
    unittest.main()
