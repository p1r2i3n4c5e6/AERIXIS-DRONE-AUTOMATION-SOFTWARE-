"""Offline AI planner (2026-11-01): the model's form is clamped and checked against the real drones
before the Plan page may apply it; the prompt carries the fleet, area and parameter limits."""
import unittest

from llm_planner import LocalPlanner, check_plan, footprint_table

CONTEXT = dict(
    mode='simulation',
    fleet=[dict(id='drone0', label='Sim 1', online=True, camera=dict(hfov_deg=65, vfov_deg=42), payload=dict(capacity=0)),
           dict(id='drone2', label='Sim 3', online=True, camera=None, payload=dict(capacity=10))],
    area=dict(loaded=True, name='arena.kml', area_ha=8.4, width_m=296, height_m=284),
    missions=[dict(id='search_deliver', title='Search, geotag & deliver', summary='...', options=['altitude_m', 'speed_mps'])],
    params=dict(altitude_m=dict(label='Altitude above take-off (m)', default=15, min=1, max=50, value=15),
                speed_mps=dict(label='Survey speed (m/s)', default=1.5, min=0.1, max=2, value=2)))


class PlannerChecks(unittest.TestCase):
    def test_values_are_clamped_and_unknown_keys_dropped(self):
        plan, warnings = check_plan(dict(mission='search_deliver', params=dict(speed_mps=5, altitude_m=20, bogus=3),
                                         roles=dict(drone0='search', drone2='delivery'), emergency='RTL'), CONTEXT)
        self.assertEqual(plan['params'], dict(speed_mps=2.0, altitude_m=20.0))
        self.assertTrue(any('outside' in w for w in warnings))

    def test_a_drone_without_packages_cannot_deliver(self):
        plan, warnings = check_plan(dict(mission='search_deliver', params={}, roles=dict(drone0='delivery', drone2='delivery'),
                                         emergency='RTL'), CONTEXT)
        self.assertEqual(plan['roles']['drone0'], 'search')
        self.assertTrue(any('no packages' in w for w in warnings))

    def test_unknown_mission_and_unknown_drone(self):
        plan, warnings = check_plan(dict(mission='fly_to_moon', params={}, roles=dict(drone9='search'), emergency='X'), CONTEXT)
        self.assertEqual(plan['mission'], '')
        self.assertEqual(plan['roles'], {})
        self.assertEqual(plan['emergency'], 'RTL')

    def test_custom_mission_features_are_made_consistent(self):
        plan, warnings = check_plan(dict(mission='custom', custom_name='Bridge watch', params={}, roles={}, emergency='LAND',
                                         features=dict(type='watch', detect=False, geotag=True, orbit=False, drop=False,
                                                       resume=True, finish='RTL', delivery_drones=True)), CONTEXT)
        self.assertTrue(plan['features']['detect'])          # geotag needs the camera
        self.assertFalse(plan['delivery_drones'])            # delivery needs coverage lanes
        self.assertEqual(plan['features']['emergency'], 'LAND')
        self.assertEqual(len(warnings), 2)

    def test_search_deliver_without_a_delivery_drone_becomes_geotag(self):
        plan, warnings = check_plan(dict(mission='search_deliver', params={}, roles=dict(drone0='search'), emergency='RTL'), CONTEXT)
        self.assertEqual(plan['mission'], 'human_geotag_resume')
        self.assertTrue(any('No drone delivers' in w for w in warnings))

    def test_lane_spacing_is_checked_against_the_camera(self):
        ctx = dict(CONTEXT, params=dict(CONTEXT['params'], spacing_m=dict(label='Lane spacing (m)', default=15, min=2, max=200, value=15)))
        _, warnings = check_plan(dict(mission='search_deliver', params=dict(altitude_m=20, spacing_m=17),
                                      roles=dict(drone0='search', drone2='delivery'), emergency='RTL'), ctx)
        self.assertTrue(any('up to 23 m apart' in w for w in warnings))          # 2*20*tan(32.5)/1.1 = 23.3
        _, warnings = check_plan(dict(mission='search_deliver', params=dict(altitude_m=15, spacing_m=30),
                                      roles=dict(drone0='search', drone2='delivery'), emergency='RTL'), ctx)
        self.assertTrue(any('sweep sideways' in w for w in warnings))

    def test_ai_avoidance_defaults_to_the_delivery_drones(self):
        plan, _ = check_plan(dict(mission='search_deliver', params={}, roles=dict(drone0='search', drone2='delivery'),
                                  emergency='RTL'), CONTEXT)
        self.assertEqual(plan['ai_drones'], ['drone2'])
        plan, _ = check_plan(dict(mission='search_deliver', params={}, roles=dict(drone0='search', drone2='delivery'),
                                  emergency='RTL', ai_drones=['drone0', 'drone9']), CONTEXT)
        self.assertEqual(plan['ai_drones'], ['drone0'])           # unknown drone dropped

    def test_state_note_says_whether_the_area_is_loaded(self):
        from llm_planner import state_note
        self.assertIn('search area LOADED (arena.kml', state_note(CONTEXT))
        self.assertIn('NOT loaded', state_note(dict(CONTEXT, area=dict(loaded=False))))

    def test_prompt_carries_fleet_area_limits_and_footprint(self):
        prompt = LocalPlanner.system_prompt(CONTEXT)
        self.assertIn('drone2 "Sim 3": camera none, packages 10', prompt)
        self.assertIn('loaded "arena.kml", 8.4 ha', prompt)
        self.assertIn('speed_mps: Survey speed (m/s), default 1.5, 0.1..2', prompt)
        self.assertIn('15 m -> 17 m', footprint_table(CONTEXT['fleet']))   # 2*15*tan(32.5)/1.1


if __name__ == '__main__':
    unittest.main()
