"""Phase 7 acceptance checks: geometry validation, determinism and constraints.

These are software checks over synthetic geometry. They establish that invalid
input is rejected and that a plan is reproducible; they do not measure terrain,
radio performance or flight behaviour.
"""
import os
import json
import math
import unittest

from as2_rescue_planner.coverage import (DEFAULT_SPACING_M, generate_lanes, manifest,
                                         partition_region, plan_route, route_length,
                                         separation_ok)
from as2_rescue_planner.geometry import (PlannerRegion, Projection, self_intersects,
                                         normalize_ring_winding, signed_area)
from as2_rescue_planner.kml import altitude_semantics, parse_kml

SQUARE = [(0.0, 0.0), (500.0, 0.0), (500.0, 300.0), (0.0, 300.0)]


def kml_document(placemarks):
    body = '\n'.join(placemarks)
    return ('<?xml version="1.0" encoding="UTF-8"?>'
            '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>%s</Document></kml>'
            % body)


def polygon_placemark(name, ring, holes=(), description=''):
    def ring_xml(coordinates):
        return '<LinearRing><coordinates>%s</coordinates></LinearRing>' % ' '.join(
            '%f,%f,%f' % (longitude, latitude, altitude) for latitude, longitude, altitude in coordinates)

    inner = ''.join('<innerBoundaryIs>%s</innerBoundaryIs>' % ring_xml(hole) for hole in holes)
    return ('<Placemark><name>%s</name>%s<Polygon><outerBoundaryIs>%s</outerBoundaryIs>%s'
            '</Polygon></Placemark>' % (name, description, ring_xml(ring), inner))


# A closed ring near the equator; 0.001 degrees is roughly 111 m.
KML_RING = [(0.0, 0.0, 0.0), (0.0, 0.001, 0.0), (0.001, 0.001, 0.0), (0.001, 0.0, 0.0)]


class GeometryTests(unittest.TestCase):
    def test_area_and_winding(self):
        ring = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        self.assertAlmostEqual(signed_area(ring), 100.0)
        self.assertAlmostEqual(signed_area(list(reversed(ring))), -100.0)
        restored = normalize_ring_winding(list(reversed(ring)))
        self.assertAlmostEqual(signed_area(restored), 100.0)

    def test_self_intersection_detected(self):
        bow = [(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)]
        self.assertTrue(self_intersects(bow))
        self.assertFalse(self_intersects(list(SQUARE)))

    def test_degenerate_rings_rejected(self):
        for label, ring in [('two points', [(0.0, 0.0), (1.0, 1.0)]),
                            ('duplicate points', [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]),
                            ('zero area line', [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)])]:
            with self.subTest(label):
                with self.assertRaises(ValueError):
                    PlannerRegion(ring)

    def test_non_finite_vertices_rejected(self):
        for bad in (float('nan'), float('inf'), float('-inf')):
            with self.subTest(bad):
                with self.assertRaises(ValueError):
                    PlannerRegion([(0.0, 0.0), (10.0, 0.0), (bad, 10.0)])

    def test_bowtie_ring_rejected(self):
        with self.assertRaises(ValueError):
            PlannerRegion([(0.0, 0.0), (10.0, 10.0), (10.0, 0.0), (0.0, 10.0)])

    def test_hole_outside_shell_rejected(self):
        with self.assertRaises(ValueError):
            PlannerRegion(list(SQUARE), [[(900.0, 900.0), (950.0, 900.0), (950.0, 950.0)]])

    def test_overlapping_holes_rejected(self):
        with self.assertRaises(ValueError):
            PlannerRegion(list(SQUARE), [[(100.0, 100.0), (300.0, 100.0), (300.0, 200.0)],
                                         [(200.0, 150.0), (400.0, 150.0), (400.0, 250.0)]])

    def test_hole_and_exclusion_reduce_area_and_containment(self):
        # Both rings are 100x100 squares, so each removes exactly 10000 m2.
        region = PlannerRegion(list(SQUARE),
                               [[(100.0, 100.0), (200.0, 100.0), (200.0, 200.0), (100.0, 200.0)]],
                               [[(300.0, 100.0), (400.0, 100.0), (400.0, 200.0), (300.0, 200.0)]])
        self.assertAlmostEqual(region.area, 150000.0 - 10000.0 - 10000.0)
        self.assertTrue(region.contains((50.0, 50.0)))
        self.assertFalse(region.contains((150.0, 150.0)), 'hole must be excluded')
        self.assertFalse(region.contains((350.0, 150.0)), 'exclusion must be excluded')
        self.assertFalse(region.contains((900.0, 900.0)), 'outside is never inside')

    def test_projection_round_trip_and_scale(self):
        projection = Projection(0.0, 0.0)
        east, north = projection.to_enu(0.0, 0.001)
        self.assertAlmostEqual(east, 111.3, delta=1.0)
        self.assertAlmostEqual(north, 0.0, delta=1e-6)
        latitude, longitude = projection.to_latlon(east, north)
        self.assertAlmostEqual(latitude, 0.0, delta=1e-9)
        self.assertAlmostEqual(longitude, 0.001, delta=1e-9)

    def test_projection_rejects_invalid_origin(self):
        for latitude, longitude in [(91.0, 0.0), (0.0, 181.0), (float('nan'), 0.0)]:
            with self.subTest((latitude, longitude)):
                with self.assertRaises(ValueError):
                    Projection(latitude, longitude)


class KmlTests(unittest.TestCase):
    def test_valid_polygon_parses(self):
        plan = parse_kml(kml_document([polygon_placemark('field', KML_RING)]))
        self.assertEqual(len(plan.areas), 1)
        self.assertEqual(plan.areas[0].name, 'field')
        # A repeated closing coordinate is dropped, leaving four corners.
        self.assertEqual(len(plan.areas[0].shell), 4)

    def test_closing_coordinate_is_tolerated(self):
        closed = KML_RING + [KML_RING[0]]
        plan = parse_kml(kml_document([polygon_placemark('closed', closed)]))
        self.assertEqual(len(plan.areas[0].shell), 4)

    def test_malformed_xml_rejected(self):
        with self.assertRaises(ValueError):
            parse_kml('<kml><unclosed>')

    def test_no_placemarks_rejected(self):
        with self.assertRaises(ValueError):
            parse_kml(kml_document([]))

    def test_point_markers_accepted_alongside_area(self):
        """Operator KML commonly mixes the survey polygon with Point waypoints."""
        point = '<Placemark><name>marker</name><Point><coordinates>0,0,0</coordinates></Point></Placemark>'
        plan = parse_kml(kml_document([polygon_placemark('field', KML_RING), point]))
        self.assertEqual(len(plan.areas), 1)
        self.assertEqual(len(plan.points), 1)
        self.assertEqual(plan.points[0]['name'], 'marker')

    def test_unsupported_geometry_rejected(self):
        """Model is still refused: ignoring it could hide intent. A LineString is a drawn route
        (never ignored either: a plan whose routes do not name every aircraft is refused)."""
        line = '<Placemark><name>drone0</name><LineString><coordinates>0,0,0 0.001,0.001,0</coordinates></LineString></Placemark>'
        self.assertEqual(sorted(parse_kml(kml_document([polygon_placemark('field', KML_RING), line])).routes), ['drone0'])
        for tag, body in [('Model', '<Location><longitude>0</longitude></Location>')]:
            with self.subTest(tag):
                placemark = '<Placemark><name>x</name><%s>%s</%s></Placemark>' % (tag, body, tag)
                with self.assertRaises(ValueError):
                    parse_kml(kml_document([polygon_placemark('field', KML_RING), placemark]))

    def test_point_only_document_rejected(self):
        point = '<Placemark><name>marker</name><Point><coordinates>0,0,0</coordinates></Point></Placemark>'
        with self.assertRaises(ValueError):
            parse_kml(kml_document([point]))

    def test_non_numeric_and_out_of_range_coordinates_rejected(self):
        bad_number = '<Placemark><name>x</name><Polygon><outerBoundaryIs><LinearRing>' \
                     '<coordinates>a,b,c 1,2,3 4,5,6</coordinates></LinearRing></outerBoundaryIs>' \
                     '</Polygon></Placemark>'
        with self.assertRaises(ValueError):
            parse_kml(kml_document([bad_number]))
        out_of_range = [(0.0, 0.0, 0.0), (0.0, 195.0, 0.0), (0.001, 0.001, 0.0)]
        with self.assertRaises(ValueError):
            parse_kml(kml_document([polygon_placemark('bad', out_of_range)]))
        bad_latitude = [(0.0, 0.0, 0.0), (95.0, 0.001, 0.0), (0.001, 0.001, 0.0)]
        with self.assertRaises(ValueError):
            parse_kml(kml_document([polygon_placemark('bad', bad_latitude)]))

    def test_ring_with_too_few_points_rejected(self):
        with self.assertRaises(ValueError):
            parse_kml(kml_document([polygon_placemark('thin', [(0.0, 0.0, 0.0), (0.0, 0.001, 0.0)])]))

    def test_exclusion_detected_by_description(self):
        document = kml_document([
            polygon_placemark('field', KML_RING),
            polygon_placemark('no-fly', KML_RING, description='<description>exclusion zone</description>'),
        ])
        plan = parse_kml(document)
        self.assertEqual(len(plan.areas), 1)
        self.assertEqual(len(plan.exclusions), 1)
        self.assertEqual(plan.exclusions[0].name, 'no-fly')

    def test_hole_parsed_and_reduces_plan_area(self):
        hole = [(0.0002, 0.0002, 0.0), (0.0002, 0.0008, 0.0),
                (0.0008, 0.0008, 0.0), (0.0008, 0.0002, 0.0)]
        plan = parse_kml(kml_document([polygon_placemark('field', KML_RING, [hole])]))
        region = plan.region(plan.areas[0])
        self.assertEqual(len(plan.areas[0].holes), 1)
        self.assertLess(region.area, 12000.0)
        self.assertGreater(region.area, 0.0)

    def test_projection_origin_is_document_centroid(self):
        plan = parse_kml(kml_document([polygon_placemark('field', KML_RING)]))
        self.assertAlmostEqual(plan.projection.latitude, 0.0005, delta=1e-6)
        self.assertAlmostEqual(plan.projection.longitude, 0.0005, delta=1e-6)

    def test_altitude_semantics_refuse_to_assume_absolute(self):
        plan = parse_kml(kml_document([polygon_placemark('field', KML_RING)]))
        semantics = altitude_semantics(plan.areas[0])
        self.assertEqual(semantics['mode'], 'relative')
        self.assertFalse(semantics['verified'])
        self.assertIsNone(semantics['flight_altitude_m'])

    def test_altitude_spread_rejected(self):
        sloped = [(0.0, 0.0, 0.0), (0.0, 0.001, 50.0), (0.001, 0.001, 0.0), (0.001, 0.0, 0.0)]
        plan = parse_kml(kml_document([polygon_placemark('slope', sloped)]))
        with self.assertRaises(ValueError):
            altitude_semantics(plan.areas[0])

    def test_amsl_altitude_needs_reference_for_relative(self):
        elevated = [(0.0, 0.0, 120.0), (0.0, 0.001, 120.0),
                    (0.001, 0.001, 120.0), (0.001, 0.0, 120.0)]
        plan = parse_kml(kml_document([polygon_placemark('ridge', elevated)]))
        semantics = altitude_semantics(plan.areas[0])
        self.assertEqual(semantics['mode'], 'amsl')
        with_reference = altitude_semantics(plan.areas[0], 100.0)
        self.assertAlmostEqual(with_reference['flight_altitude_m'], 20.0)


class CoverageTests(unittest.TestCase):
    def setUp(self):
        self.region = PlannerRegion(list(SQUARE), name='square')
        self.fleet = [{'drone_id': 'drone0', 'endurance_s': 900.0},
                      {'drone_id': 'drone1', 'endurance_s': 900.0}]

    def test_lanes_cover_and_lie_inside(self):
        lanes = generate_lanes(self.region, 50.0)
        self.assertEqual(len(lanes), 6)
        for lane in lanes:
            self.assertGreater(lane['length_m'], 0.0)
            midpoint = ((lane['start'][0] + lane['end'][0]) / 2.0,
                        (lane['start'][1] + lane['end'][1]) / 2.0)
            self.assertTrue(self.region.contains(midpoint))

    def test_lane_length_matches_endpoints(self):
        for lane in generate_lanes(self.region, 50.0):
            self.assertAlmostEqual(math.dist(lane['start'], lane['end']),
                                   lane['length_m'], places=6)

    def test_alternating_rows_reverse_direction(self):
        lanes = generate_lanes(self.region, 50.0)
        first = lanes[0]
        second = lanes[1]
        self.assertAlmostEqual(first['start'][0], 0.0, places=6)
        self.assertAlmostEqual(second['start'][0], 500.0, places=6)

    def test_invalid_spacing_rejected(self):
        for spacing in (0.0, -5.0, 0.5, 5000.0, float('nan')):
            with self.subTest(spacing):
                with self.assertRaises(ValueError):
                    generate_lanes(self.region, spacing)

    def test_route_visits_every_lane_once(self):
        lanes = generate_lanes(self.region, 50.0)
        route = plan_route(lanes)
        self.assertEqual(len(route), len(lanes))
        self.assertEqual({lane['index'] for lane in route}, {lane['index'] for lane in lanes})
        self.assertGreater(route_length(route), 0.0)

    def test_route_starts_near_given_position(self):
        lanes = generate_lanes(self.region, 50.0)
        route = plan_route(lanes, start=(490.0, 25.0))
        self.assertLess(math.dist((490.0, 25.0), route[0]['start']), 20.0)

    def test_hole_removes_coverage(self):
        holed = PlannerRegion(list(SQUARE),
                              [[(100.0, 100.0), (400.0, 100.0), (400.0, 200.0), (100.0, 200.0)]])
        for lane in generate_lanes(holed, 50.0):
            midpoint = ((lane['start'][0] + lane['end'][0]) / 2.0,
                        (lane['start'][1] + lane['end'][1]) / 2.0)
            self.assertTrue(holed.contains(midpoint))

    def test_exclusion_removes_coverage(self):
        excluded = PlannerRegion(list(SQUARE),
                                 exclusions=[[(0.0, 0.0), (250.0, 0.0), (250.0, 300.0), (0.0, 300.0)]])
        lanes = generate_lanes(excluded, 50.0)
        for lane in lanes:
            self.assertGreaterEqual(lane['start'][0], 249.0)

    def test_partition_is_deterministic(self):
        first = partition_region(self.region, self.fleet, 50.0)
        second = partition_region(self.region, self.fleet, 50.0)
        self.assertEqual(json.dumps(first, sort_keys=True), json.dumps(second, sort_keys=True))
        self.assertEqual(first['plan'] if 'plan' in first else first, second)

    def test_all_lanes_assigned_exactly_once(self):
        plan = partition_region(self.region, self.fleet, 50.0)
        assigned = [lane['index'] for item in plan['assignments'] for lane in item['lanes']]
        self.assertEqual(sorted(assigned), list(range(plan['lane_count'])))

    def test_workload_is_balanced_across_equal_aircraft(self):
        plan = partition_region(self.region, self.fleet, 50.0)
        distances = [item['distance_m'] for item in plan['assignments']]
        self.assertLessEqual(max(distances) - min(distances), 0.5 * min(distances))

    def test_assignments_keep_separation_corridor(self):
        plan = partition_region(self.region, self.fleet, 50.0)
        self.assertTrue(separation_ok(plan['assignments'], 5.0))

    def test_endurance_reserve_is_enforced_and_reported(self):
        tight = [{'drone_id': 'drone0', 'endurance_s': 10.0},
                 {'drone_id': 'drone1', 'endurance_s': 900.0}]
        plan = partition_region(self.region, tight, 50.0)
        self.assertFalse(plan['complete'])
        self.assertEqual(plan['unassigned'], ['drone0'])
        self.assertIn('drone0', plan['reason'])
        self.assertFalse([item for item in plan['assignments']
                          if item['drone_id'] == 'drone0'][0]['within_endurance'])

    def test_reserve_fraction_reduces_usable_budget(self):
        fleet = [{'drone_id': 'drone0', 'endurance_s': 400.0, 'reserve_fraction': 0.0}]
        generous = partition_region(self.region, fleet, 50.0)
        fleet = [{'drone_id': 'drone0', 'endurance_s': 400.0, 'reserve_fraction': 0.75}]
        reserved = partition_region(self.region, fleet, 50.0)
        self.assertGreater(generous['assignments'][0]['endurance_budget_s'],
                           reserved['assignments'][0]['endurance_budget_s'])

    def test_incapable_and_duplicate_aircraft_rejected(self):
        with self.assertRaises(ValueError):
            partition_region(self.region, [{'drone_id': 'd0', 'coverage': False}], 50.0)
        with self.assertRaises(ValueError):
            partition_region(self.region, [{'drone_id': 'd0', 'endurance_s': -1.0}], 50.0)
        with self.assertRaises(ValueError):
            partition_region(self.region, [{'drone_id': 'd0'}], -1.0)
        with self.assertRaises(ValueError):
            partition_region(self.region, [{'drone_id': 'd0'}, {'drone_id': 'd0'}], 50.0)
        with self.assertRaises(ValueError):
            partition_region(self.region, [], 50.0)

    def test_heading_changes_lane_layout_but_stays_inside(self):
        plan = partition_region(self.region, self.fleet, 50.0, heading_deg=90.0)
        self.assertEqual(plan['heading_deg'], 90.0)
        self.assertGreater(plan['lane_count'], 0)
        self.assertTrue(plan['complete'])

    def test_fleet_order_is_honoured(self):
        plan = partition_region(self.region, self.fleet, 50.0)
        self.assertEqual([item['drone_id'] for item in plan['assignments']], ['drone0', 'drone1'])

    def test_manifest_binds_preview_to_upload(self):
        plan = partition_region(self.region, self.fleet, 50.0)
        first = manifest(plan, 'mission-1', 1)
        second = manifest(plan, 'mission-1', 1)
        self.assertEqual(first['plan_sha256'], second['plan_sha256'])
        changed = partition_region(self.region, self.fleet, 40.0)
        self.assertNotEqual(first['plan_sha256'], manifest(changed, 'mission-1', 1)['plan_sha256'])
        with self.assertRaises(ValueError):
            manifest(plan, '', 1)
        with self.assertRaises(ValueError):
            manifest(plan, 'mission-1', 0)

    def test_plan_serializes_without_nan(self):
        plan = partition_region(self.region, self.fleet, 50.0)
        json.dumps(plan, allow_nan=False)

    def test_single_aircraft_takes_everything(self):
        plan = partition_region(self.region, [{'drone_id': 'solo'}], 50.0)
        self.assertEqual(len(plan['assignments']), 1)
        self.assertEqual(plan['assignments'][0]['lane_count'], plan['lane_count'])
        self.assertTrue(plan['complete'])


if __name__ == '__main__':
    unittest.main()


class DrawnRouteTests(unittest.TestCase):
    """LineStrings named after the aircraft are flown instead of the coverage lanes."""
    KML = open(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'sitl_missions', 'headon_2drones.kml')).read() \
        if os.path.exists(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'sitl_missions', 'headon_2drones.kml')) else None

    def plan(self, kml, fleet=('drone0', 'drone1'), template='swarm_coverage'):
        from as2_rescue_planner.mission import plan_mission
        return plan_mission(dict(kml=kml, mission_id='a' * 32, template=template, altitude_m=12.0, speed_mps=1.5,
                                 fleet=[dict(drone_id=d, endurance_s=1200) for d in fleet]))

    def test_head_on_routes_are_flown_as_drawn(self):
        if self.KML is None:
            self.skipTest('sitl_missions not present')
        result = self.plan(self.KML)
        self.assertTrue(result['complete'], result['reason'])
        self.assertEqual(result['routes'], 'kml')
        poses = {b['drone_id']: [w['pose'] for w in b['coverage']['waypoints']] for b in result['bundles']}
        self.assertEqual(poses['drone0'][0][:2], poses['drone1'][-1][:2])          # same line, opposite ways
        self.assertEqual(poses['drone0'][-1][:2], poses['drone1'][0][:2])
        self.assertIn('segment_id="drone1_route_1"', result['xml']['drone1'])

    def test_routes_must_name_every_aircraft(self):
        if self.KML is None:
            self.skipTest('sitl_missions not present')
        result = self.plan(self.KML, fleet=('drone0', 'drone1', 'drone2'))
        self.assertFalse(result['complete'])
        self.assertIn('must name every selected aircraft', result['reason'])

    def test_watch_missions_do_not_take_routes(self):
        if self.KML is None:
            self.skipTest('sitl_missions not present')
        result = self.plan(self.KML, template='human_orbit')
        self.assertFalse(result['complete'])
