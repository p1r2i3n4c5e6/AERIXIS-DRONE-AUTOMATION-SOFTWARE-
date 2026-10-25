"""Command line entry point: turn a KML file into a deterministic fleet plan.

This plans geometry only. It does not upload, start or simulate a mission.
"""
import argparse
import json
import sys
from pathlib import Path
from .mission import plan_mission
from .bundle import write_bundles

from .coverage import DEFAULT_SPACING_M, DEFAULT_CORRIDOR_M, manifest, partition_region
from .kml import altitude_semantics, parse_kml


def build_parser():
    parser = argparse.ArgumentParser(
        prog='plan_fleet',
        description='Validate a KML area and partition it across a fleet.')
    parser.add_argument('kml', help='Path to a KML file containing Polygon areas')
    parser.add_argument('--fleet', required=True,
                        help='JSON file: [{"drone_id":..,"endurance_s":..,"reserve_fraction":..}]')
    parser.add_argument('--spacing-m', type=float, default=DEFAULT_SPACING_M,
                        help='Lane spacing in metres (default %(default)s)')
    parser.add_argument('--heading-deg', type=float, default=0.0,
                        help='Coverage heading in degrees (default %(default)s)')
    parser.add_argument('--corridor-m', type=float, default=DEFAULT_CORRIDOR_M,
                        help='Separation corridor between partitions (default %(default)s)')
    parser.add_argument('--speed-mps', type=float, default=0.5,
                        help='Planned cruise speed used for endurance estimates (0, 2] m/s)')
    parser.add_argument('--mission-id', default='', help='Mission identity for the manifest')
    parser.add_argument('--area', default=None,
                        help='Area name to plan; required when the KML has several')
    parser.add_argument('--output', default=None, help='Write the plan JSON to this path')
    parser.add_argument('--output-dir', help='Write the fleet manifest, individual route bundles and XML')
    parser.add_argument('--altitude-m', type=float, help='Explicit altitude in the common origin frame')
    parser.add_argument('--template', default='survey', help='survey, search_geotag, rescue_delivery, waypoint_delivery, swarm_search or swarm_delivery')
    return parser


def main(argv=None):
    arguments = build_parser().parse_args(argv)
    try:
        with open(arguments.kml, 'rb') as handle:
            document = handle.read()
        plan_source = parse_kml(document)
        with open(arguments.fleet, 'r', encoding='utf-8') as handle:
            fleet = json.load(handle)
        if not isinstance(fleet, list):
            raise ValueError('The fleet file must contain a JSON list')
        if arguments.output_dir:
            if arguments.altitude_m is None:
                raise ValueError('--output-dir needs an explicit --altitude-m in the common origin frame')
            if not arguments.mission_id:
                raise ValueError('--output-dir needs --mission-id so bundles carry a stable identity')
            result=plan_mission(dict(kml=document.decode('utf-8'),fleet=fleet,mission_id=arguments.mission_id,
                area=arguments.area,altitude_m=arguments.altitude_m,template=arguments.template,
                spacing_m=arguments.spacing_m,heading_deg=arguments.heading_deg,
                speed_mps=arguments.speed_mps,corridor_m=arguments.corridor_m))
            if not result['complete']:raise ValueError(result['reason'])
            write_bundles(arguments.output_dir,result)
            for drone_id,xml in result['xml'].items():
                (Path(arguments.output_dir)/(drone_id+'.rev%d.xml' % result['revision'])).write_text(xml)
            sys.stdout.write('Fleet XML and route bundles written to '+arguments.output_dir+'\n')
            return 0

        areas = plan_source.areas
        if arguments.area is not None:
            areas = [area for area in areas if area.name == arguments.area]
            if not areas:
                raise ValueError('No area named %r in this KML' % arguments.area)
        if len(areas) > 1:
            raise ValueError('KML has %d areas (%s); select one with --area'
                             % (len(areas), ', '.join(area.name for area in areas)))

        area = areas[0]
        region = plan_source.region(area)
        altitude = altitude_semantics(area)
        plan = partition_region(region, fleet, arguments.spacing_m, arguments.heading_deg,
                                arguments.speed_mps, arguments.corridor_m)
        result = {
            'area': area.name,
            'altitude': altitude,
            'plan': plan,
        }
        if arguments.mission_id:
            result['manifest'] = manifest(plan, arguments.mission_id, 1)
        text = json.dumps(result, indent=2, allow_nan=False)
        if arguments.output:
            with open(arguments.output, 'w', encoding='utf-8') as handle:
                handle.write(text + '\n')
        else:
            sys.stdout.write(text + '\n')
        return 0 if plan['complete'] else 2
    except (ValueError, OSError, json.JSONDecodeError) as error:
        sys.stderr.write('plan_fleet: %s\n' % error)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
