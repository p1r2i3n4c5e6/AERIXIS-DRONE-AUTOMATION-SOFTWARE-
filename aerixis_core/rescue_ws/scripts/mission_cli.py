#!/usr/bin/env python3
"""Plan, stage, start and follow a mission on one aircraft's onboard mission manager.

For SITL and bench checks on the companion computer itself (the manager listens on
loopback). In the field the laptop GCS does the same over the telemetry radio.

  mission_cli.py plan  --kml area.kml --template human_geotag_resume --drone drone0
  mission_cli.py run   --kml area.kml --template human_geotag_resume --drone drone0 \
                       --token-file ~/.local/share/as2-missions/drone0/pairing.token
  mission_cli.py status --token-file ...
  mission_cli.py reconcile --token-file ...   (after a failed/aborted flight, on the ground)
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import urllib.error
import urllib.request
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'as2_rescue_planner'))
from as2_rescue_planner.mission import plan_mission  # noqa: E402


def call(arguments, path, body=None):
    token = Path(arguments.token_file).expanduser().read_text().strip()
    request = urllib.request.Request(arguments.manager + path, data=None if body is None else json.dumps(body).encode(),
                                     headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(request, timeout=20) as stream:
            return json.load(stream)
    except urllib.error.HTTPError as exc:
        raise SystemExit('%s: %s' % (path, json.loads(exc.read(4096)).get('error')))


def plan(arguments):
    fleet = [dict(drone_id=d, endurance_s=arguments.endurance_min * 60, reserve_fraction=.25)
             for d in arguments.fleet.split(',')]
    request = dict(kml=Path(arguments.kml).read_text(), fleet=fleet, mission_id=uuid.uuid4().hex,
                   template=arguments.template, altitude_m=arguments.altitude, spacing_m=arguments.spacing,
                   speed_mps=arguments.speed, corridor_m=arguments.corridor)
    for option in ('orbit_radius_m', 'orbit_laps', 'lost_timeout_s', 'search_pitch_deg'):
        if getattr(arguments, option) is not None:
            request[option] = getattr(arguments, option)
    result = plan_mission(request)
    if not result['complete']:
        raise SystemExit('Plan incomplete: ' + result['reason'])
    return result


def show_status(snapshot, seen):
    execution = snapshot.get('execution') or {}
    rescue = ((snapshot.get('ros') or {}).get('rescue') or {}).get('data') or {}
    for node in snapshot.get('bt_nodes', []):
        key = (node.get('path'), node.get('status'))
        if node.get('status') in ('RUNNING', 'SUCCESS', 'FAILURE') and key not in seen:
            seen.add(key)
            print('  BT %-8s %s' % (node['status'], node.get('name') or node.get('type')))
    return execution.get('state'), rescue


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('action', choices=['plan', 'run', 'status', 'reconcile'])
    parser.add_argument('--kml')
    parser.add_argument('--template', default='human_geotag_resume')
    parser.add_argument('--drone', default='drone0')
    parser.add_argument('--fleet', help='comma separated drone IDs sharing the area (default: --drone)')
    parser.add_argument('--altitude', type=float, default=12.0)
    parser.add_argument('--spacing', type=float, default=10.0)
    parser.add_argument('--speed', type=float, default=1.5)
    parser.add_argument('--corridor', type=float, default=10.0)
    parser.add_argument('--endurance-min', type=float, default=20.0)
    parser.add_argument('--orbit-radius-m', dest='orbit_radius_m', type=float)
    parser.add_argument('--orbit-laps', dest='orbit_laps', type=float)
    parser.add_argument('--lost-timeout-s', dest='lost_timeout_s', type=float)
    parser.add_argument('--search-pitch-deg', dest='search_pitch_deg', type=float)
    parser.add_argument('--payload-id', help='operator-confirmed payload ID (mission 4)')
    parser.add_argument('--manager', default='http://127.0.0.1:8790')
    parser.add_argument('--token-file', default=str(Path.home() / '.local/share/as2-missions/drone0/pairing.token'))
    parser.add_argument('--timeout', type=float, default=900)
    parser.add_argument('--out', help='write the plan JSON here')
    arguments = parser.parse_args()
    arguments.fleet = arguments.fleet or arguments.drone
    if arguments.action == 'status':
        print(json.dumps(call(arguments, '/api/status'), indent=1)[:4000])
        return
    if arguments.action == 'reconcile':  # on the ground, disarmed: clear lockout + runtime abort
        print(call(arguments, '/api/mission/reconcile', {}))
        return
    result = plan(arguments)
    xml = result['xml'][arguments.drone]
    bundle = next(b for b in result['bundles'] if b['drone_id'] == arguments.drone)
    print('Plan %s: origin %.8f, %.8f; %d waypoints for %s' % (result['template'], result['origin']['latitude'],
          result['origin']['longitude'], len(bundle['coverage']['waypoints']), arguments.drone))
    for waypoint in bundle['coverage']['waypoints']:
        print('  %s %s' % (waypoint['segment_id'][-12:], waypoint['pose']))
    if arguments.out:
        Path(arguments.out).write_text(json.dumps(result, indent=1))
    if arguments.action == 'plan':
        return
    body = dict(drone_id=arguments.drone, xml=xml, sha256=hashlib.sha256(xml.encode()).hexdigest(), route_bundle=bundle)
    if arguments.payload_id:
        body['operator_payload'] = dict(confirmed_loaded=True, payload_id=arguments.payload_id)
    mission = call(arguments, '/api/mission/stage', body)['result']
    print('Staged onboard as %s (%d nodes)' % (mission['id'], mission['node_count']))
    deadline = time.time() + 60
    while True:
        snapshot = call(arguments, '/api/status')
        if snapshot.get('can_start'):
            break
        if time.time() > deadline:
            raise SystemExit('Mission cannot start: aircraft not on the ground/ready or flight disabled')
        time.sleep(1)
    time.sleep(3)  # let the route node set the estimator origin from the staged plan
    call(arguments, '/api/mission/start', dict(mission_id=mission['id'], revision=mission['revision'],
                                               command_id=uuid.uuid4().hex))
    print('Started. Following onboard execution (Ctrl+C stops following, not the mission).')
    seen, started, last = set(), time.time(), None
    while time.time() - started < arguments.timeout:
        state, rescue = show_status(call(arguments, '/api/status'), seen)
        operation = (rescue.get('active_operation'), len(rescue.get('geotags') or []))
        if operation != last:
            last = operation
            print('  runtime: op=%s geotags=%d' % operation)
            for tag in (rescue.get('geotags') or [])[-1:]:
                print('    geotag %s %.7f %.7f +-%sm' % (tag['label'], tag['latitude'], tag['longitude'], tag['sigma_m']))
        if state in ('SUCCESS', 'FAILURE', 'CANCELLED', 'ERROR', 'INTERRUPTED'):
            print('Execution finished: %s' % state)
            return
        time.sleep(1)
    print('Stopped following after %.0fs' % arguments.timeout)


if __name__ == '__main__':
    main()
