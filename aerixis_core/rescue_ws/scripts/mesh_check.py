#!/usr/bin/env python3
"""Does the drone-to-drone mesh meet what the swarm needs?

The swarm layer (swarm_agent) treats a peer as fresh only if its last position is at most
max_peer_age_s old (default 1.0 s). PeersFresh and SeparationSafe gate every swarm search leg:
if the mesh cannot deliver each peer's position faster than that, fleet missions stop.

  mesh_check.py budget --drones 3 --air-kbps 2.4 9.6 19.2 62.5
      Encodes a realistic peer frame with the real radio codec and works out, per LoRa/serial
      air rate, whether the shared channel and the bridge pacing can keep every peer fresh.

  mesh_check.py live drone0 drone1 [--seconds 60]
      Measures the running mesh (needs ROS sourced): peer ages, freshness, separation flag,
      queued frames and bridge errors per drone, with a PASS/FAIL against the same limits.
"""
import argparse
import base64
import json
import math
import sys
import time
from pathlib import Path

RESCUE_SRC = Path(__file__).resolve().parents[1] / 'src' / 'as2_rescue_runtime'
sys.path.insert(0, str(RESCUE_SRC))
sys.path.insert(0, str(RESCUE_SRC.parent / 'as2_mission_manager'))

MAX_PEER_AGE_S = 1.0      # swarm_agent max_peer_age_s
UPDATES_PER_AGE = 2.0     # need >= 2 deliveries per max age so one lost frame is survivable
SERIAL_FPS = 2.0          # mesh_bridge frames_per_second for serial modems
CHANNEL_EFFICIENCY = 0.6  # usable share of a LoRa air rate (preamble, headers, half-duplex turnaround)


def sample_frame(with_claim=True, tags=2):
    """A peer frame as swarm_agent sends it while responding to a person."""
    frame = dict(owner='drone0', position=[-12.34, 45.67, 12.01], frame_id='earth', stamp=round(time.time(), 3),
                 sigma_m=2.5, speed=1.53)
    if with_claim:
        frame['claim'] = dict(label='L12', since=round(time.time(), 2), xy=[-5.12, 17.93])
    if tags:
        frame['tags'] = [[6.1, -28.2], [-4.9, 16.4], [12.0, 3.3]][:tags]
    return frame


def on_air_bytes(frame):
    """Bytes the mesh bridge writes for one peer message: radio codec frames, base64, newline."""
    from as2_rescue_runtime.radio import encode
    from as2_rescue_runtime.swarm import compact_peer
    message = dict(sender='drone0', kind='peer', payload=compact_peer(frame))
    parts = encode(message, b'k' * 32, time.time(), ttl=5)
    return len(parts), sum(len(base64.b64encode(p)) + 1 for p in parts)


def budget(args):
    need_hz = UPDATES_PER_AGE / args.max_age
    print(f'Requirement: every peer position <= {args.max_age:.1f} s old -> each drone must deliver '
          f'{need_hz:.1f} updates/s (2 per max age, so one lost frame is survivable).\n')
    for label, frame in (('idle (position only)', sample_frame(False, 0)), ('responding (claim + 2 geotags)', sample_frame(True, 2))):
        frames, size = on_air_bytes(frame)
        pacing_hz = SERIAL_FPS / frames
        print(f'Peer message while {label}: {len(json.dumps(frame))} B JSON -> {frames} radio frame(s), {size} B on air')
        print(f'  serial bridge pacing ({SERIAL_FPS:.0f} frames/s per drone): {pacing_hz:.1f} updates/s per drone '
              f'-> {"OK" if pacing_hz >= need_hz else "TOO SLOW"}')
        for n in args.drones:
            need_bps = n * need_hz * size * 8
            row = [f'  {n} drones on one channel need {need_bps / 1000:.1f} kbit/s usable:']
            for kbps in args.air_kbps:
                usable = kbps * 1000 * CHANNEL_EFFICIENCY
                row.append(f'{kbps:g} kbps {"OK" if usable >= need_bps else "NO"} ({100 * need_bps / usable:.0f}%)')
            print('  '.join(row))
        print()
    print('Air rate is the modem\'s over-the-air rate (LoRa E22/E32 "air data rate"), not the UART baud.\n'
          f'Assumes {CHANNEL_EFFICIENCY:.0%} of it is usable on a shared half-duplex channel; keep load under ~70 %.')


def live(args):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    node = Node('mesh_check')
    data = {d: dict(status=[], health=[]) for d in args.drones}
    for d in args.drones:
        node.create_subscription(String, f'/{d}/swarm/status', lambda m, d=d: data[d]['status'].append(json.loads(m.data)), 10)
        node.create_subscription(String, f'/{d}/swarm/radio_health', lambda m, d=d: data[d]['health'].append(json.loads(m.data)), 10)
    end = time.time() + args.seconds
    print(f'Listening {args.seconds:.0f} s to {", ".join(args.drones)} ...')
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    verdict = True
    for d in args.drones:
        st, he = data[d]['status'], data[d]['health']
        if not st or not he:
            print(f'{d}: FAIL - no swarm status / radio health (mesh bridge or swarm agent not running?)')
            verdict = False
            continue
        fresh = sum(1 for s in st if s.get('peers_fresh')) / len(st)
        safe = sum(1 for s in st if s.get('separation_safe')) / len(st)
        ages = sorted(a for h in he for a in (h.get('peers') or {}).values())
        p95 = ages[min(len(ages) - 1, int(0.95 * len(ages)))] if ages else math.inf
        errors = {h.get('error') for h in he if h.get('error')}
        queued = max((h.get('queued_frames') or 0) for h in he)
        transport = he[-1].get('transport')
        ok = fresh >= 0.99 and p95 <= MAX_PEER_AGE_S and not errors
        verdict &= ok
        print(f'{d} ({transport}): {"PASS" if ok else "FAIL"}  peers fresh {fresh:.1%}  peer age p95 {p95:.2f} s '
              f'max {ages[-1] if ages else float("nan"):.2f} s  separation safe {safe:.0%}  max queued frames {queued}'
              + (f'  errors: {"; ".join(errors)}' if errors else ''))
    print('\nMESH ' + ('MEETS' if verdict else 'DOES NOT MEET') + f' the swarm requirement (peers fresh >= 99 %, p95 age <= {MAX_PEER_AGE_S} s).')
    sys.exit(0 if verdict else 1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='action', required=True)
    b = sub.add_parser('budget')
    b.add_argument('--drones', type=int, nargs='+', default=[2, 3, 5])
    b.add_argument('--air-kbps', type=float, nargs='+', default=[2.4, 9.6, 19.2, 62.5])
    b.add_argument('--max-age', type=float, default=MAX_PEER_AGE_S, help='swarm_agent max_peer_age_s')
    l_ = sub.add_parser('live')
    l_.add_argument('drones', nargs='+')
    l_.add_argument('--seconds', type=float, default=60)
    args = parser.parse_args()
    budget(args) if args.action == 'budget' else live(args)


if __name__ == '__main__':
    main()
