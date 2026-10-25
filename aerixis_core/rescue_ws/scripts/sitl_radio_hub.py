#!/usr/bin/env python3
"""SITL stand-in for a multipoint telemetry radio: one GCS link, N aircraft.

In the field every aircraft's gateway and the GCS share one radio channel: what the GCS
sends reaches every aircraft, what any aircraft sends reaches the GCS, and MAVLink system
IDs (plus the per-aircraft radio key) keep them apart. In SITL each gateway is a UDP
socket, so this hub reproduces that channel:

  GCS  <->  hub (listen port, default 14555)  <->  gateways on 127.0.0.1:<port> ...

  sitl_radio_hub.py --listen 14555 --gcs 127.0.0.1:14551 --gateways 14560,14561,14562

Optional --rate-kbps throttles the GCS-bound direction to show what a slow radio feels like.
"""
import argparse
import selectors
import socket
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--listen', type=int, default=14555)
    parser.add_argument('--gcs', default='127.0.0.1:14551', help='where GCS-bound traffic goes until the GCS speaks')
    parser.add_argument('--gateways', required=True, help='comma separated local UDP ports of the aircraft gateways')
    parser.add_argument('--rate-kbps', type=float, default=0.0, help='0 = unlimited')
    parser.add_argument('--stats', action='store_true', help='report GCS-bound bytes per MAVLink message ID')
    args = parser.parse_args()
    host, port = args.gcs.rsplit(':', 1)
    gcs = (host, int(port))
    gateways = [('127.0.0.1', int(p)) for p in args.gateways.split(',') if p.strip()]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', args.listen))
    selector = selectors.DefaultSelector()
    selector.register(sock, selectors.EVENT_READ)
    budget, last = 0.0, time.monotonic()
    counts = {'up': 0, 'down': 0}
    by_id = {}
    report = time.monotonic() + 30
    print(f'Radio hub on UDP {args.listen}: GCS {gcs[0]}:{gcs[1]} <-> gateways {",".join(str(g[1]) for g in gateways)}', flush=True)
    while True:
        for _ in selector.select(timeout=1.0):
            while True:
                try:
                    data, source = sock.recvfrom(65535, socket.MSG_DONTWAIT)
                except (BlockingIOError, InterruptedError):
                    break
                if source in gateways or (source[0].startswith('127.') and source[1] in {g[1] for g in gateways}):
                    if args.rate_kbps > 0:
                        now = time.monotonic()
                        budget = min(args.rate_kbps * 128, budget + (now - last) * args.rate_kbps * 128)
                        last = now
                        if budget < len(data):
                            continue  # a saturated radio drops frames; it does not queue them
                        budget -= len(data)
                    sock.sendto(data, gcs)
                    counts['down'] += len(data)
                    if args.stats:
                        # MAVLink 2 frames (0xFD): message ID is bytes 7..9, little endian.
                        i = 0
                        while i + 12 <= len(data) and data[i] == 0xFD:
                            size = 12 + data[i + 1] + (13 if data[i + 2] & 1 else 0)
                            key = f"{int.from_bytes(data[i + 7:i + 10], 'little')}@{data[i + 5]}.{data[i + 6]}"
                            by_id[key] = by_id.get(key, 0) + size
                            i += size
                else:
                    gcs = source  # the GCS spoke: answer exactly where it listens
                    for gateway in gateways:
                        sock.sendto(data, gateway)
                    counts['up'] += len(data)
        if time.monotonic() >= report:
            print(f'  last 30 s: to GCS {counts["down"] / 30 / 1024:.1f} KB/s, to aircraft {counts["up"] / 30 / 1024:.2f} KB/s', flush=True)
            if by_id:
                print('    by message id (B/s): ' + ', '.join(f'{k}:{v / 30:.0f}' for k, v in sorted(by_id.items(), key=lambda kv: -kv[1])), flush=True)
            counts = {'up': 0, 'down': 0}
            by_id = {}
            report = time.monotonic() + 30


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
