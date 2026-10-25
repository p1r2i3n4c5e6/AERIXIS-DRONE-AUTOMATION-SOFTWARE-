#!/usr/bin/env bash
# AERIXIS ground station (laptop): web GCS + MAVLink link to the aircraft.
#
#   launch/gcs.sh start [--udp-port N]   start in the background and open the GCS window
#                                        (--udp-port: aircraft gateway UDP port, default 14555 =
#                                        the SITL radio hub; a field radio is chosen on the
#                                        Connect page, serial port + baud)
#   launch/gcs.sh stop | restart | status | logs | open
#
# GCS folder: $AERIXIS_GCS_DIR (default ~/custom_gcs). Web UI: http://localhost:8765
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"
URL="http://localhost:${WEB_PORT:-8765}/"

start() {
  local port=14555
  while [ $# -gt 0 ]; do case "$1" in --udp-port) port=$2; shift 2 ;; *) warn "unknown option $1"; exit 2 ;; esac; done
  [ -x "$GCS_DIR/start_gcs.sh" ] || { warn "GCS not found in $GCS_DIR (set AERIXIS_GCS_DIR)"; exit 1; }
  if ! running gcs && curl -s -o /dev/null --max-time 1 "${URL}api/state"; then
    warn "A GCS is already running on $URL (not started by this launcher). Stop it first, or just open it."; exit 1
  fi
  say "AERIXIS GCS"
  start_bg gcs env GATEWAY_PORT="$port" bash -c "cd '$GCS_DIR' && exec ./start_gcs.sh"
  if wait_http "${URL}api/state" 40; then
    say "GCS ready: $URL  (aircraft link: UDP $port, or a serial radio on the Connect page)"
  else
    warn "GCS did not answer within 40 s — see: launch/gcs.sh logs"; exit 1
  fi
}

case "${1:-start}" in
  start) shift || true; start "$@" ;;
  stop) stop_bg gcs ;;
  restart) shift; stop_bg gcs; start "$@" ;;
  status)
    status_bg gcs
    curl -s --max-time 2 "${URL}api/state" | python3 -c "import sys,json
s=json.load(sys.stdin);c=s.get('connection',{})
print('  link:', c.get('type','?'), c.get('target',''), '· heartbeat', s.get('heartbeat_hz',0), 'Hz')" 2>/dev/null || true
    curl -s --max-time 2 "${URL}api/fleet" | python3 -c "import sys,json
for d in json.load(sys.stdin)['drones']: print('  aircraft:', d['drone_id'], d['connection'])" 2>/dev/null || true ;;
  logs) logs_bg gcs ;;
  open) xdg-open "$URL" >/dev/null 2>&1 & echo "$URL" ;;
  -h|--help|help) sed -n '2,11p' "$0" ;;
  *) sed -n '2,11p' "$0"; exit 2 ;;
esac
