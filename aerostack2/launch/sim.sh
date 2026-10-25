#!/usr/bin/env bash
# AERIXIS simulation (laptop): PX4 SITL + Gazebo, plus the onboard stack of every simulated
# aircraft (AERIXIS core, rescue runtime, mission manager, MAVLink gateway, mesh, simulated A8).
#
#   launch/sim.sh start [N] [--home LAT,LON[,ALT]] [--persons "x,y;x,y"] [--px4-only]
#        N aircraft (default 1), 20 m apart; --home flies the world over your area (default:
#        PX4's Zurich); --persons = simulated people in plan metres; --px4-only = no onboard stack
#   launch/sim.sh stop | status | logs [px4|onboard] | pairing
#
# Then: launch/gcs.sh start  ->  Connect -> Simulation (UDP 14555) -> "Add aircraft manually" with
# the values from `launch/sim.sh pairing` (the GCS never reads these files itself, as with a Pi).
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"

start() {
  local n=1 home="" persons="6,-28;-5,16" onboard=1
  while [ $# -gt 0 ]; do
    case "$1" in
      --home) home=$2; shift 2 ;;
      --persons) persons=$2; shift 2 ;;
      --px4-only) onboard=0; shift ;;
      [1-9]) n=$1; shift ;;
      *) warn "unknown option $1"; exit 2 ;;
    esac
  done
  if pgrep -x px4 >/dev/null && ! running px4; then
    warn "PX4 SITL is already running (not started by this launcher). Stop it first."; exit 1
  fi
  say "AERIXIS simulation: $n aircraft${home:+ at $home}"
  start_bg px4 env SITL_HOME="$home" bash "$REPO/3_launch_simulation.sh" "$n"
  echo -n "  waiting for PX4"
  for _ in $(seq 1 150); do
    up=0; for ((i=0; i<n; i++)); do ss -uln 2>/dev/null | grep -q ":$((14580+i)) " && up=$((up+1)); done
    [ "$up" -ge "$n" ] && break; echo -n "."; sleep 2
  done; echo
  [ "$up" -ge "$n" ] || { warn "PX4 did not start — see: launch/sim.sh logs px4"; exit 1; }
  say "  PX4 ready ($n instance(s))"
  [ "$onboard" = 1 ] || return 0
  start_bg sitl-onboard env DRONES="$n" SITL_HOME="$home" bash "$REPO/rescue_ws/scripts/sitl_onboard.sh" "$persons"
  echo -n "  waiting for the onboard stacks"
  for _ in $(seq 1 120); do grep -q "^Ready:" "$STATE/logs/sitl-onboard.log" 2>/dev/null && break; echo -n "."; sleep 2; done; echo
  if grep -q "^Ready:" "$STATE/logs/sitl-onboard.log"; then
    say "  Simulation ready. GCS: Connect -> Simulation (UDP 14555) -> Add aircraft manually:"
    pairing
  else
    warn "onboard stacks still starting — see: launch/sim.sh logs onboard"
  fi
}

# Drone ID, system ID and pairing token of each simulated aircraft, to type into the GCS
# (Connect -> Add aircraft manually), exactly as for a Pi (launch/onboard.sh token).
pairing() {
  local root="${AS2_SITL_STATE:-$HOME/.local/share/as2-sitl}/missions" found=0
  for dir in "$root"/drone*; do
    [ -f "$dir/pairing.token" ] || continue
    local id; id=$(basename "$dir"); found=1
    printf '  %-8s system ID %-3s token %s\n' "$id" "$(( ${id#drone} + 1 ))" "$(cat "$dir/pairing.token")"
  done
  [ "$found" = 1 ] || warn "no simulated aircraft yet — run: launch/sim.sh start N"
}

case "${1:-start}" in
  start) shift || true; start "$@" ;;
  stop) stop_bg sitl-onboard 20; stop_bg px4 20 ;;
  status) status_bg px4 sitl-onboard; for p in $(pgrep -x px4); do echo "  px4 pid $p"; done ;;
  pairing) pairing ;;
  logs) case "${2:-onboard}" in px4) logs_bg px4 ;; *) logs_bg sitl-onboard ;; esac ;;
  -h|--help|help) sed -n '2,13p' "$0" ;;
  *) sed -n '2,13p' "$0"; exit 2 ;;
esac
