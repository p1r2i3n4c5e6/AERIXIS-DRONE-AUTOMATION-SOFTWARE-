#!/usr/bin/env bash
# AERIXIS simulation (laptop): PX4 SITL + Gazebo, plus the onboard stack of every simulated
# aircraft (AERIXIS core, rescue runtime, mission manager, MAVLink gateway, mesh, simulated A8).
#
#   launch/sim.sh start [N] [--home LAT,LON[,ALT]] [--persons "x,y;x,y"] [--px4-only]
#                       [--arena] [--no-cameras] [--no-3d]
#        N aircraft (default 1), 20 m apart; --home flies the world over your area (default:
#        PX4's Zurich); --persons = simulated people in plan metres; --px4-only = no onboard stack
#        --arena: the 300 x 300 m rescue arena (custom_gcs/simulation) centred on --home, 1-4
#        aircraft with A8-like gimbal cameras (PX4 1.16), its 50 people for the simulated tracker,
#        and a separate window with every drone's camera (--no-cameras: without it; --no-3d:
#        without Gazebo's 3D window)
#   launch/sim.sh add       one more drone in the running arena (same Gazebo world), up to 4
#   launch/sim.sh gui       (re)open the Gazebo 3D window of the running arena
#   launch/sim.sh onboard   restart every drone's onboard software (PX4 and Gazebo keep running),
#                           e.g. after rebuilding AERIXIS core or when a drone's software is stuck
#   launch/sim.sh cameras   (re)open the drone camera window
#   launch/sim.sh stop | status | logs [px4|onboard] | pairing
#
# Then: launch/gcs.sh start  ->  Connect -> Simulation (UDP 14555) -> "Add aircraft manually" with
# the values from `launch/sim.sh pairing` (the GCS never reads these files itself, as with a Pi).
set -euo pipefail
# The drones' onboard software runs at lower CPU priority than Gazebo and PX4 (2026-10-31): PX4
# runs in lock-step with Gazebo, so when 4 onboard stacks took the CPU, Gazebo stalled, PX4 froze
# with it and sent no IMU for 1-2 s, and the onboard safety check aborted every take-off.
ONBOARD_NICE="${ONBOARD_NICE:-10}"
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"
ARENA_SIM="${AERIXIS_ARENA_SIM:-$GCS_DIR/simulation/aerixis_sim}"   # rescue arena tooling (GCS repo)
port_open() { ss -uln 2>/dev/null | grep -c ":$1 " >/dev/null; }   # UDP port open (PX4 MAVLink)

start() {
  local n=1 home="" persons="6,-28;-5,16" onboard=1 arena=0 cameras=1 gui=1 sim_info=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --home) home=$2; shift 2 ;;
      --persons) persons=$2; shift 2 ;;
      --px4-only) onboard=0; shift ;;
      --arena) arena=1; shift ;;
      --no-cameras) cameras=0; shift ;;
      --no-3d) gui=0; shift ;;
      [1-9]) n=$1; shift ;;
      *) warn "unknown option $1"; exit 2 ;;
    esac
  done
  if pgrep -x px4 >/dev/null && ! running px4; then
    warn "PX4 SITL is already running (not started by this launcher). Stop it first."; exit 1
  fi
  if [ "$arena" = 1 ]; then
    [ "$n" -le 4 ] || { warn "the rescue arena fits 1 to 4 aircraft"; exit 2; }
    home=${home:-47.397742,8.545594,488}
    local run; run=$(python3 "$ARENA_SIM/prepare_arena.py" --home "$home" --drones "$n") || { warn "arena preparation failed"; exit 1; }
    sim_info="$run/sim_info.json"
    # Own ROS domain: a real Pi on the same Wi-Fi also publishes /drone0 (FC topics) on domain 0.
    export ROS_DOMAIN_ID="${SIM_ROS_DOMAIN_ID:-37}"
    echo "$ROS_DOMAIN_ID" > "$STATE/run/arena.domain"
    echo "$run" > "$STATE/run/arena.dir"
    say "AERIXIS simulation: $n aircraft in the rescue arena at $home"
    start_bg px4 bash "$ARENA_SIM/arena_px4.sh" "$run"
  else
    rm -f "$STATE/run/arena.dir"
    say "AERIXIS simulation: $n aircraft${home:+ at $home}"
    start_bg px4 env SITL_HOME="$home" bash "$REPO/3_launch_simulation.sh" "$n"
  fi
  echo -n "  waiting for PX4"
  for _ in $(seq 1 150); do
    up=0; for ((i=0; i<n; i++)); do port_open $((14580+i)) && up=$((up+1)); done
    [ "$up" -ge "$n" ] && break; echo -n "."; sleep 2
  done; echo
  [ "$up" -ge "$n" ] || { warn "PX4 did not start — see: launch/sim.sh logs px4"; exit 1; }
  say "  PX4 ready ($n instance(s))"
  if [ "$arena" = 1 ] && [ "$gui" = 1 ]; then
    start_bg gazebo-gui bash "$ARENA_SIM/gazebo_gui.sh" "$(cat "$STATE/run/arena.dir")"
  fi
  [ "$onboard" = 1 ] || return 0
  if [ "$arena" = 1 ] && [ "$cameras" = 1 ]; then
    start_bg sim-cameras bash -c "source /opt/ros/humble/setup.bash; export PYTHONPATH=/usr/lib/python3/dist-packages:\${PYTHONPATH:-}; exec python3 '$ARENA_SIM/sim_cameras.py' --drones $n"
  fi
  start_bg sitl-onboard env ONBOARD_NICE="$ONBOARD_NICE" DRONES="$n" SITL_HOME="$home" SIM_INFO="$sim_info" bash "$REPO/rescue_ws/scripts/sitl_onboard.sh" "$persons"
  echo -n "  waiting for the onboard stacks"
  for _ in $(seq 1 120); do grep -q "^Ready:" "$STATE/logs/sitl-onboard.log" 2>/dev/null && break; echo -n "."; sleep 2; done; echo
  if grep -q "^Ready:" "$STATE/logs/sitl-onboard.log"; then
    say "  Simulation ready. GCS: Connect -> Simulation (UDP 14555) -> Add aircraft manually:"
    pairing
  else
    warn "onboard stacks still starting — see: launch/sim.sh logs onboard"
  fi
}

# One more drone in the running rescue arena: its PX4 joins the same Gazebo world (same 3D window)
# at its fixed slot; the onboard stacks restart for N+1 drones (the drones on the ground keep their
# PX4, pose and pairing); the camera window reopens for N+1 if it was open.
add_drone() {
  local run n
  run=$(cat "$STATE/run/arena.dir" 2>/dev/null) || { warn "no arena simulation running (launch/sim.sh start N --arena)"; exit 1; }
  running px4 || { warn "the arena simulation is not running"; exit 1; }
  export ROS_DOMAIN_ID="$(cat "$STATE/run/arena.domain" 2>/dev/null || echo 37)"
  n=$(ls -d /tmp/px4_instance_* 2>/dev/null | wc -l)
  [ "$n" -ge 1 ] || { warn "no PX4 instance found"; exit 1; }
  # A PX4 without its onboard software (an add that stopped half-way): finish that drone, do not spawn another.
  local onboard; onboard=$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))))" "$HOME/.local/share/as2-sitl/sitl_fleet.json" 2>/dev/null || echo "$n")
  if [ "$onboard" -lt "$n" ]; then
    n=$((n-1)); say "AERIXIS simulation: finishing drone$n (its PX4 is already running)"
  else
    [ "$n" -lt 4 ] || { warn "the arena clearing fits 4 drones"; exit 2; }
    local home; home=$(python3 -c "import json,sys; h=json.load(open(sys.argv[1]))['home']; print(f\"{h['latitude_deg']},{h['longitude_deg']},{h['altitude_m']}\")" "$run/sim_info.json")
    say "AERIXIS simulation: adding drone$n to the running arena"
    python3 "$ARENA_SIM/prepare_arena.py" --home "$home" --drones $((n+1)) --out "$run" >/dev/null || { warn "arena update failed"; exit 1; }
    start_bg px4-drone$n bash "$ARENA_SIM/add_px4.sh" "$run" "$n"
  fi
  echo -n "  waiting for PX4"
  # grep -c reads all of ss's output: grep -q under pipefail made an open port look closed (SIGPIPE).
  for _ in $(seq 1 120); do port_open $((14580+n)) && break; echo -n "."; sleep 2; done; echo
  port_open $((14580+n)) || { warn "drone$n PX4 did not start — see ~/.local/share/as2-sitl/logs/px4_$n.log"; exit 1; }
  say "  PX4 ready ($((n+1)) instance(s))"
  local cams=0; running sim-cameras && cams=1
  stop_bg sim-cameras 5 >/dev/null
  stop_bg sitl-onboard 20
  start_bg sitl-onboard env ONBOARD_NICE="$ONBOARD_NICE" DRONES="$((n+1))" SIM_INFO="$run/sim_info.json" bash "$REPO/rescue_ws/scripts/sitl_onboard.sh"
  echo -n "  waiting for the onboard stacks"
  for _ in $(seq 1 150); do grep -q "^Ready:" "$STATE/logs/sitl-onboard.log" 2>/dev/null && break; echo -n "."; sleep 2; done; echo
  [ "$cams" = 1 ] && start_bg sim-cameras bash -c "source /opt/ros/humble/setup.bash; export PYTHONPATH=/usr/lib/python3/dist-packages:\${PYTHONPATH:-}; exec python3 '$ARENA_SIM/sim_cameras.py' --drones $((n+1))"
  if grep -q "^Ready:" "$STATE/logs/sitl-onboard.log"; then say "  Simulation ready: $((n+1)) drones"; else warn "onboard stacks still starting — see: launch/sim.sh logs onboard"; fi
}

# Restart the onboard stacks of the running arena (all drones), PX4 and Gazebo untouched.
restart_onboard() {
  local run; run=$(cat "$STATE/run/arena.dir" 2>/dev/null) || true
  [ -n "$run" ] && [ -f "$run/sim_info.json" ] || { warn "no arena simulation running"; exit 1; }
  local n; n=$(ls -d /tmp/px4_instance_* 2>/dev/null | wc -l)
  [ "$n" -ge 1 ] || { warn "no PX4 instance found"; exit 1; }
  export ROS_DOMAIN_ID; ROS_DOMAIN_ID=$(cat "$STATE/run/arena.domain" 2>/dev/null || echo 37)
  say "AERIXIS simulation: restarting the onboard software of $n drone(s)"
  local cams=0; running sim-cameras && cams=1
  stop_bg sim-cameras 5 >/dev/null
  stop_bg sitl-onboard 20
  start_bg sitl-onboard env ONBOARD_NICE="$ONBOARD_NICE" DRONES="$n" SIM_INFO="$run/sim_info.json" bash "$REPO/rescue_ws/scripts/sitl_onboard.sh"
  echo -n "  waiting for the onboard stacks"
  for _ in $(seq 1 150); do grep -q "^Ready:" "$STATE/logs/sitl-onboard.log" 2>/dev/null && break; echo -n "."; sleep 2; done; echo
  [ "$cams" = 1 ] && start_bg sim-cameras bash -c "source /opt/ros/humble/setup.bash; export PYTHONPATH=/usr/lib/python3/dist-packages:\${PYTHONPATH:-}; exec python3 '$ARENA_SIM/sim_cameras.py' --drones $n"
  if grep -q "^Ready:" "$STATE/logs/sitl-onboard.log"; then say "  Simulation ready: $n drones"; else warn "onboard stacks still starting — see: launch/sim.sh logs onboard"; fi
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
  onboard) restart_onboard ;;
  stop) stop_bg sim-cameras 5; stop_bg gazebo-gui 5; stop_bg sitl-onboard 20
        for f in "$STATE"/run/px4-drone*.pid; do [ -f "$f" ] && stop_bg "$(basename "$f" .pid)" 5; done
        stop_bg px4 20; rm -f "$STATE/run/arena.dir" "$STATE/run/arena.domain" ;;
  add) add_drone ;;
  gui) [ -f "$STATE/run/arena.dir" ] || { warn "no arena simulation running"; exit 1; }
       start_bg gazebo-gui bash "$ARENA_SIM/gazebo_gui.sh" "$(cat "$STATE/run/arena.dir")" ;;
  status) status_bg px4 sitl-onboard sim-cameras gazebo-gui; for p in $(pgrep -x px4); do echo "  px4 pid $p"; done
          [ -f "$STATE/run/arena.dir" ] && echo "  arena: $(cat "$STATE/run/arena.dir")" || true ;;
  cameras) export ROS_DOMAIN_ID="$(cat "$STATE/run/arena.domain" 2>/dev/null || echo "${ROS_DOMAIN_ID:-0}")"
           start_bg sim-cameras bash -c "source /opt/ros/humble/setup.bash; export PYTHONPATH=/usr/lib/python3/dist-packages:\${PYTHONPATH:-}; exec python3 '$ARENA_SIM/sim_cameras.py'" ;;
  pairing) pairing ;;
  logs) case "${2:-onboard}" in px4) logs_bg px4 ;; *) logs_bg sitl-onboard ;; esac ;;
  -h|--help|help) sed -n '2,19p' "$0" ;;
  *) sed -n '2,19p' "$0"; exit 2 ;;
esac
