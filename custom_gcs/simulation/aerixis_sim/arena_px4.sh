#!/usr/bin/env bash
# PX4 SITL (v1.16, Gazebo Harmonic) in the rescue arena, placed at the chosen map location.
#
#   arena_px4.sh ARENA_RUN_DIR
#
# ARENA_RUN_DIR comes from prepare_arena.py (world SDF with the GPS origin at home, spawn points).
# Starts Gazebo with the arena (the 3D window is gazebo_gui.sh), then one PX4 instance per
# spawn point: droneI, system ID I+1, model x500_a8 (x500 + A8-like gimbal camera), the same SITL
# home for PX4's GPS. Stops everything on Ctrl+C / TERM (launch/sim.sh stop). The 3D window is
# gazebo_gui.sh, started and reopened separately; drones can join later (add_px4.sh).
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUN="${1:?usage: arena_px4.sh ARENA_RUN_DIR (from prepare_arena.py)}"
source "$HERE/px4_env.sh"
N=$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))['spawns']))" "$RUN/sim_info.json")

killall -9 px4 gz ruby 2>/dev/null || true      # same clean start as 3_launch_simulation.sh
rm -rf /tmp/px4_instance_* 2>/dev/null || true
sleep 0.5
PIDS=()
cleanup() {
  trap - INT TERM EXIT
  echo; echo ">>> Stopping the arena simulation (PX4 + Gazebo)"
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true; done
  sleep 2
  killall -9 px4 gz ruby 2>/dev/null || true      # also drones added later (launch/sim.sh add)
  rm -rf /tmp/px4_instance_* 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM EXIT

echo "=========================================================="
echo " AERIXIS rescue arena: $N drone(s), home $HOME_LAT, $HOME_LON"
echo " World: $RUN/worlds/$WORLD.sdf   Model: x500_a8"
echo "=========================================================="
setsid $GZ_SIM -s -r -v 2 "$RUN/worlds/$WORLD.sdf" > "$LOGS/gazebo.log" 2>&1 < /dev/null &
PIDS+=($!)
for _ in $(seq 1 60); do
  gz service -i -s "/world/$WORLD/scene/info" 2>/dev/null | grep -q 'Service providers' && break; sleep 1
done
gz service -i -s "/world/$WORLD/scene/info" 2>/dev/null | grep -q 'Service providers' \
  || { echo "Gazebo did not start the arena world: see $LOGS/gazebo.log" >&2; exit 1; }
# The 3D window is separate (launch/sim.sh gui reopens it); closing it does not stop the simulation.

for ((i=0; i<N; i++)); do
  ( spawn_px4 $i ) &
  PIDS+=($!)
  sleep 4
done
echo ">>> Arena simulation running. Stop it with launch/sim.sh stop (or Ctrl+C)."
# Run until stopped (PX4 instance 0 ending also ends it).
while kill -0 "${PIDS[1]}" 2>/dev/null; do sleep 1; done
cleanup
