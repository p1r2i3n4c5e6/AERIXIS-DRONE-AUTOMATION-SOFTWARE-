#!/usr/bin/env bash
# =============================================================================
# LAUNCHER 3: PX4 SITL + Gazebo Garden Simulation (Interactive Foreground)
#
#   ./3_launch_simulation.sh        one x500 (drone0)
#   ./3_launch_simulation.sh 3      three x500s (drone0..drone2, system IDs 1..3, 20 m apart)
#   then: DRONES=3 rescue_ws/scripts/sitl_onboard.sh
#   SITL_HOME="lat,lon[,alt]" ./3_launch_simulation.sh 2   fly over your own area (default Zurich);
#   pass the same SITL_HOME to sitl_onboard.sh
#
# Features:
#   - Live simulation logs streaming in your terminal
#   - Press [Ctrl+C] in this terminal to STOP everything cleanly
#   - Automatically terminates PX4 if you close the Gazebo GUI window
# =============================================================================
set -e

PX4_DIR="/home/prince/px4_sitl"
DRONES="${1:-${SITL_DRONES:-1}}"
# Optional: fly the simulation over the real operating area, e.g. SITL_HOME="28.6139,77.2090,216"
if [ -n "${SITL_HOME:-}" ]; then
  IFS=, read -r SITL_HOME_LAT SITL_HOME_LON SITL_HOME_ALT <<< "$SITL_HOME"
  export SITL_HOME_LAT SITL_HOME_LON SITL_HOME_ALT="${SITL_HOME_ALT:-0}"
  echo ">>> SITL home: $SITL_HOME_LAT, $SITL_HOME_LON (alt ${SITL_HOME_ALT} m)"
fi
if ! [[ "$DRONES" =~ ^[1-9]$ ]]; then echo "Number of drones must be 1..9" >&2; exit 2; fi
PX4_LOGS="$HOME/.local/share/as2-sitl/logs"; mkdir -p "$PX4_LOGS"

if [ ! -d "$PX4_DIR" ]; then
  echo "Error: PX4 SITL directory $PX4_DIR not found!" >&2
  exit 1
fi

# Clean up any leftover processes
killall -9 px4 gz ruby 2>/dev/null || true
rm -rf /tmp/px4_instance_* 2>/dev/null || true
sleep 0.5

cleanup() {
  echo ""
  echo "=========================================================="
  echo ">>> [TERMINAL] Stopping PX4 SITL & Gazebo Simulator..."
  echo "=========================================================="
  killall -9 px4 gz ruby 2>/dev/null || true
  rm -rf /tmp/px4_instance_* 2>/dev/null || true
  echo ">>> [STOPPED] PX4 SITL & Gazebo simulation completely stopped."
  exit 0
}

trap cleanup INT TERM EXIT

echo "=========================================================="
echo " [LAUNCHER 3/3] Starting PX4 SITL + Gazebo Garden"
echo " Model        : x500 quadrotor"
echo " World        : as2_playground"
echo " Logs         : Live below"
echo " Stop Control : Press [Ctrl+C] OR close the Gazebo window"
echo "=========================================================="

cd "$PX4_DIR"
PX4_INSTANCE=0 PX4_UXRCE_DDS_NS=drone0 bash run_px4_sitl.sh &
PX4_PID=$!

sleep 3
if [ "$DRONES" -gt 1 ]; then
  # Extra instances join the running Gazebo world: wait for it, then spawn 20 m apart (east): the swarm gate needs 10 m plus both GNSS uncertainties.
  for _ in $(seq 1 60); do gz topic -l 2>/dev/null | grep -q "/world/.*/clock" && break; sleep 1; done
  for ((i=1; i<DRONES; i++)); do
    PX4_INSTANCE=$i PX4_UXRCE_DDS_NS=drone$i PX4_GZ_MODEL_POSE="$((i*20)),0,0.5,0,0,0" \
      bash run_px4_sitl.sh > "$PX4_LOGS/px4_$i.log" 2>&1 &
    echo ">>> drone$i (PX4 instance $i, MAV_SYS_ID $((i+1))) log: $PX4_LOGS/px4_$i.log"
    sleep 4
  done
fi

# Watchdog loop: keeps script alive, streams logs, and detects Gazebo window closing.
# Gazebo can take well over 3 s to appear on a busy laptop: only watch it once it has been
# seen running, and only stop when it has been gone for 3 checks in a row.
gz_running() { pgrep -f "gz sim" >/dev/null 2>&1 || pgrep -f "ruby.*gz" >/dev/null 2>&1; }
for _ in $(seq 1 120); do gz_running && break; kill -0 "$PX4_PID" 2>/dev/null || break; sleep 1; done
missing=0
while kill -0 "$PX4_PID" 2>/dev/null; do
  # If Gazebo was launched and closed by clicking 'X' on window, stop PX4!
  if gz_running; then missing=0; else missing=$((missing+1)); fi
  if [ "$missing" -ge 3 ]; then
    echo ""
    echo ">>> [DETECTED] Gazebo GUI window was closed by user."
    cleanup
  fi
  sleep 1
done

wait "$PX4_PID" 2>/dev/null || true
cleanup
