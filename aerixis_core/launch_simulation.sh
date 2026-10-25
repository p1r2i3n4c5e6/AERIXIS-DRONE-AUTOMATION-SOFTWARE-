#!/usr/bin/env bash
# =============================================================================
# LAUNCHER 3: PX4 SITL + Gazebo Garden Simulation (Interactive Foreground)
#
# Features:
#   - Live simulation logs streaming in your terminal
#   - Press [Ctrl+C] in this terminal to STOP everything cleanly
#   - Automatically terminates PX4 if you close the Gazebo GUI window
# =============================================================================
set -e

PX4_DIR="/home/prince/px4_sitl"

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
bash run_px4_sitl.sh &
PX4_PID=$!

sleep 3

# Watchdog loop: keeps script alive, streams logs, and detects Gazebo window closing
while kill -0 "$PX4_PID" 2>/dev/null; do
  # If Gazebo was launched and closed by clicking 'X' on window, stop PX4!
  if ! pgrep -f "gz sim" >/dev/null 2>&1 && ! pgrep -f "ruby.*gz" >/dev/null 2>&1; then
    echo ""
    echo ">>> [DETECTED] Gazebo GUI window was closed by user."
    cleanup
  fi
  sleep 1
done

wait "$PX4_PID" 2>/dev/null || true
cleanup
