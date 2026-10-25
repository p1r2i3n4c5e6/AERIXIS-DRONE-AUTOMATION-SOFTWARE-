#!/usr/bin/env bash
# =============================================================================
# LAUNCHER 1: AERIXIS Mission Control — Ground Control Station (GCS) (Interactive Terminal)
#
# Features:
#   - Live server and MAVLink logs stream in this terminal
#   - Press [Ctrl+C] in this terminal to STOP the GCS server cleanly
# =============================================================================
set -e

GCS_DIR="/home/prince/custom_gcs"
WEB_PORT="${WEB_PORT:-8765}"
# WEB_HOST=0.0.0.0 lets phones on the same network open the GCS (no login: private networks only).
WEB_HOST="${WEB_HOST:-127.0.0.1}"

# Kill previous GCS instance if running
pkill -f "custom_gcs/server.py" 2>/dev/null || true
pkill -f "python3 server.py --web" 2>/dev/null || true   # also a GCS started from inside custom_gcs
pkill -f "as2_ground_control.server" 2>/dev/null || true
tmux kill-session -t as2_gcs 2>/dev/null || true
sleep 0.5

cleanup() {
  echo ""
  echo "=========================================================="
  echo ">>> [TERMINAL] Stopping the AERIXIS GCS server..."
  echo "=========================================================="
  pkill -f "custom_gcs/server.py" 2>/dev/null || true
pkill -f "python3 server.py --web" 2>/dev/null || true   # also a GCS started from inside custom_gcs
  pkill -f "as2_ground_control.server" 2>/dev/null || true
  echo ">>> [STOPPED] AERIXIS GCS server stopped cleanly."
  exit 0
}

trap cleanup INT TERM EXIT

echo "=========================================================="
echo " [LAUNCHER 1/3] Starting AERIXIS Mission Control (GCS)"
echo " Web UI         : http://localhost:${WEB_PORT}"
echo " MAVLink Listen : 0.0.0.0:14551 (from the AERIXIS gateway)"
echo " MAVLink Send   : 127.0.0.1:14550 (to the AERIXIS gateway)"
echo " Stop Control   : Press [Ctrl+C] in this terminal to STOP"
echo "=========================================================="

cd "$GCS_DIR"
# Open the GCS window (AERIXIS loading screen first) once the server answers; GCS_OPEN_BROWSER=0 disables.
[ -x "$GCS_DIR/open_gcs_window.sh" ] && "$GCS_DIR/open_gcs_window.sh" "http://localhost:${WEB_PORT}/" &
exec python3 server.py --web-host "$WEB_HOST" --web-port "$WEB_PORT"
