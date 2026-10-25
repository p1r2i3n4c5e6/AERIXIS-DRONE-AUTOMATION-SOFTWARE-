#!/usr/bin/env bash
# =============================================================================
# AERIXIS Mission Control — Ground Control Station (GCS) launcher
# Standalone web GCS talking MAVLink (UDP/serial) to the AERIXIS gateway onboard each aircraft.
# Opens the GCS in a browser window once the server answers (GCS_OPEN_BROWSER=0 to disable).
# =============================================================================
set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-8765}"
GATEWAY_HOST="${GATEWAY_HOST:-127.0.0.1}"
GATEWAY_PORT="${GATEWAY_PORT:-14550}"
LISTEN_HOST="${LISTEN_HOST:-0.0.0.0}"
LISTEN_PORT="${LISTEN_PORT:-14551}"

echo "=========================================================="
echo " AERIXIS Mission Control — starting the ground station"
echo "=========================================================="
echo " Web UI         : http://localhost:${WEB_PORT}"
echo " MAVLink Listen : ${LISTEN_HOST}:${LISTEN_PORT} (from the AERIXIS gateway)"
echo " AERIXIS Gateway : ${GATEWAY_HOST}:${GATEWAY_PORT}"
echo "=========================================================="

OPEN_HOST="$WEB_HOST"; [ "$OPEN_HOST" = "0.0.0.0" ] && OPEN_HOST="localhost"; [ "$OPEN_HOST" = "127.0.0.1" ] && OPEN_HOST="localhost"
"$SCRIPT_DIR/open_gcs_window.sh" "http://${OPEN_HOST}:${WEB_PORT}/" &

python3 "$SCRIPT_DIR/server.py" \
  --web-host "$WEB_HOST" \
  --web-port "$WEB_PORT" \
  --gateway-host "$GATEWAY_HOST" \
  --gateway-port "$GATEWAY_PORT" \
  --listen-host "$LISTEN_HOST" \
  --listen-port "$LISTEN_PORT"
