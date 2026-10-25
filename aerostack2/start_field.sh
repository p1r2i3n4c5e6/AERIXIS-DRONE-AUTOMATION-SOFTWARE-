#!/usr/bin/env bash
# One command for the real field setup (no simulation): run on the GCS laptop.
#
#   ./start_field.sh                 find the Pi, start its onboard software, start the GCS,
#                                    connect the telemetry radio, open the GCS window
#   PI_IP=10.181.33.84 ./start_field.sh     skip the network search
#
# Pi side: the onboard Docker service (FC link, FC guardian, AERIXIS core, mission manager,
# radio gateway, camera tracker, web panel). It also starts by itself when the Pi boots.
# GCS side: the GCS server on 0.0.0.0:8765 (phones on the same Wi-Fi can open it), the radio
# opened at 57600 baud. Ctrl+C here stops the GCS; the Pi keeps running.
# Nothing here arms or flies the aircraft.
set -u
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
GCS_DIR="${GCS_DIR:-/home/prince/custom_gcs}"
PI_USER="${PI_USER:-pi}"
RADIO_BAUD="${RADIO_BAUD:-57600}"
WEB_PORT="${WEB_PORT:-8765}"

say() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

# 1. Find the Pi (its IP changes with every hotspot): saved / given IP first, then a network scan.
say "Finding the onboard computer (Pi 5)"
PI_IP="${PI_IP:-}"
if [ -z "$PI_IP" ]; then
  PI_IP="$(cd "$GCS_DIR" && python3 -c "
import onboard_discovery as d
p = d.discover(fresh=True, hints=['10.181.33.84'])
print(p[0]['address'] if p else '')" 2>/dev/null)"
fi
if [ -z "$PI_IP" ]; then
  echo "No Pi answered on port 8090. Put this laptop on the same Wi-Fi / hotspot as the Pi, check"
  echo "the Pi is powered, then run again (or: PI_IP=<address> $0)."
  exit 1
fi
echo "Pi found at $PI_IP"

# 2. Start (or keep) the onboard software on the Pi.
say "Starting the onboard software on the Pi"
if ssh -o ConnectTimeout=8 -o BatchMode=yes "$PI_USER@$PI_IP" \
    'cd ~/aerostack2/docker/raspberrypi5 && docker compose up -d onboard 2>&1 | tail -1'; then
  echo "Onboard service running. Panel: http://$PI_IP:8090 (or GCS > Onboard calibration)"
else
  echo "Could not reach the Pi over SSH (key login). The onboard service still starts by itself at boot;"
  echo "to start it by hand on the Pi: cd ~/aerostack2/docker/raspberrypi5 && docker compose up -d"
fi

# 3. Connect the telemetry radio once the GCS server answers (in the background).
RADIO="$(ls /dev/serial/by-id/*CP2102* /dev/serial/by-id/*FTDI* 2>/dev/null | head -1)"
# launch_gcs.sh replaces a GCS server that is already running: wait for the NEW one, or the radio
# would be opened on the old server just before it is stopped.
OLD_GCS="$(pgrep -f 'python3 server.py --web' | head -1)"
(
  for _ in $(seq 1 60); do
    sleep 1
    NEW_GCS="$(pgrep -f 'python3 server.py --web' | head -1)"
    [ -n "$NEW_GCS" ] && [ "$NEW_GCS" != "$OLD_GCS" ] || continue
    curl -s -o /dev/null --max-time 1 "http://127.0.0.1:$WEB_PORT/api/state" && break
  done
  if [ -n "$RADIO" ]; then
    curl -s -X POST -H 'Content-Type: application/json' \
      -d "{\"type\":\"serial\",\"port\":\"$RADIO\",\"baud\":$RADIO_BAUD}" \
      "http://127.0.0.1:$WEB_PORT/api/connect" >/dev/null \
      && printf '\n\033[1;32m==> Telemetry radio connected: %s @ %s baud\033[0m\n' "$RADIO" "$RADIO_BAUD"
  else
    printf '\n\033[1;33m==> No telemetry radio plugged in: connect it on the GCS Connect page\033[0m\n'
  fi
  LAN="$(hostname -I 2>/dev/null | awk '{print $1}')"
  printf '\033[1;36m==> GCS: http://localhost:%s   phone (same Wi-Fi): http://%s:%s\033[0m\n' "$WEB_PORT" "$LAN" "$WEB_PORT"
) &

# 4. The GCS server in this terminal (Ctrl+C stops it; the Pi keeps running).
say "Starting the GCS"
WEB_HOST=0.0.0.0 exec "$HERE/launch_gcs.sh"
