#!/usr/bin/env bash
# Opens AERIXIS Mission Control in a browser window once the GCS server answers, so the operator's
# first view is the AERIXIS loading screen. Runs in the background from start_gcs.sh.
# Disable with GCS_OPEN_BROWSER=0. Uses a Chrome/Chromium app window if available, else xdg-open.
URL="${1:-http://localhost:8765/}"
[ "${GCS_OPEN_BROWSER:-1}" = "0" ] && exit 0
[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && exit 0
for _ in $(seq 1 60); do
  curl -fsS -o /dev/null --max-time 1 "${URL%/}/api/state" 2>/dev/null && break
  sleep 0.5
done
for b in google-chrome chromium chromium-browser microsoft-edge brave-browser; do
  if command -v "$b" >/dev/null 2>&1; then
    exec "$b" --app="$URL" --new-window >/dev/null 2>&1
  fi
done
command -v xdg-open >/dev/null 2>&1 && exec xdg-open "$URL" >/dev/null 2>&1
exit 0
