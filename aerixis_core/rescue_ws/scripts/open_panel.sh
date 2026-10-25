#!/usr/bin/env bash
# Open the onboard control panel once everything is up — the LAST step of an onboard start.
# Waits until the panel answers and every onboard link has settled (running, or off in the
# settings) — at most ~4 min; a link still waiting (e.g. telemetry radio not plugged in) is
# reported and the panel opens anyway so it can be fixed there. With a desktop it opens a browser
# window; headless (Ubuntu Server) it prints the address to use from a laptop / phone.
PORT="${1:-${AS2_WEB_PORT:-8090}}"
URL="http://localhost:${PORT}/"
for _ in $(seq 1 240); do
  curl -s -o /dev/null --max-time 1 "$URL" && break
  sleep 1
done
if ! curl -s -o /dev/null --max-time 2 "$URL"; then
  echo "Onboard panel did not come up on port $PORT (check: as2_onboard_service.py status)"; exit 1
fi

settled() {  # prints the links not up yet; exit 0 = all up, 2 = only links waiting on hardware/settings
  curl -s --max-time 2 "${URL}api/state" | python3 -c '
import sys, json
links = json.load(sys.stdin).get("links", {})
pending = {n: l.get("state") for n, l in links.items() if l.get("state") not in ("running", "off")}
print(" ".join("{}={}".format(n, s) for n, s in pending.items()))
busy = not links or any(s in ("starting", "restarting") for s in pending.values())
sys.exit(1 if busy else 2 if pending else 0)'
}
echo -n "Waiting for every onboard link"
for i in $(seq 1 90); do
  pending=$(settled); rc=$?
  [ $rc = 0 ] && break
  [ $rc = 2 ] && [ "$i" -gt 15 ] && break   # only "waiting" links left for 30 s: open the panel
  echo -n "."; sleep 2
done; echo
[ -n "$pending" ] && echo "Not running yet: $pending  (fix it on the panel)"

ADDRS=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -v -e ':' -e '^$' | sed "s|.*|http://&:${PORT}|" | tr '\n' ' ')
echo "Onboard panel ready: ${ADDRS:-$URL}"
if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
  open_detached() { setsid "$@" >/dev/null 2>&1 < /dev/null & }
  for browser in chromium-browser chromium google-chrome firefox; do
    if command -v "$browser" >/dev/null; then
      case "$browser" in
        firefox) open_detached "$browser" --new-window "$URL" ;;
        *) open_detached "$browser" --app="$URL" --window-size=1280,860 ;;
      esac
      exit 0
    fi
  done
  command -v xdg-open >/dev/null && open_detached xdg-open "$URL"
fi
exit 0
