#!/usr/bin/env bash
# AERIXIS onboard software, plug and play: start it (or repair it) on the Raspberry Pi 5 and show
# the state of every link. Run on the Pi:
#
#   ~/aerostack2/docker/raspberrypi5/start_onboard.sh             start / check
#   ~/aerostack2/docker/raspberrypi5/start_onboard.sh --recreate  rebuild the container from the image
#
# What it handles:
# * The AI HAT's device number can change between boots (510 -> 240 seen on 2026-10-27). The
#   container may only open the number it was created with, so the camera AI then fails. This
#   writes the live number to .env and recreates the container when it changed.
# * Files changed after the image was built (in ~/aerixis_patches, list.txt = "<file> <path in
#   container>") are copied back after a recreate, until the next image build includes them.
# Nothing here arms or flies the aircraft.
set -u
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER=as2-rescue-onboard-1
PATCHES="${AERIXIS_PATCHES:-$HOME/aerixis_patches}"
WAIT_S="${WAIT_S:-90}"
cd "$HERE" || exit 1
say() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

recreate=0
[ "${1:-}" = "--recreate" ] && recreate=1
docker inspect "$CONTAINER" >/dev/null 2>&1 || recreate=1

say "AI HAT"
MAJOR="$(awk '$2=="hailo"{print $1}' /proc/devices)"
if [ -n "$MAJOR" ]; then
  touch .env
  if ! grep -qx "HAILO_MAJOR=$MAJOR" .env; then
    sed -i '/^HAILO_MAJOR=/d' .env && echo "HAILO_MAJOR=$MAJOR" >> .env
  fi
  if [ $recreate = 0 ] && ! docker inspect "$CONTAINER" --format '{{.HostConfig.DeviceCgroupRules}}' \
       | grep -q "c $MAJOR:\* rmw"; then
    echo "AI HAT device number is now $MAJOR: the container must be recreated"
    recreate=1
  else
    echo "AI HAT ready (device $MAJOR)"
  fi
else
  echo "AI HAT driver not loaded (no /dev/hailo0): HAT models will not run"
fi

if [ $recreate = 1 ]; then
  say "Creating the onboard container"
  docker compose up -d --force-recreate onboard || exit 1
  if [ -f "$PATCHES/list.txt" ]; then
    n=0
    while read -r file target; do
      [ -n "$file" ] || continue
      docker cp "$PATCHES/$file" "$CONTAINER:$target" && n=$((n+1))
    done < "$PATCHES/list.txt"
    echo "restored $n updated files from $PATCHES"
    docker compose restart onboard >/dev/null
  fi
else
  say "Starting the onboard software"
  docker start "$CONTAINER" >/dev/null && echo "running"
fi

say "Waiting ${WAIT_S}s for the links to come up"
sleep "$WAIT_S"
docker exec "$CONTAINER" /entrypoint.sh status 2>&1 | cut -c1-100

say "Power"
throttled="$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)"
if [ -n "$throttled" ] && [ $((throttled & 0x1)) -ne 0 ]; then
  echo "UNDERVOLTAGE NOW ($throttled): fix the 5 V supply before flying"
elif [ -n "$throttled" ] && [ $((throttled & 0x10000)) -ne 0 ]; then
  echo "undervoltage happened since boot ($throttled): check the 5 V supply and cable"
else
  echo "ok ($throttled)"
fi

say "Onboard panel"
for ip in $(hostname -I); do case "$ip" in *:*) ;; *) echo "http://$ip:8090" ;; esac; done
