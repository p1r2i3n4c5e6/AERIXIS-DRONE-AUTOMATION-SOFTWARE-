#!/usr/bin/env bash
# One command for the AERIXIS onboard stack on a Raspberry Pi 5 (native install, no Docker).
#
#   as2_pi.sh start          run everything in this terminal (Ctrl+C stops): XRCE agent ->
#                            FC guardian -> AERIXIS core + mission manager -> GCS gateway ->
#                            mesh (if a passphrase is set) -> camera (if not 'off')
#   as2_pi.sh install        install + enable the systemd units (start at every boot)  [sudo]
#   as2_pi.sh up | down | restart     start / stop the whole stack (systemd)
#   as2_pi.sh status         unit states, link states, FC guardian state
#   as2_pi.sh logs [part]    follow the journal: fc | guardian | onboard (default: all)
#   as2_pi.sh failsafe RTL|LAND [mavlink-url]
#                            set PX4's offboard-loss failsafe to the same action (default URL
#                            /dev/ttyACM0 = FC USB; run 'down' first if fc_health uses that port)
#   as2_pi.sh uninstall      disable and remove the units  [sudo]
#
# Settings: ~/.config/as2/pi_launcher.json (Pi GUI 'Save') or AS2_<KEY>=... in
# /etc/default/as2-rescue, e.g. AS2_EMERGENCY_ACTION=LAND, AS2_CAMERA_MODE=off.
# Docker on the Pi: docker/raspberrypi5 runs the same service (guardian included) in a container.
set -euo pipefail
SCRIPTS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RESCUE="$(dirname "$SCRIPTS")"
UNITS=(as2-fc-link.service as2-fc-guardian.service as2-onboard.service as2-rescue.target)
SYSTEMD=/etc/systemd/system

unit_of() { case "${1:-}" in fc) echo as2-fc-link;; guardian) echo as2-fc-guardian;; onboard) echo as2-onboard;; *) echo "";; esac; }

case "${1:-start}" in
  start)
    exec python3 "$SCRIPTS/as2_onboard_service.py" run ;;
  install)
    user="${SUDO_USER:-$USER}"
    [ "$user" != root ] || { echo "Run as the drone user with sudo, not as root" >&2; exit 1; }
    for unit in "${UNITS[@]}"; do
      sed -e "s|@USER@|$user|g" -e "s|@RESCUE@|$RESCUE|g" "$RESCUE/systemd/$unit" | sudo tee "$SYSTEMD/$unit" >/dev/null
    done
    [ -f /etc/default/as2-rescue ] || sudo install -m 0644 "$RESCUE/systemd/as2-rescue.env" /etc/default/as2-rescue
    # Serial ports for the FC UART, telemetry radio and LoRa modem.
    sudo usermod -aG dialout "$user" || true
    sudo systemctl daemon-reload
    sudo systemctl enable as2-rescue.target "${UNITS[@]:0:3}"
    echo "Installed. Start now with: $0 up   (starts automatically at boot)"
    echo "Check: python3 $SCRIPTS/as2_onboard_service.py config"
    ;;
  uninstall)
    sudo systemctl disable --now as2-rescue.target "${UNITS[@]:0:3}" || true
    for unit in "${UNITS[@]}"; do sudo rm -f "$SYSTEMD/$unit"; done
    sudo systemctl daemon-reload ;;
  up)       sudo systemctl start as2-rescue.target ;;
  down)     sudo systemctl stop as2-rescue.target "${UNITS[@]:0:3}" ;;
  restart)  sudo systemctl restart "${UNITS[@]:0:3}" ;;
  status)
    systemctl --no-pager --lines=0 status "${UNITS[@]}" 2>/dev/null | grep -E '^(●|○|×)|Active:' || true
    echo; python3 "$SCRIPTS/as2_onboard_service.py" status || true
    echo; echo "FC guardian:"
    drone="$(python3 "$SCRIPTS/as2_onboard_service.py" config | python3 -c 'import json,sys; print(json.load(sys.stdin)["drone_id"])')"
    ( set +eu; source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"; source "$RESCUE/install/setup.bash" 2>/dev/null || true
      export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
      timeout 6 ros2 topic echo --once --field data "/$drone/fc_guardian/state" 2>/dev/null ) \
      || echo "  no /$drone/fc_guardian/state (guardian not running?)" ;;
  logs)
    unit="$(unit_of "${2:-}")"
    if [ -n "$unit" ]; then exec journalctl -f -u "$unit"; fi
    exec journalctl -f -u as2-fc-link -u as2-fc-guardian -u as2-onboard ;;
  failsafe)
    action="${2:?RTL or LAND}"; url="${3:-/dev/ttyACM0}"
    # Return-height step = the fleet separation from the onboard settings (16 / 19 / 22 m with 3 m RTK).
    step="$(python3 "$SCRIPTS/as2_onboard_service.py" config | python3 -c 'import json,sys; print(float(json.load(sys.stdin)["separation_m"]))')"
    exec python3 "$SCRIPTS/px4_failsafe_params.py" --mavlink "$url" --action "$action" --return-step "$step" ;;
  *)
    sed -n '2,20p' "$0"; exit 2 ;;
esac
