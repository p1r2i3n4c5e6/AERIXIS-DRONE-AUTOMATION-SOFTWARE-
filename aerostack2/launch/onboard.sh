#!/usr/bin/env bash
# AERIXIS onboard stack (Raspberry Pi 5 / companion computer) — and the onboard control panel.
#
#   launch/onboard.sh start            start every onboard link: FC link (XRCE agent) -> FC guardian
#                                      -> AERIXIS core + mission manager -> GCS telemetry gateway ->
#                                      mesh -> camera -> onboard control panel (LAST), then opens
#                                      the panel if this computer has a desktop
#   launch/onboard.sh start --sim      same, as SITL drone0 on this laptop (PX4 via launch/sim.sh
#                                      start --px4-only; GCS link UDP 14555)
#   launch/onboard.sh start --docker   the same stack in the Pi 5 container (docker/raspberrypi5)
#   launch/onboard.sh stop | restart | status | logs | token | panel
#   launch/onboard.sh install          start at every boot (systemd, needs sudo) + open the panel
#                                      after desktop login (if the Pi has a desktop)
#   launch/onboard.sh uninstall
#
# Onboard control panel: http://<onboard computer>:8090 from any browser on the drone's network
# (works on Ubuntu Server without a screen). Changes need the panel PIN: launch/onboard.sh token.
# Settings: the panel, the Pi GUI, ~/.config/as2/pi_launcher.json or AS2_<KEY>=... variables.
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/_common.sh"
SCRIPTS="$REPO/rescue_ws/scripts"
COMPOSE=(docker compose -f "$REPO/docker/raspberrypi5/compose.yaml")
installed() { systemctl is-enabled as2-rescue.target >/dev/null 2>&1; }
ros_env() { echo "source /opt/ros/${ROS_DISTRO:-humble}/setup.bash && source '$REPO/install/setup.bash' && source '$REPO/rescue_ws/install/setup.bash'"; }

start() {
  local mode=native
  case "${1:-}" in --sim) mode=sim ;; --docker) mode=docker ;; "") ;; *) warn "unknown option $1"; exit 2 ;; esac
  say "AERIXIS onboard ($mode)"
  case "$mode" in
    docker)
      [ -f "$REPO/docker/raspberrypi5/onboard.env" ] || { warn "create docker/raspberrypi5/onboard.env first (copy onboard.env.example)"; exit 1; }
      "${COMPOSE[@]}" up -d ;;
    native)
      if installed; then bash "$SCRIPTS/as2_pi.sh" up
      else start_bg onboard bash -c "$(ros_env) && exec python3 '$SCRIPTS/as2_onboard_service.py' run"; fi ;;
    sim)
      local sitl="$HOME/.local/share/as2-sitl"
      export AS2_MISSION_STATE_ROOT="$sitl/missions"   # same token as launch/sim.sh pairing prints
      start_bg onboard env AS2_DRONE_ID=drone0 AS2_SYSTEM_ID=1 \
        AS2_FC_MODE=udp AS2_FMU_PREFIX=/drone0 AS2_FC_HEALTH_URL=udpin:0.0.0.0:14540 \
        AS2_GCS_MODE=udp AS2_GCS_HOST=127.0.0.1 AS2_GCS_PORT=14551 AS2_GCS_BIND_PORT=14555 \
        AS2_CAMERA_MODE="${AS2_CAMERA_MODE:-sim}" AS2_ALLOW_FLIGHT=true AS2_PATH_CLEAR=true AS2_MESH_PASSPHRASE= \
        bash -c "$(ros_env) && exec python3 '$SCRIPTS/as2_onboard_service.py' run" ;;
  esac
  # Last step: the onboard control panel (it is the last link to start).
  bash "$SCRIPTS/open_panel.sh" "${AS2_WEB_PORT:-8090}" &
  wait $! || true
  token
}

stop() {
  if installed; then bash "$SCRIPTS/as2_pi.sh" down; fi
  docker ps --format '{{.Names}}' 2>/dev/null | grep -q '^as2-rescue-onboard' && "${COMPOSE[@]}" down
  stop_bg onboard 30
}

token() {
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -q 'as2-rescue.*onboard'; then
    "${COMPOSE[@]}" exec onboard /entrypoint.sh token
  else
    bash -c "$(ros_env) >/dev/null 2>&1; python3 '$SCRIPTS/as2_onboard_service.py' token"
  fi
}

install() {
  bash "$SCRIPTS/as2_pi.sh" install
  if [ -d "$HOME/.config" ] && { [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] || [ -d /usr/share/xsessions ]; }; then
    mkdir -p "$HOME/.config/autostart"
    cat > "$HOME/.config/autostart/aerixis-panel.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=AERIXIS onboard panel
Comment=Opens the onboard control panel once the AERIXIS stack is up
Exec=$SCRIPTS/open_panel.sh 8090
X-GNOME-Autostart-enabled=true
EOF
    say "Desktop login will open the onboard panel ($HOME/.config/autostart/aerixis-panel.desktop)"
  else
    say "No desktop on this computer: open http://<this-computer>:8090 from a laptop or phone"
  fi
}

case "${1:-start}" in
  start) shift || true; start "$@" ;;
  stop) stop ;;
  restart) shift; stop; start "$@" ;;
  status)
    if installed; then bash "$SCRIPTS/as2_pi.sh" status; else status_bg onboard; fi
    python3 "$SCRIPTS/as2_onboard_service.py" status 2>/dev/null || true ;;
  logs) if installed; then bash "$SCRIPTS/as2_pi.sh" logs; else logs_bg onboard; fi ;;
  token) token ;;
  panel) bash "$SCRIPTS/open_panel.sh" "${AS2_WEB_PORT:-8090}" ;;
  install) install ;;
  uninstall) bash "$SCRIPTS/as2_pi.sh" uninstall; rm -f "$HOME/.config/autostart/aerixis-panel.desktop" ;;
  -h|--help|help) sed -n '2,20p' "$0" ;;
  *) sed -n '2,20p' "$0"; exit 2 ;;
esac
