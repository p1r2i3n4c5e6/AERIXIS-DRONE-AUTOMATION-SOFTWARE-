#!/usr/bin/env bash
# Onboard container entrypoint.
#   service   (default) headless onboard service: every link, supervised
#   gui       removed: open the web panel instead (http://<pi>:8090, or GCS > Onboard calibration)
#   token     print drone ID, system ID and the GCS pairing token
#   status    print link states
#   shell     interactive bash with ROS, AS2 and the rescue overlay sourced
#   anything else is run as a command in that environment
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/as2/install/setup.bash
source /opt/as2/rescue_ws/install/setup.bash
mkdir -p "$HOME/.config/as2" "$HOME/.local/share"
SCRIPTS=/opt/as2/rescue_ws/scripts
case "${1:-service}" in
  service) shift || true; exec python3 "$SCRIPTS/as2_onboard_service.py" run "$@" ;;
  gui)     echo "The launcher window is not in this image: open the web panel at http://<pi-ip>:8090" >&2; exit 2 ;;
  token)   exec python3 "$SCRIPTS/as2_onboard_service.py" token ;;
  status)  exec python3 "$SCRIPTS/as2_onboard_service.py" status ;;
  config)  exec python3 "$SCRIPTS/as2_onboard_service.py" config ;;
  # PX4 fail-safe + per-drone return height (step = the separation setting): failsafe RTL [/dev/ttyACM0]
  failsafe)
    step="$(python3 "$SCRIPTS/as2_onboard_service.py" config | python3 -c 'import json,sys; print(float(json.load(sys.stdin)["separation_m"]))')"
    exec python3 "$SCRIPTS/px4_failsafe_params.py" --mavlink "${3:-/dev/ttyACM0}" --action "${2:?RTL or LAND}" --return-step "$step" ;;
  shell)   exec bash ;;
  *)       exec "$@" ;;
esac
