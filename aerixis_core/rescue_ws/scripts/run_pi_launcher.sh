#!/usr/bin/env bash
# Start the AERIXIS onboard launcher window. It only connects links and starts the AERIXIS core; it never
# arms, flies or releases a payload by itself.
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
as2="${AS2_WORKSPACE:-$(dirname -- "$root")}"
export AS2_WORKSPACE="$as2"
# ROS + workspaces sourced here so the window can show live AERIXIS status.
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[ -f "$as2/install/setup.bash" ] && source "$as2/install/setup.bash"
[ -f "$root/install/setup.bash" ] && source "$root/install/setup.bash"
exec python3 "$root/scripts/as2_pi_launcher.py" "$@"
