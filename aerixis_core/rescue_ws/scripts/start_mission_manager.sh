#!/usr/bin/env bash
# Observation-only unless the caller explicitly supplies --allow-flight.
set -e
rescue_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
as2_root="${AS2_WORKSPACE:-$(dirname -- "$rescue_root")}"
if [[ $# -lt 1 ]]; then
  echo 'Usage: start_mission_manager.sh drone0 [--host IP --tls-cert FILE --tls-key FILE ...]' >&2
  exit 2
fi
drone_id="$1"
shift
if [[ ! "$drone_id" =~ ^[A-Za-z_][A-Za-z0-9_]{0,63}$ ]]; then
  echo 'Use a unique ROS namespace such as drone0.' >&2
  exit 2
fi
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$as2_root/install/setup.bash"
source "$rescue_root/install/setup.bash"
state_root="${AS2_MISSION_STATE_ROOT:-$HOME/.local/share/as2-missions}"
state_dir="$state_root/$drone_id"
mkdir -p -- "$state_dir"
chmod 700 "$state_dir"
token_file="$state_dir/pairing.token"
if [[ ! -e "$token_file" ]]; then
  python3 - "$token_file" <<'PY'
import os, secrets, sys
fd = os.open(sys.argv[1], os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
with os.fdopen(fd, 'w') as stream:
    stream.write(secrets.token_urlsafe(48)+'\n')
PY
fi
runner="$(ros2 pkg prefix as2_behavior_tree)/lib/as2_behavior_tree/as2_behavior_tree_node"
exec ros2 run as2_mission_manager mission_manager --drone-id "$drone_id" \
  --runner "$runner" --state-dir "$state_dir" --token-file "$token_file" "$@"
