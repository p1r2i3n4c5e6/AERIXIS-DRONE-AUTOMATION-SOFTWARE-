#!/usr/bin/env bash
# One more PX4 drone in the running rescue arena (same Gazebo world, same 3D window).
#   add_px4.sh ARENA_RUN_DIR INDEX        (sim_info.json must already list INDEX+1 spawns)
set -euo pipefail
RUN="${1:?usage: add_px4.sh ARENA_RUN_DIR INDEX}"; INDEX="${2:?index}"
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/px4_env.sh"
gz service -i -s "/world/$WORLD/scene/info" 2>/dev/null | grep -q 'Service providers' \
  || { echo "The arena world is not running" >&2; exit 1; }
spawn_px4 "$INDEX"
