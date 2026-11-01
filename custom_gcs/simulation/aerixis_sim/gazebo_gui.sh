#!/usr/bin/env bash
# The Gazebo 3D window for the running rescue arena (can be closed and reopened any time; closing
# it does not stop the simulation).   gazebo_gui.sh ARENA_RUN_DIR
set -euo pipefail
RUN="${1:?usage: gazebo_gui.sh ARENA_RUN_DIR}"
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/px4_env.sh"
exec $GZ_SIM -g
