#!/usr/bin/env bash
# World preview. Pass -s --headless-rendering to run without a window.
set -euo pipefail
arena_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$arena_dir/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
exec gz sim --force-version "${ARENA_GZ_VERSION:-8}" -r \
  "$arena_dir/worlds/rescue_arena_300.sdf" "$@"
