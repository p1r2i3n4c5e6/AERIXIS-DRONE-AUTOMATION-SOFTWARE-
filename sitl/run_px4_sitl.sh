#!/usr/bin/env bash
# Launch PX4 SITL with Gazebo Garden directly (no cmake reinvocation).
# PX4 binary must already be built by setup_px4_native.sh.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
px4_dir="${PX4_AUTOPILOT_DIR:-$script_dir/PX4-Autopilot}"
px4_bin="$px4_dir/build/px4_sitl_default/bin/px4"
rootfs="$px4_dir/build/px4_sitl_default"

if [[ ! -x "$px4_bin" ]]; then
  printf 'PX4 SITL is not built at: %s\n' "$px4_dir" >&2
  printf 'Run once to build it:\n  %s/setup_px4_native.sh\n' "$script_dir" >&2
  exit 1
fi

# Gazebo Garden (7) and Harmonic (8) are both installed: PX4 v1.14 needs Garden, v1.16 Harmonic.
if git -C "$px4_dir" describe --tags 2>/dev/null | grep -q '^v1\.14'; then
  export PATH="$script_dir/gz-garden:$PATH"
fi

if ! command -v gz > /dev/null 2>&1 || ! gz sim --version 2>/dev/null | grep -q 'version'; then
  printf 'Gazebo Garden (gz sim) is missing. Run: %s/setup_px4_native.sh\n' "$script_dir" >&2
  exit 1
fi

# ---------- tunable environment variables ----------
export PX4_SIMULATOR="${PX4_SIMULATOR:-gz}"
export PX4_UXRCE_DDS_NS="${PX4_UXRCE_DDS_NS:-drone0}"
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4001}"     # 4001 = gz x500 quadrotor
export PX4_GZ_MODEL="${PX4_GZ_MODEL:-x500}"
export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-0,0,0.5,0,0,0}"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-as2_playground}"

# PX4 >= 1.15 runs on Gazebo Harmonic: AeroStack2's Gazebo assets/plugins on these paths are built
# for Garden and stopped Harmonic's sensors from publishing (no IMU -> "Accel Sensor 0 missing",
# 2026-10-24), so a non-1.14 tree gets PX4's own models only.
if ! git -C "$px4_dir" describe --tags 2>/dev/null | grep -q '^v1\.14'; then
  unset GZ_SIM_SYSTEM_PLUGIN_PATH GZ_VERSION GZ_SIM_RESOURCE_PATH
fi
# PX4 >= 1.15: the world SDF loads no systems; they come from server.config, PX4's plugins from
# build/.../gz_plugins. `make px4_sitl gz_x500` sources this file; running the binary directly
# did not, so Gazebo published no IMU/baro/mag at all (PX4 1.16 + Harmonic, 2026-10-24).
if [[ -f "$rootfs/rootfs/gz_env.sh" ]]; then
  # shellcheck disable=SC1091
  set +u; source "$rootfs/rootfs/gz_env.sh"; set -u   # it appends to possibly unset variables
fi

# Gazebo resource paths: PX4 models/worlds + any extras
export PX4_GZ_MODELS="$px4_dir/Tools/simulation/gz/models"
export PX4_GZ_WORLDS="$px4_dir/Tools/simulation/gz/worlds"
export GZ_SIM_RESOURCE_PATH="$PX4_GZ_MODELS:$PX4_GZ_WORLDS${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# PX4 lockstep requires a work dir for params/logs; use a per-instance scratch dir.
instance="${PX4_INSTANCE:-0}"
work_dir="${PX4_WORK_DIR:-/tmp/px4_instance_${instance}}"
rm -rf "$work_dir" && mkdir -p "$work_dir"

export CCACHE_DISABLE=1

printf '[run_px4_sitl] Starting PX4 SITL + %s\n' "$(gz sim --version 2>/dev/null | head -1)"
printf '  Binary  : %s\n' "$px4_bin"
printf '  World   : %s\n' "$PX4_GZ_WORLD"
printf '  Model   : %s @ %s\n' "$PX4_GZ_MODEL" "$PX4_GZ_MODEL_POSE"
printf '  DDS ns  : %s\n' "$PX4_UXRCE_DDS_NS"
printf '  Work dir: %s\n' "$work_dir"

# PX4 SITL expects: px4 [-d] [-s startup_file] [-i instance] [-w workdir] [data_path]
# data_path must point to the etc directory containing init.d-posix/rcS
exec "$px4_bin" -d -s etc/init.d-posix/rcS -i "$instance" -w "$work_dir" "$rootfs/etc"
