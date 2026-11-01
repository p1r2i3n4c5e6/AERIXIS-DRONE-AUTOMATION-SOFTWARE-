#!/usr/bin/env bash
# Start this world and the existing, built PX4 binary. No source-tree changes.
set -euo pipefail
arena_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
px4_dir="${PX4_AUTOPILOT_DIR:-$HOME/px4_sitl/PX4-Autopilot-1.16}"
px4_build="$px4_dir/build/px4_sitl_default"
px4_bin="$px4_build/bin/px4"
if [[ ! -x "$px4_bin" ]]; then
  printf 'Built PX4 not found: %s\nSet PX4_AUTOPILOT_DIR to your built checkout.\n' "$px4_bin" >&2
  exit 1
fi
if [[ -f "$px4_dir/ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim" ]]; then
  sim_major=8
  transport_major=13
else
  sim_major=7
  transport_major=12
fi
# Pin all commands PX4 calls to the matching Gazebo / transport generation.
# A unique work directory preserves logs and never deletes another SITL instance.
arena_run_dir="$(mktemp -d "${TMPDIR:-/tmp}/rescue_arena_px4.XXXXXX")"
mkdir -p "$arena_run_dir/bin" "$arena_run_dir/work"
gz_real="$(command -v gz)"
cat > "$arena_run_dir/bin/gz" <<EOF
#!/usr/bin/env bash
command_name="\$1"; shift
case "\$command_name" in
  sim|gui|model) exec "$gz_real" "\$command_name" --force-version "$sim_major" "\$@" ;;
  service|topic|log) exec "$gz_real" "\$command_name" --force-version "$transport_major" "\$@" ;;
  *) exec "$gz_real" "\$command_name" "\$@" ;;
esac
EOF
chmod +x "$arena_run_dir/bin/gz"
export PATH="$arena_run_dir/bin:$PATH"
# Do not mix AS2 Garden plugins with PX4 Harmonic plugins.
unset GZ_SIM_SYSTEM_PLUGIN_PATH GZ_SIM_SERVER_CONFIG_PATH GZ_VERSION
export GZ_SIM_RESOURCE_PATH="$px4_dir/Tools/simulation/gz/models:$arena_dir/worlds"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$px4_build/src/modules/simulation/gz_plugins"
export PX4_GZ_MODELS="$px4_dir/Tools/simulation/gz/models"
export PX4_GZ_WORLDS="$arena_dir/worlds"
export PX4_GZ_WORLD=rescue_arena_300
export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-0,0,0.5,0,0,0}"
export PX4_SIMULATOR=gz
export PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4001}"
export PX4_UXRCE_DDS_NS="${PX4_UXRCE_DDS_NS:-drone0}"
if [[ "$sim_major" == 8 ]]; then
  export PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}"
  unset PX4_GZ_MODEL
else
  export PX4_GZ_MODEL="${PX4_GZ_MODEL:-x500}"
fi
unset PX4_GZ_MODEL_NAME
# Scope world discovery so PX4 cannot silently attach to another running world.
export GZ_PARTITION="${GZ_PARTITION:-rescue_arena_${USER:-user}}"
if gz topic -l | grep -q '^/world/'; then
  printf 'A world is already running in GZ_PARTITION=%s. Stop it or choose another partition.\n' "$GZ_PARTITION" >&2
  exit 1
fi
server_pid=''; gui_pid=''; px4_pid=''
cleanup() {
  trap - EXIT INT TERM
  for pid in "$px4_pid" "$gui_pid" "$server_pid"; do
    if [[ -n "$pid" ]]; then kill -TERM -- "-$pid" 2>/dev/null || true; fi
  done
  # Gazebo starts child processes. Stop only our process groups, with a bound.
  for ((n=0; n<30; n++)); do
    alive=0
    for pid in "$px4_pid" "$gui_pid" "$server_pid"; do
      if [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then alive=1; fi
    done
    if [[ "$alive" == 0 ]]; then break; fi
    sleep .1
  done
  for pid in "$px4_pid" "$gui_pid" "$server_pid"; do
    if [[ -n "$pid" ]]; then kill -KILL -- "-$pid" 2>/dev/null || true; fi
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
server_args=(-s -r -v 3)
if [[ -n "${HEADLESS:-}" ]]; then server_args+=(--headless-rendering); fi
setsid gz sim "${server_args[@]}" "$arena_dir/worlds/rescue_arena_300.sdf" > "$arena_run_dir/gazebo.log" 2>&1 &
server_pid=$!
ready=0
for ((attempt=0; attempt<30; attempt++)); do
  if ! kill -0 "$server_pid" 2>/dev/null; then
    cat "$arena_run_dir/gazebo.log" >&2
    exit 1
  fi
  if gz service -i -s /world/rescue_arena_300/scene/info 2>/dev/null | grep -q 'Service providers'; then
    ready=1; break
  fi
  sleep 1
done
if [[ "$ready" != 1 ]]; then
  printf 'Gazebo did not become ready. See %s/gazebo.log\n' "$arena_run_dir" >&2
  exit 1
fi
if [[ -z "${HEADLESS:-}" ]]; then
  setsid gz sim -g > "$arena_run_dir/gui.log" 2>&1 &
  gui_pid=$!
fi
export PX4_GZ_STANDALONE=1
printf 'Arena ready: 300 x 300 m, 50 humans, 180 trees.\nLogs: %s\nPartition: %s\n' "$arena_run_dir" "$GZ_PARTITION"
cd "$arena_run_dir/work"
setsid "$px4_bin" -d -s etc/init.d-posix/rcS -i "${PX4_INSTANCE:-0}" \
  -w "$arena_run_dir/work" "$px4_build/etc" &
px4_pid=$!
wait "$px4_pid"
