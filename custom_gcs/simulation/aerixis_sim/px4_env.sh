# Shared Gazebo / PX4 1.16 environment for the rescue arena (sourced by arena_px4.sh, add_px4.sh,
# gazebo_gui.sh). Needs RUN = arena run directory from prepare_arena.py.
AERIXIS_SIM_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PX4_SITL_DIR="${PX4_SITL_DIR:-$HOME/px4_sitl}"
export PX4_AUTOPILOT_DIR="${PX4_AUTOPILOT_DIR:-$PX4_SITL_DIR/PX4-Autopilot-1.16}"
ROOTFS="$PX4_AUTOPILOT_DIR/build/px4_sitl_default"
LOGS="$HOME/.local/share/as2-sitl/logs"; mkdir -p "$LOGS"
[ -x "$ROOTFS/bin/px4" ] || { echo "PX4 1.16 SITL is not built at $PX4_AUTOPILOT_DIR" >&2; return 1 2>/dev/null || exit 1; }
set +u; source "$ROOTFS/rootfs/gz_env.sh"; set -u
export GZ_SIM_RESOURCE_PATH="$AERIXIS_SIM_DIR/models:$RUN/worlds:$GZ_SIM_RESOURCE_PATH"
# Render on the NVIDIA GPU when there is one (2026-10-31): on this hybrid laptop Gazebo drew the
# 3D window and every drone camera on the Intel iGPU (iris) while the RTX 4050 sat idle, which
# slowed the whole simulation. PRIME render offload: GLX for the 3D window, EGL for the server's
# headless camera sensors. AERIXIS_SIM_GPU=0 keeps the default GPU.
if [ "${AERIXIS_SIM_GPU:-1}" != 0 ] && command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 \
   && [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi
GZ_SIM="gz sim"; gz sim --versions 2>/dev/null | grep -q '^8\.' && GZ_SIM="gz sim --force-version 8"
read -r WORLD HOME_LAT HOME_LON HOME_ALT < <(python3 -c "
import json, sys
i = json.load(open(sys.argv[1])); h = i['home']
print(i['world'], h['latitude_deg'], h['longitude_deg'], h['altitude_m'])" "$RUN/sim_info.json")

# spawn_px4 INDEX: PX4 instance INDEX (droneINDEX, system ID INDEX+1) at its fixed slot, joining the
# running arena world. Runs in the foreground; callers start it in the background.
spawn_px4() {
  local i=$1 x y
  IFS=, read -r x y < <(python3 -c "import json,sys; s=json.load(open(sys.argv[1]))['spawns'][int(sys.argv[2])]; print(f\"{s['x']},{s['y']}\")" "$RUN/sim_info.json" "$i")
  echo ">>> drone$i (PX4 instance $i, system ID $((i+1))) at ($x, $y) m — log $LOGS/px4_$i.log"
  exec env PX4_INSTANCE=$i PX4_UXRCE_DDS_NS=drone$i PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 \
      PX4_SIM_MODEL=gz_x500_a8 PX4_GZ_MODELS="$AERIXIS_SIM_DIR/models" PX4_GZ_WORLDS="$RUN/worlds" PX4_GZ_WORLD="$WORLD" \
      PX4_GZ_MODEL_POSE="$x,$y,0.3,0,0,0" SITL_HOME_LAT=$HOME_LAT SITL_HOME_LON=$HOME_LON SITL_HOME_ALT=$HOME_ALT \
      bash "$PX4_SITL_DIR/run_px4_sitl.sh" > "$LOGS/px4_$i.log" 2>&1 < /dev/null
}
