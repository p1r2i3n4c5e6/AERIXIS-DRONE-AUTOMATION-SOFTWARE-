#!/usr/bin/env bash
# Onboard side of a SITL session, as each Pi would run it, for PX4 SITL already running
# (./3_launch_simulation.sh [N]). Ctrl+C stops everything this script started.
#
#   rescue_ws/scripts/sitl_onboard.sh ["x,y;x,y" simulated people] [lose_after_s]
#   DRONES=3 rescue_ws/scripts/sitl_onboard.sh       # three aircraft (PX4 instances 0..2)
#
# Per aircraft droneI (system ID I+1): FC guardian (started first; RTL/LAND straight to PX4 if AS2
# stops, EMERGENCY_ACTION=RTL|LAND, also written to PX4's offboard-loss failsafe COM_OBL_RC_ACT),
# AS2 core + rescue runtime (SITL profile), simulated A8
# tracker, onboard mission manager (flight allowed, port 8790+I), FC health tap (PX4 MAVLink
# 14540+I), MAVLink gateway. One Micro XRCE-DDS agent (UDP 8888) serves every PX4 instance.
# With more than one aircraft the gateways sit behind sitl_radio_hub.py, which behaves like
# one multipoint telemetry radio, and a UDP mesh links the aircraft (never the GCS).
# The GCS always connects to UDP 14555 (PX4 SITL itself uses 14550):
#   GATEWAY_PORT=14555 /home/prince/custom_gcs/start_gcs.sh
# then Connect -> Simulation -> "Add aircraft manually" with the values from launch/sim.sh pairing.
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"; RESCUE="$ROOT/rescue_ws"
PERSONS="${1:-8,-2;-6,6}"; LOSE="${2:-0}"; GATEWAY_PORT="${GATEWAY_PORT:-14555}"
EMERGENCY_ACTION="${EMERGENCY_ACTION:-RTL}"
# Fleet separation for the gate and AI avoidance: 3 m in SITL (as on RTK drones); SEPARATION=10 for
# ordinary-GNSS behaviour.
SEPARATION="${SEPARATION:-3.0}"
# Same SITL_HOME as ./3_launch_simulation.sh (default: PX4's Zurich home).
IFS=, read -r HOME_LAT HOME_LON HOME_ALT <<< "${SITL_HOME:-47.397742,8.545594,488.0}"; HOME_ALT="${HOME_ALT:-0}"
if [ -z "${DRONES:-}" ]; then DRONES=$(ls -d /tmp/px4_instance_* 2>/dev/null | wc -l); [ "$DRONES" -ge 1 ] || DRONES=1; fi
STATE="${AS2_SITL_STATE:-$HOME/.local/share/as2-sitl}"; LOGS="$STATE/logs"
mkdir -p "$LOGS" "$STATE/missions"; chmod 700 "$STATE/missions"
source /opt/ros/humble/setup.bash; source "$ROOT/install/setup.bash"; source "$RESCUE/install/setup.bash"
# A pip-installed numpy 2.x in ~/.local breaks the system scipy that AS2's Python API imports;
# put the system packages first (user-only packages such as pymavlink are still found).
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
PIDS=()
start() { local name=$1; shift; setsid "$@" > "$LOGS/$name.log" 2>&1 < /dev/null & PIDS+=($!); echo "  started $name (log $LOGS/$name.log)"; }
stop_all() {
  echo; echo "Stopping onboard SITL stack..."
  for pid in "${PIDS[@]}"; do kill -INT -- -"$pid" 2>/dev/null; done; sleep 4
  for pid in "${PIDS[@]}"; do kill -KILL -- -"$pid" 2>/dev/null; done; exit 0
}
trap stop_all INT TERM

NAMES=(); for ((i=0; i<DRONES; i++)); do NAMES+=("drone$i"); done
PEERS_YAML="[$(printf "'%s'," "${NAMES[@]}" | sed 's/,$//')]"
# One shared SITL mesh key (the field fleet uses the key written by the Pi launcher).
MESH_KEY="$STATE/mesh_key.json"
python3 - "$MESH_KEY" "${NAMES[@]}" <<'PY'
import json, os, secrets, sys
path, names = sys.argv[1], sys.argv[2:]
key = json.load(open(path))['send_key'] if os.path.exists(path) else secrets.token_urlsafe(32)
fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
os.write(fd, json.dumps(dict(send_key=key, receive_keys={n: key for n in names + ['base']})).encode())
PY

echo "Onboard SITL stack for $DRONES aircraft"
start xrce_agent bash -c "export LD_LIBRARY_PATH=$ROOT/install/px4_microxrce_agent/lib:\$LD_LIBRARY_PATH; exec $ROOT/install/px4_microxrce_agent/bin/MicroXRCEAgent udp4 -p 8888"
sleep 3
GATEWAYS=()
FLEET_JSON="["
for ((i=0; i<DRONES; i++)); do
  DRONE=drone$i; SYSID=$((i+1)); MANAGER_PORT=$((8790+i))
  mkdir -p "$STATE/missions/$DRONE" "$STATE/rescue/$DRONE" "$STATE/payload/$DRONE"; chmod 700 "$STATE/missions/$DRONE"
  cat > "$STATE/${DRONE}_sitl.yaml" <<YAML
/**/rescue_runtime: {ros__parameters: {minimum_separation_m: $SEPARATION, enable_motion: true, allow_operator_load: true, overhead_envelope_verified: true, min_overhead_agl_m: 5.0, max_overhead_agl_m: 20.0, state_dir: $STATE/rescue/$DRONE}}
/**/rescue_route_publisher: {ros__parameters: {origin_source: estimator, altitude_frame_verified: true, path_clear_verified: true, bundle_directory: $STATE/missions/$DRONE}}
/**/payload_aux_adapter: {ros__parameters: {enabled: true, allow_operator_load: true, actuator_set: 1, release_value: 1.0, target_system: $SYSID, mission_directory: $STATE/missions/$DRONE, state_dir: $STATE/payload/$DRONE}}
/**/swarm_agent: {ros__parameters: {minimum_separation_m: $SEPARATION, expected_peers: $([ "$DRONES" -gt 1 ] && echo "$PEERS_YAML" || echo "['']")}}
YAML
  TOKEN="$STATE/missions/$DRONE/pairing.token"
  [ -f "$TOKEN" ] || python3 -c "import os,secrets,sys; fd=os.open(sys.argv[1],os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600); os.write(fd,(secrets.token_urlsafe(48)+'\n').encode())" "$TOKEN"
  echo "-- $DRONE (system ID $SYSID, mission manager :$MANAGER_PORT, emergency $EMERGENCY_ACTION)"
  python3 "$RESCUE/scripts/px4_failsafe_params.py" --sitl-instance $i --action "$EMERGENCY_ACTION" --return-step "$SEPARATION" \
    || echo "  WARNING: PX4 offboard-loss failsafe not confirmed for $DRONE"
  start ${DRONE}_guardian ros2 run as2_rescue_runtime fc_guardian --ros-args -r __ns:=/$DRONE -p fmu_prefix:=/$DRONE \
    -p target_system:=$SYSID -p emergency_action:=$EMERGENCY_ACTION
  start ${DRONE}_onboard ros2 launch as2_rescue_runtime onboard.launch.py drone_id:=$DRONE fmu_prefix:=/$DRONE target_system_id:=$SYSID \
    override:="$STATE/${DRONE}_sitl.yaml" fc_mavlink_url:=udpin:0.0.0.0:$((14540+i))
  start ${DRONE}_sim_vision ros2 run as2_a8_tracker a8_sim_vision --ros-args -r __ns:=/$DRONE -p "persons:=$PERSONS" -p lose_after_s:=$(python3 -c "print(float('$LOSE'))")
  start ${DRONE}_mission_manager env AS2_MISSION_STATE_ROOT="$STATE/missions" bash "$RESCUE/scripts/start_mission_manager.sh" $DRONE --allow-flight --port $MANAGER_PORT
  if [ "$DRONES" -gt 1 ]; then
    BIND=$((14570+i)); GCS_PORT=$GATEWAY_PORT; GATEWAYS+=($BIND)   # 14560 is named in PX4's SITL params
    # Mesh ports below Linux's ephemeral range (32768-60999): at 47800+ a guardian's outgoing UDP
    # socket once took drone1's mesh port and its mesh bridge died at start (SITL 2026-10-23).
    MESH_PEERS=$(for ((j=0; j<DRONES; j++)); do [ $j -ne $i ] && printf "'127.0.0.1:%d'," $((24800+j)); done | sed 's/,$//')
    start ${DRONE}_mesh ros2 run as2_rescue_runtime mesh_bridge --ros-args -r __ns:=/$DRONE -p identity:=$DRONE -p key_file:=$MESH_KEY \
      -p transport:=udp -p bind_port:=$((24800+i)) -p "peers:=[$MESH_PEERS]"
  else
    BIND=$GATEWAY_PORT; GCS_PORT=14551
  fi
  start ${DRONE}_gateway ros2 run as2_mavlink_gateway mavlink_gateway --ros-args -r __node:=mavlink_gateway_$DRONE -p drone_namespace:=$DRONE -p system_id:=$SYSID \
    -p fleet_token_file:="$TOKEN" -p fleet_endpoint:=http://127.0.0.1:$MANAGER_PORT -p bind_address:=0.0.0.0 -p udp_port:=$BIND \
    -p gcs_host:=127.0.0.1 -p gcs_port:=$GCS_PORT -p home_latitude:=$HOME_LAT -p home_longitude:=$HOME_LON -p home_altitude:=$HOME_ALT
  FLEET_JSON+="{\"drone_id\":\"$DRONE\",\"system_id\":$SYSID,\"manager_port\":$MANAGER_PORT},"
done
FLEET_JSON="${FLEET_JSON%,}]"
# Index of the simulated aircraft (for launch/sim.sh; the GCS never reads it).
echo "$FLEET_JSON" > "$STATE/sitl_fleet.json"
if [ "$DRONES" -gt 1 ]; then
  start radio_hub python3 "$RESCUE/scripts/sitl_radio_hub.py" --stats --listen $GATEWAY_PORT --gcs 127.0.0.1:14551 --gateways "$(IFS=,; echo "${GATEWAYS[*]}")"
fi
sleep 15
# Clear what a previous session left latched (manager lockout, runtime abort). The stacks can take
# well over 30 s to settle on a loaded laptop, so keep trying for 2 minutes and say how it went.
for ((i=0; i<DRONES; i++)); do
  ok=0
  for k in $(seq 1 24); do python3 "$RESCUE/scripts/mission_cli.py" reconcile --manager http://127.0.0.1:$((8790+i)) \
    --token-file "$STATE/missions/drone$i/pairing.token" >/dev/null 2>&1 && { ok=1; break; }; sleep 5; done
  [ "$ok" = 1 ] && echo "  drone$i reconciled on the ground" || echo "  WARNING: drone$i not reconciled — press Reconcile on its card in the GCS"
done
echo
echo "Ready: $DRONES aircraft. GCS: Connect -> Simulation -> 'Add aircraft manually' (values: launch/sim.sh pairing)."
echo "Manual add: drone ID droneI, transport MAVLink, system ID I+1, token from $STATE/missions/droneI/pairing.token"
echo "Simulated people (plan frame, metres): $PERSONS    Track log: ~/.local/share/as2-rescue-logs/"
echo "Press Ctrl+C here to stop the onboard stack."
while true; do sleep 10; done
