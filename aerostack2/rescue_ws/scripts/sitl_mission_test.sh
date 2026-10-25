#!/usr/bin/env bash
# One-command SITL test of a rescue mission on this laptop (no camera needed).
#
#   rescue_ws/scripts/sitl_mission_test.sh human_geotag_resume
#   rescue_ws/scripts/sitl_mission_test.sh human_track_rtl "8,-2" 15      # person hides after 15 s
#   rescue_ws/scripts/sitl_mission_test.sh human_orbit "8,-2" 0 --orbit-radius-m 8 --orbit-laps 1
#   rescue_ws/scripts/sitl_mission_test.sh human_geotag_drop_resume "8,-2;-6,6" 0 --payload-id kit1
#   rescue_ws/scripts/sitl_mission_test.sh swarm_coverage
#   rescue_ws/scripts/sitl_mission_test.sh stop
#
# Starts headless PX4 SITL + Micro XRCE-DDS agent, the onboard AERIXIS core/rescue stack with a
# simulation-only profile, the simulated A8 tracker (people at x,y metres in the plan frame
# of sample_field_mission.kml), the onboard mission manager, then plans, uploads, starts
# and follows the mission. State lives in ~/.local/share/as2-sitl, never in flight folders.
# (no "set -u": ROS setup scripts use unset variables)
TEMPLATE="${1:-human_geotag_resume}"; PERSONS="${2:-8,-2;-6,6}"; LOSE="${3:-0}"; shift $(( $# < 3 ? $# : 3 ))
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
RESCUE="$ROOT/rescue_ws"; PX4_DIR="${PX4_SITL_DIR:-$HOME/px4_sitl}"
STATE="${AS2_SITL_STATE:-$HOME/.local/share/as2-sitl}"; LOGS="$STATE/logs"; DRONE=drone0
# Own DDS domain: on the same Wi-Fi as a real drone (domain 0) the simulated PX4 and the real
# onboard nodes otherwise see each other's topics (seen 2026-10-24 with the Pi on the hotspot).
# PX4 SITL copies ROS_DOMAIN_ID into UXRCE_DDS_DOM_ID, so the agent follows too.
export ROS_DOMAIN_ID="${AS2_SITL_DOMAIN_ID:-77}"
# Gazebo transport on loopback: with Wi-Fi/hotspot changes it may advertise a stale interface and
# PX4's gz_bridge then gets no sensor data (PX4 1.16 + Harmonic, 2026-10-24).
export GZ_IP="${GZ_IP:-127.0.0.1}"
mkdir -p "$LOGS" "$STATE/missions/$DRONE" "$STATE/rescue" "$STATE/payload"; chmod 700 "$STATE/missions" "$STATE/missions/$DRONE"

stop_all() {
  for pattern in "ros2 launch as2_rescue_runtime onboard" "as2_platform_pixhawk_node" "as2_state_estimator_node" \
      "as2_motion_controller_node" "behavior_node" "rescue_ws/install/as2_rescue_runtime/lib" \
      "as2_behavior_tree_node --ros-args" "a8_sim_vision" "mission_manager --drone-id $DRONE" \
      "MicroXRCEAgent udp4" "px4_sitl_default/bin/px4 -d" "gz sim" "as2_rescue_rate_gate/rate_gate"; do
    pkill -INT -f -- "$pattern" 2>/dev/null; done
  sleep 4
  # Launches started in the background ignore SIGINT: kill what is left, rate gate included (an
  # orphaned gate from the previous run doubled every gated topic, 2026-10-25).
  for pattern in "as2_platform_pixhawk_node" "as2_state_estimator_node" "as2_motion_controller_node" "behavior_node" \
      "rescue_ws/install/as2_rescue_runtime/lib" "MicroXRCEAgent udp4" "px4_sitl_default/bin/px4 -d" "gz sim" \
      "as2_rescue_rate_gate/rate_gate" "ros2 launch as2_rescue_runtime onboard"; do
    pkill -KILL -f -- "$pattern" 2>/dev/null; done
}
if [ "$TEMPLATE" = stop ]; then stop_all; echo "SITL stack stopped"; exit 0; fi

source /opt/ros/humble/setup.bash; source "$ROOT/install/setup.bash"; source "$RESCUE/install/setup.bash"
stop_all
echo "[1/5] PX4 SITL (headless) + XRCE agent"
(cd "$PX4_DIR" && HEADLESS=1 PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}" setsid nohup bash run_px4_sitl.sh > "$LOGS/px4.log" 2>&1 < /dev/null &)
LD_LIBRARY_PATH="$ROOT/install/px4_microxrce_agent/lib:${LD_LIBRARY_PATH:-}" setsid nohup \
  "$ROOT/install/px4_microxrce_agent/bin/MicroXRCEAgent" udp4 -p 8888 > "$LOGS/agent.log" 2>&1 < /dev/null &
for i in $(seq 1 60); do grep -q "Ready for takeoff" "$LOGS/px4.log" 2>/dev/null && break; sleep 1; done

echo "[2/5] Onboard AERIXIS core + rescue stack (SITL profile: all hardware gates asserted for simulation only)"
cat > "$STATE/${DRONE}_sitl.yaml" <<EOF
/**/rescue_runtime: {ros__parameters: {enable_motion: true, allow_operator_load: true, overhead_envelope_verified: true, min_overhead_agl_m: 5.0, max_overhead_agl_m: 20.0, state_dir: $STATE/rescue}}
/**/rescue_route_publisher: {ros__parameters: {origin_source: estimator, altitude_frame_verified: true, path_clear_verified: true, bundle_directory: $STATE/missions/$DRONE}}
/**/payload_aux_adapter: {ros__parameters: {enabled: true, allow_operator_load: true, actuator_set: 1, release_value: 1.0, target_system: 1, mission_directory: $STATE/missions/$DRONE, state_dir: $STATE/payload}}
/**/swarm_agent: {ros__parameters: {expected_peers: ['']}}
EOF
setsid nohup ros2 launch as2_rescue_runtime onboard.launch.py drone_id:=$DRONE fmu_prefix:=/$DRONE target_system_id:=1 \
  override:="$STATE/${DRONE}_sitl.yaml" > "$LOGS/onboard.log" 2>&1 < /dev/null &
echo "[3/5] Simulated A8 tracker: people at $PERSONS"
setsid nohup ros2 run as2_a8_tracker a8_sim_vision --ros-args -r __ns:=/$DRONE -p "persons:=$PERSONS" \
  -p lose_after_s:=$(python3 -c "print(float('$LOSE'))") > "$LOGS/sim_vision.log" 2>&1 < /dev/null &
echo "[4/5] Onboard mission manager (flight allowed: simulation)"
AS2_MISSION_STATE_ROOT="$STATE/missions" setsid nohup bash "$RESCUE/scripts/start_mission_manager.sh" $DRONE --allow-flight \
  > "$LOGS/manager.log" 2>&1 < /dev/null &
sleep 18
TOKEN="$STATE/missions/$DRONE/pairing.token"
for i in 1 2 3 4 5 6; do python3 "$RESCUE/scripts/mission_cli.py" reconcile --token-file "$TOKEN" >/dev/null 2>&1 && break; sleep 5; done
echo "[5/5] Mission $TEMPLATE"
python3 -u "$RESCUE/scripts/mission_cli.py" run --kml "$ROOT/sample_field_mission.kml" --template "$TEMPLATE" \
  --altitude 12 --spacing 8 --speed 1.5 --token-file "$TOKEN" --timeout 900 "$@"
echo "Logs: $LOGS   Track log: $(ls -t ~/.local/share/as2-rescue-logs/track-*.jsonl 2>/dev/null | head -1)"
echo "The simulator keeps running; stop it with: $0 stop"
