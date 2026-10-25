#!/usr/bin/env bash
# =============================================================================
# LAUNCHER 2: AeroStack2 (AS2) Full System Stack (Interactive Terminal)
#
# Features:
#   - Micro-XRCE-DDS Autopilot Connection (SITL UDP vs. Real Hardware Serial)
#   - Live ROS 2 logs stream directly in this terminal
#   - Press [Ctrl+C] in this terminal to STOP all AS2 nodes instantly
# =============================================================================
set -e

WORKSPACE="/home/prince/aerostack2"
DRONE_NS="drone0"

# Check ROS 2 setup
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  echo "Error: /opt/ros/humble/setup.bash not found!" >&2
  exit 1
fi
if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  echo "Error: $WORKSPACE/install/setup.bash not found! Run colcon build first." >&2
  exit 1
fi

SETUP_CMD="source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash && export PYTHONPATH=/usr/lib/python3/dist-packages:\$PYTHONPATH"

# Parse connection mode (Option 1 = SITL, Option 2 = Real Hardware)
MODE_CHOICE="${1:-}"

if [ -z "$MODE_CHOICE" ]; then
  echo "=========================================================="
  echo " Select Autopilot (Micro-XRCE-DDS) Connection Mode:"
  echo "  [1] PX4 SITL / Gazebo Simulation (UDP port 8888)"
  echo "  [2] Real Flight Controller (Pixhawk/Cube via Serial)"
  echo "=========================================================="
  read -t 5 -p "Enter selection [1 or 2, default: 1]: " USER_INPUT || true
  echo ""
  MODE_CHOICE="${USER_INPUT:-1}"
fi

case "$MODE_CHOICE" in
  2|real|hardware)
    SERIAL_PORT="${2:-/dev/ttyACM0}"
    SERIAL_BAUD="${3:-921600}"
    echo ">>> [MODE: REAL FLIGHT CONTROLLER] Using Serial: $SERIAL_PORT @ $SERIAL_BAUD baud"
    XRCE_EXEC="export LD_LIBRARY_PATH=$WORKSPACE/install/px4_microxrce_agent/lib:\${LD_LIBRARY_PATH:-} && exec $WORKSPACE/install/px4_microxrce_agent/bin/MicroXRCEAgent serial --dev $SERIAL_PORT -b $SERIAL_BAUD"
    ;;
  *)
    echo ">>> [MODE: PX4 SITL SIMULATION] Using UDP port 8888"
    XRCE_EXEC="export LD_LIBRARY_PATH=$WORKSPACE/install/px4_microxrce_agent/lib:\${LD_LIBRARY_PATH:-} && exec $WORKSPACE/install/px4_microxrce_agent/bin/MicroXRCEAgent udp4 -p 8888"
    ;;
esac

# Pre-clean any leftover processes
killall -9 MicroXRCEAgent as2_platform_pixhawk_node as2_state_estimator_node as2_motion_controller_node follow_path_behavior_node go_to_behavior_node land_behavior_node takeoff_behavior_node generate_polynomial_trajectory_behavior_node mission_executor mavlink_gateway 2>/dev/null || true
sleep 0.5

# PIDs tracker
CHILD_PIDS=()

cleanup() {
  echo ""
  echo "=========================================================="
  echo ">>> [TERMINAL] Stopping all AERIXIS core components..."
  echo "=========================================================="
  for pid in "${CHILD_PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 0.5
  killall -9 MicroXRCEAgent as2_platform_pixhawk_node as2_state_estimator_node as2_motion_controller_node follow_path_behavior_node go_to_behavior_node land_behavior_node takeoff_behavior_node generate_polynomial_trajectory_behavior_node mission_executor mavlink_gateway 2>/dev/null || true
  echo ">>> [STOPPED] All AERIXIS nodes and controllers stopped cleanly."
  exit 0
}

# Only trap manual interruption (Ctrl+C) or termination signal, NOT normal subshell EXIT
trap cleanup INT TERM

# Disable exit on error so minor background node exit does not abort the entire stack
set +e

echo "=========================================================="
echo " [LAUNCHER 2/3] Launching AERIXIS core stack"
echo " Drone Namespace  : ${DRONE_NS}"
echo " MAVLink Gateway  : UDP port 14550 -> GCS port 14551"
echo " Live Logs        : Streaming in this terminal below"
echo " Stop Control     : Press [Ctrl+C] in this terminal to STOP ALL"
echo "=========================================================="

# 1. Micro XRCE-DDS Agent
echo "[1/8] Starting Micro XRCE-DDS Agent..."
bash -c "$XRCE_EXEC" 2>&1 | sed -u 's/^/[XRCE] /' &
CHILD_PIDS+=($!)
sleep 1

# 2. Platform Pixhawk
echo "[2/8] Launching AERIXIS Platform Pixhawk..."
bash -c "$SETUP_CMD && exec ros2 launch as2_platform_pixhawk pixhawk_launch.py namespace:=$DRONE_NS platform_config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml" 2>&1 | sed -u 's/^/[PLATFORM] /' &
CHILD_PIDS+=($!)
sleep 1.5

# 3. State Estimator
echo "[3/8] Launching AERIXIS State Estimator..."
bash -c "$SETUP_CMD && exec ros2 launch as2_state_estimator state_estimator_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=raw_odometry" 2>&1 | sed -u 's/^/[ESTIMATOR] /' &
CHILD_PIDS+=($!)
sleep 1

# 4. Motion Controller
echo "[4/8] Launching AERIXIS Motion Controller..."
bash -c "$SETUP_CMD && exec ros2 launch as2_motion_controller controller_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=pid_speed_controller plugin_config_file:=$WORKSPACE/projects/px4_vision/config/pid_speed_controller.yaml" 2>&1 | sed -u 's/^/[CONTROLLER] /' &
CHILD_PIDS+=($!)
sleep 1

# 5. Motion Behaviors
echo "[5/8] Launching AERIXIS Motion Behaviors (Takeoff, Land, GoTo, FollowPath)..."
bash -c "$SETUP_CMD && exec ros2 launch as2_behaviors_motion motion_behaviors_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml use_sim_time:=false" 2>&1 | sed -u 's/^/[BEHAVIORS] /' &
CHILD_PIDS+=($!)
sleep 1

# 6. Trajectory Generation
echo "[6/8] Launching AERIXIS Trajectory Generation..."
bash -c "$SETUP_CMD && exec ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=dynamic_mav_trajectory_generator use_sim_time:=false" 2>&1 | sed -u 's/^/[TRAJ_GEN] /' &
CHILD_PIDS+=($!)
sleep 1

# 7. Mission Executor
echo "[7/8] Launching AERIXIS Mission Executor..."
bash -c "$SETUP_CMD && exec $WORKSPACE/install/as2_python_api/lib/as2_python_api/mission_executor --n $DRONE_NS" 2>&1 | sed -u 's/^/[MISSION] /' &
CHILD_PIDS+=($!)
sleep 1

# 8. MAVLink Gateway
echo "[8/8] Launching AERIXIS MAVLink Gateway..."
bash -c "$SETUP_CMD && exec ros2 run as2_mavlink_gateway mavlink_gateway --ros-args -p drone_namespace:=$DRONE_NS -p bind_address:=0.0.0.0 -p udp_port:=14550 -p gcs_host:=127.0.0.1 -p gcs_port:=14551 -p home_latitude:=47.397742 -p home_longitude:=8.545594 -p home_altitude:=488.0 -p use_sim_time:=false" 2>&1 | sed -u 's/^/[GATEWAY] /' &
CHILD_PIDS+=($!)

echo ""
echo "=========================================================="
echo ">>> [SUCCESS] All 8 AERIXIS core components are running!"
echo ">>> Live logs streaming below. Press [Ctrl+C] anytime to stop."
echo "=========================================================="

while true; do
  sleep 2
done
