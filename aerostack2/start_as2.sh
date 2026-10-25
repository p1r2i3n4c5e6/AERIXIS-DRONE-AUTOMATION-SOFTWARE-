#!/usr/bin/env bash
# =============================================================================
# Aerostack2 (AS2) Stack Launcher
# Connects to PX4/ArduPilot via Micro XRCE-DDS and to Custom GCS via MAVLink.
# =============================================================================
set -e

WORKSPACE="/home/prince/aerostack2"
DRONE_NS="drone0"
TMUX_SESSION="as2_stack"

echo "=========================================================="
echo " Starting AERIXIS core stack"
echo " Drone Namespace: ${DRONE_NS}"
echo " MicroXRCE-DDS  : UDP port 8888"
echo " MAVLink Gateway: UDP port 14550 (sends to GCS on 14551)"
echo "=========================================================="

# Check ROS 2 and workspace
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  echo "Error: /opt/ros/humble/setup.bash not found!" >&2
  exit 1
fi
if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  echo "Error: $WORKSPACE/install/setup.bash not found! Run colcon build first." >&2
  exit 1
fi

SETUP_CMD="source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash"

# Kill existing AS2 session if running
tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true

# 1. Start Micro XRCE-DDS Agent
echo "[1/7] Launching Micro XRCE-DDS Agent (port 8888)..."
tmux new-session -d -s "$TMUX_SESSION" -n xrce_agent \
  "bash -c 'export LD_LIBRARY_PATH=$WORKSPACE/install/px4_microxrce_agent/lib:\${LD_LIBRARY_PATH:-}; exec $WORKSPACE/install/px4_microxrce_agent/bin/MicroXRCEAgent udp4 -p 8888'"
sleep 1

# 2. Launch AS2 Platform Pixhawk
echo "[2/7] Launching AERIXIS Platform Pixhawk..."
tmux new-window -t "$TMUX_SESSION" -n platform \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_platform_pixhawk pixhawk_launch.py namespace:=$DRONE_NS platform_config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml'"
sleep 2

# 3. Launch AS2 State Estimator
echo "[3/7] Launching AERIXIS State Estimator..."
tmux new-window -t "$TMUX_SESSION" -n estimator \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_state_estimator state_estimator_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=raw_odometry'"
sleep 1

# 4. Launch AS2 Motion Controller
echo "[4/7] Launching AERIXIS Motion Controller..."
tmux new-window -t "$TMUX_SESSION" -n controller \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_motion_controller controller_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=pid_speed_controller plugin_config_file:=$WORKSPACE/projects/px4_vision/config/pid_speed_controller.yaml'"
sleep 1

# 5. Launch AS2 Motion Behaviors
echo "[5/8] Launching AERIXIS Motion Behaviors..."
tmux new-window -t "$TMUX_SESSION" -n behaviors \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_behaviors_motion motion_behaviors_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml use_sim_time:=false'"
sleep 1

# 6. Launch AS2 Trajectory Generation Behavior
echo "[6/8] Launching AERIXIS Trajectory Generation Behavior..."
tmux new-window -t "$TMUX_SESSION" -n traj_gen \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=dynamic_mav_trajectory_generator use_sim_time:=false'"
sleep 1

# 7. Launch Mission Executor
echo "[7/8] Launching AERIXIS Mission Executor..."
tmux new-window -t "$TMUX_SESSION" -n mission \
  "bash -c '$SETUP_CMD; exec $WORKSPACE/install/as2_python_api/lib/as2_python_api/mission_executor --n $DRONE_NS'"
sleep 1

# 8. Launch AS2 MAVLink Gateway (bridges to custom GCS)
echo "[8/8] Launching AERIXIS MAVLink Gateway..."
tmux new-window -t "$TMUX_SESSION" -n gateway \
  "bash -c '$SETUP_CMD; exec ros2 run as2_mavlink_gateway mavlink_gateway --ros-args -p drone_namespace:=$DRONE_NS -p bind_address:=0.0.0.0 -p udp_port:=14550 -p gcs_host:=127.0.0.1 -p gcs_port:=14551 -p home_latitude:=47.397742 -p home_longitude:=8.545594 -p home_altitude:=488.0 -p use_sim_time:=false'"

echo "=========================================================="
echo " AERIXIS core stack launched successfully in tmux session: ${TMUX_SESSION}"
echo " To view/monitor windows : tmux attach -t ${TMUX_SESSION}"
echo " To detach from tmux     : Press Ctrl+b then d"
echo " To stop the stack       : bash $WORKSPACE/stop_all.sh"
echo "=========================================================="
