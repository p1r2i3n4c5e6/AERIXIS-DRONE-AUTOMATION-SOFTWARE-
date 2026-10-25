#!/usr/bin/env bash
# =============================================================================
# AS2 + PX4 SITL + Custom GCS Full-Stack Startup Script
# Starts each decoupled component in named tmux windows:
#   1. Custom GCS (in /home/prince/custom_gcs)
#   2. PX4 SITL + Gazebo (in /home/prince/px4_sitl)
#   3. AS2 Stack (Micro-XRCE-DDS, Platform, Estimator, Controller, Behaviors, Gateway)
# =============================================================================
set -e

WORKSPACE="/home/prince/aerostack2"
GCS_DIR="/home/prince/custom_gcs"
PX4_DIR="/home/prince/px4_sitl"
SESSION="as2_sitl"
DRONE_NS="drone0"

# Kill existing session if present
tmux kill-session -t "$SESSION" 2>/dev/null || true

SETUP_CMD="source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash"

echo "=========================================================="
echo " Launching Full Stack into tmux session '$SESSION'"
echo " 1. AERIXIS GCS       : $GCS_DIR"
echo " 2. PX4 SITL Sim     : $PX4_DIR"
echo " 3. AERIXIS core stack : $WORKSPACE"
echo "=========================================================="

# Window 0: Custom GCS
echo "[1/10] Starting AERIXIS GCS web server (http://localhost:8765)..."
tmux new-session -d -s "$SESSION" -n gcs \
  "bash -c 'bash $GCS_DIR/start_gcs.sh; exec bash'"

sleep 1

# Window 1: PX4 SITL + Gazebo Garden
echo "[2/10] Starting PX4 SITL + Gazebo Garden..."
tmux new-window -t "$SESSION" -n px4_sitl \
  "bash -c '$SETUP_CMD; bash $PX4_DIR/run_px4_sitl.sh; exec bash'"

sleep 3

# Window 2: Micro XRCE-DDS Agent
echo "[3/10] Starting Micro XRCE-DDS Agent (port 8888)..."
tmux new-window -t "$SESSION" -n xrce_agent \
  "bash -c 'export LD_LIBRARY_PATH=$WORKSPACE/install/px4_microxrce_agent/lib:\${LD_LIBRARY_PATH:-}; exec $WORKSPACE/install/px4_microxrce_agent/bin/MicroXRCEAgent udp4 -p 8888'"

sleep 1

# Window 3: AS2 Platform Pixhawk
echo "[4/10] Starting AERIXIS Platform Pixhawk..."
tmux new-window -t "$SESSION" -n platform \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_platform_pixhawk pixhawk_launch.py namespace:=$DRONE_NS platform_config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml'"

sleep 2

# Window 4: AS2 State Estimator
echo "[5/10] Starting AERIXIS State Estimator..."
tmux new-window -t "$SESSION" -n estimator \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_state_estimator state_estimator_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=raw_odometry'"

sleep 1

# Window 5: AS2 Motion Controller
echo "[6/10] Starting AERIXIS Motion Controller..."
tmux new-window -t "$SESSION" -n controller \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_motion_controller controller_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=pid_speed_controller plugin_config_file:=$WORKSPACE/projects/px4_vision/config/pid_speed_controller.yaml'"

sleep 1

# Window 6: AS2 Behaviors
echo "[7/10] Starting AERIXIS Motion Behaviors..."
tmux new-window -t "$SESSION" -n behaviors \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_behaviors_motion motion_behaviors_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml use_sim_time:=false'"

sleep 1

# Window 7: AS2 Trajectory Generation Behavior
echo "[8/10] Starting AERIXIS Trajectory Generation Behavior..."
tmux new-window -t "$SESSION" -n traj_gen \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=dynamic_mav_trajectory_generator use_sim_time:=false'"

sleep 1

# Window 8: Mission Executor
echo "[9/10] Starting AERIXIS Mission Executor..."
tmux new-window -t "$SESSION" -n mission \
  "bash -c '$SETUP_CMD; exec $WORKSPACE/install/as2_python_api/lib/as2_python_api/mission_executor --n $DRONE_NS; exec bash'"

sleep 1

# Window 9: AS2 MAVLink Gateway
echo "[10/10] Starting AERIXIS MAVLink Gateway..."
tmux new-window -t "$SESSION" -n gateway \
  "bash -c '$SETUP_CMD; exec ros2 run as2_mavlink_gateway mavlink_gateway --ros-args -p drone_namespace:=$DRONE_NS -p bind_address:=0.0.0.0 -p udp_port:=14550 -p gcs_host:=127.0.0.1 -p gcs_port:=14551 -p home_latitude:=47.397742 -p home_longitude:=8.545594 -p home_altitude:=488.0 -p use_sim_time:=false'"

echo ""
echo "=========================================================="
echo " All components launched into tmux session: $SESSION"
echo " Web UI        : http://localhost:8765"
echo " Attach tmux   : tmux attach -t $SESSION"
echo " Windows list  : tmux list-windows -t $SESSION"
echo " Stop all      : bash $WORKSPACE/stop_all.sh"
echo "=========================================================="
