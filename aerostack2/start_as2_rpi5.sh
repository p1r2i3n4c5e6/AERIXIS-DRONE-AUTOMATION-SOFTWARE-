#!/usr/bin/env bash
# =============================================================================
# Aerostack2 (AS2) Companion Computer Launcher (Raspberry Pi 5 / Jetson)
#
# Hardware Connections:
#   1. Flight Controller (Pixhawk / Cube / Holybro):
#      - Connected via UART (/dev/ttyAMA0 or /dev/ttyUSB1) @ 921600 baud
#      - Protocol: Micro XRCE-DDS client on PX4/ArduPilot
#   2. Telemetry Radio (SiK / Holybro 915MHz/433MHz / RFD900 / WiFi / LTE):
#      - Serial Radio: /dev/ttyUSB0 @ 57600 baud (or UDP over IP: 14550)
#      - Protocol: Standard MAVLink to Custom GCS
# =============================================================================
set -e

WORKSPACE="/home/prince/aerostack2"
DRONE_NS="${DRONE_NS:-drone0}"
TMUX_SESSION="as2_companion"

# Flight Controller (PX4 uXRCE-DDS) Serial Configuration
FC_SERIAL_DEV="${FC_SERIAL_DEV:-/dev/ttyAMA0}"
FC_BAUDRATE="${FC_BAUDRATE:-921600}"

# Telemetry Radio / Network Configuration for Custom GCS
TELEMETRY_MODE="${TELEMETRY_MODE:-udp}" # 'udp' (WiFi/LTE) or 'serial' (SiK radio)
RADIO_DEV="${RADIO_DEV:-/dev/ttyUSB0}"
RADIO_BAUD="${RADIO_BAUD:-57600}"
GCS_HOST="${GCS_HOST:-192.168.1.100}"
GCS_PORT="${GCS_PORT:-14551}"
LOCAL_PORT="${LOCAL_PORT:-14550}"

echo "=========================================================="
echo " Starting AERIXIS core Companion Computer Stack (RPi 5)"
echo " Drone Namespace  : ${DRONE_NS}"
echo " Flight Controller : ${FC_SERIAL_DEV} @ ${FC_BAUDRATE}"
echo " Telemetry Mode    : ${TELEMETRY_MODE}"
if [ "$TELEMETRY_MODE" = "serial" ]; then
  echo " Radio Port        : ${RADIO_DEV} @ ${RADIO_BAUD}"
else
  echo " UDP Gateway       : Local ${LOCAL_PORT} -> GCS ${GCS_HOST}:${GCS_PORT}"
fi
echo "=========================================================="

if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  echo "Error: /opt/ros/humble/setup.bash not found!" >&2
  exit 1
fi
if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  echo "Error: $WORKSPACE/install/setup.bash not found! Run colcon build first." >&2
  exit 1
fi

SETUP_CMD="source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash && export PYTHONPATH=/usr/lib/python3/dist-packages:\$PYTHONPATH"

# Kill existing companion session if present
tmux kill-session -t "$TMUX_SESSION" 2>/dev/null || true

# 1. Micro XRCE-DDS Agent (Serial to Pixhawk)
echo "[1/8] Starting Micro XRCE-DDS Agent on ${FC_SERIAL_DEV}..."
tmux new-session -d -s "$TMUX_SESSION" -n xrce_agent \
  "bash -c 'source /opt/ros/humble/setup.bash; export LD_LIBRARY_PATH=$WORKSPACE/install/px4_microxrce_agent/lib:\${LD_LIBRARY_PATH:-}; exec $WORKSPACE/install/px4_microxrce_agent/bin/MicroXRCEAgent serial --dev ${FC_SERIAL_DEV} -b ${FC_BAUDRATE}'"
sleep 2

# 2. AS2 Platform Pixhawk
echo "[2/8] Launching AERIXIS Platform Pixhawk..."
tmux new-window -t "$TMUX_SESSION" -n platform \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_platform_pixhawk pixhawk_launch.py namespace:=$DRONE_NS platform_config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml'"
sleep 2

# 3. AS2 State Estimator
echo "[3/8] Launching AERIXIS State Estimator..."
tmux new-window -t "$TMUX_SESSION" -n estimator \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_state_estimator state_estimator_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=raw_odometry'"
sleep 1

# 4. AS2 Motion Controller
echo "[4/8] Launching AERIXIS Motion Controller (PID + Trajectory)..."
tmux new-window -t "$TMUX_SESSION" -n controller \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_motion_controller controller_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=pid_speed_controller plugin_config_file:=$WORKSPACE/projects/px4_vision/config/pid_speed_controller.yaml'"
sleep 1

# 5. AS2 Motion Behaviors (Takeoff, Land, GoTo, FollowPath)
echo "[5/8] Launching AERIXIS Motion Behaviors..."
tmux new-window -t "$TMUX_SESSION" -n behaviors \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_behaviors_motion motion_behaviors_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml use_sim_time:=false'"
sleep 1

# 6. AS2 Trajectory Generation (Dynamic MAV Polynomial Splines)
echo "[6/8] Launching AERIXIS Trajectory Generation Behavior..."
tmux new-window -t "$TMUX_SESSION" -n traj_gen \
  "bash -c '$SETUP_CMD; exec ros2 launch as2_behaviors_trajectory_generation generate_polynomial_trajectory_behavior_launch.py namespace:=$DRONE_NS config_file:=$WORKSPACE/projects/px4_vision/config/config.yaml plugin_name:=dynamic_mav_trajectory_generator use_sim_time:=false'"
sleep 1

# 7. AS2 Mission Executor
echo "[7/8] Launching AERIXIS Mission Executor..."
tmux new-window -t "$TMUX_SESSION" -n mission \
  "bash -c '$SETUP_CMD; exec $WORKSPACE/install/as2_python_api/lib/as2_python_api/mission_executor --n $DRONE_NS'"
sleep 1

# 8. AS2 MAVLink Gateway (Bridges to Custom GCS via Serial or UDP)
echo "[8/8] Launching AERIXIS MAVLink Gateway..."
if [ "$TELEMETRY_MODE" = "serial" ]; then
  GATEWAY_ARGS="-p drone_namespace:=$DRONE_NS -p serial_device:=$RADIO_DEV -p serial_baud:=$RADIO_BAUD -p use_sim_time:=false"
else
  GATEWAY_ARGS="-p drone_namespace:=$DRONE_NS -p bind_address:=0.0.0.0 -p udp_port:=$LOCAL_PORT -p gcs_host:=$GCS_HOST -p gcs_port:=$GCS_PORT -p use_sim_time:=false"
fi

tmux new-window -t "$TMUX_SESSION" -n gateway \
  "bash -c '$SETUP_CMD; exec ros2 run as2_mavlink_gateway mavlink_gateway --ros-args $GATEWAY_ARGS'"

echo "=========================================================="
echo " Companion Stack active in tmux session: ${TMUX_SESSION}"
echo " Monitor with : tmux attach -t ${TMUX_SESSION}"
echo " Detach with  : Ctrl+b then d"
echo " Stop with    : tmux kill-session -t ${TMUX_SESSION}"
echo "=========================================================="
