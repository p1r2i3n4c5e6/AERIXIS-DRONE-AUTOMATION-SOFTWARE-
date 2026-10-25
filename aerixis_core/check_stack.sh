#!/usr/bin/env bash
# =============================================================================
# AS2 + PX4 SITL + Custom GCS Stack Diagnostics
# =============================================================================

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

pass() { echo -e "  [${GREEN}PASS${NC}] $1"; }
fail() { echo -e "  [${RED}FAIL${NC}] $1"; }
warn() { echo -e "  [${YELLOW}WARN${NC}] $1"; }
info() { echo -e "  [${CYAN}INFO${NC}] $1"; }

echo "=========================================================="
echo " Running Stack Diagnostics"
echo "=========================================================="

echo ""
echo "--- 1. Process Checks ---"
for proc in px4 MicroXRCEAgent as2_platform_pixhawk_node as2_state_estimator_node as2_motion_controller_node mavlink_gateway; do
  if pgrep -f "$proc" >/dev/null 2>&1; then
    pass "Process running: $proc"
  else
    fail "Process NOT running: $proc"
  fi
done

if pgrep -f "server.py" >/dev/null 2>&1 || pgrep -f "as2_ground_control" >/dev/null 2>&1; then
  pass "Process running: Custom GCS server"
else
  fail "Process NOT running: Custom GCS server"
fi

echo ""
echo "--- 2. Port Checks ---"
# Check GCS Web Port
if curl -s -o /dev/null -w "%{http_code}" http://localhost:8765/api/state 2>/dev/null | grep -q "200"; then
  pass "GCS Web UI responding on http://localhost:8765 (HTTP 200)"
else
  fail "GCS Web UI not responding on http://localhost:8765"
fi

# Check UDP Ports
for port in 8888 14550 14551; do
  if ss -uln 2>/dev/null | grep -q ":$port "; then
    pass "UDP port $port listening"
  else
    warn "UDP port $port not detected in ss (might be using socket without SO_REUSEADDR or active)"
  fi
done

echo ""
echo "--- 3. ROS 2 Topics Checks ---"
if command -v ros2 >/dev/null 2>&1; then
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  source /home/prince/aerostack2/install/setup.bash 2>/dev/null || true

  # Check PX4 Odometry through Micro-XRCE
  if timeout 2 ros2 topic echo /drone0/fmu/out/vehicle_odometry --qos-reliability best_effort --once >/dev/null 2>&1; then
    pass "PX4 uXRCE-DDS odometry receiving: /drone0/fmu/out/vehicle_odometry"
  else
    warn "No messages on /drone0/fmu/out/vehicle_odometry (ensure Gazebo simulation is unpaused)"
  fi

  # Check AS2 Platform Info
  if timeout 2 ros2 topic echo /drone0/platform/info --once >/dev/null 2>&1; then
    pass "AS2 Platform info published: /drone0/platform/info"
  else
    fail "No messages on /drone0/platform/info"
  fi

  # Check AS2 Pose
  if timeout 2 ros2 topic echo /drone0/self_localization/pose --qos-reliability best_effort --once >/dev/null 2>&1; then
    pass "AS2 State Estimator pose published: /drone0/self_localization/pose"
  else
    warn "No messages on /drone0/self_localization/pose"
  fi
else
  warn "ros2 command not available in this subshell; source /opt/ros/humble/setup.bash first."
fi

echo ""
echo "--- 4. GCS Live State ---"
curl -s http://localhost:8765/api/state 2>/dev/null | python3 -c '
import sys, json
try:
    d = json.load(sys.stdin)
    c_on = "\033[0;32mCONNECTED\033[0m"
    c_off = "\033[0;31mDISCONNECTED\033[0m"
    conn = c_on if d.get("connected") else c_off
    as2 = c_on if d.get("as2_link",{}).get("connected") else c_off
    px4 = c_on if d.get("px4_link",{}).get("connected") else c_off
    armed = d.get("armed", False)
    lat = d.get("latitude")
    lon = d.get("longitude")
    alt = d.get("altitude")
    print(f"  GCS <-> AS2 Gateway MAVLink : {conn}")
    print(f"  AS2 Core Link               : {as2}")
    print(f"  PX4 uXRCE-DDS Link          : {px4}")
    print(f"  Armed State                 : {armed}")
    print(f"  Latitude / Longitude        : {lat}, {lon}")
    print(f"  Altitude                    : {alt} m")
except Exception as e:
    print(f"  Could not parse state: {e}")
' || echo "  GCS state API not reachable."

echo "=========================================================="
