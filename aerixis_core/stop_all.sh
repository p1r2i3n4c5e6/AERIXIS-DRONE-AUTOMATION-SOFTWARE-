#!/usr/bin/env bash
# =============================================================================
# Master Stop Script: Terminates all 3 Launchers & Sessions cleanly
# =============================================================================
echo "Stopping all GCS, AERIXIS core stack, and Simulation processes..."

# Stop launcher 1 (GCS)
tmux kill-session -t as2_gcs 2>/dev/null || true
pkill -f "custom_gcs/server.py" 2>/dev/null || true
pkill -f "as2_ground_control.server" 2>/dev/null || true

# Stop launcher 2 (AS2 Stack)
tmux kill-session -t as2_stack 2>/dev/null || true
tmux kill-session -t as2_companion 2>/dev/null || true
tmux kill-session -t as2_sitl 2>/dev/null || true

killall -9 MicroXRCEAgent 2>/dev/null || true
killall -9 as2_platform_pixhawk_node 2>/dev/null || true
killall -9 as2_state_estimator_node 2>/dev/null || true
killall -9 as2_motion_controller_node 2>/dev/null || true
killall -9 follow_path_behavior_node 2>/dev/null || true
killall -9 go_to_behavior_node 2>/dev/null || true
killall -9 land_behavior_node 2>/dev/null || true
killall -9 takeoff_behavior_node 2>/dev/null || true
killall -9 generate_polynomial_trajectory_behavior_node 2>/dev/null || true
killall -9 mission_executor 2>/dev/null || true
killall -9 mavlink_gateway 2>/dev/null || true

# Stop launcher 3 (Simulation)
tmux kill-session -t px4_sim 2>/dev/null || true
killall -9 px4 2>/dev/null || true
killall -9 gz 2>/dev/null || true
killall -9 ruby 2>/dev/null || true
rm -rf /tmp/px4_instance_* 2>/dev/null || true

echo "All components stopped cleanly."
