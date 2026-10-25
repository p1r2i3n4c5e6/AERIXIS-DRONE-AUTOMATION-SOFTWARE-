#!/usr/bin/env bash
echo "Stopping AERIXIS core stack..."
tmux kill-session -t as2_stack 2>/dev/null || true
tmux kill-session -t as2_companion 2>/dev/null || true

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
echo "AERIXIS core stack stopped."
