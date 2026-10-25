#!/usr/bin/env bash
echo "Stopping PX4 SITL & Gazebo simulation..."
tmux kill-session -t px4_sim 2>/dev/null || true
killall -9 px4 2>/dev/null || true
killall -9 gz 2>/dev/null || true
killall -9 ruby 2>/dev/null || true
rm -rf /tmp/px4_instance_* 2>/dev/null || true
echo "Simulation stopped."
