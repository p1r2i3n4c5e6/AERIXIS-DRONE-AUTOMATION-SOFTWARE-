#!/usr/bin/env bash
echo "Stopping the AERIXIS GCS..."
tmux kill-session -t as2_gcs 2>/dev/null || true
pkill -f "custom_gcs/server.py" 2>/dev/null || true
pkill -f "as2_ground_control.server" 2>/dev/null || true
echo "AERIXIS GCS stopped."
