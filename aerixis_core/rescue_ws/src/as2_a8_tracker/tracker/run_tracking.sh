#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export A8_PREVIEW_ONLY=0
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec python3 src/hybrid_tracker.py --model model --no-pin --threads 2 --rtsp rtsp://192.168.144.25:8554/main.264 --gimbal-mode siyi_udp --mavlink none --no-scan-on-start --kd-yaw 0 --kd-pitch 0 --siyi-command-cap 35 --headless "$@"
