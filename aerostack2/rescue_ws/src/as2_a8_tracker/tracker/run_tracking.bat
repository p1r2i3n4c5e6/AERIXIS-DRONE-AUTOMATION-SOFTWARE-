@echo off
setlocal
cd /d "%~dp0"
set A8_PREVIEW_ONLY=0
set OMP_NUM_THREADS=2
set OPENBLAS_NUM_THREADS=1
echo A8-20261020-dualtrack2 -- TRACKING: CAPPED GIMBAL COMMANDS ENABLED
python src\hybrid_tracker.py --model model --no-pin --threads 2 --rtsp rtsp://192.168.144.25:8554/main.264 --gimbal-mode siyi_udp --mavlink none --no-scan-on-start --kd-yaw 0 --kd-pitch 0 --siyi-command-cap 35 %*
exit /b %errorlevel%
