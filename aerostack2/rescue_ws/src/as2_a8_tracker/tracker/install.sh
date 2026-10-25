#!/usr/bin/env bash
# ==============================================================================
# Raspberry Pi 5 — One-Click Dependency Installer (Direct Raspberry Pi OS)
# No virtualenv needed — installs directly into Raspberry Pi OS (Debian 12 Bookworm)
# ==============================================================================

set -e

echo "=========================================================="
echo "  🚀 Raspberry Pi 5 Drone Vision & Gimbal Installer"
echo "  YOLO26n NCNN (FP16) + OC-SORT Center Lock + PyMAVLink"
echo "  Hardware: SIYI A8 Mini Gimbal Camera + Flight Controller"
echo "=========================================================="

# 1. Update APT and install required system libraries
echo ""
echo "📦 [1/3] Installing system packages directly in Raspberry Pi OS..."
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-opencv \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libopenblas-dev \
    iproute2 \
    net-tools \
    v4l-utils

# 2. Install NCNN runtime & PyMAVLink directly to system Python
echo ""
echo "📦 [2/3] Installing NCNN & PyMAVLink for autonomous gimbal control..."
sudo python3 -m pip install --upgrade --no-cache-dir --break-system-packages ncnn pymavlink 2>/dev/null || \
python3 -m pip install --upgrade --no-cache-dir --break-system-packages ncnn pymavlink 2>/dev/null || \
python3 -m pip install --upgrade --no-cache-dir ncnn pymavlink

# 3. Verify installation
echo ""
echo "🔍 [3/3] Verifying Python libraries in Raspberry Pi OS..."
python3 -c "
import ncnn
import cv2
import numpy as np
import pymavlink
print('   ✅ NCNN Runtime : Ready (ARM Cortex-A76 NEON acceleration)')
print('   ✅ OpenCV       : Version', cv2.__version__)
print('   ✅ NumPy        : Version', np.__version__)
print('   ✅ PyMAVLink    : Ready for Autonomous Gimbal Tracking!')
"

# 4. Set execution permissions
chmod +x run.sh
chmod +x install.sh
chmod +x src/hybrid_tracker.py
chmod +x src/test_gimbal.py
chmod +x src/ros2_aerostack2_bridge.py

echo ""
echo "=========================================================="
echo "  🎉 Installation Successful!"
echo "  Connect your SIYI A8 Mini & Flight Controller to Pi 5,"
echo "  then run:"
echo "      ./run.sh"
echo "=========================================================="
