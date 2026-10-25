#!/usr/bin/env bash
# ==============================================================================
# Raspberry Pi 5 — One-Click Dependency Installer (SIYI A8 Mini Ethernet Edition)
# Supports Raspberry Pi OS Bookworm (Debian 12) & Ubuntu 22.04/24.04
# ==============================================================================

set -e

echo "=========================================================="
echo "  🚀 Raspberry Pi 5 Drone Vision Installer"
echo "  YOLO26n NCNN (Trained Model) + Kalman Hybrid Tracker"
echo "  Hardware: SIYI A8 Mini (Ethernet RTSP) + Aerostac2"
echo "=========================================================="

# 1. Update APT and install required system libraries
echo ""
echo "📦 [1/3] Installing system packages, OpenCV, FFmpeg & networking tools..."
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-opencv \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    iproute2 \
    net-tools \
    v4l-utils

# 2. Install NCNN runtime (handling Debian Bookworm externally-managed-environment)
echo ""
echo "📦 [2/3] Installing optimized NCNN runtime for ARM NEON..."
python3 -m pip install --upgrade --no-cache-dir --break-system-packages ncnn 2>/dev/null || \
python3 -m pip install --upgrade --no-cache-dir ncnn

# 3. Verify installation
echo ""
echo "🔍 [3/3] Verifying Python libraries..."
python3 -c "
import ncnn
import cv2
import numpy as np
print('   ✅ NCNN Runtime : Ready (ARM Cortex-A76 NEON acceleration)')
print('   ✅ OpenCV       : Version', cv2.__version__)
print('   ✅ NumPy        : Version', np.__version__)
"

# 4. Set execution permissions
chmod +x run.sh
chmod +x install.sh
chmod +x src/hybrid_tracker.py
chmod +x src/ros2_aerostack2_bridge.py

echo ""
echo "=========================================================="
echo "  🎉 Installation Successful!"
echo "  Connect your SIYI A8 Mini Ethernet cable to the Pi 5,"
echo "  then run:"
echo "      ./run.sh"
echo "=========================================================="
