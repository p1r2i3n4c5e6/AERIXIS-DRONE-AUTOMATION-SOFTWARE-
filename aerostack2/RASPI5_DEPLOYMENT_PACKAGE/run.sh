#!/usr/bin/env bash
# ==============================================================================
# Raspberry Pi 5 — SIYI A8 Mini Drone Vision Runner (Ethernet RTSP)
# Optimized for 10-15m Aerial Human Detection & Aerostac2 Integration
# ==============================================================================
# CPU Core Isolation:
#   - CORES 0 & 1 : Reserved 100% for Aerostac2 & ROS 2 flight controller
#   - CORES 2 & 3 : Dedicated to Vision (A8 Mini RTSP + NCNN + Kalman Tracker)
# ==============================================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MODEL_DIR="$SCRIPT_DIR/model"
if [ ! -f "$MODEL_DIR/model.ncnn.param" ] || [ ! -f "$MODEL_DIR/model.ncnn.bin" ]; then
    echo "❌ Error: NCNN model files not found in $MODEL_DIR"
    exit 1
fi

echo "=========================================================="
echo "  🚁 Raspberry Pi 5 + SIYI A8 Mini Drone Vision"
echo "  YOLO26n (2 Threads) + 8-State Kalman Tracker (25 FPS)"
echo "  Detection Altitude : 10m to 15m Ground Distance"
echo "=========================================================="

# ── 1. Auto-Detect Ethernet Port & Configure Subnet for SIYI A8 Mini ─────────
# Raspberry Pi 5 gigabit Ethernet interface is named 'end0' (fallback 'eth0' or 'enx*')
ETH_IFACE=$(ip -br link | grep -E "end0|eth0|enx" | awk '{print $1}' | head -n 1)

if [ -n "$ETH_IFACE" ]; then
    echo "🔌 [Ethernet] Detected camera port interface: $ETH_IFACE"
    
    # Ensure link is UP
    sudo ip link set "$ETH_IFACE" up 2>/dev/null || true

    # Check if Pi's Ethernet port has an IP on the 192.168.144.x subnet
    if ! ip addr show dev "$ETH_IFACE" | grep -q "192.168.144."; then
        echo "⚙️ [Network Auto-Config] Assigning IP 192.168.144.20/24 to $ETH_IFACE..."
        sudo ip addr add 192.168.144.20/24 dev "$ETH_IFACE" 2>/dev/null || true
    else
        CURRENT_IP=$(ip -br addr show dev "$ETH_IFACE" | awk '{print $3}')
        echo "✅ [Network] Interface $ETH_IFACE configured with IP: $CURRENT_IP"
    fi
else
    echo "⚠️ [Warning] No physical Ethernet port detected. Will check fallback options."
fi

# ── 2. Display Hardware Allocation ──────────────────────────────────────────
echo "  🔒 Hardware CPU Pinning : Cores 2 & 3"
echo "  🕊️ Aerostac2 Free Cores : Cores 0 & 1"
echo "  📹 Camera Stream        : SIYI A8 Mini (Sub/Main RTSP)"
echo "  📐 Resolution Feed      : 640x360 (Optimal for 10-15m altitude & low CPU)"
echo "  📡 Telemetry Output     : UDP 127.0.0.1:9090 (JSON for Aerostac2)"
echo "=========================================================="

# Run strictly pinned to CPU Cores 2 and 3
exec taskset -c 2,3 python3 src/hybrid_tracker.py \
    --model "$MODEL_DIR" \
    --cores 2 3 \
    --threads 2 \
    --skip 4 \
    --res 640x360 \
    --conf 0.45 \
    "$@"
