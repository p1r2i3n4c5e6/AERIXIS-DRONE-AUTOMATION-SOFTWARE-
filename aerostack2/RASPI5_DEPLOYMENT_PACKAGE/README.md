# Raspberry Pi 5 + SIYI A8 Mini Person Detection & Tracking Package
### (Calibrated for 10m to 15m Aerial Detection & AERIXIS Drone Autonomy)

Complete standalone deployment package for **Raspberry Pi 5** featuring our newly trained **YOLO26n NCNN model** with zero-latency RTSP streaming and 8-state Kalman Filter tracking.

---

## 🎯 10m to 15m Altitude Detection & Camera Feed Resolution

When detecting humans from **10 to 15 meters above the ground** with the SIYI A8 Mini 8mm lens:
* Full 4K or 1080p video stream decoding causes severe CPU overload on the Raspberry Pi 5.
* **Optimal Feed Resolution**: **`640x360`** (or `720p`).
  * At 10–15m altitude, a 1.7m human occupies **~35 to 70 pixels**, which is the exact sweet-spot for YOLO detection.
  * Decoding takes **<8 ms** instead of >35 ms, allowing the Pi 5 to sustain **25 FPS smooth tracking** while keeping CPU temperatures cool.
* The system connects to the camera's **sub-stream (`rtsp://192.168.144.25:8554/sub.264`)** automatically with fallback to the main stream.

---

## 🛠️ Physical Wiring: SIYI A8 Mini to Raspberry Pi 5

1. Connect the **SIYI A8 Mini 8-pin / 4-pin Ethernet cable** to a standard RJ45 connector.
2. Plug the RJ45 cable directly into the **Raspberry Pi 5 Ethernet port (`end0`)**.
3. Power the SIYI A8 Mini gimbal with **11V–26V (3S–6S LiPo or BEC)**.
4. The camera defaults to IP: `192.168.144.25` with RTSP URLs:
   * Sub-Stream: `rtsp://192.168.144.25:8554/sub.264` (Recommended)
   * Main-Stream: `rtsp://192.168.144.25:8554/main.264`

---

## ⚡ 1-Minute Installation on Raspberry Pi 5

Copy this entire folder (`RASPI5_A8MINI_DEPLOYMENT`) to your Raspberry Pi 5 (e.g. to `~/RASPI5_A8MINI_DEPLOYMENT`).

In your Raspberry Pi 5 terminal:

```bash
cd ~/RASPI5_A8MINI_DEPLOYMENT
chmod +x install.sh run.sh src/*.py
./install.sh
```

---

## 🚀 Running the Vision System

### 1. Standard Autonomous Run (A8 Mini Ethernet)
```bash
./run.sh
```
* Auto-configures Pi 5 Ethernet IP to `192.168.144.20`.
* Pulls camera feed at **640x360** for ultra-low latency and 10–15m human detection.
* Pins to **CPU Cores 2 & 3**, leaving **Cores 0 & 1 100% free for Aerostac2**.
* Broadcasts target telemetry to **UDP port 9090**.

---

### 2. Drone Flight Mode (Headless, Maximum Speed)
When the drone is flying without an HDMI monitor:
```bash
./run.sh --headless
```

---

### 3. Wireless Ground-Station Web View (View on Laptop)
View the camera feed and detection bounding boxes from your laptop browser over Wi-Fi without any HDMI cable:
```bash
./run.sh --headless --stream 8080
```
Then open in your laptop browser:
```
http://<RASPBERRY_PI_IP>:8080
```

---

### 4. Resolution Tuning Options
You can select different feed resolutions depending on your altitude:
```bash
# 640x360 (Default, recommended for 10-15m altitude, fastest FPS):
./run.sh --res 640x360

# 720p (For higher altitude 15-25m):
./run.sh --res 720p
```

---

### 5. Offline / Benchtop Testing (Without Camera)
```bash
# Test with USB Webcam:
./run.sh --webcam

# Test with a video file:
./run.sh --video /path/to/test.mp4
```

---

## 📡 Aerostac2 ROS 2 Integration

The vision pipeline broadcasts JSON telemetry over `127.0.0.1:9090`.

To publish these targets into Aerostac2 ROS 2 topics:
```bash
source /opt/ros/humble/setup.bash
python3 src/ros2_aerostack2_bridge.py
```

### Published ROS 2 Topics:
* `/drone/vision/target_found` (`std_msgs/Bool`): True when a confirmed person is tracked.
* `/drone/vision/target_offset` (`geometry_msgs/PointStamped`):
  * `point.x`: Normalized horizontal error (`-1.0` left to `+1.0` right, `0.0` is center).
  * `point.y`: Normalized vertical error (`-1.0` top to `+1.0` bottom).
  * `point.z`: Calibrated distance in meters (for 10–15m altitude).
* `/drone/vision/target_telemetry` (`std_msgs/String`): Raw JSON string with tracking IDs and velocities.

---

## 🛡️ Anti-False-Alarm Guarantee
* **Aspect Ratio Gate**: Rejects any box with $H/W < 1.1$ (filters out horizontal shadows, vehicles, road markings).
* **Min Height Gate**: Rejects noise under 18 pixels.
* **3-Frame Temporal Confirmation**: Discards 1-frame camera shake and foliage motion glitches.
