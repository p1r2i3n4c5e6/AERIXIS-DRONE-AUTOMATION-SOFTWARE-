# AERIXIS — automation software for drones (plug and play)

Flood-rescue / search drone software: autonomous survey missions planned from a KML area, person
detection and gimbal tracking with an A8 camera, geotagging, payload drop, multi-drone (swarm)
coverage with collision avoidance, an emergency FC guardian, and a web ground control station.

Built on AeroStack2 (ROS 2 Humble) and PX4 v1.16 (uXRCE-DDS). Onboard computer: Raspberry Pi 5
(Docker image). Ground station: laptop or phone browser.

## What is in this repository

| Folder | What it is |
|---|---|
| `aerixis_core/` | **AERIXIS core**, the onboard software: AeroStack2 (modified), the rescue stack in `rescue_ws/` (mission runtime, FC guardian, mission manager, planner, avoidance, A8 tracker + AI, rate gate), the Raspberry Pi 5 Docker image (`docker/raspberrypi5/`), launch scripts and docs (`docs/IMPLEMENTATION_STATUS.md` is the detailed engineering log) |
| `custom_gcs/` | AERIXIS Mission Control: the ground control station (Python server + web UI), works with a telemetry radio (MAVLink) |
| `sitl/` | PX4 v1.16.1 SITL changes for testing (`px4-v1.16.1-aerixis.patch`, `px4-rc.as2`) and the SITL runner `run_px4_sitl.sh` |

Not included on purpose: build outputs, logs, real pairing tokens (`custom_gcs/config/fleet.json`),
the onboard settings file with the panel PIN (`onboard.env`). Create your own from the examples.

## Quick start

### 1. Laptop (Ubuntu 22.04, ROS 2 Humble)
```bash
git clone <this repo> ~/aerixis && cd ~/aerixis && ./setup.sh
```
`setup.sh` links `~/aerostack2` to `aerixis_core/` and `~/custom_gcs` to `custom_gcs/` (the launchers
expect those places; see *Known limitations*). The commands below use those links.

### 2. Build the Raspberry Pi 5 image on the laptop and deploy it
One-time: `sudo apt-get install -y docker-buildx qemu-user-static binfmt-support`
```bash
cd ~/aerostack2
docker buildx build --platform linux/arm64 -f docker/raspberrypi5/Dockerfile.onboard \
  --build-arg MAKE_JOBS=4 --build-arg COLCON_WORKERS=3 -t as2-rescue-onboard:pi5 --load .
docker save as2-rescue-onboard:pi5 | gzip -1 | ssh pi@<pi-ip> 'gunzip | docker load'
ssh pi@<pi-ip> "cd ~/aerostack2/docker/raspberrypi5 && docker compose up -d --force-recreate onboard"
```
The Pi needs `docker/raspberrypi5/` (compose file) and its own `onboard.env`
(copy `onboard.env.example`, set the drone ID, system ID and panel PIN). See
`aerixis_core/docker/raspberrypi5/README.md` for the first Pi setup (network, camera, FC link).

### 3. Field start (real drone, no simulation)
```bash
cd ~/aerostack2 && ./start_field.sh
```
Finds the Pi on the Wi-Fi/hotspot, starts its onboard software, starts the GCS on port 8765
(phones on the same network can open it) and connects the telemetry radio at 57600 baud.
The onboard panel is at `http://<pi-ip>:8090` or in the GCS under *Onboard calibration*.

### 4. Simulation test (PX4 v1.16 + Gazebo Harmonic)
Apply `sitl/px4-v1.16.1-aerixis.patch` to a PX4 v1.16.1 checkout, put `run_px4_sitl.sh` next to it, then:
```bash
PX4_AUTOPILOT_DIR=~/px4_sitl/PX4-Autopilot-1.16 ~/aerostack2/rescue_ws/scripts/sitl_mission_test.sh human_geotag_resume
```

## Hardware this was built and tested with
Raspberry Pi 5 (16 GB), PX4 v1.16.1 flight controller over Ethernet (uXRCE-DDS, UDP 8888),
SIYI A8 mini camera (Ethernet, H.264 main stream), SiK telemetry radios (57600 baud).
The Pi needs a stable 5.1 V / 5 A supply: undervoltage throttles the CPU and causes lag.

## Licences — read before any commercial use
- AeroStack2, PX4 / px4_msgs and this project's own packages: BSD-3-Clause. ROS 2, Fast DDS,
  OpenCV, ncnn: Apache-2.0 / BSD. OC-SORT and filterpy (vendored): MIT. pymavlink: LGPL-3.0.
- **The bundled detection models are Ultralytics YOLO models under AGPL-3.0**
  (see each `metadata.yaml`). Closed-source commercial use needs an Ultralytics Enterprise
  licence or models retrained on a permissively licensed detector.
- Not legal advice; check licences and your country's drone regulations before selling.

## Known limitations
- Several laptop launchers and docs use the absolute path `/home/prince/...`; `setup.sh` covers
  the default layout, a portable-paths pass is planned. The onboard Docker image does not depend on it.
- Field validation is ongoing; see `aerixis_core/docs/IMPLEMENTATION_STATUS.md`.
