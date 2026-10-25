# AERIXIS — A8 mini + Raspberry Pi 5 rescue perception overlay

Initial bench integration for the user's Pi 5 (16 GB), A8 Ethernet camera and supplied 512x512 YOLO11n NCNN person model. Flight approach, gimbal control, geolocation and payload release are not implemented in this package.

## Build

From this directory, after the main AERIXIS workspace has been built:

```bash
source /opt/ros/humble/setup.bash
source /home/prince/aerostack2/install/setup.bash
PYTHONNOUSERSITE=1 colcon build --base-paths src --packages-select as2_rescue_perception
source install/setup.bash
```

The build-only `PYTHONNOUSERSITE=1` avoids the current host's incompatible user-site setuptools/packaging combination; it does not change installed packages. Leave it unset when running if NCNN is installed in the user site.

The root `COLCON_IGNORE` prevents the parent AERIXIS build from discovering this overlay twice. Building with explicit `--base-paths src` is intentional.

Dependencies include ROS 2 rclpy, sensor_msgs, std_msgs, vision_msgs; Python OpenCV, NumPy, PyYAML and NCNN. This wrapper reuses `src/hybrid_tracker.py` preprocessing/tracker from the supplied deployment package. It loads the NCNN files directly; it does not run the standalone script or its network configuration commands. Model metadata declares AGPL-3.0; wrapper code does not change the model's license.

## Launch on the Pi

Configure the camera Ethernet subnet separately. The user-provided deployment profile uses camera `192.168.144.25` and example companion address `192.168.144.20/24`; confirm these are unique on the actual network. This launch does not modify network interfaces.

```bash
ros2 launch as2_rescue_perception a8_person_detection.launch.py \
  drone_id:=drone0 \
  deployment_dir:=/home/prince/aerostack2/RASPI5_DEPLOYMENT_PACKAGE \
  vision_cores:=2,3
```

Adjust `deployment_dir` for the actual Pi filesystem. Default stream: `rtsp://192.168.144.25:8554/main.264`; override with `rtsp_url:=...` if firmware/configuration differs. Supply `calibration_file:=/absolute/path/camera.yaml` for a ROS camera calibration file matching the streaming resolution and zoom. No calibration is invented: CameraInfo K[0] remains zero if calibration is absent or mismatched. Mounting transforms and gimbal attitude are still required for world geolocation.

The launch pins the camera and detector processes to cores 2 and 3, limits NCNN to two threads and OpenCV to one. CPU affinity does not reserve the other cores exclusively or guarantee control deadlines. Validate scheduling, thermals and inference latency on the Pi. No Pi FPS/latency claim has been verified here.

## Interfaces (relative to /drone0)

- `camera/image_raw`: bgr8 `sensor_msgs/Image`, latest fresh frame only.
- `camera/camera_info`: matching header/resolution, calibrated intrinsics only when supplied.
- `camera/health`: JSON `std_msgs/String`; connection state, age, calibration and error.
- `perception/detections`: `vision_msgs/Detection2DArray`; person hypotheses in image pixels; includes empty arrays.
- `perception/tracks`: JSON `std_msgs/String`; track IDs, boxes, confidence, confirmation, prediction flag and observation age. `world_position_valid` is always false.
- `perception/health`: JSON `std_msgs/String`; latest successful output age, inference duration, error.

Camera timestamps are decoder receipt times, not verified exposure timestamps. A bounded cache prevents replaying the same decoded frame; it cannot prove that the camera/decoder has no upstream buffering. Measure end-to-end latency on hardware.

The detector rejects old, future or non-increasing image stamps. It discards results that become too old during inference and clears stale tracks through a watchdog. BGR8 serialization avoids the current host's cv_bridge/NumPy binary mismatch without changing system packages.

Do not use the legacy UDP bridge's `dist_m` as flight geometry: it assumes person height/lens parameters and contains an extra scale division. This wrapper does not publish that estimate and never maps image offsets into a metric PointStamped.

## Optional BT observation condition

The main AERIXIS engine now registers:

```xml
<IsPersonDetected topic="perception/detections" max_age_s="0.5" min_confidence="0.35"/>
```

This condition requires a fresh `person` detection and fails when absent, stale, future-dated or below confidence. It does not require a confirmed persistent track and does not authorize approach or delivery. Those belong to the rescue behavior phase.

A harmless 10-second observation tree is provided at `as2_behavior_tree/resource/person_detection_check.xml`. After the main workspace rebuild and camera launch:

```bash
ros2 launch as2_behavior_tree behavior_trees.launch.py \
  drone_id:=drone0 \
  tree:=/home/prince/aerostack2/as2_behavior_tree/resource/person_detection_check.xml \
  mission_id:=camera_bench_check
```

It reports success when a fresh detection appears, failure on timeout, and contains no flight/payload commands. Use a dedicated namespace when testing alongside another executor.

## Open acceptance gates

Real A8 stream/reconnect and latency tests; calibrated image/attitude synchronization; Pi resource/thermal measurements; representative aerial flood-person footage accuracy; persistent target identity under occlusion; gimbal SDK feedback and control. This is a software bench foundation, not a completed autonomous rescue system.

## Onboard mission execution

The overlay also contains `as2_mission_manager`. See [onboard mission setup and GCS workflow](../docs/ONBOARD_MISSIONS.md). The camera/perception nodes and mission manager have separate launch procedures.
