# Implementation status — 2026-10-24

What is built, what is verified, and what is still open. This file is written to be
accurate rather than optimistic: SITL success is not flight qualification.

## 2026-10-25: Pi image built on the laptop and deployed; measured before/after

Laptop cross-build for the Pi 5: docker-buildx 0.30.1 + qemu-user-static (binfmt, flags PF) on
Ubuntu 22.04 (docker.io 29.1.3). `docker buildx build --platform linux/arm64 -f
docker/raspberrypi5/Dockerfile.onboard --build-arg MAKE_JOBS=4 --build-arg COLCON_WORKERS=3 -t
as2-rescue-onboard:pi5-laptop --load .` (new ARG COLCON_WORKERS, default 1 = Pi behaviour):
~70 min from scratch, exit 0. Shipped with `docker save | gzip -1 | ssh pi 'gunzip | docker load'`
(~1.6 MB/s over the phone hotspot, ~12 min), retagged pi5, old image kept as pi5-prev
(rollback: docker tag as2-rescue-onboard:pi5-prev as2-rescue-onboard:pi5 && docker compose up -d
--force-recreate onboard). compose now has cap_add SYS_NICE.

On the Pi (tracking live, disarmed, 2.4 GHz, no undervoltage during the runs):
- before (old image): 3.94 cores busy; detection median 142 ms, p90 257 ms (~7/s); ~18 fps;
  video stalls every ~35-40 s with H.265 (none since H.264).
- 2 detector threads: detection worse (median 177 ms, p90 590 ms; NCNN threads spin-wait on
  each other on busy cores; detector 67 % CPU). Default changed back to 1 thread.
- **1 thread + raised priority (nice -7 detector, -2 tracker): detection median 71 ms (~14/s,
  above the 10/s needed at 5 m/s), p90 86 ms, tracker 25 fps (full camera rate), 0 stalls.**
  CPU still ~100 % busy (freed time is used); rescue Python layer measured 1.32-1.70 cores
  depending on contention, AS2 C++ core 0.79-1.01.
- All links running, FC connected, tracker locked a person and the gimbal followed.
- Pi: model_library.py (1-thread default) hot-patched; source updated (next build bakes it in).

## 2026-10-25: Where the Pi's CPU goes with tracking live, and the no-feature-loss optimisations

30 s per-process measurement on the Pi (tracking live, no mission, before this change): 3.97 of 4
cores. Camera + AI ~1.07 (tracker 63 %, detector 28 %, bridge 16 %); AS2 C++ core ~0.96 (PX4
platform 21 %, estimator 13 %, controller 11 %, 4 behaviours 27 %, XRCE agent 10 %, rate gate 15 %);
rescue Python layer ~1.9 (runtime 42 %, gateway 29 %, geolocator 16 %, swarm 15 %, delivery 15 %,
manager 14 %, guardian 13 %, route 13 %, logger 13 %, surface 9 %, payload 6 %). The panel's "AS2"
figure counts everything ROS (rescue layer and camera bridge too), measured over 0.5 s.
Why the idle Python nodes cost 9-16 % each: ROS 2 Humble's Python executor cost per message, fed
by camera topics at 23 Hz and pose at 25 Hz into nodes whose own loops run at 5-10 Hz.

Changes (nothing removed, no node or topic dropped, the brain keeps full rate):
- Camera bridge: vision/frame every frame (the runtime's input); vision/target, gimbal/state and
  perception/tracks at most 10 Hz (param aux_topics_hz) and at once on any lock change.
  Test: test_vision.test_side_topics_are_limited_but_frames_and_lock_changes_are_not.
- Track logger reads vision/frame through the rate gate at 10 Hz (records changes).
- Swarm agent, delivery zone, route: pose/twist at 12.5 Hz (keep-latest for their 5-10 Hz loops;
  <= 80 ms old against their 0.5 s freshness limits). The runtime keeps 25 Hz pose/twist, 50 Hz IMU.
- Detector: nice -5 in its own process (tracker loop nice -2) via compose cap_add SYS_NICE; at
  least 2 NCNN threads on a 4-core board (it owns cores 2-3).
- Not done (would remove something): dropping parameter services from nodes (ros2 param get/set
  for debugging), lowering the gateway's DroneInterface spin rate, merging nodes into one process.

## 2026-10-25: A8 camera bring-up on the Pi: video reader fix, gimbal/power finding

Camera came up at 192.168.144.25 (MAC 7c:ea:68:8b:e3:27); RTSP main.264 = H.265 1280x720.
- Camera + network are fine when the camera is up: a standalone reader with the tracker's own
  settings gets 25.0 fps for 60 s with no gaps, also with gimbal command load. HEVC decode costs
  ~45 % of one Pi 5 core. The person model still benchmarks 43 Hz.
- **Bug fixed: tracker video froze for good.** The tracker ran ~20 fps for ~30 s, then
  cap.read() blocked forever (py-spy: grabber thread stuck at hybrid_tracker.py:808; eth0 traffic
  fell to the FC's ~540 kbit/s). Reproduced: the A8 stops a session whose reader falls behind 25
  fps (~40 s at 17.5 fps), and a camera reboot kills it silently; with no timeout the reader never
  noticed. hybrid_tracker.open_rtsp(): OpenCV open timeout 5 s, read timeout 2 s -> the grabber
  logs "stalled > 2000 ms (reconnect #n)" and reconnects. Test: tests/test_rtsp_timeout.py
  (59 tracker tests pass). Live on the Pi: a stall was caught in 2 s and video resumed by itself
  when the camera returned. Hot-patched into the container; source synced (next image build).
- After the camera power fix (16:4x): gimbal centred, start-up check passed, tracking running
  (~4300 frames, preview shows person detection). Tracking latency high: detection 130-260 ms in
  the tracker vs 23 ms benchmarked, CPU 99 %, GCS "Telemetry link lost". Cause: Pi 5 undervoltage,
  79 kernel "Undervoltage detected!" events since boot, EXT5V 4.80 V; the firmware drops the CPU
  to 1.0-2.1 GHz (seen at 1.0 GHz) -> everything up to 2.4x slower. With a steady 5.1 V supply
  the Pi has headroom (53 % idle measured between sags). Needs: official 27 W supply or 5 V/5 A
  BEC set to 5.1-5.2 V with short thick wires; camera on its own supply.
- **3 s video latency explained and fixed (camera setting):** the A8 main stream was H.265
  1280x720 1570 kbps. Alone, the Pi decodes it at 25 fps (~47 % of a core); inside the tracker
  (decode pinned to cores 0-1 with the tracker loop and floating AS2 processes, plus undervoltage
  clock drops) it fell to ~10-22 fps, frames queued in the TCP buffer (seconds of delay) until the
  camera cut the session every ~35-40 s (31 auto-reconnects in the old run). Switched the main
  stream to H.264 1280x720 1570 kbps over SIYI cmd 0x21 (ack 01, reversible with codec 2; recording
  stream untouched): decode 28 % of a core; new run 0 stalls in 3+ min, ~18 fps processed.
  (The stream's timestamps tick at 30 fps while it delivers 25: a PTS-vs-wall "lag" is misleading.)
- **Still limited: CPU is full with everything on** (0 % idle): tracker process ~1 core (decode +
  loop; overlay + JPEG preview only while someone watches), detector 1 thread on cores 2-3 gets
  ~25 % because AS2 Python nodes share those cores (detection median 142 ms, p90 257 ms ~ 7 Hz,
  below the 10 Hz wanted at 5 m/s); AS2 grew to ~2.9 cores with the camera on because the bridge
  publishes vision/frame, perception/tracks, vision/target and gimbal/state at 23 Hz each to
  several Python nodes. Next options: gate those camera topics for the Python nodes, give the
  detector priority/2 threads, fix the power; or a Hailo-8L AI HAT to take detection off the CPU.
- **Hardware, not software (open):** the camera dropped off the network for ~60 s at a time
  (reboot), and the gimbal ACKs the centre command (01) but does not move (yaw stays -25 deg,
  stable). The tracker's start-up check (centre ACK + |yaw|,|pitch| <= 3 deg, steady) therefore
  refuses to start tracking, by design. Earlier the same check passed (yaw -1.5 deg). Points at
  the camera power supply (motors draw the current; a sagging supply resets the camera) or a
  blocked yaw axis. Fix the supply/wiring; the camera link retries every 60 s and starts by itself.

## 2026-10-25: One-command field start (start_field.sh)

`./start_field.sh` on the GCS laptop: finds the Pi (saved IP, else scans for its panel on port
8090; or PI_IP=...), runs `docker compose up -d onboard` on it over SSH (the service also starts by
itself at boot), starts the GCS on 0.0.0.0:8765 (phones on the same Wi-Fi) and opens the telemetry
radio at 57600 once the *new* GCS server answers (first run opened it on the old server just before
it was replaced; fixed). Ctrl+C stops the GCS; the Pi keeps running. Never arms or flies.
Camera at that time: answered ARP once (MAC 00:30:1b:ba:02:db) then nothing for 60 s on
192.168.144.0/24 or 192.168.10.0/24: power or cable, not software. The camera link retries by itself.

## 2026-10-25: Full bench health check on the Pi (new image, real FC, disarmed)

Working: all links (FC Ethernet/UDP 8888, guardian, AERIXIS core, mission manager, radio gateway,
web panel); FC connected; flight modes from the GCS; mission upload over the radio (VALIDATED);
AI avoidance ON/OFF from the GCS reaches the runtime in ~1 s ("brain" mode, off by default);
camera AI: active model person-n512-320x192 benchmarks 43.2 Hz on the Pi CPU (need 10 Hz at 5 m/s).
With AS2 down to ~1.4 cores, person-n512-512x288 now benchmarks 14.2 Hz and is unlocked; 640/960
models (2.7-8.1 Hz) stay locked. CPU idle ~59 %.
Not working / bench only: A8 camera not answering at 192.168.144.25 (cable/power) so no live
tracking; no GPS fix indoors (position/home invalid, preflight fails); battery from PX4 flags only
(no battery sensing on the bench); PX4 falls back from OFFBOARD to POSCTL when disarmed and idle.

## 2026-10-25: Flight mode buttons in the GCS (Offboard / Position / Altitude / RTL / Land)

The new GCS had no way to change the FC mode, and the onboard gateway could not do it either:
DO_SET_MODE for POSCTL/ALTCTL called AS2's manual(), which never changed PX4's mode, and its
heartbeat always said OFFBOARD or POSCTL (so LOITER etc. were never shown).

- GCS sidebar: new "Mode: <PX4 mode>" button -> Flight mode panel: tick aircraft, pick Offboard,
  Position hold, Altitude hold, RTL or Land; the current PX4 mode is highlighted and each request
  shows sending / sent to PX4 / accepted / refused (custom_gcs/web/ui/js/flightctl.js, gcs.css;
  main.js?v=15, gcs.css?v=13). RTL / Land use the existing emergency path (FC guardian).
- GCS server: /api/mode with drone_ids -> DO_SET_MODE per MAVLink system ID, re-sent each second
  until the aircraft answers; COMMAND_ACK tracked in state.mode_cmds and logged.
- Onboard gateway: OFFBOARD -> AERIXIS core (it streams PX4's setpoints); POSCTL / ALTCTL ->
  VehicleCommand DO_SET_MODE straight to PX4, PX4's own ack forwarded (IN_PROGRESS then the result;
  3 s timeout = FAILED); heartbeat custom_mode = PX4's real nav_state (protocol.px4_custom_mode).
  Tests: test_protocol FlightModeTests.
- Verified: PX4 1.16 SITL (OFFBOARD -> POSCTL -> ALTCTL -> POSCTL -> OFFBOARD, acks 5 then 0) and on
  the real FC on the bench, disarmed, through GCS -> radio -> Pi -> PX4: POSCTL, ALTCTL, OFFBOARD all
  accepted in ~15 ms, heartbeat showed each. RTL / Land on the ground are refused by design (guardian:
  not airborne). Manual modes need RC sticks/joystick in flight; PX4 refuses POSCTL without position.
- Pi: image rebuilt with the gateway fix (build ok, ~1 h 15 min, no resets) and the onboard
  service recreated, so the fix is permanent now. Rechecked on the bench through the radio: POSCTL,
  ALTCTL, OFFBOARD accepted, GCS showed each; CPU idle 59 %. Camera still unreachable (A8 not
  answering at 192.168.144.25; tracker stops at "Startup center not verified").

## 2026-10-25: Pi CPU fix (rate gate), Pi-only image, bugs found and fixed

**Why AS2 used ~3.9 of 4 cores on the Pi.** Measured (top, py-spy): every Python rescue node used
20-50 % of a core, ~90 % of it inside ROS 2 Humble's Python executor, which rebuilds its wait set
for every message (a few ms on a Pi 5). With the real PX4 1.16 FC on Ethernet, AS2 publishes pose,
twist, IMU and /tf at ~100 Hz, and each Python node woke 100-370 times per second for data it uses
at <= 20 Hz. On PX4 1.14 the Pi's FC link was the old USB path and never connected, so no data
flowed: that is why it looked light. Plus ~65 log lines/s from AS2 behaviours ("Could not get
transform: earth" on every twist while there is no GPS origin), each handled by the gateway and
`ros2 launch` in Python. No simulator was running on the Pi.

Fixes:
- New C++ package rescue_ws/src/as2_rescue_rate_gate (`rate_gate`): takes pose/twist/IMU (25 Hz)
  and /tf (30 Hz *per frame*, so a slow gimbal transform is never dropped) once, without decoding,
  and republishes them under `<drone>/rescue/in/...`. rescue.launch.py remaps the Python nodes
  (runtime, geolocator, payload, swarm agent, delivery zone, gimbal, surface, route) onto it; code
  unchanged; `rate_gate:=false` restores direct topics. AS2's C++ nodes keep the full rate.
- FC guardian: new launch/guardian.launch.py = guardian + its own gate for PX4 local position
  (20 Hz, height only). Own gate so it never depends on the AERIXIS core; either process exiting
  restarts the link. PX4 status / land detector / acks / emergency commands stay direct.
- MAVLink gateway: PX4 link liveness from vehicle_status_v1 (2 Hz) instead of 100 Hz odometry
  (param `px4_status_topic`); pose/twist through the gate.
- AS2 C++: the transform warning throttled to once per 5 s in takeoff/land/go_to/follow_path/
  follow_reference behaviours, the motion controller and as2_core tf_utils.
- Test: test_rate_gate.py (25 Hz pose, 30 Hz odom frame, every 10 Hz gimbal frame kept).

**Bugs found and fixed (deep check):**
1. *Battery reserve could never fire* (safety). AS2's PX4 platform publishes BatteryState.percentage
   as 0..100; the runtime compared it with the 0.25 reserve as a fraction, so a real battery at 18 %
   (18.0) never aborted. Live since PX4 1.16 bridges battery_status (1.14 did not: the PX4 failsafe
   flags were used, so SITL never showed it). runtime_node.battery_fraction(): 0..100 scale
   (param battery_percentage_scale), "no reading" (-1, NaN, not present) = unknown -> PX4 failsafe
   flags. Test: test_battery_fraction.py.
2. *Last stop-and-hover never sent; tracebacks on every link restart.* Humble's SIGINT handler shut
   ROS down before the nodes' finally blocks: the runtime's stop/hover publish failed ("publisher's
   context is invalid"), other nodes raised "rcl_shutdown already called". New
   as2_rescue_runtime/node_main.py: signals only stop the spin; the shutdown step runs with ROS up;
   all rescue nodes use it; gateway fixed inline. Test: test_node_main.py (SIGINT and SIGTERM).
3. *MAVLink waypoint missions failed on the Pi*: the gateway built them as polynomial-trajectory
   missions, a behaviour the Pi never starts. Pi gateway now `use_trajectory:=false` (go_to legs).
4. (earlier today) FC guardian and payload adapter used /drone0/fmu/... with a real FC (/fmu/...).
5. Found by the SITL check of the rate gate itself (PX4 1.16 Gazebo):
   a) *Landing aborted by touchdown.* The runtime's vibration check ran on the ground too: the
      touchdown bump (vibration 12.5, tilt 0.08 deg, altitude -0.04 m) latched AIRCRAFT_UNSTABLE,
      cancelled ManagedLand and hovered. The vibration window ("1 s" in code, 0.5 s in practice
      through a 50-sample buffer at 100 Hz) had become a full 1 s with gated IMU, so the bump
      outlasted the 0.5 s persistence; at 100 Hz it was luck. Now vibration does not count while
      AS2 is LANDING / LANDED / DISARMED (tilt and rates still do; an unstable aircraft should land
      anyway). Test: test_core.test_touchdown_vibration_does_not_abort_the_landing.
   b) *Takeoff cancelled, "vibration_rms unknown".* A strict 0.5 s window left the busy Python
      runtime (which drops IMU messages from its small queue) under the 10 samples needed. Now the
      original rule scaled to the gate: last 25 samples no older than 1 s (VIBRATION_SAMPLES), IMU
      through the gate at 50 Hz. The "data incomplete" warning now names the missing value.
   c) SITL script: an orphaned rate gate from the previous run doubled the gated topics (launches
      started in the background ignore SIGINT); the stop lists now include the gate and launches.
      On the Pi links stop as a process group, so this cannot happen there.

**Pi image is aircraft-only:** no trajectory generation (not used onboard; long compile), no
simulated camera (panel hides "Simulated" when it is not installed), no SITL/radio-hub scripts, no Tk
launcher window (web panel replaces it; entrypoint `gui` explains), no python3-tk, no tests or
avoidance evaluation results. The Pi's source copy (build input only) now holds just what the
Dockerfile copies: 93 MB -> 48 MB (Gazebo/simulator platforms, trajectory generator, laptop scripts,
docs removed; onboard.env kept). Nothing on the Pi's boot path used the removed files.

**Verified on the Pi 5 (new image, real PX4 1.16 FC over Ethernet, disarmed, no flying):**
CPU idle 0.5 % -> 58 % (onboard stack ~3.9 -> ~1.7 of 4 cores). Per process before -> after:
rescue_runtime 52 -> 41 %, mavlink_gateway 40 -> 21 %, swarm_agent 28 -> 10 %, fc_guardian 25 -> 7 %,
geolocator/surface/delivery/route ~20 -> 5-7 % each; rate_gate 5.5 %. PX4 odometry and AS2 pose
still 100 Hz; Python nodes get pose 25 Hz, IMU 47 Hz. "Could not get transform" ~3900/min -> 48/min.
Vehicle connected, battery 100 % from the PX4 failsafe flags (no battery reading on the bench: the
false 0 % / BATTERY_RESERVE abort is gone). Panel PIN works; "Simulated" camera option hidden.
Restart of the as2 + guardian links: 13 processes "finished cleanly", no tracebacks (AS2's own C++
nodes still report exit code -2 = stopped by SIGINT, upstream behaviour). Build: no-cache not needed,
~2 h with the AS2 layer; Pi did not reset (27 W supply). Camera still unreachable (A8 not on the switch).

Laptop checks: rescue 308 + gateway 22 + tracker 57 tests pass (gateway tests need
PYTHONNOUSERSITE=1 on this laptop: numpy 2 in ~/.local; the mesh test needs pyserial, only in
~/.local here, system-wide on the Pi).

## 2026-10-25: GCS "Onboard calibration" finds the Pi by itself

The page no longer needs the Pi's IP typed in (it changes with every hotspot). New
custom_gcs/onboard_discovery.py + GET /api/onboard/discover on the GCS server: scans the /24 of each
of the laptop's real interfaces on port 8090 (second pass with a longer timeout on a cold network)
and keeps hosts whose /api/state has the panel's own fields (drone_id, web_port). onboardcal.js:
saved address first; if empty or not answering, search, pick the panel with the selected drone's
ID (else the first), connect and remember it. "Search again" button forces a fresh scan; typing an
IP still works. Checked in the GCS: wrong saved IP -> searching -> connected to 10.181.33.84 in
4.4 s, Pi panel Calibration & checks shown. Backups: server.py.before_onboard_discovery_20261025,
onboardcal.js.before_autofind_20261025. main.js?v=14.

## 2026-10-25: Pi image clean-rebuilt for PX4 1.16; FC link switched to Ethernet/UDP 8888

Clean rebuild on the Pi 5 (build cache and old images deleted; the as2-data volume and onboard.env
kept): `docker compose build --no-cache --pull`, exit 0 in about 70 min, no power resets on the
27 W supply (throttled 0x50000 = earlier undervoltage, none during the build). The new image
as2-rescue-onboard:pi5 (3.66 GB) contains the PX4 1.16 code (fc_guardian on vehicle_status_v1,
GPS deg fields), the radio fix and the empty fmu_prefix fix, so no hot patches are needed any more.

Onboard service recreated on the new image. Panel PIN drone1 works. FC link changed in the panel
from the old USB serial (ArduPilot device path) to `fc_mode udp`, `fc_udp_port 8888`: the
MicroXRCEAgent now listens on udp4 8888. Radio: CP2102 present, gateway running on serial 57600.

**Verified later the same day (cable reconnected): PX4 1.16 data reaches AS2 over Ethernet.**
eth0 up at 100 Mb/s with 192.168.144.20 + 192.168.10.20 (also after an accidental Pi reboot); FC
192.168.10.14 answers ping (0.2 ms); `/fmu/out/vehicle_odometry` at 101 Hz; vehicle_status_v1,
GPS, battery, failsafe_flags, sensor_combined bridged; AS2 `platform/info` connected=true; panel
shows the vehicle connected, disarmed. No flying.

Bug found and fixed on the real FC: with an empty fmu_prefix (real FC publishes `/fmu/...`) the
FC guardian used the relative `fmu/...` = `/drone0/fmu/...` and never saw PX4 status (so it could
not command RTL/land); the payload AUX adapter had the same relative topics and no prefix setting.
fc_guardian.py: empty prefix = `/fmu`; payload_node.py: new `fmu_prefix` parameter; rescue.launch.py
and onboard.launch.py pass it on. SITL never showed it because it runs with `/drone0`. After the fix
the guardian reports ON_GROUND/disarmed from the real FC. Runtime tests 178 pass (run with an own
ROS_DOMAIN_ID: on the same Wi-Fi as the Pi the guardian test picks up the real FC's /fmu topics).
Hot-patched into the Pi container (survives reboot, lost on `--force-recreate`); source copied to
the Pi, so the next image build includes it.

Bench state seen (indoors): no GPS fix, so local/global/home position invalid and preflight fails;
battery reads 0 (no battery sensing on the bench); PX4 mode was RTL while disarmed. A8 camera at
192.168.144.25 does not answer ping (tracker restarting): check the camera's cable/power on the switch.

## 2026-10-24 (night, later): PX4 1.16 + Gazebo Harmonic SITL — geotag mission SUCCESS

Gazebo Harmonic 8.15 installed next to Garden 7.9 (nothing removed). PX4 1.16.1 SITL rebuilt
against it (gz-transport13/sim8). Fixes needed in the simulator runner (`~/px4_sitl/run_px4_sitl.sh`,
backup `.before_harmonic_20261024`): a v1.14 tree gets a `gz-garden/gz` shim (forces Garden), a
newer tree drops AeroStack2's Garden-built Gazebo assets from the paths and sources PX4's
`build/.../rootfs/gz_env.sh` (without it Harmonic loaded no sensor systems: "Accel Sensor 0
missing"); `sitl_mission_test.sh` sets GZ_IP=127.0.0.1. PX4 1.16 lost the `px4-rc.params` hook:
our SITL parameter block is now `ROMFS/.../init.d-posix/px4-rc.as2` (sourced after the airframe,
COM_OBL_ACT→COM_OBL_RC_ACT) plus NAV_DLL_ACT 0 — gz_x500 sets it to 2 and 1.16 then refuses to arm
without a GCS heartbeat on the FC. The user's real FC has NAV_DLL_ACT 0 (checked over USB), so the
field set-up (GCS via the Pi gateway) is not blocked.
Result: human_geotag_resume **SUCCESS** on PX4 1.16 + Gazebo — two people geotagged (47.39776,
8.54573 / 47.39783, 8.54554, ±0.97 m), coverage completed, RTL, landing. The 3 m corridor
failure seen with SIH did not occur: it was the SIH model's drift. Seen but recovered by the tree:
two "ResumeCoverage: Route refused (crosses outside/excluded area)" retries after the first geotag
— to be looked at; the mission still succeeded. Pi image still to be rebuilt (Pi offline).

## 2026-10-24 (late night): onboard AS2 moved to PX4 v1.16 messages; tested in PX4 1.16.1 SITL

Why: the user's FC is an Aeromind FMU v6X (Pixhawk 6X class, Ethernet) on PX4 **v1.16.1**; AS2 was
built for v1.14 messages. FC programmed over USB from the laptop: static IP 192.168.10.14
(net.cfg), UXRCE_DDS_CFG=1000 (Ethernet), UXRCE_DDS_AG_IP=192.168.10.20 (the Pi), port 8888,
MAV_SYS_ID 1 — verified after reboot (`netman show`, `uxrce_dds_client status`). It was in DHCP
"fallback" with DDS off before.
Changes: `projects/px4_msgs` → release/1.16 (pinned in as2_cli/projects.repos; v1.14 copy backed
up in the scratchpad). as2_platform_pixhawk: GPS from latitude_deg/longitude_deg/
altitude_ellipsoid_m (the old ÷1e7/÷1e3 scaling removed), BatteryStatus.design_capacity gone (NaN),
attitude setpoint roll/pitch/yaw_body gone. Only VehicleStatus is versioned in 1.16 →
`vehicle_status_v1` in fc_guardian, fc_health (and its test); failsafe `geofence_breached`,
`navigator_failure`; nav states POSITION_SLOW / EXTERNAL1. Pi gateway now listens to
`<fmu_prefix>/fmu/out/vehicle_odometry` (a real FC has no /drone0 namespace; before, the GCS would
never get position from the real drone). Panel FC mode label "Ethernet / UDP".
SITL: PX4 1.16 needs Gazebo Harmonic (laptop has Garden 7.9, kept for the 1.14 simulator), so the
test used PX4's built-in SIH simulator (`PX4_SIMULATOR=sihsim PX4_SYS_AUTOSTART=10040`,
`PX4_AUTOPILOT_DIR=~/px4_sitl/PX4-Autopilot-1.16`). sitl_mission_test.sh now runs in its own DDS
domain (77): with the Pi on the same hotspot, the simulated PX4 and the real onboard nodes saw each
other's topics.
Result (human_geotag_resume): take-off, survey, two geotags at the right coordinates (47.39783,
8.54555 / 47.39776, 8.54573, ±0.97 m — GPS reaches AS2 in degrees), RTL at 16 m, landing; FC
guardian reads vehicle_status_v1 (ON_GROUND/disarmed), fc_health decodes status, GPS (3D, 10
sats), no faults. **Found and fixed:** ResumeCoverage re-saved its own resume point at leg start, so
its goal moved ("Approved leg goal changed") — engine.py + test_resume_coverage.py (runtime 186
pass). **Still failing:** the run ends FAILURE because a survey leg left the 3 m route corridor
(about 4 m south drift); SIH also drifted ~2 m on take-off and ~6 m on landing, so SIH dynamics are
the likely cause, not the messages — not proven without Gazebo Harmonic. Pi image not rebuilt yet
(Pi off the network). No real flight until this is understood.

## 2026-10-24 (late): Pi image rebuilt with the radio fix; PX4 platform crash found and fixed

Image rebuilt on the Pi (detached build, BUILD_EXIT=0; full rebuild because the gateway sits in the
AS2 source copied early). After the restart: radio link OK (GCS "MAVLink active via Serial",
heartbeat 1 Hz, drone0 CONNECTED, PIN drone1 accepted, both radio ends 57600).
New defect seen on the real Pi: `as2_platform_pixhawk` died at start with InvalidTopicNameError —
an empty `fmu_prefix` became the text `''` (as2_core's config-file launch helper reads YAML as
text), so topics were `''/fmu/out/...`. SITL never hit it (it uses `/droneN`). Fixed in
`as2_rescue_runtime/launch/onboard.launch.py`: the key is left out of the generated config when
empty and `fmu_prefix` is passed to the platform launch explicitly; the platform now logs an empty
prefix and stays up. Hot-patched in the running container; the next image build keeps it.
The camera link is off in the Pi's saved settings (camera_mode off).

## 2026-10-24 (night): GCS telemetry radio works on the real Pi 5

Root causes, measured on the hardware: (1) both SiK radios' serial speed is **57600** (a test
pattern sent laptop → Pi arrived 120/120 at 57600, garbled at 115200); both ends now 57600.
(2) **Gateway bug:** with a serial radio `self.peer` stayed None (pymavlink serial links have no
sender address), so the gateway never sent a heartbeat or telemetry over the radio — fixed in
`as2_mavlink_gateway/gateway_node.py` (peer = the radio). (3) Pi power: the UBEC gave 4.59 V
(constant under-voltage, CP2102 errors -110); after the user's fix 4.84-5.0 V, no throttling.
Result: GCS "MAVLink active via Serial", heartbeat 1 Hz, drone0 CONNECTED / fresh. The fix is
hot-patched in the running Pi container; the image must be rebuilt to keep it after a recreate.

## 2026-10-24 (night): GCS menu "Onboard calibration"

`custom_gcs/web/ui/js/onboardcal.js` (+ CSS in gcs.css, entry added in main.js after Logs & data;
backups `*.before_onboardcal_20261024`). Opens the drone's onboard panel full window, straight on
its Calibrate page (`http://<onboard-ip>:8090/#calib`): aircraft selector, address field remembered
per aircraft, Connect / reload / new-tab buttons, reachability check with a "connect to the
onboard computer" screen when it does not answer. Back button top left (also Esc, or going to
another GCS page) returns to the GCS unchanged; slide/blur animation (off with reduced motion);
phone layout. Above the app but under toasts and the emergency view. No file link with AS2: it
is the drone's web page over the drone's network (not the telemetry radio). Laptop test: opens,
loads the panel test copy, back/Esc, no browser-history entries added, unreachable address
handled, phone width without sideways scroll.

## 2026-10-24 (night): onboard panel redesigned — sidebar pages, widgets, calibration page, PIN

`rescue_ws/scripts/onboard_web.py` page rebuilt (all earlier features kept): icon sidebar (bottom bar
on a phone) with six pages — **Home** (vehicle tiles: battery, GNSS, altitude, mode, camera,
mission; onboard links with restart), **Connect** (GCS radio, flight controller, mesh as segmented
buttons; baud lists; UDP-only fields appear only in UDP mode; plugged-in ports), **Calibrate**
(pre-flight checklist ✅/❌, FC sensor health + PX4 messages, gimbal check buttons centre / 45° /
straight down with live video), **Camera & AI** (camera mode + live video side by side, the AI
model card with its help folded away), **Drone** (identity + pairing token, safety switches as
toggles, change PIN), **Log**. Red dots on the sidebar flag a problem page.
New API: `POST /api/pin` (change PIN, 4-32 chars), `POST /api/gimbal` (pose centre/down45/down;
ground only, Track mode only; drops any lock for 10 s, then an absolute angle via the tracker's
control port). `AS2_PANEL_PIN` in onboard.env sets the PIN when the panel first starts.
Sensor calibration itself (compass, accel) is not in the panel: it runs on the FC via QGroundControl.
User's PIN: `drone1` (laptop test panel set; the Pi is set when it is next on the network).
Tested on the laptop: every page, wrong/right PIN, gimbal refused in Preview, PIN rule, toggle and
segmented saves, phone width without horizontal scroll.

## 2026-10-24 (evening): lock until the target leaves the frame, speed-based camera defaults

User's conditions: tracking 19-25 fps, fast detection, the lock must stay on the target until it
is no longer in the gimbal video, at the drone's flying speed.
**Defaults (panel card 7):** drone speed while tracking 5 m/s, min tracking 19 fps (A8 video is
25 fps), detection needed = speed ÷ 0.5 m (the drone moves at most ~one person width between two
AI checks), never below 5 Hz: 2 m/s → 5 Hz, 5 m/s → 10 Hz, 8 m/s → 16 Hz. Models too slow for
that on the board are locked. Pi 5 estimate: the 640x384 person model ≈ 10 Hz (just enough at
5 m/s), 512x288 ≈ 17 Hz, 320x192 ≈ 41 Hz — the benchmark on the Pi decides.
**Lock logic (`dual_tracking.py`), each change found with a ground-truth crowd-pan test:**
- a detector miss while other people are seen is a miss, not a conflict (was every "ambiguous"
  stop); a detection overlapping the target that belongs to someone else still stops the gimbal
  (ratio test: second overlap ≥ half the best);
- detector quiet (small target): a confident NanoTrack (score ≥ 0.6, away from the edge) keeps
  driving up to 4 s, unless the detector sees others but not ours (then the 2 s grace);
- NanoTrack may not jump more than ¾ of the box size per frame (was 1.25×/50 px: it hopped to
  neighbours);
- at the frame edge only fresh detections (≤ 0.3 s) move the gimbal; a target last confirmed at
  the edge is never re-attached to a "new ID" (that was always the neighbour);
- the selection is released when the target left the frame (1 s), was not seen for 5 s, or its
  identity stayed in doubt for 1.5 s; then the target nearest the centre is selected. Before, a
  selection could stay held forever with the gimbal stopped.
**Measured (laptop, same pan videos, detector every 2/3/5 frames, 53/120 px/s, gimbal not moving =
worst case):** original dualtrack2 logic: one lock never released, 10-60 frames driving on the
wrong person per run, on target 23-100 %; now: 0-7 wrong frames (only while a person half-exits),
on target 86-100 %, release + next target in ≤ 3 frames. Live pipeline on the paced video: gimbal
driving 91 % of frames (was 66 %), 25 fps. At 200 px/s with 5 Hz detection (faster than the speed
rule allows) OC-SORT loses IDs: 28 % on target — the reason detection must grow with speed.
Tests: tracker 57. Not verified: real A8 video and gimbal on the Pi 5 (tonight).

## 2026-10-24 (later): any-class detection, model library in the onboard panel, geotag fusion

**Detection of any class.** `tracker/src/detector_models.py` replaces the person-only NCNN
detector: any YOLO detect model (4 + classes rows, YOLO26/YOLO11/YOLOv8, end2end off), class
names and input size from metadata.yaml (or the ONNX metadata), class-aware NMS, person-only
shape rules (aspect, minimum height), other classes ≥ 8 px. Backends: NCNN, ONNX (onnxruntime,
now in the Pi image), RKNN (written, not run: needs an RK3588). HEF (Hailo) and TensorRT models are
recognised but their inference is not written yet: the panel keeps them locked. ONNX and NCNN of
the same model gave identical boxes on the laptop test image.
**Tracking of any class.** `dual_tracking.DualTracker`: one OC-SORT per class (track id =
class × 10000 + id, class 0 unchanged), the selection keeps its class (a dog box never continues a
person's track, nor confirms it), `--lock-classes` limits what the gimbal may lock on, other
detected classes are shown. Telemetry carries `label`, `box`, `model`, `selected_class`; distance
only for classes with a known height. **Safety:** the bridge reports a lock on a class not in its
`rescue_classes` parameter (default `person`) as *no lock* (`lock_state LOCKED_OTHER`): rescue
missions never geotag, drop or claim on a dog/car lock; the gimbal still follows it.
**Model library (onboard panel card 7 · Camera & AI models).** `tracker/src/model_library.py`:
- Board: the operator picks Raspberry Pi 5 / Pi 5 + AI HAT+ 13 or 26 TOPS / Vicharak Axon /
  Jetson Nano / Jetson Orin Nano / other; the panel reads the real hardware (device tree, CPU
  type, cores/threads, clock, RAM, /dev/hailo0 + hailortcli, RK NPU, Tegra, temperature) and warns
  when they disagree.
- CPU budget: measured from /proc for 0.5 s — AS2/flight processes, camera tracker, other; the
  detector gets the cores left after AS2, the camera pipeline (0.6 core) and a 0.25 reserve,
  capped by the detector cores setting. Chosen threads go to the tracker (`--threads`).
- Speed and lock: detection rate per model measured by the Benchmark button on the board (tracker
  Python, synthetic frames, detector cores), else estimated from published figures scaled by input
  pixels, model size and threads. Locked when the board has no runtime/accelerator for the format,
  or detection < minimum Hz (default 5) or tracking < minimum fps (default 16) — both editable.
  Locked models cannot be made active. Filters by type and resolution.
- Upload (PIN, after landing): zip of an export folder or a single .onnx/.rknn/.engine/.hef, up to
  300 MB; zip paths/links checked, class names / input size can be given when metadata is missing;
  .pt refused with the export command. Delete (not shipped models, not the active one). Activate
  with detect / lock classes → camera link restarts. All refused while armed or a mission runs.
- Shipped models (`tracker/models`): person 512-trained at 320x192 and 512x288, 640-trained at
  640x384 and 960x544, 80-class objects (yolo26n COCO) at 640x384; `model/` stays the default
  (person 512-trained at 640x384). Laptop CPU (2 threads): 85 / 55 / 32 / 14 / 31 Hz. A 960x544
  model gets 960x540 video (no upscaling): a person at 20 m is ~24 px there instead of ~16 px.
  Pi 5 estimates (2 threads): 41 / 17 / 10 / 4.7 (locked at 5 Hz) / 10 Hz — to be measured tonight.
**Geotag fusion.** `GeotagOverhead` now averages ≥ 5 overhead samples over 1 s (robust mean:
samples > 3× the median residual, min 1 m, from the median are dropped), each with its own GNSS
fix; the tag records samples, rejected samples, spread and a sigma (GNSS, angle error / √n,
spread). Timeout 3 s with at least one sample (never worse than before); a different person
under the camera adds no samples. Research basis: repeated measurements of a tracked target
reduce geolocation error (EKF / sliding-window least squares in the literature); GNSS bias is not
reduced by averaging and stays in the sigma.
**Tested:** tracker 48 (any-class decode, class-aware NMS, per-class lock, library rules, upload
safety, wrapper), package 9 (bridge class filter), runtime 176 (geotag fusion + outlier), manager
18, planner 70, avoidance 19, perception 3. Panel API exercised end to end on the laptop
(board, upload ONNX zip, benchmark 24 Hz, refusals). Tracker on a paced test video with the
80-class model: persons + backpack detected, lock on the chosen class (backpack) at 25-28 fps.
Not verified: anything on the Pi 5 (rebuild tonight), Hailo/RKNN/TensorRT inference.

## 2026-10-24: A8 tracker replaced by dualtrack2 + AERIXIS layer, nano model at 640x384

User's choice: the tracker code is now `A8_DUAL_TRACKER-dualtrack2` (its own gimbal control and
single-target DualTracker; the multilock1 multi-person policy, "fast" control profile and
single-person recovery are gone). Ported onto it: the AERIXIS layer (`as2_link.py` mission link +
gimbal attitude, telemetry fields for the bridge, SUPPRESSED state, rate pause during commanded
moves, options --min-person-height/--detector-cores/--as2-link/--control-port/--telemetry-port/
--attitude-in-preview) and the 2026-10-23 fixes (gimbal replies matched by time, detector on its own
cores, preview drawn only for viewers, ncnn pinned). dualtrack2 hard-coded a 50 px minimum person
height; it is now the option (24 px via the onboard wrapper). Backup of the previous tracker:
scratchpad `tracker_before_dualtrack2_20261024`.

Model: the user's `yolo26n_512_person` re-exported to NCNN at **640x384** (16:9; a 640x360 frame
fills it, 12 px bars instead of 44 % square padding; people ~25 % larger than at 512 for the same
cost). Validation (2 163 aerial images, .pt): 512 square P 0.695 R 0.443 mAP50 0.491 → 640 rect
P 0.708 R 0.474 mAP50 0.526 (the previous 9.45 M model: R 0.517 mAP50 0.578 at 512). NCNN export
checked against the .pt (top scores equal to 3 decimals). The tracker refuses a model exported for
another input size (metadata.yaml). Laptop run on a people-walking video: 527 detections, 0 stale,
OC-SORT → NanoTrack lock held (0.89–0.95). Tests: tracker 27 (8 new), package 7.
Person size vs altitude (A8 mini 720p, 45° down): ~26 px at 12 m, ~16 px at 20 m in the 640x360
frame — 20 m needs a lower size limit (16 px) or zoom; not changed (user's choice).
Open: Pi image rebuild + camera test (the Pi is not on the laptop's network today).

## 2026-10-23: real Pi 5 bring-up — camera, gimbal, person detection speed

Pi 5 (Ubuntu 24.04, 10.140.40.84) runs the arm64 onboard image built on the Pi; laptop GCS on the
telemetry radio, drone0 paired. Findings and fixes:
- **FC runs ArduPilot** (USB "ArduPilot Pixhawk1"): AERIXIS needs PX4 (uXRCE-DDS). Open: flash PX4
  v1.14 (operator's decision; erases ArduPilot calibration).
- **A8 mini unreachable**: eth0 had no address → 192.168.144.20/24 (netplan 60-aerixis-camera.yaml).
- **Gimbal never verified / no gimbal attitude**: this A8 firmware replies with its own sequence
  counter, the tracker matched replies by ours → startup centring always failed ("tracking was not
  started") and AERIXIS never got gimbal angles (GeolocationValid false). Now one query in flight,
  matched by command and time, late replies never taken as fresh (`siyi_startup.py`, `as2_link.py`,
  7 new tests incl. stale/no-ACK/NACK/moving still failing).
- **Tracker death hidden**: launch kept running → link showed "running"; now the launch ends with the
  tracker so the service restarts it (5–60 s backoff).
- **Detection 450–600 ms (all results discarded as stale > 0.4 s)**: the model is the 9.45 M "small"
  network (mislabelled nano). Kept: on 2 163 validation images it finds more people than the real
  nano model (recall 0.52 vs 0.45, mAP50 0.58 vs 0.50). Pi benchmark: 240 ms on 2 free cores, 450 ms
  when sharing cores 2–3 with video decoding and the tracker loop; more threads are slower. Now the
  detector process has cores 2–3 to itself (`--detector-cores`), the tracker loop 0–1, flight nodes
  unpinned. ncnn pinned to 1.0.20260526 (1.0.20261020 was 25 % slower). Measured on the Pi in track
  mode: 450–600 ms -> 310–350 ms per inference, ~80 % of results used (before: 0 %).
- **CPU**: `rescue/state` 20 Hz -> 10 Hz plus immediate on any safety-relevant change (flags,
  operation, hold, abort/mode, payload transaction): every reader accepts 0.5 s (guardian 3 s). The
  preview HUD is drawn and JPEG-encoded only while someone watches :8080. The camera link backs off
  5→60 s when the tracker keeps failing its start-up check (it reset to 5 s before). Remaining cost is
  rclpy per-message overhead for the camera topics (3 per frame at ~26 fps) — merging them is a
  larger follow-up. `a8_gimbal` (runtime gimbal_node, not launched) still matches by sequence number.
- **Steady state on the rebuilt image** (track mode, 2 min, 100 inferences): median 363 ms (p10 346,
  p90 378), result age median 399 ms → ~60 % of results used (0 % before), tracking starts. Still
  above the tracker's 0.4 s observation window (not changed: it bounds how old a person position may
  be when the gimbal acts). Pi CPU ~85 %: detector 1.3 cores, tracker loop 0.75 (was 0.96 before the
  preview change), AS2/AERIXIS nodes ~1.2 (runtime 37 %, MAVLink gateway 28 % with no FC, track
  logger 11 %, camera bridge 12 %). Next: cut that node cost (merge the 3 camera topics per frame,
  gateway idle loop, logger) so the detector gets its 240 ms.
- **Node CPU cut (same day, later)**: runtime + track logger take the camera data as one
  `vision/frame` per frame (target + gimbal + tracks; same data and rate, both bridges publish it, the
  separate topics stay for the other readers; runtime `camera_topics:=separate` for other sources);
  the track logger takes position/speed from rescue/state instead of pose/twist at odometry rate;
  MAVLink gateway polls at 50 Hz (still drains everything, 2 reply frames per poll = same 100/s).
  Not done on purpose: throttling vision/target (the runtime uses it for centring, overhead and
  OverheadDropReady) or the tracking topics. Measured on the Pi, track mode, 90 s: inference median
  **300 ms** (p90 319), result age mean 340 ms, 6/76 over 400 ms → ~92 % used. Note: with PX4
  connected, pose/twist/IMU arrive at odometry rate in several Python nodes — measure again then.
- **Camera** went offline once during testing (no ping, ARP incomplete), back ~10 min later; the
  first centre check after that failed and the retry passed (yaw sits near the ±3° limit: −2.9…−3.0°
  centred once, 0.3° / 2.0° other times) — worth a gimbal centre calibration in SIYI's tool.

## 2026-10-23: GCS emergency view only on a real emergency, automatic reset after a flight, Pi image

- **Emergency view** (`custom_gcs/web/ui/js/emergency.js`) opens by itself only when AERIXIS core has
  failed on an aircraft in the air (not running on the Pi, or the FC guardian took over after losing it)
  and closes by itself when that is over ("Emergency over"). Aborts, PX4 faults, battery, link loss,
  mission failures and the operator's own RTL/LAND are toasts only (alerts.js). The Emergency button
  still opens it for every aircraft. SITL: drone0's runtime killed at 10 m -> view opened within ~3 s,
  guardian RTL landed it, view closed after 26 s.
- **Automatic reset after a flight** (`autoreconcile.js`): the manager clears its lockout itself only
  after a SUCCESS mission; a FAILURE/cancelled/aborted one that landed normally needed Reconcile every
  time. The GCS now reconciles by itself once no mission runs, PX4 reports disarmed, status is fresh and
  the FC reports no fault (the aircraft re-checks disarmed-on-ground); a pop-up names each aircraft and
  what is reset. FC faults are never cleared automatically. Setting: bell -> "Reset after each flight".
  SITL: both drones reset 6–10 s after landing from a FAILURE mission; the next mission started at once.
- **Pi mesh: base station + LoRa fallback** (user: the image lacked the swarm/AI design): the agreed
  design (base-station IP network main, LoRa long-range fallback) was not implemented — the bridge ran
  LoRa *or* UDP. `mesh_bridge transport:=both` (now the default `mesh_mode`): every frame over UDP at
  full rate and over LoRa at the modem rate, always on, duplicates dropped by the receivers, missing
  modem retried while UDP carries on; per-link health. Panel/GUI offer the mode; the panel's Mesh card
  shows both links, the separation and the AI avoidance state. Real-process tests with a pty "radio":
  LoRa-only delivery with UDP dead, no duplicates with both, UDP without a modem (3 tests). Swarm and AI
  still need a mesh passphrase on every drone (without it the drone flies alone).
- **Pi 5 image**: `docker/raspberrypi5/Dockerfile.onboard` rebuilt (amd64 test image on the laptop:
  brain v4 with 43 inputs, area walls, take-off/arm checks, driver fix). The Pi builds its own arm64
  image (`docker compose build`, ~40–60 min the first time). `as2_pi.sh failsafe` and the container's
  `/entrypoint.sh failsafe RTL /dev/ttyACM0` now pass the separation setting as the return-height step.

## 2026-10-23: avoidance stage 4 — 3 m separation (RTK), AI return at mission height, brain v4

Operator choices: 3 m separation (RTK on every drone, experts only; default stays 10 m) and, with AI on,
return home at mission height through the avoidance layer. One `minimum_separation_m` now drives the
gate, the avoidance radius, the brain and the return step; PX4 emergency RTL heights 16 / 19 / 22 m
with 3 m (16 / 26 / 36 m with 10 m). Brain v4 retrained for both rules (43 inputs, same neighbour rule
as onboard); 6 000 unseen encounters per rule: 0 collisions, 0 under 3 m, all completed, 1.7 % faster
than ORCA (10 m: all completed vs 2968/3000, under 10 m 2 vs 4). Gazebo SITL faults found and fixed:
take-off on a disarmed FC (PX4 refused to arm, AS2 reported ARM) now fails in 8 s / no climb in 20 s;
return/land on the ground end at once; landing waits for the velocity hand-back; AI commands limited to
1.5 m/s^2 and the AI return keeps the nose; driver sent a yaw pose with position (0,0,0) into a controller
still in POSITION mode (PX4 got 41 m/s for one cycle) — now real position + own SPEED ack; planning-area
edges are hard ORCA limits (operator's 26 x 28 m area: "detour left the planning area"); refused legs fail
in 5 s with the reason instead of hovering. SITL: head-on and the operator's small-area mission both
complete cleanly (heights <= 15.9 m). Tests: runtime 169, avoidance 19, manager 18, planner 69.
Details `docs/AVOIDANCE_STAGE4.md`.

## 2026-10-23: in-flight avoidance stage 3 — onboard, GCS "AI" switch, Gazebo SITL

Avoidance layer in the rescue runtime (brain + ORCA shield fly route legs by velocity when ON; OFF = usual
GoTo), velocity in mesh frames, 25 m avoidance corridor inside the area, signed manager switch (OFF at
every start), GCS menu "AI" button + card status, planner flies KML LineStrings named after drones.
Gazebo SITL head-on (2 drones, same line): AI off -> both missions stopped by the gate (9.5 m); AI on ->
both completed, closest 15.1 m during the legs. Found and fixed: route corridor (3 m) too tight for
avoidance; return height step 5 m -> 10 m (returning drones crossed 5.6 m apart vertically). 3-drone
run not completed (laptop overload, drone2 SITL GPS/estimator). Retry: fixed the SITL ground (60x40 m,
drone2 fell off it), vehicle-data age mismatch ('NoneType' on legs), avoidance of grounded drones, mesh
ports in the ephemeral range (now 24800+), origin only from a steady GNSS fix; 2-drone head-on with all
fixes: both completed, 15.1 m on the legs, 10.6 m on the way home. 3 drones still not flown: PX4's own
simulated sensors time out on this laptop. Details `docs/AVOIDANCE_STAGE3.md`.

## 2026-10-23: in-flight avoidance stage 2 — learned brain with ORCA shield

PPO-trained policy shared by all drones (39 inputs from its own mesh view, decides every 0.5 s, ORCA
shield every 0.1 s, 28 KB NumPy model, ~8 µs per decision). On 9 000 unseen simulated encounters vs ORCA:
0 collisions (both), under 10 m 1/4500 vs 6/4500, mission time +6 % vs +11 %, overtaking fixed, head-on
always on the right; crowds 897/900 completed vs 899/900. Three training runs (run 1 speeded, run 2
slowed down — both caught and fixed in the reward). Details `docs/AVOIDANCE_STAGE2.md`. Model shipped in
`as2_rescue_avoidance/models/brain.npz`; tests 15 pass. Simulation only — stage 3 (onboard + GCS AI
toggle + Gazebo SITL) not started.

## 2026-10-23: in-flight avoidance stage 1 — simulator + ORCA baseline

New package `rescue_ws/src/as2_rescue_avoidance` (simulator with base-station / LoRa link models, ORCA
shield with right-hand passing, 5 encounter families, evaluation CLI). 13 500 simulated episodes:
today's stop gate never collides but aborts ~98 % of missions that meet another drone and still dips
to 7.7 m; ORCA completes 99.9 % with 0 collisions, head-on always passes on the right; 5 brief dips to
9.2–9.9 m (GNSS error) and 3 unfinished encounters remain. Details: `docs/AVOIDANCE_STAGE1.md`. Tests 11
pass. Not flown in Gazebo or on drones yet; nothing in the onboard runtime uses it yet. Next: stage 2
(learned policy), stage 3 (onboard + GCS AI toggle).

## 2026-10-23 (later): cancel lands, menu flight controls, GCS without AS2 files, safety review

User report: cancelling a mission left the drone hovering; Orbit person: drone0 only yawed, drone1 failed.

- **Orbit person "only yaw"** is mission 2 working as designed (hold a station, sweep yaw until a person is
  seen). The run used `sample_field_mission.kml` (26x28 m); the simulated people (6,-28) and (-5,16) are
  outside it, so nobody could ever be detected. Use a larger KML, or `launch/sim.sh start N --persons ...`
  inside the area.
- **drone1 failed 0.65 s after arming**: the outer SafetyGate (VehicleHealthy) dropped while AS2's platform
  paused its topics for the PX4 arm request (laptop load average ~16). Fix: on the ground (last state
  DISARMED/LANDED) FC data may be up to 3 s old (`ground_vehicle_max_age_s`); airborne stays 1 s. The runtime
  now logs why readiness is lost (`Vehicle readiness lost: <stale data | incomplete | tilt/rate/vibration |
  abort reason>`), also as `flags.health`. Tests in test_core.py.
- **Cancel in flight now returns or lands**: the manager halts the tree and asks the FC guardian for the
  mission's emergency action (Plan setting, RTL by default); PX4 flies it; nothing is sent on the ground.
  SITL: cancel at 10 m → PX4 RTL confirmed at once → landed and disarmed in 21 s.
- **AS2 platform stayed FLYING after PX4 landed by itself** (guardian RTL/LAND, auto-disarm): AS2 only
  accepts DISARM from LANDED, so the next arm/takeoff and Reconcile were refused until a restart.
  `as2_platform_pixhawk` now walks LAND → LANDED → DISARM when PX4 disarms. SITL: FLYING → DISARMED at
  PX4's disarm, Reconcile then accepted.
- **Start refused while the runtime holds an abort** (manager `can_start` and `start`), e.g. an abort that
  survived a restart; before, the drone armed and the tree failed on the ground. Sim start-up reconcile
  retries 2 min and prints the result.
- **Lost `get_origin` reply** (DDS discovery settling) left drone0 without an earth frame for good (take-off
  "earth → base_link" TF errors, platform EMERGENCY). Route publisher now retries after 5 s.
- **Menu flight controls** (custom_gcs, every page, bottom of the menu): Safety switch (server-side, SAFE at
  every GCS start: Start refused from every window; RTL/LAND/DISARM never blocked), RTL, Land (FC guardian →
  PX4), Disarm (all aircraft ticked by default in a picker). DISARM: AERIXIS core force-disarms PX4 (it would
  stop the motors in flight), so the onboard **gateway refuses DISARM unless platform info is fresh and
  LANDED/DISARMED** (test GcsDisarmTest). Menu icon hides/shows the menu (icon rail on wide screens, drawer
  on phones/tablets; checked at 1024 and 375 px, no sideways scroll).
- **GCS has no file link to AS2 any more**: removed "Add simulated aircraft" (it read
  `~/.local/share/as2-sitl` tokens) — add every aircraft with ID, system ID and token (`launch/sim.sh
  pairing` / `launch/onboard.sh token`); the planner is a copy in `custom_gcs/vendor/as2_rescue_planner`
  (`tools/sync_planner.sh`, version stamp) instead of an import from `~/aerostack2`; the laptop BT runner
  (`/api/tree/run`, unused) needs `AERIXIS_LOCAL_BT=1`. Backups `*.before_no_file_link_20261023`,
  `*.before_sidebar_controls_20261023`.
- Tests: runtime 148, manager 17, gateway 22, planner 67 pass; as2_platform_pixhawk built.

**Safety review — still to do (not changed):**
1. ~~Person approach/orbit ignores peer separation~~ — **done 2026-10-23**: in a fleet mission ApproachOverhead
   and OrbitTarget hover in place (height kept, camera on the person, orbit laps not counted) while
   SeparationSafe or PeersFresh is false, and continue when it is safe again; a single hold longer than
   `SEPARATION_HOLD_S` (60 s) fails the response with `SEPARATION_HOLD_TIMEOUT` (the tree's lost-person
   branch releases the person and resumes). Shown as `rescue/state.hold`, runtime log "Person response
   holding …", GCS drone card "Now" row. Tests: approach hold/resume, orbit hold bounded without lap
   progress, single drone never holds (runtime 153 pass). SITL with the run-4 layout: both SUCCESS, min
   spacing 18.1 m — separation never went unsafe, so the hold path is verified by unit tests only.
2. ~~"RTL all" at one height~~ — **done 2026-10-23**: every drone returns at 30 m + 10 m x (system ID - 1) (step raised from 5 m after Gazebo SITL: two returning drones crossed 5.6 m apart vertically)
   (`as2_rescue_runtime/rtl.py`). PX4 RTL: `px4_failsafe_params.py` sets `RTL_RETURN_ALT` and
   `RTL_CONE_ANG` 0 (SITL: drone0 30 m, drone1 35 m confirmed). AERIXIS core ReturnToLaunch climbs in place
   to that height, then flies home level (`system_id` passed by the onboard launch; params
   `return_altitude_base_m` / `_step_m`). SITL RTL-all from the GCS: peaks 30.3 m and 34.2 m, both landed.
   Tests: runtime 150 pass. The base (30 m) must clear obstacles and the mission altitude.
3. PX4 geofence from the KML area (`GF_ACTION` RTL) so PX4 enforces the boundary even if AERIXIS core fails;
   today only AERIXIS core checks the area.
4. Telemetry (GCS) link loss: drones keep flying their mission by design; decide whether a long loss should
   return them (onboard timer in the gateway).
5. Low battery on real hardware: PX4 v1.14 does not bridge battery_status; check `COM_LOW_BAT_ACT` /
   `BAT_CRIT_THR` on each FC and do a bench test.
6. Real FC: RC kill switch and mode switch configured; compass/accel calibration; arming checks visible in
   Swarm start (already shown from PX4 data).
7. Two GCS windows can overwrite each other's upload (see below).
8. GCS emergency view opened once with "No aircraft registered" after a GCS restart (empty fleet poll) —
   should not open on an empty fleet.

## 2026-10-23: custom builder mission and watch re-station flown in 2-drone SITL; 2 runtime bugs fixed

Custom mission made in the GCS builder (camera, Watch stations, geotag + orbit 10 m x 1 lap + continue
searching, RTL/RTL), 100x80 m KML (stations drone0 (0,-20), drone1 (0,20)), planned/uploaded/started from
the GCS, `launch/sim.sh start 2 --persons ...`.

- **Fixed: a person seen during the watch itself was never handled** (every watch mission, built-in mission 2
  too). `ScanArea` was not in the runtime's `PREEMPTIBLE` list, so the response branch's `PauseCoverage` was
  refused ("Another rescue operation owns the runtime"); the geotag/orbit branch failed, the fallback
  released the person as lost and the watch restarted, every sweep (~30 s), forever. Earlier SITL runs only
  ever detected people during the station leg (`SurveyWaypoint`, pre-emptible). Fix: `engine.py`
  `PREEMPTIBLE` + `ScanArea`; test `test_detection_preempts_running_watch`.
- **Fixed: watch re-station aborted the mission** the first time it ran in ROS. `runtime_node.tick()` logged
  "Watch <state>" with warning or info from one call site; rclpy raises "Logger severity cannot be changed
  between calls", the tick exception latched an abort → `ScanArea failed: ABORT_LATCHED` → RTL. Fix: one call
  site per severity; ROS test `RuntimeWatchLogTests` (fails on the old code). `RescueRuntime(args=None)`
  now takes CLI args like `RoutePublisher` (for the test).
- **Fixed (GCS)**: builder orbit Radius/Laps boxes were 22x17 px (`.tgl input` hit the number inputs);
  now `.tgl input[type=checkbox]` (custom_gcs `ui/css/gcs.css`, backup `gcs.css.before_builder_input_fix_20261023`).
- Also: rescue_ws install was stale (planner install had no `features.py`); rebuilt.

SITL results (after the fixes):
- Run 2, person (-5,16): drone1 found the person on its station leg, geotag 0.7 m from truth (±1.9 m),
  orbit 1 lap, release, back to watching; drone0 watched 300 s; both RTL + landed. SUCCESS.
- Run 6, person (-10,0), drone1 started when drone0 claimed: drone0 (watching) pre-empted ScanArea, geotag
  0.6 m off, orbit; drone1's station leg **stopped short** ("drone0 orbit nearby"), **re-stationed** to its
  own alternate (35,25), watched there; drone0 flew back to its planned station after the orbit. Both
  300 s watches SUCCESS ("watch re-stationed 1 time"), RTL, landed. Min 3-D spacing 17.7 m, separation
  never unsafe, drone1 >= 28 m from the person during the orbit (zone ≈ 26 m). **Re-station path now
  verified in SITL.**

- **Fixed: Start could hang forever ("RUNNING", nothing flies, Cancel ignored).** `bt_service_node.hpp`
  (Arm/Disarm/Offboard) called `wait_for_service()` with no timeout in the node constructor, before the
  runner's first event; SIGINT/SIGTERM did not stop it. Seen when the laptop's Wi-Fi changed network
  (06:03 drop, new IP 06:06): the running SITL nodes kept talking to each other (GCS all green) but the
  runner started at 06:10 could not discover `set_arming_state`. Now bounded by the runner's existing
  `wait_for_service_timeout` (5 s) and throws → `finished ERROR "Service set_arming_state not available
  after 5000 ms"` (checked: exit after 5.2 s). Also fixed the action node's broken error string
  (`std::string("…%s…", c_str())`). as2_behavior_tree rebuilt, 6/6 gtests pass. Troubleshooting note in
  `launch/README.md` (restart the stack after a network change).

- **Changed: the ground lockout clears itself after a normal flight.** The manager sets `hold_lockout` at
  every Start and used to clear it only on the operator's Reconcile, even after a perfect landing. Now, when
  the run ended SUCCESS and the FC reports the aircraft disarmed and landed for >= 1 s (`platform/info` from
  PX4 over XRCE-DDS, the same evidence Reconcile uses), the manager runs the same reconcile by itself (log:
  "Ground-state reconciliation complete (automatic: …)"; retried every 5 s if refused). Failed, cancelled
  or interrupted runs keep the lockout for the operator's Reconcile. GCS Swarm start text now says which
  case it is and where the Reconcile button is (`preflight.js`, backup `.before_lockout_text_20261023`).
  Test `test_lockout_clears_itself_only_after_success_on_the_ground`; manager suite 15 passed. SITL: 2-drone
  orbit → both SUCCESS, both unlocked within seconds of landing.

Open, found today (not changed — design decisions):
- **Person response ignores peer separation.** Approach/Orbit/Geotag are not under the SeparationSafe gate
  (only search and watch legs are). Peers avoid a claimed orbit through keep-out zones, but an aircraft
  taking off or landing inside the zone cannot: run 4 (person 18 m from drone1's launch) drone0's orbit
  passed 9.1 m from drone1 climbing out / returning (drone1's own gate tripped → RTL). Options: hold the
  orbit while separation is unsafe, or keep the take-off of a peer blocked while its launch point is in an
  orbit zone.
- Two GCS windows can overwrite each other's upload without warning (the onboard tree is whatever came last;
  Start uses the onboard mission). Seen today when a second window uploaded another plan 19 s later.
- GCS log noise: watch missions report "PersonIncident failed (×N)" every tick (ForceFailure by design).

## 2026-10-22 (late): one launcher per system, onboard control panel (headless Pi)

- `launch/gcs.sh`, `launch/sim.sh`, `launch/onboard.sh` (see `launch/README.md`): start / stop / status / logs,
  each in the background with pid and log files in `~/.local/share/aerixis`. `onboard.sh start [--sim|--docker]`,
  `onboard.sh install` (systemd + desktop autostart of the panel).
- Onboard control panel `rescue_ws/scripts/onboard_web.py`: the last supervised link (`web`, port 8090). It is a
  browser page served by the Pi, so it works on Ubuntu Server without a screen. It sets up the GCS radio, FC link,
  A8 camera (plus a link to the A8 tracker's own dashboard), mesh, identity/pairing and safety switches; shows link
  states, live PX4 data and the onboard log. Changes need a PIN (`token` action). Flight links are refused while
  armed or on a mission. Settings fixed by `AS2_*` env are read-only, so the Docker `onboard.env.example` now
  only sets the identity and leaves the rest to the panel.
- `open_panel.sh` (the last step of an onboard start) waits until every link is running, or until only links
  waiting on hardware remain for 30 s, then opens a browser window (desktop) or prints the addresses (headless).
- Fixed: `route_node` blocked the provisional GNSS origin on a fresh install (no `state.json`), which caused a
  transform-warning flood. The manager and route node used different mission folders when
  `AS2_MISSION_STATE_ROOT` was set. Launcher stop left the SITL onboard processes running: a background bash
  ignores SIGINT, so the launcher now sends TERM after INT.
- Verified on the laptop: `sim.sh start 1` → GCS shows drone0 CONNECTED; `sim.sh start 1 --px4-only` +
  `onboard.sh start --sim` → all 7 links running in 16 s, panel opened last, token + PIN printed; stop leaves no
  processes. Not verified: `onboard.sh install`/autostart on a real Pi desktop, the panel inside an arm64 image.

## 2026-10-22 (late): product rebranded to AERIXIS (user-visible text only), logo + GCS loading splash

Spelling: AERIXIS (the user corrected an earlier "AERXIS"). The raster logo has no I after R; the SVG logo set adds it (user's choice) and drops the raster's second small A (Λ) after the mark, so the logo reads A-ERIXIS with the mark as the A.

The product the operator sees is now **AERIXIS** (all caps, as in the user's logo). Scope chosen by the user:
"everything you see" — internal names are unchanged so builds, systemd and the Pi image keep working
(`as2_*` packages/dirs, Python modules, C++ namespaces, ROS topics/services/actions, message types, file names,
env vars `AS2_*`, storage keys `as2gcs.*`, the `X-AS2-GCS` header, STATUSTEXT protocol `LINK|AS2|…`, guardian state
`WAITING_FOR_AS2`, BSD licence/copyright notices). In this file and the other docs, "AERIXIS core" = the onboard
AeroStack2-based stack. Credit line kept in the GCS About dialog and READMEs: built on the open-source AeroStack2
framework (BSD licence).

- **Logo** (`custom_gcs/web/ui/brand/`): user's original raster (`aerixis-logo-original.webp/.png`), SVGs traced from it
  (`aerixis-logo.svg` navy #0B2347 + teal #04A9B3, `aerixis-logo-dark.svg` light on dark, `aerixis-mark*.svg`,
  `aerixis-icon.svg` + 512/192 PNG, `web/favicon.ico`), generator `custom_gcs/tools/make_brand.py`.
- **GCS** (`/home/prince/custom_gcs`, not git; originals in this session's scratchpad `gcs_backup/custom_gcs_orig.tar`):
  title, favicon, sidebar brand tile (logo + "Mission control", click = About), footer, all UI text, toasts, emergency
  view, preflight/connect/logs pages, classic.html + web/js, BT catalogue descriptions, download names
  (`aerixis-*.json/.xml`), server messages/banner (server.py; **live after the next GCS restart** — not restarted),
  start_gcs.sh, README, GUI-README. Guardian state names are shown via `guardianText()` (links.js), so
  `WAITING_FOR_AS2` reads "waiting for core".
- **Loading splash** (`ui/css/splash.css`, `ui/js/splash.js`, top of `index.html`): frosted card, logo settles in
  (A rises, swoosh sweeps, letters follow), slim bar over the real steps (interface, `/api/state`, `/api/fleet`,
  mission catalogue, maps + imagery probe). Fades into the app; error card (Retry / Open anyway) on a failed step,
  missing script, 8 failed state polls or 25 s; Esc / "Open now" skips it; reloads in the same browser session skip
  the intro. Light + dark checked in the built-in browser. `start_gcs.sh` / `1_launch_gcs.sh` / `launch_gcs.sh` now
  open the GCS in a Chrome app window (else xdg-open) once the server answers (`open_gcs_window.sh`,
  `GCS_OPEN_BROWSER=0` disables).
- **Onboard, text only**: Pi launcher GUI (`as2_pi_launcher.py` title/labels/messages), `as2_pi.sh` usage,
  `as2_onboard_service.py` banner/log lines, `onboard_links.py` errors, systemd `Description=` lines, guardian detail
  texts (`guardian.py`: "AERIXIS core alive / lost …"), runtime/engine/route error texts that reach the GCS, gateway
  and A8-tracker log lines, laptop launch-script echo output, docs prose/headings (`mdbrand` pass skipping code).
  Python suite: 290 passed (rescue_ws/src), gateway tests 21 passed; all GCS JS `node --check` ok.
- **Intentional leftovers**: the repo-root README keeps the upstream AeroStack2 README (framework docs, licence)
  below a new AERIXIS intro; `Aerostack2` in licence headers and `setup.py` maintainer fields; `ros_inspector` shows real
  ROS node/package names (`as2_*`); script names such as `./2_launch_as2.sh` quoted in UI hints are real file names.

## 2026-10-22 (night): external mode change ends the operation; gateway obeys only its own system ID

Closes two follow-ups of the FC guardian work below. Python only (rescue runtime, MAVLink gateway);
no BT, C++ or PX4 change.

- **External mode change** (`as2_rescue_runtime` `engine.py` `Engine.mode_guard`, `runtime_node.py`):
  the runtime now reads PX4's own OFFBOARD flag from `platform/info` (AERIXIS's PX4 platform copies
  `vehicle_control_mode.flag_control_offboard_enabled`). Armed, AERIXIS platform state not
  DISARMED/LANDED, OFFBOARD seen earlier in this arming, and OFFBOARD now off = another mode flies
  the aircraft (RC pilot, FC guardian, GCS RTL/LAND, PX4 failsafe). Then, at once:
  - the running operation (GoTo leg, velocity op, return, land, hold…) ends **FAILURE** with
    `EXTERNAL_MODE_CHANGE: PX4 left OFFBOARD (now RTL); AERIXIS core stopped its operation and does not
    resume it` (PX4 mode name from `fc_guardian/state` when fresh); the GoTo is cancelled, velocity
    streaming stops (one zero twist), gimbal tracking off;
  - the mission is aborted (`ABORT_LATCHED` with this reason in `rescue/state.flags.reason`);
  - while PX4 stays out of OFFBOARD, **every** operation except `ReportObservation` fails at once,
    recovery ones included — AERIXIS never fights the external mode;
  - AERIXIS's hover reference is re-frozen at the measured position every 2 s, and again the moment
    PX4 is switched back to OFFBOARD, so a switch back holds where the aircraft is and never flies
    the old leg or back to where the takeover began. After that only return/land operations are
    accepted (the existing abort rule); a new leg or `ResumeCoverage` is refused. The latch is
    cleared only by the ground reconcile (disarmed, landed).
  - Not a takeover: OFFBOARD set/dropped on the ground (armed or not), PX4 disarming at touchdown,
    a flight that never was in OFFBOARD this arming, silent `platform/info` (last decision kept;
    the guardian covers a dead AERIXIS).
  - Shown: `rescue/state.mode_change` (latched reason), `rescue/state.external_mode`,
    `vehicle.offboard`; runtime log error on the change, warning on the switch back.
- **Gateway target filter** (`as2_mavlink_gateway` `protocol.command_route`/`addressed_to`,
  `gateway_node.handle_message`): on the shared radio every COMMAND_LONG and COMMAND_INT is ignored
  (no ACK, reply path untouched) unless `target_system` is this aircraft's `system_id`. Target 0
  (all aircraft) is honoured only for the fleet-wide emergency RTL/LAND (NAV_RETURN_TO_LAUNCH,
  NAV_LAND to component 0/1, DO_SET_MODE AUTO/RTL|LAND). Same rule (own ID only, never 0) for
  MISSION_COUNT / MISSION_ITEM(_INT) / MISSION_CLEAR_ALL and MANUAL_CONTROL (`target`), which
  would otherwise start, clear or stick-fly every drone in range. COMMAND_INT is accepted for the
  emergency path only; any other COMMAND_INT for this system gets ACK UNSUPPORTED. Before this,
  an ARM/TAKEOFF/mission sent to system 1 acted on every gateway on the radio.
- **Stale test fixed**: `test_protocol.py` now expects `trajectory_generation` (the default) and
  checks `go_to` with `use_trajectory=False`.

Verified (software only):
- `as2_rescue_runtime/test/test_mode_change.py` (11): takeover mid-leg → leg FAILURE with reason,
  stop + track off, no motion; back to OFFBOARD → hover re-frozen, no move, new leg and
  ResumeCoverage refused, ReturnToLaunch accepted; no motion of any kind while external (RTL, land,
  recovery, hold, orbit, track, waypoint) while ReportObservation still works; takeover between
  operations; re-anchor every 2 s; stale platform keeps the decision; no false takeover on the
  ground / at disarm / in a never-OFFBOARD flight; no platform evidence = unchanged behaviour;
  ground reconcile clears; two ROS wiring tests (`platform/info` + `fc_guardian/state` → engine,
  driver re-anchor).
- `as2_mavlink_gateway/test/test_target_filter.py` (10): routing table (own → AERIXIS, foreign
  ignored, broadcast only for emergency RTL/LAND) and the real `handle_message` against a fake
  node (foreign/broadcast ARM, TAKEOFF, AERIXIS land, pause, other drone's RTL → nothing sent, no ACK;
  own commands reach AERIXIS; COMMAND_INT emergency only; mission upload/clear and sticks need own ID).
  Gateway tests: 21 passed. Full rescue_ws suite: 290 passed.
- Built sequentially (`MAKEFLAGS=-j1`, `--parallel-workers 1`): `as2_mavlink_gateway`, then
  `as2_rescue_runtime`.

Not verified yet: SITL (PX4 LAND / RC mode switch mid-leg with AERIXIS core alive, then back to OFFBOARD);
the running SITL was not touched. The GCS's legacy single-drone buttons (ARM, takeoff, land, mode,
mission upload/stop in `custom_gcs/server.py`) always address system 1, so with this filter they
reach only the drone whose `system_id` is 1 (before, every drone on the radio obeyed them).
If an AERIXIS land plugin that hands landing to PX4 (`land_plugin_platform`) is ever configured, its
PX4 LAND would read as a takeover; the rescue configs use `land_plugin_speed`.

## 2026-10-22 (evening): GCS swarm start page, toasts, emergency view, custom missions

Request: a drone start page that shows the whole swarm is ready to fly; camera only if the
mission uses it; flight data / errors / logs as small pop-ups; full-screen emergency with an RTL
or LAND choice (also a mission setting); own missions saved for later, built-ins never deletable;
a simple builder (sensors → mission type → features) that applies all nodes automatically with
full safety. GCS details: `custom_gcs/web/GUI-README.md` (custom_gcs is outside this repo).

- **Planner: feature spec → tree** (`as2_rescue_planner/features.py`): `feature_spec()` checks the
  operator's choices (camera, payload, type coverage/watch/track, detect, geotag, orbit, drop,
  resume, finish RTL/LAND, emergency RTL/LAND) with readable errors (e.g. detection needs the
  camera, drop needs payload + geotag); `custom_mission()` composes the tree inside the same
  mandatory shell as the built-ins (wait + SafetyGate VehicleHealthy, PeersFresh wait and
  SeparationSafe/PeersFresh gates in a fleet, RecoverThenFail with return/land recovery, end
  return/land + Disarm). Only node types the built-ins use; no new C++ nodes. Without camera (or
  detection off) no SetGimbalSearch / VisionHealthy / SetSearchArea / HumanDetected / ScanArea.
  `BUILTIN_FEATURES` expresses missions 1–5 as specs and `custom_mission` reproduces their XML
  byte for byte (tested), so "edit a copy" starts from the real mission. `describe()` /
  `requirements()` give the GCS its "what it will do" text and readiness needs.
  Watch + resume: no checkpoint; after a person ScanArea restarts and flies back to the planned
  station (Restation goal), the watch window restarts.
- `plan_mission`: `template: "custom"` + `features`; `emergency_action` (RTL|LAND) for every
  mission changes only the recovery branch (built-in XML unchanged with the default RTL);
  result carries `features` and `emergency_action`. `trees.py` gained `_end_steps/_recovery/...`
  helpers; built-in output verified identical before/after.
- Tests: `test_features.py` (10): built-in equivalence, rule errors, no camera nodes without
  camera, mandatory safety in every valid combination (single + fleet, ~200 trees), every tree
  passes `as2_mission_manager.validation.validate_xml` against the installed runner registry,
  KML plans pass `validate_route_bundle`, emergency setting only touches recovery.
  Full suite: 279 passed.
- **GCS** (custom_gcs): Readiness became **Swarm start** — fleet verdict "Whole swarm is ready to
  fly", per drone a pass/fail list by link (telemetry radio + per-system heartbeat, gateway &
  manager, AERIXIS nodes, FC: XRCE-DDS / PX4 arming checks / sensors / failsafe / battery, GNSS,
  mesh, camera only if the mission uses it, payload only if it drops, FC guardian) with the reason
  per failure; operator selects optional checks; old status cards kept. Data: existing polls only.
  Toasts for flight warnings/errors/AS2 log lines (bottom-right, setting in the bell), modal
  pop-ups removed for flight data. Full-screen **Emergency** view (abort, PX4 failsafe, link loss,
  guardian acting, AERIXIS core down, battery < 20 %, mission failure airborne, top-bar button): RTL / LAND
  per drone or all, mission setting pre-highlighted, ACK state shown; minimise → red banner.
  Server: `POST /api/emergency` → COMMAND_LONG 20/21 to component 1 per system ID, retried until
  COMMAND_ACK (accepted / refused / no-ack in `/api/state.emergency`); `GET /api/missions`,
  `POST /api/missions/{save,delete,describe}` (`mission_store.py`, `config/missions/*.json`,
  built-ins refused). Plan: built-ins (copy-to-edit), My missions (edit / copy / delete), 4-step
  builder, "On emergency RTL/LAND" per mission; Upload sends `emergency_action` in the stage.
  Guardian contract (agreed with the FC guardian session): RTL/LAND to component 1, ACK 0 accepted /
  2 on ground / 4 refused; guardian state from `links.guardian` or the gateway's `LINK|GUARDIAN|…`
  STATUSTEXT when AERIXIS core is silent (`/api/state.guardian`), `EMERGENCY|…` replies logged.

Verified: planner tests; GCS server on a separate port against fake gateways (RTL accepted,
refused MAV_RESULT 3, no-ack after 5 tries, one command per system, unknown drone refused);
packets classified RTL/LAND by the gateway's `protocol.emergency_action`; browser: start page
on live SITL data (flying drone → "not ready", reasons), builder save/plan via the API, emergency
view opened by the button and automatically from an injected abort + guardian state, banner.
SITL end-to-end (PX4 v1.14 + `DRONES=1 sitl_onboard.sh`, GCS restarted): Swarm start showed "Whole
swarm is ready to fly" (guardian ON_GROUND via poll and STATUSTEXT); a saved custom mission (no
camera, coverage, emergency LAND) planned from Plan (6 wp, 27 nodes), uploaded (guardian action
became LAND from the stage request) and started; at 8 m on a survey leg, Emergency → Land now:
ACK 0 after 8 ms on the first try, guardian EMERGENCY_ACTIVE (source gcs), PX4 LAND, disarmed
~15 s later; the view listed abort (external mode change), guardian active, mission failure.
Found and fixed: the "Before you leave" step guard (a modal) could cover the emergency view after
Start auto-navigates — the guard is now skipped during an emergency and for just-started drones.
Open (guardian session's follow-up): after an external mode change the BT keeps running, so
Reconcile is refused until the onboard stack restarts. Not yet: multi-drone emergency "all",
custom missions with camera features flown in SITL.

## 2026-10-22 (evening): FC guardian, GCS emergency path without AERIXIS, PX4 failsafe, fast Pi start

Request: if the AERIXIS brain stops, talk to the FC directly and RTL or LAND (emergency only); a GCS
emergency must work with AERIXIS core down; fast one-command start on the Pi 5. Guide:
`docs/RASPI5_ONBOARD_GUIDE.md` sections 2, 4a, 4b.

- **FC guardian** (`as2_rescue_runtime/fc_guardian.py`, decision core `guardian.py`, entry point
  `fc_guardian`): one per drone, rclpy + px4_msgs + the as2_msgs message only. Watches
  `platform/info` and `rescue/state`; armed + airborne + PX4 in OFFBOARD/HOLD + AERIXIS silent >
  `as2_timeout_s` (3 s) → `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` or `NAV_LAND` on `fmu/in/vehicle_command`,
  repeated every 1 s until PX4's nav_state changes, never again after that. Never on the ground,
  never against a pilot mode, not when PX4 already returns/lands, LAND if AERIXIS core was landing, RTL
  refused 5x → LAND. Clear states + reasons on `fc_guardian/state` (2 Hz) and in the log.
  PX4 v1.14 does not bridge `vehicle_command_ack` / `vehicle_land_detected`: confirmation is the
  nav_state change; airborne = armed + `takeoff_time` + height above the disarmed ground level
  (both topics are used when a PX4 build bridges them).
- **Emergency action RTL/LAND**: parameter `emergency_action` (settable in flight), latched topic
  `fc_guardian/set_action`, Pi setting `emergency_action` (GUI Safety tab / `AS2_EMERGENCY_ACTION`),
  per-mission `"emergency_action"` in the mission-manager stage request (manager hands it over
  at start). Mission-manager status poll gains `links.guardian`. GCS UI: separate session.
- **GCS emergency without AERIXIS** (`as2_mavlink_gateway`): COMMAND_LONG `NAV_RETURN_TO_LAUNCH`,
  `NAV_LAND` to the autopilot component (1) or 0, and `DO_SET_MODE` AUTO/RTL|LAND go to the guardian
  (`fc_guardian/request`), for one drone or all (`target_system` 0); ACK ACCEPTED/DENIED/FAILED;
  `LINK|GUARDIAN|...` and `EMERGENCY|...` STATUSTEXT. NAV_LAND to component 190 is unchanged (AERIXIS
  land); no other bridge logic changed. Before this, `DO_SET_MODE` RTL/LAND fell into the
  `drone.manual()` branch. Commands for another system ID are now ignored for these three commands
  only (other COMMAND_LONGs are still not filtered by target — see open items; fixed in the
  "external mode change" section above).
- **PX4 failsafe layer**: `rescue_ws/scripts/px4_failsafe_params.py` sets `COM_OBL_RC_ACT` (3 RTL /
  4 LAND) and `COM_OF_LOSS_T` via MAVLink (hardware, FC USB) or `px4-param` (SITL), and prints
  `NAV_DLL_ACT`, RC-loss and RTL parameters; `sitl_onboard.sh` runs it per instance
  (`EMERGENCY_ACTION`) and starts a guardian per drone.
- **AERIXIS PX4 platform** (`projects/as2_platform_pixhawk`, C++; on top of the earlier local changes in
  that file): no setpoints to PX4 while armed outside OFFBOARD after having been in it (heartbeat
  only). See the SITL finding below.
- **Fast Pi start**: `rescue_ws/scripts/as2_pi.sh start|install|up|down|status|logs|failsafe`;
  systemd units in `rescue_ws/systemd/` (`as2-fc-link`, `as2-fc-guardian` with `Type=notify` + 3 s
  watchdog + cores 0–1, `as2-onboard`, `as2-rescue.target`); `as2_onboard_service.py` gains the
  `guardian` link (restart 1–2 s, in the healthcheck), `run --only/--skip`, `exec NAME`, start gaps
  0.5–4 s instead of 2–14 s; `onboard_links.ros_run()` execs installed programs directly (0.13 s
  less per link on the laptop). Pi GUI starts the guardian with the FC link.

Verified (software only):
- 27 guardian tests (`test_fc_guardian.py`): AERIXIS core alive/dead, ground/airborne, RTL vs LAND, retries,
  RTL→LAND fallback, no double action, pilot takeover, PX4 failsafe first, GCS repeat/escalation,
  stand-down and re-arm, plus two ROS node tests. Gateway routing tests (`test_emergency.py`, 6).
  Manager option test. Full rescue_ws suite: 279 passed.
- Isolated end-to-end (ROS domain 77, fake PX4 topics, real gateway + guardian, pymavlink GCS):
  RTL on the ground → DENIED; RTL for another system ID → ignored; LAND to all → ACCEPTED and
  `NAV_LAND` sent; AERIXIS land via component 190 → guardian untouched; AERIXIS killed in the air → RTL
  2.04 s later (timeout 2 s), stopped after PX4 confirmed.
- Guardian under systemd (user unit, `Type=notify`, `WatchdogSec=3`): ready, 0 restarts.
- `systemd-analyze verify` of the units: clean.

PX4 SITL v1.14.3, one x500, mission human_geotag_resume at 12 m (`sitl_onboard.sh`):
- `px4_failsafe_params.py --sitl-instance 0`: COM_OBL_RC_ACT=3, COM_OF_LOSS_T=1.0 confirmed.
- Normal mission: guardian MONITORING through AERIXIS's own RTL and land, no action (correct).
- A: PX4 failsafe set to Hold, whole AERIXIS onboard launch SIGKILLed in the air → PX4 HOLD after 1 s →
  guardian `RTL (platform/info silent > 3 s)` once, ~3 s after the kill → PX4 RTL → landed, disarmed,
  guardian ON_GROUND.
- C: only `rescue_runtime` killed (platform alive, PX4 stays OFFBOARD — PX4's failsafe cannot see
  this) → guardian RTL once, 3 s after the kill → PX4 RTL.
- D: AERIXIS core alive, mission running, GCS COMMAND_LONG over the gateway: RTL for system 2 → ignored (no
  ACK); LAND to system 1 component 1 → ACK ACCEPTED in 0.04 s, PX4 LAND from 12.5 m, repeat → ACK,
  no second command.
- **Found in D, fixed in AERIXIS's PX4 platform** (`projects/as2_platform_pixhawk/src/pixhawk_platform.cpp`
  `sendCommand()`): after PX4 left OFFBOARD, the platform kept streaming trajectory setpoints
  (~58 Hz, here the still-running GoTo's velocity). PX4 v1.14's uXRCE bridge writes them into the
  same uORB topics PX4's own RTL/LAND/position modes use: the GCS LAND touched down but PX4's land
  detector never declared landed, so it stayed armed until AERIXIS core was stopped (then it disarmed at
  once). This affects every non-AS2 mode with AERIXIS core alive (RC pilot, GCS/guardian RTL/LAND, PX4
  failsafe). Now the platform sends only the offboard heartbeat (no setpoints) while armed and
  PX4 is not in OFFBOARD after having been in it, so a deliberate switch back still works.
  Re-test after the fix (rebuilt `as2_platform_pixhawk`): GCS LAND from 12.4 m with AERIXIS core alive →
  landed and **disarmed 14 s** after the command; a full normal mission still ends SUCCESS
  (takeoff, survey, AERIXIS RTL, land, disarm) with the guardian never acting.

Not verified yet: the MAVLink path of `px4_failsafe_params.py`; real Pi/FC; `ROS_LOCALHOST_ONLY=1`
with the XRCE agent (left opt-in). Follow-up: the rescue runtime keeps its operation (GoTo) after
PX4 leaves OFFBOARD and the BT fails, so a later switch back to OFFBOARD would resume the old leg;
it should abort on an external mode change. (Fixed: see "external mode change" above.)
Pre-existing, not from this change: `as2_mavlink_gateway/test/test_protocol.py` expects `go_to` but
the gateway emits `trajectory_generation`; the gateway does not filter other COMMAND_LONGs (ARM,
TAKEOFF…) by `target_system` on a shared radio (both fixed above); gateway start takes ~8 s (AERIXIS
behaviour waits).

## 2026-10-22 (later still): mission 2 watch keeps working next to a peer's orbit

Fixes the open point of SITL run 2 below (drone1's station lay inside drone0's orbit; flying
there tripped the separation gate and drone1 returned without watching). The separation gate is
unchanged and as strict as before; the watch now avoids walking into it. Python only, no BT/C++
or GCS<->AERIXIS bridge change.

- **Keep-out zones** (`swarm_agent`, `swarm/status.keep_out`): around each peer a circle as wide as
  the gate margin (minimum separation + both GNSS sigmas + peer speed x (frame age + one period))
  plus `keep_out_buffer_m` (3 m); around each peer's claimed person the orbit radius plus the same.
  Claims now carry the orbit radius (`r`, whole metres) in the mesh frame (still <= 2 LoRa frames).
  Horizontal circles: a point outside a zone is outside the gate's 3-D margin.
- **Planned stations to move to** (`as2_rescue_planner/watch.py`): for each drone up to 8 alternates
  along its own lanes (inset 5 m from the boundary, spread out) plus the other drones' planned
  stations. They travel in the tree (ScanArea `pose`, `segment_id`, `area` ports — existing generic
  ports) so the XML checksum binds them; `validate_route_bundle` checks the ScanArea station equals
  the staged waypoint and every listed station is inside the region.
- **Route approval**: new route operation `Restation` in `route_node`. Approved only if the goal is a
  station listed in the staged tree, at the station height, and the leg passes the normal region /
  entry-transit / corridor rules. Each re-station leg has its own command id.
- **Station leg**: the mission-2 station `SurveyWaypoint` (segment `*_station`) stops short (hover)
  when a peer's *orbit* zone covers the station or the way there, and ends SUCCESS with
  `STATION_BLOCKED: ...`; the segment is not recorded as completed. The existing separation check
  runs first, as before.
- **ScanArea**: if the watch is not at its planned station, or an orbit zone covers its station, it
  picks a station in this order: planned station; the planned station of a peer busy with a person
  (that share is unwatched); nearest clear own alternate. A station must clear every zone by 3 m more
  (no flip-flop); a leg may never enter a zone, and from inside one may only move outward. A
  preferred station whose leg a peer aircraft is crossing is waited for up to 20 s. Legs are flown
  at the mission speed (0.3..2 m/s) facing the way; the remaining leg is re-checked every tick and
  the aircraft stops short if a zone moves onto it. Separation lost while flying a leg fails the
  watch (same rule as SurveyWaypoint); a leg not yet started is simply not started. If no station
  is clear the drone holds where it is (outside the zones) and keeps sweeping; it resumes when a
  claim is released. Total time without a station is bounded (`WATCH_HOLD_S` = 240 s), then
  `WATCH_BLOCKED` → the normal recovery return and land.
- **Handled people**: after OrbitTarget/ReleaseTargetLock the person's position is kept
  (`rescue/state.handled`) and sent with the geotags, so neither this drone (new tracker label)
  nor a peer takes the same person again once the claim is gone.
- **Shown**: `rescue/state.watch` {state, note, station, planned}; runtime log line
  `Watch <state>: <note>`; GCS drone card "Now" row, e.g. `watch re-stationed (peer orbit nearby):
  station of drone0 (busy with a person)` or `no clear watch station: holding ...`.
- Tests: `as2_rescue_runtime/test/test_watch_restation.py` (23: zones, choice order, passing-peer
  wait, never entering a zone with the vehicle following the commands, stop short and resume,
  hold-and-resume, bounded wait, route refusal, separation loss, single drone unchanged, handled
  person), `test_plumbing.py` RestationRouteTests (3, ROS), planner tests (4). Suite 176 passed.
  Packages build.
- **2-drone SITL run 3** (mission 2, same KML and people A (6,-28), B (-5,16), `mission_cli.py run
  --template human_orbit --fleet drone0,drone1` on both managers): both **SUCCESS**. drone0 claimed
  B (with `r` = 10 over the compact mesh frame) and orbited it; drone1 saw A from its launch point
  and orbited A. Each person was orbited by exactly one drone; minimum spacing while both were
  airborne 20.4 m; separation never reported unsafe. Keep-out zones in swarm/status matched the
  peer (~16 m) and the peer's orbit (~26 m around the other person). drone0 published B as handled
  after the orbit. The re-station path itself was **not triggered** in this run (timing: drone1
  found A before its station leg); it is covered by the unit tests above, not yet by SITL.
- Trade-offs: a drone may watch a busy peer's share instead of its own (deliberate: mission 2 gives
  each drone one response, so the peer's share is otherwise unwatched for the rest of the mission).
  Only mission 2 station legs stop short; coverage legs of missions 3-5 still rely on the gate.
  Zones need the peers' frames: with stale mesh data the watch holds instead of moving.

## 2026-10-22 (later): person de-confliction across the fleet

Two drones must never respond to the same person. Added in the onboard runtime (Python only;
no BT/C++ or GCS<->AERIXIS bridge change):

- **Claim**: when a vision op binds a person (`VisionOps.bind_label`: TrackTarget, OrbitTarget,
  ApproachOverhead) the runtime claims them: label, start time, estimated plan-frame x,y (kept
  current while locked). Published in `rescue/state.claim`; `swarm_agent` puts it (and the last 4
  geotags as x,y) in its existing mesh peer frame (`swarm/peer_tx`, ~2 compressed LoRa frames, no
  new message kind, `mesh_bridge.py` unchanged). Released on ReleaseTargetLock, CompleteRescue,
  ReturnToLaunch/ManagedLand/RequestRecovery, mission change and ground reconcile.
- **Expiry**: a peer's claim lives while the peer keeps repeating it (5 Hz UDP, ~1 Hz LoRa) and expires
  3 s (`claim_ttl_s`) after the last frame carrying it; a newer frame without it releases it at once.
  Peers' claims and geotags come back in `swarm/status` (`peer_claims`, `peer_tags`, `person_claim`).
- **Exclusion**: `HumanDetected` is false for a person within 8 m (`claim_exclusion_radius_m`) of a
  peer's live claim, or within 6 m of a peer's geotag (missions 3/4). Housekeeping now also releases
  such a lock and looks away while watching/covering (ScanArea, SurveyWaypoint, ResumeCoverage) —
  before, an excluded person stayed locked during ScanArea and blocked new detections.
- **Tie-break**: two claims on the same person started within 2 s (`CLAIM_TIE_WINDOW_S`) → the lower
  drone ID keeps the person; otherwise the earlier claim keeps it. Both drones compare the same two
  start times, so exactly one keeps it even with clock skew. The other gives way: ApproachOverhead
  fails `YIELDED` (missions 3/4 resume coverage); TrackTarget/OrbitTarget move to where the response
  began, pushed out of the peer's orbit (radius + 10 m), sweep yaw and take the next unclaimed person
  (up to 120 s, then fail → normal return). Shown in `rescue/state.yielded`.
- **GCS**: drone card "Now" row shows `responding to L1 (claimed)` or `L1 left to drone0`
  (`custom_gcs/web/ui/js/dronecard.js`); observer `links.swarm` carries `person_claim`/`peer_claims`.
- Trade-offs: two different people closer than 8 m count as one (one is left for the next pass);
  a claim without a range estimate (gimbal near horizon) cannot be matched by peers; claims are
  in memory only (a runtime restart drops it). Tracker labels are per drone, so matching is by
  position, never by label.
- Tests: `as2_rescue_runtime/test/test_person_claims.py` (17: claim publish/expiry/release, frame size, centred-estimate gate, detection hold,
  peer-claim and peer-geotag exclusion, tie-break both ways, give-way motion and re-acquire, bounded
  search). Full suite 145 passed. Packages build.
- **Identity decisions need a centred person**: the range estimate is where the gimbal points; on the
  first ticks of a new selection the gimbal still points at the search attitude. Search-area,
  geotag and claim decisions (HumanDetected, housekeeping, the claim position) now use the estimate
  only when the image error is within 0.2 (`CENTRED`). A detection just accepted is not released by
  housekeeping for 5 s (`DETECTION_HOLD_S`) or until the response binds it.
- **2-drone SITL run 1** (mission 2, KML 100x80 m, people A (6,-28), B (-5,16)): drone0 locked B
  from the ground, claimed it at bind and orbited it; drone1 (which also had B locked) released B
  as soon as the claim arrived. No sample had both drones orbiting the same person (the 10-22
  double orbit did not recur). Defect found: 3 s later drone1's tracker re-selected B; the
  uncentred first-tick estimate put B > 8 m from the claim → HumanDetected → housekeeping (no op
  active between PauseCoverage and OrbitTarget) released B with the correct estimate →
  `OrbitTarget failed: No locked person` → drone1 returned without orbiting A. Fixed by the two
  points above; 17 claim tests, full suite 145 passed.
- **2-drone SITL run 2** (with the fix, started with `mission_cli.py run --template human_orbit
  --fleet drone0,drone1` on both managers): drone0 claimed B and orbited it, SUCCESS. drone1 released
  B as soon as the claim arrived; when its tracker re-selected B (new label) 3 s later it was **not**
  taken as a detection (run-1 defect gone). No sample with both drones on the same person.
  drone1 then failed its watch with `Peer freshness/separation unavailable`: its planned station
  (0,20) is 5 m from B, inside drone0's 10 m orbit; closing to 14 m tripped the existing
  separation gate → recovery → return and land (safe, but drone1 never watched; person A was not
  orbited). **Open**: when a peer's claimed orbit overlaps this drone's station, re-station (or hold
  and wait for the gate) instead of failing the mission; the planner does not know where people are. → addressed in the section above.

## 2026-10-22 (later): Pi 5 onboard Docker image, mesh requirement check, maps

- **Pi 5 onboard image** (`docker/raspberrypi5/`): `Dockerfile.onboard` builds only the 13 AERIXIS packages an
  aircraft runs + the rescue overlay + Micro XRCE-DDS agent 2.4.2; the A8 tracker gets its own Python
  environment (`A8_TRACKER_PYTHON`) so ROS keeps its numpy. `compose.yaml` (host network, serial devices
  only, volume `/data`, restart unless-stopped), `onboard.env.example`, `setup_pi5.sh` (UART, permissions,
  build, start), README. The container runs `as2_onboard_service.py`: the same link commands as the Pi GUI
  (now shared in `rescue_ws/scripts/onboard_links.py`), supervised with back-off, waits for unplugged
  radios, prints the GCS pairing token, healthcheck via `status`.
- **Mesh requirement**: `rescue_ws/scripts/mesh_check.py budget|live`. Swarm gates need peers <= 1 s old
  (~2 updates/s per aircraft). A responding drone's peer frame needed 2 radio frames (only 1 update/s at
  the serial bridge's 2 frames/s) — fixed with a compact wire form (`swarm.compact_peer/expand_peer`,
  backwards compatible) and rounded values: now 1 frame (~190-255 B on air). Hardware rule: LoRa air rate
  >= 19.2 kbps for 2 aircraft, >= 62.5 kbps for 3-5 (2.4 / 9.6 kbps defaults are too slow), or IP mesh.
  SITL UDP mesh live check: PASS (peers fresh 100 %, max age 0.24 s).
- **Image verified (amd64 build of the same Dockerfile, 3.7 GB)**: container as drone0 against PX4 SITL;
  GCS added it with the container's own pairing token over MAVLink (CONNECTED, tree uploaded and validated
  onboard, started); single-drone mission 2 (watch station, orbit, RTL, land) SUCCESS. Rebuild after a
  rescue-only change: ~8 min (AERIXIS layers cached). Not yet built on an actual Pi 5 (arm64): pip wheels for
  ncnn / opencv-contrib-headless aarch64 are expected but unverified.
- Bugs found by that test and fixed: `ros2 launch` rejects empty `fmu_prefix:=""`/`fc_mavlink_url:=""`
  (AERIXIS never started with the Pi default prefix); SITL launcher watchdog killed PX4 when Gazebo started
  slower than 3 s; planning included offline aircraft, so fleet trees waited forever for a mesh peer (Plan now
  uses only aircraft that answer). The laptop suspended twice (10:52, 11:53), which dropped internet and SITL.
- **Maps**: Street / Satellite / Hybrid (Esri imagery + labels), live GCS location (browser geolocation),
  maps open on the operator's last location (India) instead of PX4's default; drone markers/trails in their
  card colour. SITL can fly anywhere: `SITL_HOME="lat,lon,alt"` for `3_launch_simulation.sh` and
  `sitl_onboard.sh` (PX4 `SIM_GZ_HOME_*`). SITL gateways moved to UDP 14570+.

## 2026-10-22 update: GCS redesign, low-bandwidth radio, FC health, multi-drone SITL

Verified in SITL with **two drones** (PX4 x2 + Gazebo, AERIXIS x2, mesh, radio hub), driven entirely from the GCS:
mission 2 (orbit) planned from a 100x80 m KML, uploaded and started for both; each detected a person inside the
area, orbited at ~8 m for 1 lap, returned and landed — both SUCCESS. 128 rescue tests pass (the ROS bridge test
can time out while SITL loads the laptop).

- **Radio**: idle link load for 2 drones 49 KB/s → 1.7 KB/s. Delta status polls (`status_delta.py`, merged by the
  GCS), adaptive poll rate, gateway position telemetry fixed (an exception made it send at 20 Hz instead of 5) and
  set to 2 Hz, ROS log relay scoped to the drone's own nodes and rate-limited, repeated log lines folded,
  compact planner XML.
- **FC health** (`fc_health` node, `fc/health`): GNSS fix/satellites/HDOP/jamming, PX4 arming/mode/preflight,
  failsafe flags; optional read-only MAVLink tap (`fc_mavlink_url`, Pi: FC USB `/dev/ttyACM0`) for unhealthy
  sensors (MAG/GPS/ACCEL…) and PX4 STATUSTEXT warnings (e.g. compass). Shown on every drone card.
- **Mission 2 (orbit)**: no survey lanes — one watch station per drone (`ScanArea`: hold + yaw sweep); every vision
  mission now loads the KML polygon (`SetSearchArea`) so people outside the area are ignored.
- **Estimator origin**: a provisional origin from GNSS until a plan is staged (stops TF error floods), then the
  plan origin is applied on the ground (raw_odometry accepts an origin change; route_node only does it disarmed).
  A new KML no longer needs an AERIXIS core restart.
- **Runtime fixes**: command freshness window 1 s → 3 s (late valid commands failed missions under load);
  take-off waits for a fresh position before recording home (crashed with stale evidence); return-to-launch keeps
  yaw (path-facing turn while accelerating out of an orbit tripped the stability monitor); failure reasons are
  logged by the runtime and shown in the GCS.
- **Multi-drone SITL**: `./3_launch_simulation.sh N` (20 m apart) + `DRONES=N sitl_onboard.sh`; GCS "Add simulated
  aircraft" button.

**Open / next**: (1) ~~person de-confliction between drones~~ — implemented, see the section above. (2) Real
hardware validation. (3) Pi 5 Docker packaging.

## 2026-10-21 update: A8 tracker, five missions, Pi 5 GUI, SITL-verified

### What was added
| Area | What exists |
|---|---|
| A8 tracker in AERIXIS | `rescue_ws/src/as2_a8_tracker`: your multilock1 tracker vendored unchanged except a small `as2_link.py` hook (gimbal attitude read-back, release/suppress/set-attitude commands on UDP 9091). ROS bridge publishes `vision/target`, `gimbal/state`, `perception/tracks`, `camera/health`; `vision/command` goes back to the tracker. Modes: `preview`, `track`, `sim` (SITL camera/gimbal simulator). Original 54 tracker tests still pass. |
| New BT actions | `TrackTarget`, `ApproachOverhead`, `OrbitTarget`, `GeotagOverhead`, `DropPayload`, `ReleaseTargetLock`, `SetGimbalSearch`, `ReturnToLaunch` |
| New BT conditions | `HumanDetected`, `TargetLocked`, `OverheadTarget`, `PayloadAvailable`, `VisionHealthy`, `FlightAvailable` (113 node types registered) |
| Five missions (planner templates, auto-generated per drone from the KML) | 1 `human_track_rtl` · 2 `human_orbit` · 3 `human_geotag_resume` (default) · 4 `human_geotag_drop_resume` · 5 `swarm_coverage` |
| GCS | Mission dropdown (5 rescue missions first, default remembered), trees generated automatically when a KML is loaded or the mission type changes, first tree previewed, "Assign all drones" |
| Pi 5 GUI | `rescue_ws/scripts/as2_pi_launcher.py` (run `run_pi_launcher.sh`): GCS telemetry (serial port list + baud or UDP), mesh (LoRa serial or IP mesh/base station with fleet passphrase), Micro XRCE-DDS (UART/UDP), Start AERIXIS (core + rescue + logger + mission manager), camera (off/detect/track/sim), safety profile tab, live status tab, pairing token |
| Onboard bring-up | `ros2 launch as2_rescue_runtime onboard.launch.py` (GNSS estimator, PID controller with tuned gains, behaviours, rescue nodes, track logger) |
| GPS frame | Route node writes the staged KML plan origin into the AERIXIS estimator (`set_origin`) on the ground and verifies it on every leg — no manual origin entry |
| Logging | Track log JSONL: operations, gimbal aim, guidance samples (gimbal yaw/pitch + aircraft position/speed while steering to nadir), geotags, payload outcomes |
| SITL test | `rescue_ws/scripts/sitl_mission_test.sh <mission>` — PX4 SITL + full stack + simulated A8 + mission manager + mission |

### Verified in PX4 SITL (v1.14.3, headless Gazebo, sample_field_mission.kml)
| Mission | Result |
|---|---|
| 1 Track & RTL | SUCCESS: search → lock → hold & face person → person hidden → return to launch → land |
| 2 Orbit | SUCCESS: one lap at 8.0–8.2 m radius, 355° swept, gimbal locked (pitch −48..−56°) → RTL → land |
| 3 Geotag & resume | SUCCESS: two people geotagged ~0.3 m from truth, lock released, coverage resumed, full survey, RTL, land |
| 4 Geotag + drop | SUCCESS: payload command sent once over person 1 at nadir (`COMMAND_SENT_UNVERIFIED`), person 2 geotagged only |
| 5 Swarm coverage | SUCCESS (single aircraft; two-drone separation correctly rejected for the 26×28 m sample area) |

### Defects found by SITL and fixed (all would have broken real flights)
1. Track logger could never start (`self.handle` clashes with rclpy; `_aim()` missing argument).
2. PX4 v1.14 does not bridge `battery_status` over uXRCE-DDS → vehicle never "healthy". Now falls back to PX4 `failsafe_flags` battery warning.
3. AERIXIS platform pauses all topics ~0.5 s while arming → readiness dropped and takeoff failed. Vehicle evidence now tolerates 1 s; takeoff/land wait 3 s.
4. PID controller started with all-zero gains (packaged default) → no mode could be set. Onboard launch now ships the px4_vision tuning.
5. Swarm agent read `self_localization/odom`, which AERIXIS never publishes → `PeersFresh` always false. Now uses pose/twist + GNSS sigma.
6. Detection could not interrupt a running waypoint (pause rejected) → now preempts coverage.
7. Yaw turns and take-off acceleration were treated as instability. Now roll/pitch rate only, 30° tilt, and instability must persist 0.5 s.
8. Resume point taken from a measured corner position 3 cm outside the area → resume blocked forever. Now projected onto the planned lane.
9. Lane ends on the boundary + normal overshoot revoked route approval mid-leg → approved legs now only require a 3 m corridor.
10. Launch points outside the KML area could never enter it → one straight entry leg allowed if it crosses no hole/no-fly zone (≤500 m).
11. First DDS service response can be lost by a fresh BT runner → Offboard/Arm retried.
12. Geotagged person was immediately auto-released before the drop; gimbal stayed staring at an excluded person; release raced the payload authority. All fixed.
13. Mission-manager reconcile did not clear the rescue runtime's latched abort → now clears both on the ground.

### 2026-10-21 (later): new GCS interface and fixes found while flying it
- New 6-step GCS interface (Connect → Readiness → Plan → Upload & start → Live → Logs), light/dark, see `custom_gcs/web/GUI-README.md`. Classic UI kept as `classic.html`.
- Mission 3 started from the new GUI in SITL: SUCCESS, both people geotagged within ~0.5 m.
- Fixed: GCS MAVLink receive loop fell behind the gateway (status timeouts); route-bundle checksum broke after the browser's JSON round trip (15.0 → 15), so no KML plan from the GUI could be staged; AERIXIS GPS estimator anchored the frame without subtracting the PX4 local position (frame shifted by however far the drone moved since PX4 boot) and could crash on `set_origin` before a GPS fix; survey legs kept the take-off yaw so the camera looked backwards on return lanes (now path-facing); geotag exclusion zone reduced 15 → 6 m with a 3 m duplicate check.

### Still open — must not be represented as complete
1. **Real hardware:** no A8 gimbal/stream on the aircraft, no servo, no Pi thermal run. Bench-check gimbal yaw sign (`gimbal_yaw_sign` on the bridge), attitude feedback and `set_attitude` (CMD 0x0E) with the real A8 firmware.
2. **Ground model:** range to the person assumes flat ground at take-off height; overhead check itself does not.
3. **Overhead drop** stays physically risky: verify package mass, height envelope and release latency with inert payloads before setting `overhead_verified`.
4. **Multi-drone SITL** not yet run (only one PX4 instance tested); LoRa/IP mesh not measured on radios.
5. **GCS radio path** (laptop → SiK radio → gateway → onboard manager) not re-tested in this update; missions were staged over loopback.
6. **GCS UI** not checked in a browser in this update (JS syntax-checked and API tested).

## Earlier status (2026-10-17)

## Deployment topology this design assumes

- **Laptop:** the custom GCS only (`/home/prince/custom_gcs`). It plans missions, uploads
  them, and monitors. It has no AERIXIS installation and no ROS graph.
- **Onboard computer (Raspberry Pi 5 16 GB):** all of AERIXIS, the rescue runtime, perception,
  the mission manager, and the mesh radio bridge.
- **Laptop ↔ onboard:** long-range telemetry only (SiK / RFD900x serial, or a MAVLink
  relay), and optionally HTTPS over an SSH tunnel. Never a shared ROS domain.
- **Onboard ↔ onboard:** a separate drone-to-drone mesh (UART LoRa modem or IP mesh).
  It is not connected to the laptop GCS by design.
- **A8 mini:** Ethernet at `192.168.144.25`, RTSP `rtsp://192.168.144.25:8554/main.264`,
  Pi interface addressed `192.168.144.20/24`.
- **Payload:** servo on a flight-controller AUX output; **no load/latch sensor**, so the
  operator confirms loading before takeoff and every drop is reported as
  `COMMAND_SENT_UNVERIFIED`.
- **Delivery geometry:** you chose **directly over the person**. This is implemented as an
  explicit per-mission opt-in with its own gates (see below), not as a default.

## Built and passing software tests

| Area | What exists | Evidence |
|---|---|---|
| KML planning | `as2_rescue_planner`: ring validation, self-intersection rejection, holes, exclusions, altitude semantics, boustrophedon coverage, endurance/reserve partition, per-drone bundles with checksums, per-drone BT XML | 45 tests pass |
| Planner CLI | `plan_fleet` with `--output-dir`, template, altitude, spacing, heading, corridor | End-to-end run produced 2 drone bundles + XML + manifest |
| GCS planning | `/api/plan/kml`, `/api/plan/status`, `planner_bridge.py`, fleet planner panel in Mission Designer | Plans the supplied sample KML into 2 matching XML/bundle pairs |
| Route approval | `rescue/route` publisher bound to the exact requested command, command/pose match, region containment, origin match | Part of 17 plumbing tests |
| Surface estimate | `rescue/surface` publisher; fixed / operator / range sources; unverified never yields a usable elevation | Part of 17 plumbing tests |
| Camera transform | measured world→optical TF at image timestamp; nadir = optical +Z on world −Z; calibration gate | Rotation verified orthonormal and right-handed at 48 angle combinations |
| Delivery zone | `delivery/zone` from calibrated fix + explicit policy (beside or overhead) | Node builds, policy enforced by safety model |
| Overhead drop | `safety.py` overhead mode: verified envelope, mission authorization, nadir, centring, AGL bounds, horizontal alignment | Runtime core tests pass |
| Operator payload | preflight load confirmation bound to the mission; drop reported as unverified | Runtime core tests pass |
| Rescue runtime | 99 registered BT node types (was 96); `ReportPayloadOutcome`, `CenteredOnTarget`, `PayloadOperatorConfirmed` added | Catalog export confirms 99 |
| Pi launcher | `rescue_ws/scripts/as2_pi_launcher.py` — GCS link, mesh link, FC link, camera mode | Written; not yet exercised on a Pi |
| Track log | `track_logger.py` — JSONL flight log of gimbal aim, track acquire/loss, geotag, aircraft pose; size-capped rotation | Written; not yet run against live ROS |
| Overlay build | `as2_rescue_planner`, `as2_rescue_runtime`, `as2_mission_manager`, `as2_rescue_perception` all build | `colcon build` finished 4 packages |

**Total: 96 tests passing** across planner (45), runtime core (21), plumbing (17), radio RPC (1),
driver (2), mission manager (9), observer (1).

## Defects found and fixed during the audit

1. **Circular package dependency** — planner imported runtime templates while runtime imported
   the planner. Would have broken `colcon` ordering. Trees moved to `as2_rescue_planner.trees`;
   runtime `templates.py` is now a re-export. Planner depends on nothing.
2. **Planner rejected the supplied sample KML** — it refused any document containing `Point`
   waypoints. Points are now collected as reference markers; `LineString`/`Model` still rejected.
3. **Offset drop mode broken** — `person_clearance_m` measured from the person, not from the
   aircraft, so the 5 m offset actually released over the recipient. Now offset from the fix.
4. **Weighted partition silently dropped aircraft** — a band that rounded to zero lanes vanished.
   Now reported in `unassigned` with a reason.
5. **Boundary rounding rejected valid surveys** — millimetre-rounded waypoints fell outside the
   exact region. Containment now allows a 1 mm tolerance.
6. **GCS planner import used a hardcoded path** — would fail as soon as AERIXIS left the laptop.
   Now searched via `AS2_RESCUE_SRC`, an installed package, or conventional locations.
7. **Catalog builder hardcoded the GCS path** and crashed the whole export. Now configurable
   via `AS2_GCS_WEB`, skips with a warning if absent.
8. **CLI default speed (5 m/s) exceeded its own 2 m/s limit**, and a missing altitude raised
   `TypeError`. Both fixed with clear errors.

## Still open — must not be represented as complete

1. **Hardware validation.** No A8 stream, gimbal SDK, servo or Pi thermal test has been run.
   Gimbal axis signs, mount offset, attitude reference (body vs world) and CameraInfo all still
   need measuring on the aircraft before `calibration_verified` may be set.
2. **No SITL or flight test.** Pause/resume, target loss, hold handover and overhead alignment
   are proven only against synthetic evidence.
3. **`path_clear` defaults to false.** The planner validates geometry, not obstacles. Until an
   operator asserts a checked corridor, coverage legs are refused. That is intentional.
4. **Laptop-local tree execution still exists** in `custom_gcs/server.py`
   (`run_behavior_tree` / `_prepare_behavior_tree`). It contradicts the radio-only topology and
   should be removed or gated before deployment.
5. **JavaScript not syntax-checked** — `node` is not installed in this environment. All eight
   GCS JS files are unverified by a parser here.
6. **Launcher untested.** Written for the Pi; cannot be exercised without the hardware.
7. **Overhead drop remains physically risky.** A released package can strike the person below.
   Package mass, altitude envelope and release latency must be established with inert payloads.
8. **Mesh not measured.** LoRa airtime, latency, packet loss and fleet size are unvalidated; the
   bridge is a transport, not a routing mesh.
9. **Operator payload mode has no physical proof of delivery.** It is labelled as such everywhere.

## How to run what exists

```bash
# Laptop: plan a fleet from KML (no ROS needed)
cd /home/prince/aerostack2
./plan_fleet.sh sample_field_mission.kml --fleet fleet.json \
  --mission-id demo-1 --altitude-m 15 --output-dir ./bundle_out

# Onboard: build and start the rescue services
cd /home/prince/aerostack2/rescue_ws
PYTHONNOUSERSITE=1 colcon build --base-paths src
source install/setup.bash
ros2 launch as2_rescue_runtime rescue.launch.py drone_id:=drone0

# Onboard: the companion launcher (GCS link, mesh link, FC link, camera)
cd /home/prince/aerostack2/rescue_ws
./scripts/run_pi_launcher.sh
```

## Test commands

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTHONPATH=$PWD/rescue_ws/src/as2_rescue_planner:$PWD/rescue_ws/src/as2_rescue_runtime:$PWD/rescue_ws/src/as2_mission_manager:$PYTHONPATH
python3 -m pytest rescue_ws/src/as2_rescue_planner/test rescue_ws/src/as2_rescue_runtime/test rescue_ws/src/as2_mission_manager/test -q
```
