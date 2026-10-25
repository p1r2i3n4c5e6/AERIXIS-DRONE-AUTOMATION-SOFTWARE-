# AERIXIS — Raspberry Pi 5 onboard guide

Everything AERIXIS runs on the Pi. The laptop runs only the custom GCS and talks to the Pi over
the long-range telemetry radio. The mesh (LoRa or base-station IP network) links drones to
each other only.

## 1. Wiring

| Link | Pi side | Other side |
|---|---|---|
| Flight controller | UART `/dev/ttyAMA0` (GPIO14/15, `dtparam=uart0=on`) | PX4 TELEM2 |
| GCS telemetry radio | USB `/dev/ttyUSB0` (SiK/RFD900x) | radio pair on the laptop |
| Mesh | USB `/dev/ttyUSB1` (LoRa modem) **or** Ethernet/Wi-Fi to the mesh radio | other drones |
| A8 mini | Ethernet, Pi address `192.168.144.20/24` | camera `192.168.144.25` |
| Payload servo | — | flight controller AUX pin |

## 2. PX4 parameters
- `UXRCE_DDS_CFG = TELEM2`, `SER_TEL2_BAUD = 921600` (same baud in the GUI).
- `MAV_SYS_ID` = the drone's system ID (GUI "MAVLink system ID", GCS fleet entry).
- Servo: in *Actuators*, map the AUX pin to **Offboard Actuator Set N**; enter N in the GUI
  Safety tab (actuator set), and the release value. Test with props removed.
- **Offboard-loss failsafe** (the last layer, when the whole Pi or the XRCE link is dead):
  `COM_OBL_RC_ACT` = **3** (Return) or **4** (Land) — the same choice as the guardian's emergency
  action (section 4b) — and `COM_OF_LOSS_T` = 1.0 s. `NAV_DLL_ACT` must stay **0**: PX4 never sees
  the GCS directly (the radio goes to the Pi), so a data-link-loss action would fire after arming.
  Set them in QGC, or with the FC USB cable on the Pi:
  `rescue_ws/scripts/as2_pi.sh failsafe RTL` (prints all related parameters afterwards); in the
  Docker image: `docker compose exec onboard /entrypoint.sh failsafe RTL /dev/ttyACM0`.
  These are the PX4 v1.14 names (`src/modules/commander/commander_params.c`); older PX4 used
  `COM_OBL_ACT`.
- **Own return height per drone** (the same command sets it): `RTL_RETURN_ALT` = 16 m + one fleet
  separation x (MAV_SYS_ID - 1), the separation being the onboard setting *Min separation*:
  16 / 26 / 36 m with 10 m (ordinary GNSS), 16 / 19 / 22 m with 3 m (RTK on every drone, experts
  only), and `RTL_CONE_ANG` = 0 (always climb the full height), so PX4's own RTL-all never puts two
  drones at one height. AERIXIS core's own return uses the same heights with AI avoidance off; with AI
  on it flies home at mission height through the avoidance layer. Give every drone a different
  MAV_SYS_ID, set the same *Min separation* on every drone (re-run the command after changing it), and
  raise the base if anything around the area is taller than 16 m: `px4_failsafe_params.py --mavlink
  /dev/ttyACM0 --return-base 30 --return-step <separation>` plus `return_altitude_base_m: 30.0` in the
  aircraft profile.

## 3. Software (Ubuntu 22.04 + ROS 2 Humble on the Pi, or the `humble-pi5` Docker image)
```bash
cd ~/aerostack2 && colcon build --executor sequential        # AS2 (MAKEFLAGS=-j2 on the Pi)
cd ~/aerostack2/rescue_ws && colcon build --base-paths src    # rescue overlay incl. as2_a8_tracker
python3 -m pip install ncnn pymavlink pyserial psutil opencv-contrib-python-headless
```
The tracker needs OpenCV with `TrackerNano` (opencv-contrib ≥ 4.8). Keep the Pi cooled: the
tracker stops itself at 78 °C.

## 4. Start
```bash
~/aerostack2/rescue_ws/scripts/run_pi_launcher.sh
```
(or copy `rescue_ws/deploy/as2-launcher.desktop` to `~/.local/share/applications/`).

1. Set **Drone ID** (`drone0`, `drone1` …) and **MAVLink system ID**.
2. **Flight controller**: UART `/dev/ttyAMA0`, 921600 → Connect.
3. **Start AERIXIS** (needs the flight-controller link). Click **Pairing token** and enter the token
   and system ID for this drone in the GCS Fleet page.
4. **GCS telemetry**: pick the radio port and baud (57600 for SiK) → Connect.
5. **Mesh**: mode *Base station + LoRa fallback* (default): the base station's subnet broadcast
   (e.g. `192.168.1.255`) or the other drones' IPs, plus the LoRa port/baud; list every drone ID,
   same passphrase on all drones → Connect. The base station carries every frame (up to 0.5–1 km),
   the LoRa modem runs alongside at its own rate, so out of base-station range the drones keep
   hearing each other. Without a passphrase the drone flies alone: no separation gate peers and no
   AI avoidance. The panel's Mesh card shows both links, the separation and the AI avoidance state
   (on/off is switched from the GCS menu "AI"; the drone runs the trained brain with NumPy, no PyTorch).
6. **Camera**: *Detect only* for bench tests, *Detect + gimbal track* for missions. Live video:
   `http://<pi-ip>:8080`.
7. **Safety profile** tab: tick *Allow flight* and *Flight area checked* only when ready to fly;
   payload switches only after inert-payload tests. Settings are remembered.

*Connect all* does steps 2–6 in order. *Stop all* stops everything. The FC guardian starts
with the flight-controller link (its state is shown on that row); the Safety profile tab has
*Emergency action if AERIXIS core stops in flight* (RTL or LAND).

## 4a. Fast headless start (no screen, starts at boot)
One command, same links and settings as the GUI (`~/.config/as2/pi_launcher.json`):
```bash
rescue_ws/scripts/as2_pi.sh start        # in this terminal, Ctrl+C stops
rescue_ws/scripts/as2_pi.sh install      # once: systemd units, start at every boot (sudo)
rescue_ws/scripts/as2_pi.sh up | down | status | logs [fc|guardian|onboard]
```
Order: XRCE agent → FC guardian (ready in well under a second) → AERIXIS onboard + mission manager →
GCS gateway → mesh → camera (if not *Off*). Links start 0.5–4 s apart (they used to wait up to
14 s), and links run their installed program directly instead of through `ros2 run`.
Three systemd units, so AERIXIS can crash or be restarted without touching the emergency layer:

| Unit | Runs | Restart |
|---|---|---|
| `as2-fc-link` | Micro XRCE-DDS agent | always, 1 s |
| `as2-fc-guardian` | FC guardian, cores 0–1, `Type=notify` + 3 s watchdog | always, 1 s |
| `as2-onboard` | AERIXIS, mission manager, gateway, mesh, camera (`as2_onboard_service.py run --skip fc,guardian`) | on failure, 2 s |

`as2-rescue.target` groups them. Override settings in `/etc/default/as2-rescue`
(`AS2_EMERGENCY_ACTION=LAND`, `AS2_CAMERA_MODE=off`, …). The Docker image
(`docker/raspberrypi5`) runs the same service, guardian included, in one container.
Still slow: the GCS gateway waits about 8 s for AERIXIS's behaviour servers at start (AERIXIS Python API).

## 4b. Emergency layers when AERIXIS core stops
| Layer | When | What happens |
|---|---|---|
| AERIXIS mission safety | AERIXIS running | battery / position / separation aborts → AERIXIS return and land |
| **FC guardian** (`fc_guardian`) | AERIXIS core stopped: `platform/info` or `rescue/state` silent > 3 s (`as2_timeout_s`), aircraft armed **and** airborne, PX4 in OFFBOARD or HOLD | commands PX4 directly over the XRCE link: **RTL** or **LAND** (`VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` / `NAV_LAND`), repeated every 1 s until PX4 shows the new mode, then never again |
| **GCS emergency** | operator presses RTL/LAND in the GCS, AERIXIS core alive or not | radio → MAVLink gateway on the Pi → guardian → PX4 |
| **PX4 failsafe** | Pi or XRCE agent dead (no offboard setpoints for `COM_OF_LOSS_T`) | PX4 runs `COM_OBL_RC_ACT` (set it to the same RTL/LAND) |

Once PX4 has left OFFBOARD (for whatever reason), AERIXIS's PX4 platform stops sending setpoints and
keeps only the offboard heartbeat, so it cannot disturb PX4's RTL/LAND or a pilot mode (PX4 v1.14
would otherwise mix AERIXIS's setpoints into its own modes; found and fixed in SITL).

Guardian rules: it never acts on the ground or while disarmed; it never overrides a pilot mode
(MANUAL/ALTCTL/POSCTL/ACRO/STAB) — a pilot who takes over during an emergency makes it stand down;
it does nothing if PX4 is already returning/landing by itself (`PX4_FAILSAFE`); if AERIXIS core was landing
when it stopped, it uses LAND instead of RTL (RTL would climb first); if PX4 refuses RTL (no home)
5 times, it switches to LAND. It only supervises AERIXIS after it has seen AERIXIS once (so a guardian
restarted mid-flight while AERIXIS core is already dead waits — PX4's failsafe still covers that case).

**Choosing RTL or LAND** (default RTL; flood areas: LAND only where landing is safe):
- per aircraft: GUI Safety tab, `AS2_EMERGENCY_ACTION` in `/etc/default/as2-rescue`, or in
  flight `ros2 param set /drone0/fc_guardian emergency_action LAND`;
- per mission: the mission upload (`/api/mission/stage`) accepts `"emergency_action": "RTL"|"LAND"`;
  the mission manager hands it to the guardian when the mission starts (latched topic
  `fc_guardian/set_action`, so a restarted guardian gets it back). Without it the guardian keeps
  its parameter. (GCS button: separate GCS work.)

**What the GCS sees**: STATUSTEXT `LINK|GUARDIAN|UP|<state> action=<RTL|LAND> [emergency=…]` on
change and every 10 s (`DOWN` when the guardian is not running), `EMERGENCY|<action>|<result>`
after a GCS emergency command, the guardian's warnings as `ROSLOG` lines, and `links.guardian`
in the mission-manager status poll (`state, action, detail, as2_alive, airborne, nav, emergency`). Full state on the Pi: `ros2 topic echo /drone0/fc_guardian/state`.
States: `NO_FC`, `ON_GROUND`, `WAITING_FOR_AS2`, `MONITORING`, `PILOT_CONTROL`, `OTHER_MODE`,
`PX4_FAILSAFE`, `COMMANDING`, `EMERGENCY_ACTIVE`, `STOOD_DOWN`.

**GCS command format** (for the GCS session): MAVLink `COMMAND_LONG`
- `MAV_CMD_NAV_RETURN_TO_LAUNCH` (20) to any component → emergency RTL;
- `MAV_CMD_NAV_LAND` (21) to component **1** (autopilot) or **0** → emergency LAND
  (to component 190 it stays the normal AERIXIS land);
- `MAV_CMD_DO_SET_MODE` custom main mode 4 with sub mode 5 (RTL) / 6 (LAND) → same;
- `target_system` = the drone's system ID, or **0 for all drones** on the radio.
The gateway answers `COMMAND_ACK`: ACCEPTED (sent to guardian), DENIED (not airborne) or FAILED
(guardian not running / no PX4 link). Like ARM today, these commands are not token-authenticated:
protect the radio link itself (SiK net ID / AES key).

## 5. Fly from the GCS
Mission Designer → *Plan a fleet mission from KML* → choose the mission (default: Mission 3) →
load the KML: one tree per drone is generated automatically → review → *Assign all drones* →
upload and start from the Fleet page.

| Mission | What the drone does |
|---|---|
| 1 Track & target | Search; on a person: hold, face them, keep the gimbal locked; person lost → return to launch, land |
| 2 Orbit | Search; circle the person (radius/laps) with the gimbal locked → return to launch |
| 3 Geotag & resume | For every person: pause, fly until the gimbal looks straight down, geotag, release, resume search |
| 4 Geotag + drop | Mission 3 + one payload release over the first person at nadir |
| 5 Swarm coverage | Each drone flies its share of the area with mesh/separation gates |

All missions end with return to launch and landing, and an abort (low battery, lost
position, persistent instability) also returns to launch and lands.

## 5a. Onboard panel and Camera & AI models
Open `http://<pi-ip>:8090` and enter the panel PIN (top right). Pages (sidebar): **Home** status,
**Connect** radio / FC / mesh, **Calibrate** pre-flight checklist, FC sensors and gimbal check,
**Camera & AI**, **Drone** identity, pairing token, safety switches and PIN change, **Log**.
On **Camera & AI → AI models**:

1. **Onboard computer** — pick the board (Raspberry Pi 5, Pi 5 + AI HAT+ 13/26 TOPS, Vicharak
   Axon, Jetson Nano, Jetson Orin Nano). The card shows what it really found (CPU, cores/threads,
   clock, RAM, accelerator, runtimes, temperature) and warns if the choice does not match.
   *Drone speed while tracking*, *Min detection (Hz)* and *Min tracking (fps)* decide which models
   unlock. Defaults: 5 m/s, 5 Hz, 19 fps → detection needed = speed ÷ 0.5 m = 10 Hz, tracking ≥ 19
   fps (NanoTrack follows the target on every frame between AI scans). The lock stays on the target
   until it leaves the picture (released 1 s later) or nothing has seen it for 5 s; then the target
   nearest the centre is locked.
2. **CPU budget** — measured now: how much AS2 / flight software, the camera tracker and the rest
   use; the detector gets the cores that remain (threads shown).
3. **Models** — every shipped and uploaded model with type, input size, classes, detection rate,
   tracking rate and 🔒 reasons. Press **Benchmark** on the Pi to replace the estimate by a
   measurement. **Use…** picks the classes to detect and the classes the gimbal may lock on, makes
   the model active and restarts the camera. Filters: type, resolution, unlocked only.
4. **Upload** — a zip of the exported folder (NCNN: `yolo export model=best.pt format=ncnn
   imgsz=384,640 half=True`) or a single `.onnx`. Class names / input size only if the export has
   no `metadata.yaml`. **Delete** removes an uploaded model (not a shipped one, not the active one).

Models live in the `as2-data` volume (`/data/models`) and survive image rebuilds. Changes are
refused while armed or flying. Rescue missions (geotag, drop, peer claims) act only on **person**
locks; a lock on another class is followed by the gimbal and shown, nothing more.
For 20 m altitude use the `person-n640-960x544` model (960x540 video, people ~1.5x larger) on a
board that unlocks it, or lower *Min detection* on a bare Pi 5 after benchmarking it.

## 6. After a flight
Logs: `~/.local/share/as2-rescue-logs/track-*.jsonl` (gimbal aim, guidance towards nadir,
geotags, payload outcome) and `a8_tracker/` (tracker log). After a failed or aborted flight,
the drone must be reconciled on the ground (GCS reconcile, or
`rescue_ws/scripts/mission_cli.py reconcile --token-file ...`) before the next mission.

## 7. Test without hardware (laptop)
```bash
rescue_ws/scripts/sitl_mission_test.sh human_geotag_resume       # or any mission name
rescue_ws/scripts/sitl_mission_test.sh stop
```
`rescue_ws/scripts/sitl_onboard.sh` starts one guardian per drone and sets each PX4 instance's
`COM_OBL_RC_ACT`/`COM_OF_LOSS_T` to `EMERGENCY_ACTION` (default RTL) with
`px4_failsafe_params.py --sitl-instance N`.
