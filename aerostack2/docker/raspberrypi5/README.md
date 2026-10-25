# AERIXIS onboard stack on a Raspberry Pi 5 (Docker)

One container runs everything an aircraft needs, supervised and restarted automatically:

| Link | What | Default device |
|---|---|---|
| fc | Micro XRCE-DDS agent: PX4 (TELEM2, uXRCE-DDS) ↔ ROS 2 | `/dev/ttyAMA0` @ 921600 |
| as2 | AERIXIS core (PX4 platform, GNSS estimator, PID speed controller, behaviours) + rescue runtime + FC health tap | FC USB `/dev/ttyACM0` (read-only) |
| manager | onboard mission manager (validates trees from the GCS, runs them) | — |
| gcs | MAVLink gateway to the laptop GCS over the long-range telemetry radio | `/dev/ttyUSB0` @ 57600 |
| mesh | drone-to-drone mesh: base-station IP network as the main link + LoRa modem as an always-on fallback (or either alone); carries positions for the separation gate and the AI avoidance; only if a passphrase is set | base station + `/dev/ttyUSB1` |
| camera | A8 mini detection / gimbal tracking | `192.168.144.25` |
| web | **onboard control panel**, started last: set up and check every link from a browser | port `8090` |

The GCS (laptop) is never on the mesh; it talks to each aircraft only through the telemetry radio.

## Install (on the Pi)

1. 64-bit Raspberry Pi OS (Bookworm), Pi 5 with active cooling (the tracker heats the CPU).
2. Install Docker: `curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh`
3. Copy this repository to the Pi, then from its root: `bash docker/raspberrypi5/setup_pi5.sh`
   It enables the UART, frees it from the Linux console, sets serial permissions, creates
   `onboard.env` from the example and opens it for editing, builds the image (~40–60 min) and starts it.
4. Reboot if the script says so.

## Configure: the onboard control panel (http://\<pi-address\>:8090)

The Pi can run Ubuntu Server or Raspberry Pi OS Lite without a screen. Once every link is up, the
panel answers on port 8090. Open it from any laptop or phone on the drone's network (Wi-Fi or
Ethernet). On a Pi with a desktop, `launch/onboard.sh start --docker` opens it automatically as
the last step. In the panel:

- **GCS telemetry radio**: serial port + baud (or UDP: laptop IP + port).
- **Flight controller**: UART + baud (or UDP for SITL / Ethernet), USB health tap. Shows live PX4 mode, GNSS and faults.
- **A8 camera**: off / preview / track, camera IP. **Open camera dashboard** shows the A8 tracker page (port 8080).
- **Mesh**: modem or IP mesh, fleet names, passphrase (identical on every aircraft).
- **Identity**: drone ID, system ID (= PX4 `MAV_SYS_ID`), pairing token for the GCS.
- **Safety switches**: start OFF. Turn on *Allow flight* only after bench and tether tests.

Every change needs the panel PIN: `docker compose exec onboard /entrypoint.sh token` prints it
with the pairing token. Saving restarts only the affected link. Flight links are never restarted
while the aircraft is armed or flying a mission.

`onboard.env` only sets the identity (`AS2_DRONE_ID`, `AS2_SYSTEM_ID`) and ROS settings. Every other `AS2_*` line in
it is commented out. Uncommenting one fixes that setting from the file, and the panel then shows it read-only.
Apply file changes: `docker compose up -d --force-recreate`.

Use `/dev/serial/by-id/...` names for USB radios so the telemetry radio and the mesh modem can never swap.

PX4 side: `UXRCE_DDS_CFG = TELEM2`, `SER_TEL2_BAUD = 921600`, `MAV_SYS_ID` = the system ID above.

## Connect the GCS (laptop)

1. GCS → Connect → **Field radio**: the radio's serial port and the same baud as the aircraft radio.
2. On the Pi: `docker compose exec onboard /entrypoint.sh token` prints drone ID, system ID and the pairing token
   (also on the panel: *Identity & GCS pairing → Show pairing token*).
3. GCS → Connect → *Add aircraft manually*: drone ID, system ID, token. The card turns green within seconds.

## Operate

```bash
cd docker/raspberrypi5
docker compose logs -f onboard                 # every link, live
docker compose exec onboard /entrypoint.sh status   # link states (also the container healthcheck)
docker compose restart onboard                 # restart all links
docker compose exec onboard /entrypoint.sh shell    # ROS shell: ros2 topic list, ros2 node list ...
```

Optional window on the Pi desktop to set everything up with the familiar GUI (same settings file):
`xhost +local: && docker compose stop onboard && docker compose --profile gui run --rm gui`, then
`docker compose up -d` again. Don't run the GUI and the service at the same time.

Data that survives updates (volume `as2-data` → `/data`): settings, pairing token, staged missions, payload
journal, logs. Update the software: `git pull && docker compose build && docker compose up -d`.

## Mesh: what the radios must deliver

The swarm safety gates (PeersFresh, SeparationSafe) need every peer's position to be at most 1 s old, i.e.
about 2 updates/s from each aircraft. Each update is one radio frame of ~190–255 bytes. Check a fleet before
buying or configuring modems:

```bash
python3 rescue_ws/scripts/mesh_check.py budget --drones 3 --air-kbps 9.6 19.2 62.5
python3 rescue_ws/scripts/mesh_check.py live drone0 drone1 --seconds 60      # on running aircraft / SITL
```

Rule of thumb from the budget: LoRa air data rate ≥ 19.2 kbps for 2 aircraft, ≥ 62.5 kbps for 3–5 aircraft
(the common 2.4 / 9.6 kbps defaults are too slow), or an IP mesh radio. Air rate is the modem's radio setting,
not the UART baud.

## Bench test of the same image on a laptop (SITL)

The image also builds for amd64. With PX4 SITL running (`./3_launch_simulation.sh`) and the laptop GCS on
UDP 14555, run one simulated aircraft from the container:

```bash
docker run --rm -it --network host --ipc host -v as2-sitl-data:/data \
  -e AS2_DRONE_ID=drone0 -e AS2_SYSTEM_ID=1 -e AS2_FC_MODE=udp -e AS2_FMU_PREFIX=/drone0 \
  -e AS2_GCS_MODE=udp -e AS2_GCS_HOST=127.0.0.1 -e AS2_GCS_PORT=14551 -e AS2_GCS_BIND_PORT=14555 \
  -e AS2_CAMERA_MODE=sim -e AS2_ALLOW_FLIGHT=true -e AS2_PATH_CLEAR=true \
  as2-rescue-onboard:amd64-test
```
