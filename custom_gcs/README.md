# AERIXIS Mission Control (GCS)

The AERIXIS ground control station: a standalone web GCS for the AERIXIS flood-rescue drone swarm (real aircraft and PX4 SITL).

AERIXIS is built on the open-source AeroStack2 framework (BSD licence); internal package names (`as2_*`) keep their original names.

## Architecture

- **Web Interface**: Runs on `http://localhost:8765`
- **Telemetry Link**: Listens on UDP `0.0.0.0:14551` (receives MAVLink common telemetry from AERIXIS Gateway)
- **Command Link**: Transmits to AERIXIS Gateway on UDP `127.0.0.1:14550`

## How to Run

The updated glass interface is installed in `web/` and uses this same launcher and existing backend APIs. It includes the left workspace menu, mission builder, per-aircraft execution viewer, notifications and link activity displays. See [GUI integration details](web/GUI-README.md), including the current limits of GPS telemetry. The MAVLink bridge and backend logic are unchanged.

1. **Start the GCS**:
   ```bash
   cd /home/prince/custom_gcs
   bash start_gcs.sh
   ```

2. **Open in Browser**:
   `start_gcs.sh` opens [http://localhost:8765](http://localhost:8765) in a browser window once the server answers (the AERIXIS loading screen shows first). Set `GCS_OPEN_BROWSER=0` to start without opening a window.

3. **Status Indicators**:
   - `✦ AERIXIS Connected with GCS · Ready`: Shows **Connected** as soon as AERIXIS is started.
   - `AERIXIS ↔ PX4 · uXRCE-DDS`: Shows **Connected** when PX4 SITL odometry is received via Micro-XRCE-DDS.
   - `Arm / Takeoff / Land / Mission`: Controls unlock automatically when all systems are connected.
