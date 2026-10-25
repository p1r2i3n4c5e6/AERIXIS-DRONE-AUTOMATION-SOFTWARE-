# AERIXIS (flood-rescue drone swarm, built on AeroStack2) + PX4 SITL + AERIXIS GCS: MASTER PROJECT HANDOFF

> **Document Version:** 1.0.0  
> **Target Audience:** Any AI Assistant / Robotics Engineer taking over this codebase.  
> **Workspace Path:** `/home/prince/aerostack2`  
> **Custom GCS Path:** `/home/prince/custom_gcs`  
> **PX4 Autopilot Path:** `/home/prince/px4_sitl/PX4-Autopilot`

---

## 1. Executive Summary & System Objectives
This repository contains a production-grade, decoupled autonomous aerial robotics platform combining:
1. **PX4 Autopilot & Gazebo Harmonic Simulation:** Simulates multicopter dynamics, IMU, GPS, and offboard control interfaces.
2. **AERIXIS core (open-source AeroStack2 framework, BSD licence; ROS 2 Humble):** Autonomous flight software providing platform abstraction, state estimation, PID motion controllers, trajectory generation (`dynamic_mav_trajectory_generator`), and BehaviorTree.CPP v4 mission management.
3. **Decoupled Custom Web GCS Studio (`localhost:8765`):** A custom Ground Control Station featuring:
   - 3D interactive Behavior Tree visualizer built with Three.js.
   - Leaflet GIS map with polygon boundary draw, KML import, and automatic Boustrophedon (lawnmower) survey grid generation.
   - Bidirectional MAVLink v2 telemetry router and diagnostics socket.
   - Live network data transfer monitor (TX/RX KB, packet counts, bitrate, XML payload size).
   - Multi-drone swarm planning roadmap for long-range (20–30 km) mesh networks.

---

## 2. Decoupled 3-Tier System Architecture

```
┌──────────────────────────────────────────────────────────────────────────────────┐
│ TIER 1: CUSTOM GCS WEB STUDIO (Ground Laptop / Station)                          │
│  - Web Interface: HTML5, CSS3, Vanilla JS, Three.js 3D Scene, Leaflet GIS        │
│  - Backend: custom_gcs/server.py (HTTP port 8765 + UDP MAVLink port 14551/14550) │
│  - Features: KML Survey Planner, BehaviorTree.CPP Builder, Live Traffic Card     │
└────────────────────────────────────────┬─────────────────────────────────────────┘
                                         │ MAVLink Telemetry & BT XML Upload
                                         ▼ (UDP 14550 <-> 14551)
┌──────────────────────────────────────────────────────────────────────────────────┐
│ TIER 2: AERIXIS ONBOARD CORE STACK (AeroStack2 framework, ROS 2 Humble)           │
│  - as2_mavlink_gateway: MAVLink <-> ROS 2 Bridge                                 │
│  - as2_platform_pixhawk: Bridge to PX4 FMU Topics                                │
│  - as2_state_estimator: Raw odometry plugin                                     │
│  - as2_motion_controller: PID speed controller                                   │
│  - as2_behaviors_motion: Takeoff, Land, GoTo, FollowPath Action Servers          │
│  - as2_behaviors_trajectory_generation: dynamic_mav_trajectory_generator         │
│  - as2_behavior_tree: BehaviorTree.CPP v4 Engine (/tmp/as2_mission_tree.xml)     │
└────────────────────────────────────────┬─────────────────────────────────────────┘
                                         │ Micro-XRCE-DDS Middleware
                                         ▼ (UDP localhost:8888 / Serial @ 921600 baud)
┌──────────────────────────────────────────────────────────────────────────────────┐
│ TIER 3: AUTOPILOT / FLIGHT CONTROLLER (PX4 SITL / Hardware)                      │
│  - PX4 Autopilot v1.14 / Gazebo Harmonic Simulator (`gz sim`)                   │
│  - MicroXRCEAgent (Bridge between PX4 uORB and ROS 2 DDS)                        │
│  - Flight Modes: OFFBOARD, POSCTL, ALTCTL, MANUAL, AUTO_LAND                     │
└──────────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Operational Workflows & Shell Scripts

All components are launched and stopped using interactive bash scripts in `/home/prince/aerostack2`:

### Launching the System (In 3 Separate Terminals)
```bash
# Terminal 1: Launch Custom Ground Control Station
cd /home/prince/aerostack2
./1_launch_gcs.sh
# -> Web UI runs at http://localhost:8765

# Terminal 2: Launch the AERIXIS core stack (Mode 1 = SITL, Mode 2 = Real Hardware)
cd /home/prince/aerostack2
./2_launch_as2.sh 1

# Terminal 3: Launch PX4 SITL & Gazebo Simulation
cd /home/prince/aerostack2
./3_launch_simulation.sh
```

### Stopping the System
```bash
cd /home/prince/aerostack2
./stop_simulation.sh  # Kills PX4 SITL and Gazebo Harmonic cleanly
./stop_as2.sh         # Stops all 8 AERIXIS core nodes, MicroXRCEAgent, and MAVLink gateway
./stop_all.sh         # Stops entire stack including GCS server
```

---

## 4. Key Solved Issues & Code Modifications

### A. Behavior Tree 3D Hierarchy Restructuring (`custom_gcs/web/js/bt_studio.js`)
* **Old Bug:** All 36+ generated lawnmower scan waypoints were linked directly to `<Sequence name="FlightMissionSequence">`, creating an unreadable 40-node flat horizontal line across the 3D screen.
* **Solution Implemented:**
  - **Level 0:** `MainTree` $\rightarrow$ `FlightSequence` (`<Sequence name="FlightMissionSequence">`).
  - **Level 1 (Primary Branches):**
    1. `Set Offboard Mode` (Action)
    2. `Arm Drone` (Action)
    3. `Autonomous Takeoff` (Action)
    4. **`AreaCoverageTrajectory`** (`<Sequence name="AreaCoverageScanTrajectory">`)
    5. `Autonomous Land` (Action)
    6. `Disarm Motors` (Action)
  - **Level 2 (Under `AreaCoverageTrajectory`):** All scan waypoints (`Scan WP 1`, `Scan WP 2`, ..., `Scan WP N`) branch downwards directly from `AreaCoverageTrajectory`.
  - **Sidebar Tree Hierarchy:** All waypoints are cleanly nested inside an expandable/collapsible `4. Area Coverage Trajectory` container with a badge displaying `${N} Waypoints`.

### B. Prevention of Sudden Disconnect & False Instant "SUCCESS" (`custom_gcs/server.py` & `2_launch_as2.sh`)
* **Launcher Trap Bug:** In `2_launch_as2.sh`, `trap cleanup INT TERM EXIT` with `set -e` caused bash to catch `EXIT` when any minor background subshell exited, running `killall -9` on the entire AERIXIS stack!
  - *Fix:* Trap now only catches `INT` and `TERM` (Ctrl+C or kill), and uses a stable keepalive loop.
* **Pre-Flight Connection Gate:** In `server.py`, `run_behavior_tree()` now validates that the AERIXIS telemetry heartbeat is active (`< 7.0s`). If offline, execution is blocked with an explicit error:
  `"Cannot execute mission: AERIXIS is DISCONNECTED! Telemetry heartbeat is offline. Please launch ./2_launch_as2.sh first."`
* **Execution Status Integrity:** In `_exec_bt()`, if `proc.returncode != 0`, remaining steps are marked as `failed`, never falsely marked as `success`.

### C. Live Telemetry & Mission Data Transfer Card (`index.html`, `server.py`, `app.js`)
Added a telemetry traffic monitor card in the GCS sidebar:
- **⬆️ Uploaded (TX):** Total KB, packets transmitted, live KB/s transfer rate.
- **⬇️ Downloaded (RX):** Total KB, packets received, live KB/s transfer rate.
- **Mission BT XML Payload:** Exact byte size of the uploaded Behavior Tree XML file.
- **Telemetry Link Rate:** Heartbeat frequency (Hz) and link status.

---

## 5. BehaviorTree.CPP v4 Reference & Taxonomy

AERIXIS executes missions via BehaviorTree.CPP v4 using 4 fundamental node types:

### 1. Control Nodes (`<Control>`) — Flow Controllers
- **`Sequence`:** Ticks children 1 by 1. If all succeed $\rightarrow$ `SUCCESS`. If one fails $\rightarrow$ `FAILURE`.
- **`ReactiveSequence`:** Re-evaluates earlier conditions on every clock tick while an action is running. If a condition fails, running actions are aborted instantly.
- **`SequenceWithMemory`:** Remembers where it paused; does not re-tick previously completed children.
- **`Fallback` (Selector):** Tries children 1 by 1 to find the first that succeeds. Only falls back to the next child if the previous one fails.
- **`ReactiveFallback`:** Continuously checks higher-priority children to override lower-priority running tasks.
- **`Parallel` & `ParallelAll`:** Executes multiple children simultaneously.
- **`IfThenElse`, `WhileDoElse`, `Switch`:** Conditional branching nodes.

### 2. Decorator Nodes (`<Decorator>`) — Single-Child Modifiers
- **`Inverter`:** Flips `SUCCESS` $\leftrightarrow$ `FAILURE`.
- **`RetryUntilSuccessful`:** Retries a failing child up to $N$ times.
- **`Repeat`:** Repeats a successful child $N$ times.
- **`Timeout`:** Aborts child if execution takes longer than specified milliseconds.
- **`Delay`:** Adds a pause before ticking the child.
- **`ForceSuccess` / `ForceFailure`:** Overrides child output.

### 3. Action Nodes (`<Action>`) — Physical Work Execution
- **`SyncActionNode`:** Completes in < 1 ms (e.g. `Arm`, `Offboard`).
- **`AsyncActionNode`:** Runs in background thread, reporting `RUNNING` (e.g. `TakeOff`, `GoTo`, `FollowPath`, `Land`).
- **`RosActionNode`:** Bridges to a ROS 2 Action Server with goal, feedback, and cancel handles.
- **`RosServiceNode`:** Calls a ROS 2 Service (request $\rightarrow$ response).

### 4. Condition Nodes (`<Condition>`) — Instant Queries
- Read-only queries taking 0 time and returning `SUCCESS` or `FAILURE` (e.g. `IsMeshConnected`, `IsBatteryLow`, `IsPeerTooClose`).
- **The Blackboard:** Shared key-value memory where nodes read/write dynamic data (e.g. target coordinates, drone IDs) using `<input_port>` and `<output_port>`.

---

## 6. Long-Range (20–30 km) Swarm & Mesh Architecture Specification

### Network & RF Topology
- **Hardware:** MANET IP Mesh Radios (2.4 GHz / 1.4 GHz MIMO, e.g. Doodle Labs Smart Radio, Silvus StreamCaster, or Microhard Nano IP) or Dual-Link (868/915 MHz Air-to-Ground + 2.4 GHz Inter-Drone Mesh).
- **IP Subnet:**
  - GCS Laptop: `192.168.1.100`
  - Drone 0 (Companion Computer): `192.168.1.10` (Namespace `/drone0`)
  - Drone 1 (Companion Computer): `192.168.1.11` (Namespace `/drone1`)
  - Drone 2 (Companion Computer): `192.168.1.12` (Namespace `/drone2`)
- **Inter-Drone State Broadcast:** Each drone broadcasts a lightweight ~48-byte Swarm State Packet at 10 Hz over UDP/DDS (ID, GPS, local ENU pose, velocity, BT step index). Bandwidth consumption is `< 0.5 KB/s` per drone.
- **Decentralized Coordination:**
  - Common `ROS_DOMAIN_ID=42`.
  - Collision avoidance: Local potential field repulsion if inter-drone Euclidean distance $< 5.0\text{ m}$.
  - Formation tracking via `as2_behaviors_swarm_flocking` and virtual leader coordinate frames (`tf2`).
  - GCS multiplexes telemetry using MAVLink System IDs (`sysid=1, 2, 3`) with independent 3D Behavior Tree instances per drone.

---

## 7. Current System State & Immediate Next Steps

### Current Status
- All simulation and AERIXIS processes are cleanly stopped.
- The GCS Web Server is operational on `http://localhost:8765`.
- Behavior Tree hierarchy, KML generation, telemetry card, and safety gates are fully implemented and verified.

### Recommended Next Steps for Successor AI / Engineer
1. **SITL Verification:** Run `./3_launch_simulation.sh` $\rightarrow$ `./2_launch_as2.sh 1` $\rightarrow$ open `http://localhost:8765` $\rightarrow$ load a KML polygon $\rightarrow$ generate scan grid $\rightarrow$ click "Generate BT Trajectory & Run" to observe full flight execution.
2. **Multi-Drone GCS Expansion:** Add multi-drone tab switching (`Drone 0`, `Drone 1`, `Drone 2`) in `custom_gcs/web/index.html` and demux incoming telemetry by `sysid`.
3. **Swarm Launch Script:** Create `./launch_swarm_sitl.sh` launching 2 or 3 Gazebo drone models with unique ROS 2 namespaces (`drone0`, `drone1`).
