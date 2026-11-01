# AERIXIS simulation with the rescue arena

Runs every AERIXIS mission in PX4 SITL inside the 300 × 300 m rescue arena
(`../rescue_arena_300`: 50 people, 180 trees), placed at the location you choose on the map.

## From the GCS (normal use)

1. Click **Connect → Simulation (SITL)**. The simulation starts at once with the drone count and
   location set in the Simulation panel (location: what you typed, else your location, else where
   the map is looking), and a loading window opens over the blurred GCS — no setup step.
2. The window shows each stage live until everything is up (1–3 minutes): arena at your location
   → Gazebo world → PX4 per drone → onboard software → drones added to the GCS → link open with
   every drone answering. Nothing to type: the drones go into Simulation's own aircraft list
   (`config/fleet_sim.json`, never your field list) and the link opens on UDP 14555. The
   **Gazebo 3D window** opens by itself. Start-up warnings (uXRCE-DDS, mission interpreter) are
   muted while the simulation settles. *Run in background* hides the window; clicking
   Simulation (SITL) again shows it.
3. **Add drone** (loading window when ready, or the Simulation panel) puts one more drone into the
   **same running arena and the same Gazebo window** — up to 4, on fixed take-off spots 20 m apart
   (−10/−10, +10/−10, −10/+10, +10/+10). The drones already there stay on the ground and connected;
   only the onboard software restarts (about 1–2 minutes). If an add fails, the window says so with
   the launcher output, and **Retry** finishes that same drone (no extra PX4 is spawned).
   Not possible while a mission runs.
4. **3D window** reopens Gazebo if you closed it (closing it no longer stops the simulation).
5. **Go to Plan** → **Use arena as search area**, or turn on **Draw area** on the map and click the
   corners of your own area inside the orange outline (click the first corner to finish), or load
   a KML file. Pick a mission, Generate, then **Upload & start**. Mission 6 **Search & orbit**
   needs no Generate: once the area is set and the conditions accepted (and, with several drones,
   the drones that may orbit are ticked) it plans and assigns by itself.
6. When a mission that **uses the camera** starts (Track, Orbit, Geotag, Geotag + drop, or your own
   with detection), the drone camera window opens by itself; Swarm coverage does not open it.
7. The maps show the arena outline and the 50 ground-truth people (orange dots); each geotag says
   how far it is from the nearest real person.
8. **Restart** (Simulation panel) applies a new location or starts over; **Stop** closes everything.

Simulation and Field are fully separate: Field radio closes and refuses the simulation link,
Simulation closes and refuses the radio, and each has its own aircraft list.
The simulation-control buttons work only on the GCS laptop itself (not from a phone).

## What runs

| Part | File | What it does |
|---|---|---|
| Arena at your location | `prepare_arena.py` | Regenerates the arena with its GPS origin at the chosen point (`~/.local/share/aerixis/arena/<lat>_<lon>/`), picks take-off spots in the central clearing 20 m apart and ≥ 4 m from trees, writes `sim_info.json` |
| PX4 + Gazebo | `arena_px4.sh`, `px4_env.sh` | Gazebo Harmonic server with the arena world, then one PX4 1.16 per drone: `droneI`, system ID I+1, model `x500_a8` |
| Add a drone | `add_px4.sh` (`sim.sh add`) | One more PX4 into the running world on the next take-off spot |
| 3D window | `gazebo_gui.sh` (`sim.sh gui`) | The Gazebo window, separate from the server: close and reopen any time |
| Drone model | `models/x500_a8`, `models/gimbal_a8` | PX4's x500 with its 3-axis gimbal camera, set to an A8 mini-like 65° view, 640×360, 15 fps |
| Camera window | `sim_cameras.py` | One window with every drone's camera, the lock state, target, distance and gimbal angles; points each Gazebo gimbal where the simulated tracker points its gimbal |
| Launcher | `aerostack2/launch/sim.sh start N --arena --home LAT,LON[,ALT] [--no-3d] [--no-cameras]` | Runs all of the above plus the onboard stack of every drone (`sitl_onboard.sh`) |

Same from a terminal:

```bash
PX4_AUTOPILOT_DIR=$HOME/px4_sitl/PX4-Autopilot-1.16 ~/aerostack2/launch/sim.sh start 2 --arena --home 28.6139,77.2090
~/aerostack2/launch/sim.sh add         # one more drone in the same arena
~/aerostack2/launch/sim.sh gui         # reopen the Gazebo 3D window
~/aerostack2/launch/sim.sh cameras     # reopen the camera window
~/aerostack2/launch/sim.sh stop
```

## Detection and geotagging in the arena

The simulated A8 tracker (`a8_sim_vision`) knows the 50 arena people exactly and computes what
the camera sees from the drone's **true Gazebo pose** (as a real camera sees the real world),
so detection is perfect and the camera window's target cross sits on the rendered person.
Geotags are computed by the normal rescue software from its **own** position and heading
estimate, exactly as on a real drone, so the arena check on the map shows the real geotagging
accuracy of the stack.

## Closing

- **Camera window:** press **q** or **Esc** in it, or close it with its **X**. Reopen it with
  **Camera window** in the GCS.
- **Gazebo 3D window:** close it any time; the simulation keeps running. **3D window** reopens it.
- **Whole simulation:** **Stop** in the GCS Simulation panel (or `launch/sim.sh stop`). If PX4 and
  Gazebo end on their own, the GCS stops the drones' onboard software and the camera window too.
- The camera window belongs to the simulation, not to a mission: it stays open between missions
  so you can watch the next one.

## Tested (2026-10-31)

Two drones started from the GCS, mission *Geotag & resume* over a 120 × 80 m box with 4 arena
people at 15 m: all 4 geotagged overhead, **0.02–0.11 m** from their true positions, both drones
landed after about 9 minutes.

## Notes

- The arena simulation uses its own ROS domain (37): a real Pi on the same Wi-Fi also publishes
  `/drone0` on domain 0.
- Trees are 6.5–10.5 m tall: keep the mission altitude ≥ 15 m (the default).
- Up to 4 drones fit the clearing; 3–4 drones with both windows load this laptop heavily.
- In this simulated world PX4's estimated heading is 6–7° off the true heading (the world's
  magnetic field does not match the real declination at the chosen place). It is a simulation
  artefact and shows up in the geotag check for people tagged from a distance.
