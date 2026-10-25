# In-flight avoidance — stage 3: onboard, GCS switch, Gazebo SITL (2026-10-23)

Stage 2 trained the brain in a 2-D simulator. Stage 3 puts it on the drones' onboard software, adds the
GCS switch, and flies it in PX4 + Gazebo SITL.

## What runs where

| part | where | what |
|---|---|---|
| `as2_rescue_runtime/avoidance.py` | each drone (Pi 5) | **AI ON**: a route leg (SurveyWaypoint / ResumeCoverage) is flown with velocity commands: route velocity -> brain (every 0.5 s) -> ORCA shield -> AS2 speed reference, 10 Hz; leg ends within 1.5 m of its waypoint. **AI OFF**: nothing here runs, every leg is the usual AS2 GoTo. No trained model -> plain ORCA. |
| `engine.py` | each drone | starts each leg through the layer when it is on; switched off mid-leg -> the rest of the leg is the usual GoTo at once; switched on -> from the next leg |
| neighbour radius | each drone | the separation gate's own margin (10 m + both GNSS sigmas + both speeds x 1 s) + 1 m, so avoidance keeps the drones outside the gate and the gate stays the last resort |
| mesh frames (`swarm_agent`, `swarm.py`) | each drone | now carry the earth-frame velocity (`w`, ~16 bytes); `swarm/status.neighbours` gives the runtime each peer's position, velocity, sigma, age |
| route approval (`route_node`) | each drone | avoidance legs may leave the planned line by up to 25 m (`avoidance_corridor_m`; the layer stops pushing outward at 20 m) but never the planning area; normal legs keep 3 m |
| mission manager | each drone | signed `/api/avoidance` (over the telemetry radio like every mission command); publishes the latched `avoidance/enabled`; **always OFF after a (re)start** |
| GCS menu "AI" button | laptop | after Safety, before RTL: shows what the drones report (AI: ON / AI: off / AI: n/total), confirm dialog to switch on, off at once; drone cards show "AI on" / "AI avoiding (nearest N m)" |
| planner: drawn routes | laptop + drones | a KML LineString named after a drone (`drone0`, `drone1`, ...) is flown instead of its coverage lanes (every selected drone needs one); used for the head-on test, also for real crossing missions |

Test missions: `rescue_ws/sitl_missions/headon_2drones.kml` (same line, opposite directions) and
`headon_crossing_3drones.kml` (+ a third drone crossing north-south), both on the 100 x 80 m SITL field.

## Gazebo SITL results (PX4 v1.14, 2 x500, mission 5 "Swarm coverage" with drawn routes, from the GCS)

| run | AI | result |
|---|---|---|
| A | off | as today: both flew at each other on the same line, the separation gate stopped **both missions**, both returned and landed; still came within **9.5 m** while stopping |
| B1 | on | avoided (closest 20.1 m) but both legs failed: "Aircraft left the approved leg corridor" — the route check allowed 3 m off the line. Fixed (25 m corridor for avoidance legs, inside the area only). |
| B2 | on | **both missions completed** (both legs SUCCESS, return, land). Closest during the head-on legs **15.1 m**, gate never tripped. |

B2 found a second problem, outside the avoidance itself: after the head-on mission each drone ended at the
other's end, their paths home crossed, and with the return heights 30 m / 35 m they passed **5.6 m apart
directly above each other**. The per-drone return step is now **10 m** (30 / 40 / 50 m; PX4 `RTL_RETURN_ALT`
confirmed 30/40/50 in SITL), so returning drones keep the 10 m separation by height alone.

**3 drones (C): not completed.** The run started from the GCS with AI on, but the laptop was overloaded
(load average 20–34 on 16 cores: 3 PX4 + Gazebo + 3 onboard stacks, plus four of my test recorders left
running by mistake — stopped): drone0 lost readiness while taking off (IMU samples too slow for the
vibration check), drone2's estimator never published a pose (its origin was set from an invalid SITL GPS
altitude, −232 673 m), drone1 then failed waiting for fresh peers. Nothing flew into anything; all three
were recovered on the ground. To be repeated with less load.

## Retry, later on 2026-10-23 (3 drones on a quiet laptop)

Before the retry: twelve leaked test recorders of mine were found and stopped (they had loaded the
laptop all day), and the route publisher now sets the plan origin only from a steady, plausible GNSS
fix (`steady_fix`: finite, -500..9000 m, >= 5 fixes over 3 s agreeing within 5 m).

What the retry found and fixed:
- **drone2 fell through the SITL world**: the ground of `as2_playground.sdf` was a 60 x 40 m box, the third
  spawn point (x = 40 m) is off it — hence the −232 673 m GPS altitudes. Ground now 500 x 500 m
  (PX4 world file, outside the repo; backup `.before_ground_500m_20261023`). With it all three drones
  reconcile with plausible origins (487–488 m).
- **`'NoneType' object is not subscriptable`** on a leg: readiness accepted vehicle data up to 1.0 s old,
  the leg code read it with a 0.5 s limit. Every read now uses `vehicle_max_age_s` (regression test
  fails on the old engine).
- **A flying drone swerved around drones still on the ground** 12 m below: the layer now ignores peers
  more than the separation (10 m) above or below.
- **Mesh port collision**: drone1's mesh bridge died at start because its port 47801 had been handed to
  its FC guardian as a random outgoing UDP port. Mesh ports moved below Linux's ephemeral range
  (24800+; SITL script, mesh_bridge default, Pi launcher default, Docker example).

**3 drones: still not flown.** With the fixes the three stacks are healthy on the ground, but in flight
PX4 itself reported its simulated sensors timing out (gyro, magnetometer, barometer "TIMEOUT"),
landed blind and left OFFBOARD: 3 PX4 + Gazebo + 3 onboard stacks keep this laptop at load ~20 on 16
cores. Needs a faster computer (or fewer processes per drone) for 3-drone SITL.

**2 drones again with every fix** (head-on, AI on): both missions completed, closest 15.1 m on the
head-on legs, closest 10.6 m on the way home with 10.5 m height difference (the 10 m return step).

## Tests

Runtime 158 (new: layer OFF = GoTo, ON bends right for a head-on peer and finishes, OFF mid-leg -> GoTo,
gate-margin radius; route corridor inside/outside the area; return heights 30/40/50), manager 18 (switch:
OFF at every start, reaches the runtime, true/false only), planner 69 (drawn routes), avoidance 15.

## Open

- 3-drone SITL with AI on (above).
- Route publisher should refuse an implausible GPS altitude when it sets the origin (drone2 above).
- The layer does not know the planning area; it relies on the route check refusing a detour outside it
  (the leg then fails safely). Give it the area so it bends inwards instead.
- ReturnToLaunch is not flown through the layer (height separation covers it now).
- Pi 5: image rebuild (the package and its 28 KB model are included by the existing Dockerfile), then real
  drones: two drones, large spacing, slow speed first.
