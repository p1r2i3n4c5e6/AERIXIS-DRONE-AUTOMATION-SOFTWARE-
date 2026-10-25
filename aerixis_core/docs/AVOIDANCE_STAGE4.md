# In-flight avoidance — stage 4: 3 m separation (RTK), return at mission height (2026-10-23)

Operator question after stage 3: drone0 flew at 15 m but another drone climbed to 32 m on the way home —
was that the AI? And: train it for the shortest, safest paths, close passes, no extra height or width.

## Where heights and paths come from (before this stage)

| what | made by | AI? |
|---|---|---|
| the route (drawn line / coverage lanes) and its height | the planner (mission altitude, e.g. 12–15 m) | no |
| sideways bend when two drones meet on a leg | brain + ORCA shield (avoidance layer) | **yes**, horizontal only — it never changes height |
| the climb on the way home (30 / 40 / 50 m, i.e. 32 m seen) | fixed per-drone return heights (`rtl.py`), base 30 m + 10 m per drone | no — a fixed safety rule |

## What changed (operator choices: "3 m (RTK, experts)", "mission height + AI")

1. **One separation setting** (`minimum_separation_m`) drives the separation gate, the avoidance radius,
   the brain's input and the return-height step. Default stays **10 m** (ordinary GNSS); **3 m only
   with RTK on every drone** (`SEPARATION=3.0` in `rescue_ws/scripts/sitl_onboard.sh`).
2. **Gate margin from the real message age**: separation + both GNSS sigmas + both speeds x (age +
   0.2 s). With RTK over the base station at 1.5 m/s that is about 4 m; the avoidance keeps 1 m more.
3. **Return home with AI on: at mission height**, flown through the avoidance layer (no climb). With
   AI off, and for PX4's own emergency RTL (Pi or link dead), the per-drone heights stay, now **16 m +
   one separation per drone**: 16 / 19 / 22 m with 3 m, 16 / 26 / 36 m with 10 m
   (`px4_failsafe_params.py --return-step <separation>`).
4. **Brain v4**, one model for both rules (trained on 2/3 RTK 3 m, 1/3 GNSS 10 m episodes):
   - the simulator now has `Rules` (`sim.GNSS`, `sim.RTK`: separation, GNSS error, collision distance
     1 m for RTK — x500 V2 is ~0.85 m across with props);
   - neighbours are avoided with exactly the onboard rule (`orca.neighbour_for_gate`: position
     extrapolated by the message age, radius = gate margin + 1 m; a runtime test checks both agree);
   - a new input per peer: clearance in units of its radius (43 inputs; an older model is refused and
     the layer flies plain ORCA instead of misreading it);
   - 5x stronger penalty for leaving the planned line (width), best model chosen by safety, then
     completed missions, then time, then width.

## Training

`python3 -m as2_rescue_avoidance.train --rules rtk,gnss --iterations 300 --workers 10 --out models/brain_v4`
(28 min on the laptop, next to a running 2-drone SITL). Validation (900 episodes, both rules) was safe at
every check (0 collisions, 0 under the separation); best at iteration 280: 900/900 completed, time x1.040.
Shipped as `models/brain.npz` (43 inputs, 29 KB); the stage-2/3 model is kept as
`models/brain_run3_gnss10m.npz` (it cannot be loaded by this brain version — the layer would fly ORCA).

## Results on unseen encounters (seeds 100 000+, 200 per family and link)

`results/stage4_rtk.md`, `results/stage4_gnss.md`. "Detour" = largest distance from the planned line
(the width a pass costs), mean over episodes.

| rule | policy | episodes | collisions | under separation | completed | closest | detour | time x ideal |
|---|---|---|---|---|---|---|---|---|
| 3 m RTK | today's gate | 3000 | 0 | **10** (2.72 m) | **344** (2656 aborted) | 2.72 m | 0.1 m | — |
| 3 m RTK | ORCA | 3000 | 0 | 0 | 3000 | 4.39 m | 4.80 m | 1.006 |
| 3 m RTK | **brain v4** | 3000 | 0 | 0 | 3000 | 4.50 m | **4.74 m** | **0.989** |
| 10 m GNSS | ORCA | 3000 | 0 | 4 | 2968 | 9.39 m | 10.52 m | 1.151 |
| 10 m GNSS | **brain v4** | 3000 | 0 | **2** | **3000** | 9.43 m | **10.03 m** | **1.101** |

Head-on on the same line, 3 m RTK, base-station link: closest 4.8 m, each drone ~3.0 m to the side of its
line, 100 % passed on the right. Over LoRa (messages up to ~1 s old) the margin grows: closest ~6.8 m,
~6 m to the side.

**What limits the width**: not the AI. The ORCA shield keeps every pair outside the separation gate's
margin + 1 m (about 5 m at 1.5 m/s with RTK over the base station), so the narrowest safe head-on pass
is each drone moving about half of that sideways. Both ORCA and the brain are at that limit; the
brain's gain is time (it does not slow down more than needed) and finishing crowded encounters. The
avoidance never climbs or descends: height stays the route's (mission) height.

Passing closer than ~4.5 m would mean shrinking the gate margin itself (the 1 m buffer, or the speed x
latency term) — a safety-rule decision, not something to train.

## Gazebo SITL (PX4 v1.14, 2 x500, 3 m separation, AI on, mission height 15 m, from the GCS)

| run | mission | result |
|---|---|---|
| 1 | head-on (same line) | drone0's PX4 refused to arm (pre-flight checks) but AS2 reported ARM; take-off ran 90 s on the ground, then ReturnToLaunch with AI waited for ever on the ground ("simulation stuck"). drone1's landing after its AI return failed "Motion ownership unavailable". **Fixed** (below). |
| 2 | head-on | both completed; drone1's return first aborted AIRCRAFT_UNSTABLE (13 deg, 67 deg/s) when the AI return reversed 1.35 m/s in one step and turned the nose 180 deg. **Fixed**: 1.5 m/s^2 command limit, return keeps the nose. Heights max 15.1 m; legs closest 7.2 m, both passed right; return closest 7.8 m at 0.3 m height difference. |
| 3 | head-on | drone1's return aborted again (6.6 deg, **224 deg/s**). PX4 log: one velocity setpoint of 41 m/s = -(drone1's position): our yaw pose carried position (0,0,0) and reached the controller while it was still in POSITION mode after the stop. **Fixed** in the driver (below). |
| 4 | head-on | **both completed cleanly**, every step SUCCESS first time. |
| operator runs 12:58–13:15 | small area 26 x 28 m, 2 lanes | lanes along the edge: "Avoidance detour left the planning area" (drone0); drone1 (taking off 3.5 m outside the area) hovered 3 min on its first leg until the operator pressed RTL — the engine waited silently. AI off: the gate stopped both (expected). |
| 5 | small area (reproduced) | drone0: "detour left the planning area" reproduced. |
| 6 | small area, area-aware | **both completed** (drone0 4 waypoints, drone1 2, AI returns, landings); drone0 stayed >= 1.25 m inside the area; heights max 15.1 / 15.9 m. |

Fixes from these runs (all with regression tests):
- take-off fails within 8 s when the FC is not armed, or 20 s without 0.5 m climb (engine);
- ReturnToLaunch / ManagedLand on a disarmed aircraft end at once ("Already on the ground");
- take-off / landing wait up to 5 s for the previous (velocity) motion to hand back;
- AI commands change by at most 1.5 m/s^2; the AI return keeps the nose;
- driver: velocity is published only after the controller acknowledged *this* switch to SPEED, and the
  yaw pose carries the aircraft's own position (never 0,0,0);
- the route approval carries the planning area; the ORCA shield treats its edges as hard limits
  (1.5 m margin, RVO2 obstacle lines: never relaxed) — near an edge the pass goes to the inside;
- a leg the route check keeps refusing fails after 5 s with the reason; what a leg waits for is shown
  in `rescue/state.waiting`.

Not covered: 3-drone SITL (laptop capacity, stage 3), real drones. A ROS bridge integration test is
timing-sensitive under SITL load (passes alone).
