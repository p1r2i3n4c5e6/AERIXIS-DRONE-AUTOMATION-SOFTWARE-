# In-flight avoidance — stage 1: simulator and ORCA baseline (2026-10-23)

Goal of the avoidance work: when drones share their positions over the mesh, each drone bends its own
path around the others (passing on the right, like people do) and still completes its mission, instead
of today's "stop the mission and return". Stage 1 builds the tools and measures the classic method.
Nothing here flies a real or simulated (Gazebo) drone yet: it is a fast 2-D simulator.

Package: `rescue_ws/src/as2_rescue_avoidance` (pure Python, no ROS needed for the simulator).

| file | what |
|---|---|
| `orca.py` | ORCA velocity filter (port of RVO2's 2-D agent part) + right-hand passing + message-age handling. Pure Python (no numpy) so the same code runs on the Pi 5 as the safety shield. |
| `links.py` | mesh link models: **base_station** (main: IP network, 10 Hz, ~50 ms, 2 % loss), **edge** (base station near its range limit: 2 Hz, 30 % loss), **lora** (fallback: 1 Hz, 0.4–0.7 s, 10 % loss) |
| `sim.py` | drones with acceleration/speed limits, drifting GNSS error (0.8 m), route following that rejoins the planned line after a detour; peers known only through the mesh (late, noisy, lost messages); policies `straight` (no avoidance), `stop_gate` (today's logic), `orca` |
| `scenarios.py` | **head_on** (same path, opposite directions — the operator's case), **crossing** (30–150°), **overtaking**, **blocker** (a drone orbiting a person on another drone's path), **random** (3–5 drones through one area); every scenario is timed so the drones meet |
| `evaluate.py` | `ros2 run as2_rescue_avoidance avoidance_eval --seeds 300 --out results/stage1` — every policy meets exactly the same encounters; all CPU cores |

## Results — 13 500 episodes (300 per family x 5 families x 3 links x 3 policies), 114 s

All drones at one height (the worst case). "Below 10 m": the true distance between two drones went
under the fleet's 10 m separation at some moment.

| policy | link | episodes with a collision (<2 m) | below 10 m | missions completed | episodes with an aborted mission | closest (m) | head-on passes on the right |
|---|---|---|---|---|---|---|---|
| straight | base_station | 1028/1500 (69%) | 1415/1500 (94.3%) | 1500/1500 (100.0%) | 0/1500 | 0.01 | 55% |
| straight | edge | 1009/1500 (67%) | 1412/1500 (94.1%) | 1500/1500 (100.0%) | 0/1500 | 0.0 | 50% |
| straight | lora | 1025/1500 (68%) | 1415/1500 (94.3%) | 1500/1500 (100.0%) | 0/1500 | 0.01 | 51% |
| stop_gate | base_station | 0/1500 (0%) | 112/1500 (7.5%) | 40/1500 (2.7%) | 1465/1500 | 8.31 | 53% |
| stop_gate | edge | 0/1500 (0%) | 77/1500 (5.1%) | 17/1500 (1.1%) | 1492/1500 | 8.39 | 51% |
| stop_gate | lora | 0/1500 (0%) | 65/1500 (4.3%) | 7/1500 (0.5%) | 1496/1500 | 7.73 | 53% |
| orca | base_station | 0/1500 (0%) | 4/1500 (0.3%) | 1499/1500 (99.9%) | 0/1500 | 9.18 | 100% |
| orca | edge | 0/1500 (0%) | 1/1500 (0.1%) | 1500/1500 (100.0%) | 0/1500 | 9.95 | 100% |
| orca | lora | 0/1500 (0%) | 0/1500 (0.0%) | 1498/1500 (99.9%) | 0/1500 | 10.33 | 100% |

Per-family table (detour, delay, passing side): `rescue_ws/src/as2_rescue_avoidance/results/stage1.md`
(+ every episode in `stage1.json`).

What this says:
- **Today's stop gate** never collides, but it ends almost every mission that meets another drone
  (1465–1496 of 1500), and because a drone stops only once it is already close it still goes under 10 m
  in 4–8 % of encounters (closest 7.7 m).
- **ORCA** completes 99.9 % of the missions with no collision; head-on on the same path both drones
  always pass on the right; typical detour 4–15 m, missions take 3–30 % longer (overtaking the most).
- ORCA's remaining weak spots — what the learned policy (stage 2) has to beat:
  - 5 episodes went to 9.2–9.9 m for 0.3–3.4 s, 4 of them on one seed with a large GNSS error on both
    drones (each believes they are further apart than they are). GNSS-based separation can not see
    this; RTK GNSS or a larger buffer would.
  - 3 episodes did not finish within 2.5x the ideal time (two drones circling during an overtake, two
    5-drone crowds) — ORCA's known "reciprocal dance".
  - Overtaking and crowds make long detours (up to 30–44 m).
- Only horizontal avoidance so far; drones keep their height.

Tuning found here (kept in `orca.py`): right-hand bias applied in full once a collision course is
predicted (a gradual bias passed on the left in ~40 % of head-on encounters), 2 m buffer on top of
10 m + both GNSS sigmas + message age.

## Next stages

2. Train the learned policy (shared by all drones, decides only from its own mesh view) on these
   scenarios, ORCA as the safety shield on its output; must beat ORCA on completion/detour/dance
   without losing safety on new seeds.
3. Onboard: avoidance layer in the rescue runtime (ORCA shield + policy, Pi 5 CPU), mesh frames with
   velocity and next waypoint, route corridor approval, GCS menu "AI avoidance" on/off (OFF = today's
   logic unchanged), 2–3 drone Gazebo SITL including a head-on mission drawn by the operator.
