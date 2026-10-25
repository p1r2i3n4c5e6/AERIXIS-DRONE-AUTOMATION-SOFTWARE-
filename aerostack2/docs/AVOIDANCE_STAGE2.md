# In-flight avoidance — stage 2: the learned brain (2026-10-23)

Stage 1 (`AVOIDANCE_STAGE1.md`) built the simulator and measured ORCA. Stage 2 trains a policy that
every drone runs on its own mesh view, with ORCA as its safety shield, and compares it with ORCA on
encounters it never saw. Still simulation only (2-D, all drones at one height): not in Gazebo, not on
the Pi, not on drones yet.

## How the brain works

- **Input** (39 numbers, in the drone's own route frame so it works in any direction): own velocity,
  route velocity, distance from the planned line, route left, mission speed; the 4 nearest drones from
  the newest mesh messages (position projected by message age, relative velocity, separation radius,
  message age, distance).
- **Output** every 0.5 s: speed along the route (0.1–1.2x) and a sideways velocity. An untrained brain
  outputs 0 = plain ORCA, so training starts from the baseline (checked by a test).
- **Shield**: ORCA with right-hand passing checks the brain's velocity every 0.1 s; the executed velocity
  is always ORCA-safe. The existing gates (10 m stop, holds, return heights) stay underneath in flight.
- **Size and speed**: 2 hidden layers of 64, 28 KB file (`rescue_ws/src/as2_rescue_avoidance/models/brain.npz`,
  installed to `share/as2_rescue_avoidance/models`), NumPy only, ~8 µs per decision on the laptop CPU.
- **Training**: PPO, one network shared by all drones, each learns from its own view only; 13 CPU
  workers simulate, the RTX 4050 learns; random mix of all 5 encounter families and all 3 links.

## Training — three runs (each problem found from the numbers, not assumed)

| run | reward | what happened |
|---|---|---|
| 1 | progress + finish − closeness − shield − off-line | learned to fly 1.2x the operator's mission speed the whole time (missions *faster* than planned) — wrong for a camera search. Stopped at iteration 40. |
| 2 | + progress capped at mission speed, overspeed penalty | safe, but learned to slow down: only discounting pushed it to finish; time got worse every validation (x1.15 → x1.19). Stopped at 50. |
| 3 | + time cost 0.4 per 0.5 s until done, closeness band 12 m, −2 per step under 10 m | x1.074 after 10 iterations, best x1.056 at iteration 20, then flat (x1.06–1.09) for 100 iterations; stopped at 120. **Shipped: iteration 20** (selection: collisions, then time under 10 m, then completed, then time, on 450 validation episodes). |

Logs and histories of all three runs: `rescue_ws/src/as2_rescue_avoidance/models/`. Training 10 M+, validation
500 000+ and test 100 000+ seeds never overlap.

## Test — 9 000 unseen encounters (seeds 100 000–100 299, 5 families x 3 links x 300, both policies)

| link | policy | collisions | below 10 m | completed | closest | mission time | detour | head-on on the right |
|---|---|---|---|---|---|---|---|---|
| base station | ORCA | 0 | 2/1500 | 1499/1500 | 9.65 m | x1.113 | 8.1 m | 100 % |
| base station | **brain** | 0 | 1/1500 | 1499/1500 | 9.85 m | **x1.057** | 7.8 m | 100 % |
| edge of range | ORCA | 0 | 2/1500 | 1500/1500 | 9.69 m | x1.105 | 9.5 m | 100 % |
| edge of range | **brain** | 0 | 0/1500 | 1498/1500 | 10.21 m | **x1.058** | 9.1 m | 100 % |
| LoRa | ORCA | 0 | 2/1500 | 1496/1500 | 9.29 m | x1.109 | 10.0 m | 100 % |
| LoRa | **brain** | 0 | 0/1500 | 1500/1500 | 10.75 m | **x1.063** | 9.9 m | 100 % |

Per family (all links):

| family | ORCA: below 10 m / completed / time / detour | brain: below 10 m / completed / time / detour |
|---|---|---|
| head-on, same path | 0 / 900 / x1.033 / 10.0 m | 0 / 900 / **x1.007** / 9.5 m |
| crossing | 0 / 900 / x1.067 / 7.2 m | 0 / 900 / **x1.037** / 8.2 m |
| overtaking | 2 / 896 / x1.283 / 14.4 m | **0 / 900 / x1.160 / 11.0 m** |
| passing an orbiting drone | 1 / 900 / x1.043 / 4.1 m | 0 / 900 / **x1.007** / 5.0 m |
| crowd of 3–5 drones | 3 / 899 / x1.117 / 10.2 m | 1 / **897** / x1.080 / 10.9 m |

**Result**: the brain halves the time lost to avoidance (about +6 % instead of +11 %), goes under 10 m in
1 of 4 500 encounters instead of 6, never collides, always passes head-on traffic on the right, and fixes
ORCA's overtaking problem. One regression: in crowds 3 of 900 missions did not finish in time (ORCA 1).

## Limits — read before trusting it

- Simulation only, horizontal only, drones at one height, x500-like point-mass dynamics, modelled GNSS
  and links. Gazebo SITL and real flights can and will differ.
- The one dip under 10 m (9.85 m, base station) is the GNSS-error case from stage 1: neither method can
  see an error both drones share; RTK GNSS or a larger buffer would.
- Crowd completion is slightly worse than ORCA; a longer or larger training (more crowd scenarios) is the
  first thing to try.
- The network is only as good as its training scenarios: no wind, no obstacles, no non-cooperating
  aircraft, at most 5 drones.

## Reproduce

```bash
cd rescue_ws/src/as2_rescue_avoidance
PYTHONPATH=$PWD python3 -m as2_rescue_avoidance.train --iterations 120 --out models/brain
PYTHONPATH=$PWD python3 -m as2_rescue_avoidance.evaluate --policies orca,learned --model models/brain.npz --seed-start 100000 --out results/stage2
```
(Training needs PyTorch; do not put `/usr/lib/python3/dist-packages` first on PYTHONPATH for it. The
colcon build does need it first.)

## Next — stage 3 (onboard)

Avoidance layer in the rescue runtime (brain + ORCA shield on the Pi 5 CPU), mesh frames carrying
velocity, route corridor approval for the detours, GCS menu "AI avoidance" on/off (OFF = today's logic
unchanged), 2–3 drone Gazebo SITL including a head-on mission drawn by the operator, Pi Docker image.
