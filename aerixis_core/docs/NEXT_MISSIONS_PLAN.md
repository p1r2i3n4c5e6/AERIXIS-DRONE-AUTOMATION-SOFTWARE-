# AERIXIS — plan for the next mission work (written 2026-11-01)

Decided with the user: geotag behaviour first (done 2026-11-01, see IMPLEMENTATION_STATUS.md),
then the items below, one round each. New speed-critical code is written in C++; the tested
Python mission logic is kept and moved to C++ only where measurements show it is slow.

## Round 2 — Orbit from a KML only, with "which drone orbits"

DONE 2026-11-01 as GCS mission 6 "Search & orbit" (see IMPLEMENTATION_STATUS.md). The per-drone
`features` in the plan request is also the base for round 3's search/delivery roles.

Today mission 2 holds one watch station per drone and scans; it needs Generate.
Plan:
- KML upload is enough: the drone plans its own search path inside the area (lanes from the
  existing coverage planner, run onboard or by the GCS automatically on upload, no Generate
  step), camera searching as in geotag missions.
- On a person: lock, orbit (radius/laps as now), then resume the search path.
- Several drones connected: the GCS asks once "Which drones may orbit?" (checkboxes). The others
  only geotag the person and continue, so one drone watches while the rest keep searching.
- Planner: per-drone role in the plan request (fleet[i].role = 'orbit' | 'search'); the tree
  gets OrbitTarget only for orbit drones. Existing claim de-confliction keeps two orbit drones
  off the same person.

## Round 3 — 2 search drones + 2 delivery drones over the mesh

DONE 2026-11-01 as GCS mission 7 "Search, geotag & deliver" (see IMPLEMENTATION_STATUS.md).
Differences from the sketch below: no new lease authority — the existing person-claim rule
(earlier claim wins, ties to the lower drone ID) splits the targets; the geotags already shared
as plan-frame points are used directly; one package per delivery drone.

Goal: drones A and B fly swarm coverage + geotag; drones C and D carry payloads. Every geotag is
shared over the mesh; C and D split the targets between them automatically and drop.
Missing today (found in the code): delivery-only drones (every drone must get lanes), geotags
sent as delivery targets (peers only share the last 4 x,y points to avoid re-tagging), a split
rule, and an op that flies to a geotag and drops without its own camera lock.
Plan:
- Planner: roles per drone (search / delivery); delivery drones get no lanes, a holding point
  near launch and a delivery tree: wait for targets -> claim -> fly at a safe altitude ->
  descend -> verify -> drop -> report -> next target -> RTL when none left or payload empty.
- Mesh message "geotag" with a stable tag ID (drone + sequence), lat/lon, sigma, time; sent
  until acknowledged; every drone keeps the full list (also for the GCS map).
- Split rule (deterministic, no central computer needed): each delivery drone ranks the open
  targets by its own flying distance; a target is claimed through the existing lease authority
  (AssignmentAuthority, swarm.py) keyed by tag ID; if two claim at once the earlier claim wins,
  ties go to the lower drone ID (same rule as person claims today). Lease loss = drop nothing,
  pick the next target. So if C takes target 1, D automatically takes target 2.
- New C++ BT nodes (as2_behavior_tree): AcquireTarget (next unclaimed geotag), FlyToTarget,
  DropAtTarget (release at a verified hover above or beside the point, operator-confirmed load,
  existing one-shot payload journal). Requires a Pi image rebuild.
- Tested in the arena simulation with 4 drones (2 + 2) and 50 people.

## Round 4 — Flexible multi-condition missions

Extend the custom mission builder from one linear feature list to phases with conditions,
e.g. "search until 5 people tagged OR 10 min, then deliver to the nearest 2, then RTL",
compiled into the same safe tree shell (safety gates, recovery branch, emergency action).

## Scenario test bench (instead of reinforcement learning, for now)

- C++ batch runner around the existing simulation: hundreds of missions with random people
  positions, drone counts, wind and start slots; headless Gazebo (no 3D window, GPU rendering).
- Score each run: area covered, people found / missed, geotag error vs ground truth, time,
  aborts, separation holds, battery used. Results table + charts in the GCS.
- Auto-tune mission settings (lane spacing, speed, sweep angle, approach distance, timeouts)
  with Bayesian optimisation over these scores.
- Reinforcement learning later only as a planner that SUGGESTS paths/assignments inside the
  safety envelope, trained in a fast 2-D simulator, never flying the drone directly; the bench
  above is what would measure whether it is better than the current planner.
