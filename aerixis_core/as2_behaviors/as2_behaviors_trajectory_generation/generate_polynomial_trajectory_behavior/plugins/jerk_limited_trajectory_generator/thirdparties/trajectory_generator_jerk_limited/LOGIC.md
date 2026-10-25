# Trajectory generation logic and configuration

This document is the single technical reference of the
`trajectory_generator_jerk_limited` library.
The algorithm is a clean reimplementation of the strategy used by [PX4
Autopilot](https://github.com/PX4/PX4-Autopilot) in `VelocitySmoothing` and `PositionSmoothing` [1, 2], with
two deliberate departures:

- All bounds on velocity and acceleration are expressed as **3D
  Euclidean norms** (`‖v‖ ≤ v_max`, `‖a‖ ≤ a_max`) rather than
  per-axis bounds.
- The library is purely reactive: it owns no waypoint manager. The
  caller drives the segment progression by rotating the active triplet
  `{prev, target, next}` on arrival.

## What the library does

Given an ordered list of 3D waypoints in the world ENU frame and a set
of kino-dynamic limits, the library produces, on every tick of period
$\Delta t$, a setpoint tuple

$$
\bigl(\mathbf{r},\ \dot{\mathbf{r}},\ \ddot{\mathbf{r}},\ \mathbf{r}^{(3)}\bigr) \in \mathbb{R}^{3 \times 4}
$$

— position, velocity, acceleration and jerk — that:

- **passes through (or near) every waypoint** in order,
- respects the **3D-norm bounds** $\|\dot{\mathbf{r}}\| \le v_{\max}$
  and $\|\ddot{\mathbf{r}}\| \le a_{\max}$, plus the **per-axis jerk
  bound** $|r^{(3)}_j| \le j_{\max}$,
- keeps the path a **straight line in configuration space** between
  consecutive waypoints (no axis-induced curvature),
- optionally **slows down** if the vehicle lags too far behind, so the
  trajectory waits instead of running away,
- and is **$C^2$ in position** during nominal operation (a momentary
  $C^2$ dip can occur when the 3D-norm post-clamp activates — see Section 5).

Everything is computed in **closed form** per tick: no optimisation
problem is solved, no global polynomial spline is fit. The library is
essentially a stateful integrator that re-derives a few S-curve phase
durations on every step.

What the library deliberately does **not** do:

- It does not own a waypoint manager. The streaming
  [`WaypointTrajectoryController`](include/trajectory_generator_jerk_limited/waypoint_controller.hpp)
  expects the caller to rotate the active triplet
  $\{\mathbf w_{-1},\mathbf w_0,\mathbf w_{+1}\}$ when the vehicle
  enters the acceptance radius.
- It does not emit yaw or attitude. Only translational kinematics.
- It does not solve a global optimal-time problem. The S-curve solver
  is locally minimum-time per tick under the active limits.

The two public entry points are:

| API | Header | Use case |
|---|---|---|
| `WaypointTrajectoryController` | [`waypoint_controller.hpp`](include/trajectory_generator_jerk_limited/waypoint_controller.hpp) | streaming, real-time control loop |
| `TrajectoryGenerator` (one-shot) | [`trajectory_generator.hpp`](include/trajectory_generator_jerk_limited/trajectory_generator.hpp) | offline planning + spline post-fit |

The one-shot wrapper drives the streaming controller forward at a
fixed $\Delta t$ until the last waypoint is settled, records every
sample, and exposes both `evaluate(t)` (exact at sample times) and a
piecewise-polynomial `spline()` (serialisation-grade).

## Pipeline at a glance

The same five steps run on every tick, in this order. Each step is
strictly deterministic and runs in closed form. The right column points
to the formal section that describes that step in full.

```
                 input: triplet {w-1, w0, w+1}, current state, params
                                  │
                                  ▼
   ┌──────────────────────────────────────────────────────────────┐
   │ 1. Build velocity setpoint  v_sp                             │  Section 2
   │    - braking-distance cap (single-waypoint)                  │
   │    - corner-speed cap, backward sweep, L1 corner-cut (multi) │
   ├──────────────────────────────────────────────────────────────┤
   │ 2. Compute tracking-error time stretch  β ∈ [0, 1]           │  Section 3
   │    - slows the trajectory when the vehicle lags              │
   ├──────────────────────────────────────────────────────────────┤
   │ 3. Per-axis S-curve: solve T1/T2/T3 + integrate β·Δt          │  Section 4
   │    - jerk = ±j_max; T1 saturated by a_max                    │
   ├──────────────────────────────────────────────────────────────┤
   │ 4. 3D-norm post-clamp + state re-injection                   │  Section 5
   │    - clamp ‖v‖ ≤ v_max, ‖a‖ ≤ a_max as 3D norms              │
   │    - feed clamped state back into the per-axis profiles      │
   ├──────────────────────────────────────────────────────────────┤
   │ 5. Axis time-synchronization                                 │  Section 6
   │    - extend shorter axes to T_max for the next tick          │
   └──────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
                 output: (r, v, a, j)
```

The key invariant is that step 4 always restores feasibility: if any
intermediate computation drifts, the next tick still starts from a
state inside the kino-dynamic envelope. The remainder of this document
expands each step in turn.

## 1. Problem statement

Given the current vehicle position $\mathbf{p}(t) \in \mathbb{R}^{3}$ in
the world ENU frame, an ordered triplet of waypoints
$\{\mathbf{w}_{-1}, \mathbf{w}_{0}, \mathbf{w}_{+1}\}$ (previous, current
target, next), and a fixed time step $\Delta t$, produce on each tick a
smooth setpoint tuple

$$
\mathbf{s}(t) =
\bigl(\mathbf{r}(t),\ \dot{\mathbf{r}}(t),\ \ddot{\mathbf{r}}(t),\ \mathbf{r}^{(3)}(t)\bigr) \in \mathbb{R}^{3 \times 4},
$$

with position, velocity, acceleration and jerk in the world ENU frame, such
that:

- $\mathbf{r}(t)$ converges to $\mathbf{w}_{0}$ in finite time and stays
  there, OR — when the multi-waypoint mode is active — passes through a
  neighbourhood of $\mathbf{w}_{0}$ with a non-zero crossing speed dictated
  by the geometry of the next turn at $\mathbf{w}_{0}$.
- $\Vert \dot{\mathbf{r}}(t) \Vert \le v_{\max}$,
  $\Vert \ddot{\mathbf{r}}(t) \Vert \le a_{\max}$, and the per-axis jerk
  satisfies $|r^{(3)}_{j}(t)| \le j_{\max}$ for $j \in \{x, y, z\}$.
- The three axes reach the target *simultaneously*, so the path between
  $\mathbf{w}_{-1}$ and $\mathbf{w}_{0}$ remains a straight line in
  configuration space (no axis-induced curvature).
- Optionally, when the vehicle's position $\mathbf{p}$ lags behind
  $\mathbf{r}$, the trajectory time is *stretched* so the generator waits
  for the vehicle, gated by an upper bound on the tracking error.

The generator is **incremental and greedy**: it carries an internal
state $(\mathbf{r}, \dot{\mathbf{r}}, \ddot{\mathbf{r}})$ between calls
and re-derives the active S-curve phase at every tick from a small set
of analytically computable durations. No optimization problem is solved
and no global polynomial spline is fit.

The initial state $(\mathbf{r}(0), \dot{\mathbf{r}}(0),
\ddot{\mathbf{r}}(0))$ is a parameter of the problem and defaults to
$(\mathbf{p}_0, \mathbf{0}, \mathbf{0})$ with $\mathbf{p}_0$ the initial
vehicle position. The caller may inject non-zero values through the
`velocity` and `acceleration` fields of the first waypoint (one-shot
API; see Section 9 for the outer multi-waypoint driver) or through the
extended `reset(...)` overload (streaming API).
A non-zero $\dot{\mathbf{r}}(0)$ is the typical use case (chaining a
trajectory onto a vehicle already in flight without a velocity
discontinuity at the join). A non-zero $\ddot{\mathbf{r}}(0)$ is
accepted for API symmetry but, due to the reactive nature of the
integrator, is not propagated to the first emitted setpoint: the
post-trajectory branch (no S-curve phase active yet) emits zero
acceleration on the first tick. The seeded acceleration is still
recorded internally and shapes subsequent ticks.

## 2. Velocity-setpoint construction

Per tick, the generator turns the spatial input
$\{\mathbf{w}_{-1}, \mathbf{w}_{0}, \mathbf{w}_{+1}\}$ into a single
velocity setpoint $\mathbf{v}_{\mathrm{sp}} \in \mathbb{R}^{3}$ that is
then fed to the per-axis profiles of Section 4. This is the **first**
operation each tick and is implemented in
`computeVelocitySetpoint{,Multi}` inside
[`trajectory_generator_3d.cpp`](src/trajectory_generator_3d.cpp).

### 2.1 Single-waypoint mode

The current target is $\mathbf{w}_{0}$. The position error is
$\mathbf{e} = \mathbf{w}_{0} - \mathbf{r}$ and its norm is the *braking
distance*. A feasible speed is bounded by

$$
v^{\star}
= \min\!\Bigl(
    v_{\max},\;
    v_{\mathrm{brake}}\bigl(\Vert\mathbf{e}\Vert,\, j_{\max},\, a_{\max},\, v_{\mathrm{exit}} = 0\bigr)
  \Bigr),
$$

where $v_{\mathrm{brake}}$ is the maximum speed that still allows
decelerating to $v_{\mathrm{exit}}$ over a remaining distance $d$, modelled
as a constant-acceleration phase with a $2 a / j$ jerk-ramp delay [4]:

$$
v_{\mathrm{brake}}^{2} \;=\; v_{\mathrm{exit}}^{2}
+ 2\, a\, \Bigl( d - v_{\mathrm{brake}} \cdot \tfrac{2 a}{j} \Bigr).
$$

Solving for $v_{\mathrm{brake}}$ yields the closed form used in PX4
`computeMaxSpeedFromDistance` [1]. The instantaneous velocity setpoint is
then

$$
\mathbf{v}_{\mathrm{sp}} = v^{\star}\, \widehat{\mathbf{e}},
\qquad \widehat{\mathbf{e}} = \mathbf{e} / \Vert \mathbf{e} \Vert,
$$

with feedforward velocity (when provided) added component-wise and the
result clamped to the 3D norm $v_{\max}$ before integration.

### 2.2 Multi-waypoint mode

When `acceptance_radius > 0` and the triplet has at least one non-trivial
extension, two additional pieces of geometry are introduced.

**(a) Corner-speed cap.** The waypoint $\mathbf{w}_{0}$ is treated as the
apex of a circular arc tangent to both segments $(\mathbf{w}_{-1},
\mathbf{w}_{0})$ and $(\mathbf{w}_{0}, \mathbf{w}_{+1})$. With the V-shape
internal angle

$$
\alpha \;=\; \operatorname{acos}\!\Bigl(
  \widehat{(\mathbf{w}_{-1} - \mathbf{w}_{0})} \cdot
  \widehat{(\mathbf{w}_{+1} - \mathbf{w}_{0})}
\Bigr),
$$

the maximum tangential speed compatible with a centripetal acceleration
$g_{\mathrm{traj}}\, a_{\max}$ over an acceptance radius $d$ is

$$
v_{\mathrm{corner}} \;=\; \sqrt{g_{\mathrm{traj}}\, a_{\max}\, d\, \tan(\alpha/2)},
$$

with the convention $\alpha = \pi$ ⇒ $v_{\mathrm{corner}} = +\infty$
(straight extension, no curvature) and $\alpha = 0$ ⇒
$v_{\mathrm{corner}} = 0$ (180° reversal, must stop). The dimensionless
gain $g_{\mathrm{traj}} \ge 0$ is the user-tunable `trajectory_gain`.

**(b) Backward sweep.** With the corner cap defined, the speed permitted
*at the start of segment $(\mathbf{w}_{-1}, \mathbf{w}_{0})$* is

$$
v_{\mathrm{sweep}}
= v_{\mathrm{brake}}\!\Bigl(
    \Vert \mathbf{w}_{0} - \mathbf{w}_{-1} \Vert,\,
    j_{\max},\, a_{\max},\,
    v_{\mathrm{exit}} = \min\bigl( v_{\mathrm{corner}},\, v_{\mathrm{exit},+1},\, v_{\max} \bigr)
  \Bigr),
$$

where $v_{\mathrm{exit},+1}$ is the speed permitted at $\mathbf{w}_{0}$
*after* propagating one more segment back from $\mathbf{w}_{+1}$. For the
$N = 3$ triplet the recursion is fully unrolled:

$$
\begin{aligned}
v_{\mathrm{exit},+1}
  &= v_{\mathrm{brake}}\bigl(\Vert \mathbf{w}_{+1} - \mathbf{w}_{0} \Vert,\, j_{\max},\, a_{\max},\, v_{\mathrm{exit,target}}\bigr), \\
v_{\mathrm{sweep}}
  &= v_{\mathrm{brake}}\bigl(\Vert \mathbf{w}_{0} - \mathbf{w}_{-1} \Vert,\, j_{\max},\, a_{\max},\, \min(v_{\mathrm{corner}}, v_{\mathrm{exit},+1}, v_{\max})\bigr).
\end{aligned}
$$

The post-target exit speed $v_{\mathrm{exit,target}}$ is set by the
fly-through discriminator of Section 2.2(e); the legacy choice $v_{\mathrm{exit,target}} = 0$
forces every triplet to brake to a halt at $\mathbf{w}_{0}$, which is
the right behaviour only when $\mathbf{w}_{0}$ is the last waypoint of
the user's path. The instantaneous velocity setpoint cap is then
$v^{\star} = \min(v_{\max}, v_{\mathrm{sweep}})$.

**(c) L1 corner-cut crossing point.** When the vehicle's velocity is
already mis-aligned with the direction to $\mathbf{w}_{0}$ (i.e. it is
*turning*), the target of the unsmoothed velocity command is replaced by
an *L1 crossing point* on the segment
$(\mathbf{w}_{-1}, \mathbf{w}_{0})$. With

$$
\widehat{\mathbf{u}} = \widehat{(\mathbf{w}_{0} - \mathbf{w}_{-1})},
\qquad
\mathbf{q} = \mathbf{w}_{-1} + \bigl((\mathbf{p} - \mathbf{w}_{-1}) \cdot \widehat{\mathbf{u}}\bigr)\, \widehat{\mathbf{u}},
$$

the closest point on the line is $\mathbf{q}$, the cross-track error is
$\eta = \Vert \mathbf{q} - \mathbf{p} \Vert$, and the L1 point is advanced
by $\sqrt{L_1^{2} - \eta^{2}}$ along $\widehat{\mathbf{u}}$ (with $L_1
= \max(d, L_{\min})$, $L_{\min} = 5$ m as in PX4 [2]). The crossing point
is *not* clipped to the segment end: extending past $\mathbf{w}_{0}$ is
exactly what keeps the trajectory pulled along the local path even after
reaching the target's neighbourhood, so the caller's external waypoint
rotation engages naturally.

The turn detector is

$$
\text{turning}
\;\Longleftrightarrow\;
\begin{cases}
\Vert \dot{\mathbf{r}} \Vert > v_{\mathrm{turn,min}}, \\
\widehat{\dot{\mathbf{r}}} \cdot \widehat{(\mathbf{w}_{0} - \mathbf{p})} < c_{\mathrm{turn}}, \\
\Vert \mathbf{w}_{0} - \mathbf{p} \Vert > d,
\end{cases}
$$

with thresholds $v_{\mathrm{turn,min}} = 0.2$ m/s and $c_{\mathrm{turn}} =
0.98$ ported from PX4 [2]. When `turning` is false, the crossing point
collapses to $\mathbf{w}_{0}$.

**(d) In-turn speed lock.** While `turning` is true, the speed cap is
further bounded by the previous-tick value $v_{\mathrm{prev}}^{\star}$.
This prevents the trajectory from re-accelerating in the middle of the
corner — a subtle but observable artefact when the vehicle's actual
position briefly lags and the L1 point is recomputed.

**(e) Fly-through vs stop discriminator.** The braking-distance cap of
Section 2.1 and the backward sweep of Section 2.2(b) both need a target speed
$v_{\mathrm{exit,target}}$ at $\mathbf{w}_{0}$. PX4 [2] derives this
externally from the navigator's mission state (fly-through vs stop). To
keep the library agnostic to any waypoint manager, the choice is folded
into the existing collapsed-triplet convention used at the end of the
path:

$$
v_{\mathrm{exit,target}}
\;=\;
\begin{cases}
0
  & \text{if } \mathbf{w}_{+1} = \mathbf{w}_{0}
    \quad (\text{stop: caller signalled "this is the last waypoint"}), \\[4pt]
\min\bigl( v_{\mathrm{corner}},\, v_{\max} \bigr)
  & \text{otherwise}
    \quad (\text{fly-through: } \mathbf{w}_{0} \text{ is intermediate}).
\end{cases}
$$

The corner cap $v_{\mathrm{corner}}$ used here is the same one defined
in Section 2.2(a). For a collinear straight-line stretch the V-shape angle is
$\alpha = \pi$, so $v_{\mathrm{corner}} = +\infty$ and
$v_{\mathrm{exit,target}} = v_{\max}$: the trajectory crosses every
intermediate waypoint at full cruise speed and only decelerates at the
final one. For a sharp turn $v_{\mathrm{corner}}$ shrinks accordingly,
recovering the geometric corner braking of Section 2.2(a) without additional
tuning.

This discriminator is what lets the library reproduce PX4's
fly-through behaviour for an arbitrary $N$-waypoint list without
adding a per-waypoint flag to the public API: the one-shot
@ref TrajectoryGenerator already produces a collapsed triplet at the
end of the path (and only there), and the streaming @ref WaypointTrajectoryController
example follows the same convention. A caller who needs to stop at an
intermediate waypoint can express it by splitting the path into two
generate() calls.

### 2.3 Degeneracy guarantees

When `acceptance_radius ≤ 0` or the triplet collapses to a single
waypoint (`prev = target = next`), the multi-waypoint branch is bypassed
and the single-waypoint formula of Section 2.1 is applied verbatim,
guaranteeing **bit-for-bit regression** with the legacy single-waypoint
behaviour.

## 3. Tracking-error time stretch

Once the velocity setpoint of Section 2 is known and **before** the
per-axis polynomial integration of Section 4.3 advances the state, the
generator computes a scalar gate $\beta \in [0, 1]$ that optionally slows
the trajectory to wait for a lagging vehicle. With the emitted internal
position $\mathbf{r}$, the vehicle's reported position $\mathbf{p}$, and
the user-tunable threshold $\eta_{\max}$ (the `max_tracking_error`
parameter):

$$
\beta \;=\;
\begin{cases}
1 & \text{if } (\mathbf{r} - \mathbf{p}) \cdot \dot{\mathbf{r}} < 0, \\
1 - \min\!\Bigl(1,\, \tfrac{\Vert \mathbf{r} - \mathbf{p} \Vert}{\eta_{\max}}\Bigr) & \text{otherwise},
\end{cases}
$$

and the per-axis polynomial integration of Section 4.3 advances local
time by $\beta \Delta t$ instead of $\Delta t$. The first branch — vehicle
ahead of the trajectory — disables the stretch so the generator does not
stall when the follower over-shoots. Setting $\eta_{\max} \le 0$ disables
the gate entirely.

## 4. Per-axis trajectory representation

With the velocity setpoint of Section 2 and the time-stretch factor of
Section 3 in hand, the generator drives each ENU axis along a
**three-phase** S-curve. This is the actual numerical work each tick;
implementation in
[`velocity_profile_1d.cpp`](src/velocity_profile_1d.cpp).

### 4.1 Three-phase polynomial

For each ENU axis $j \in \{x, y, z\}$, the generator drives the velocity
$\dot{r}_j$ from its current value $v_{0,j}$ toward an instantaneous
**velocity setpoint** $v_{\mathrm{sp},j}$ along a **three-phase** profile
characterized by phase durations $T_{1,j}, T_{2,j}, T_{3,j} \ge 0$:

| Phase | Jerk | Duration | Effect |
|-------|------|----------|--------|
| 1 | $+\sigma_j\, j_{\max}$ | $T_{1,j}$ | accelerate the acceleration |
| 2 | $0$                    | $T_{2,j}$ | constant acceleration phase |
| 3 | $-\sigma_j\, j_{\max}$ | $T_{3,j}$ | decelerate the acceleration |

Here $\sigma_j \in \{-1, 0, +1\}$ is the *jerk direction* needed to drive
$\dot{r}_j$ from $v_{0,j}$ to $v_{\mathrm{sp},j}$, computed by comparing
$v_{\mathrm{sp},j}$ to the velocity that would result from immediately
applying the maximum opposite jerk to cancel the current acceleration:

$$
\sigma_j \;=\; \operatorname{sign}\!\bigl( v_{\mathrm{sp},j} - v_{a=0,j} \bigr),
\quad\text{where}\quad
v_{a=0,j} = v_{0,j} + a_{0,j}\, t_{a=0,j} + \tfrac{1}{2}\, j_{a=0,j}\, t_{a=0,j}^{2},
$$

with $j_{a=0,j} = -\operatorname{sign}(a_{0,j})\, j_{\max}$ and
$t_{a=0,j} = -a_{0,j} / j_{a=0,j}$.

### 4.2 Closed-form phase durations

For a positive jerk direction (the negative case is symmetric), the
durations are obtained analytically by solving the polynomial system
formed by integrating the constant-jerk dynamics over each phase and
imposing $\dot{r}_j(T_{1}+T_{2}+T_{3}) = v_{\mathrm{sp},j}$ and
$\ddot{r}_j(T_{1}+T_{2}+T_{3}) = 0$:

$$
\Delta v = v_{\mathrm{sp},j} - v_{0,j},
$$

$$
T_{1,j} = \frac{ - a_{0,j} + \tfrac{1}{2}\sqrt{2 a_{0,j}^{2} + 4\, \sigma_j\, j_{\max}\, \Delta v} }{ \sigma_j\, j_{\max} },
$$

then $T_{1,j}$ is **clipped** so the peak acceleration after phase 1 stays
within $[-a_{\max}, +a_{\max}]$:

$$
T_{1,j} \leftarrow \min\!\Bigl( T_{1,j},\; \tfrac{a_{\max} - a_{0,j}}{\sigma_j\, j_{\max}} \Bigr) \quad \text{(or symmetric for the negative branch)}.
$$

The decreasing-acceleration phase has duration

$$
T_{3,j} = \frac{a_{0,j}}{\sigma_j\, j_{\max}} + T_{1,j},
$$

so that the acceleration reaches zero at the end of phase 3, and the
constant-acceleration phase consumes whatever time is needed to actually
hit $\Delta v$:

$$
T_{2,j} = \frac{\Delta v - \frac{1}{2}\bigl(T_{1,j}^{2} - T_{3,j}^{2}\bigr) \sigma_j j_{\max} - (T_{1,j} + T_{3,j}) a_{0,j}}{a_{0,j} + \sigma_j\, j_{\max}\, T_{1,j}}.
$$

All three durations are clipped to be non-negative. This formulation is the
classical jerk-limited "S-curve" used in industrial motion control [3] and
is the same one PX4's `VelocitySmoothing` uses [1].

### 4.3 Polynomial integration

Within each phase, the state evolves analytically over local time
$\tau = \beta \Delta t$ (the time-stretch factor of Section 3 scales
the local integration step):

$$
\begin{aligned}
\ddot{r}_{j}(\tau) &= a_{0,j} + j_\phi\, \tau, \\
\dot{r}_{j}(\tau)  &= v_{0,j} + a_{0,j}\, \tau + \tfrac{1}{2}\, j_\phi\, \tau^{2}, \\
r_{j}(\tau)       &= r_{0,j} + v_{0,j}\, \tau + \tfrac{1}{2}\, a_{0,j}\, \tau^{2} + \tfrac{1}{6}\, j_\phi\, \tau^{3},
\end{aligned}
$$

where $j_\phi \in \{+\sigma_j j_{\max}, 0, -\sigma_j j_{\max}\}$ is the
phase-constant jerk and $\tau$ is the local time elapsed inside the phase.
Crucially, on every tick the generator integrates **from the snapshot
state captured at the last `updateDurations` call**, not from the previous
emitted state. This pull-from-snapshot strategy keeps the analytical
trajectory exact and prevents floating-point drift across many ticks.

## 5. Hard 3D-norm clamps and re-injection

The per-axis profile of Section 4 enforces $j_{\max}$ exactly (per axis,
by construction), and the velocity-setpoint solver of Section 2 keeps
$v_{\max}$, $a_{\max}$ as **soft** caps. Because PX4-style
synchronization is axis-aligned and the bounds are 3D Euclidean norms,
isolated profile transitions can momentarily exceed those norms by a
small amount. To make the emitted setpoints strictly feasible, after
integration the generator post-clamps:

$$
\dot{\mathbf{r}}\, \leftarrow\, \dot{\mathbf{r}}\, \cdot\,
\min\!\Bigl(1,\, \tfrac{v_{\max}}{\Vert \dot{\mathbf{r}} \Vert}\Bigr),
\qquad
\ddot{\mathbf{r}}\, \leftarrow\, \ddot{\mathbf{r}}\, \cdot\,
\min\!\Bigl(1,\, \tfrac{a_{\max}}{\Vert \ddot{\mathbf{r}} \Vert}\Bigr),
$$

and *re-injects* the clamped values back into the per-axis profile state
so subsequent ticks integrate from a feasible initial state. This is the
identifying departure from PX4 (whose bounds are per-axis and need no
post-clip): the price is a momentary dip in $C^{2}$ continuity exactly
when the clamp activates; the gain is strict 3D-norm feasibility on the
emitted setpoints. The clamp is conservatively kept for both single- and
multi-waypoint paths.

## 6. Axis time-synchronization

The S-curve solver of Section 4 produces three independent profiles
whose total durations $T_{j} = T_{1,j} + T_{2,j} + T_{3,j}$ generally
differ. Once the post-clamp of Section 5 has re-injected the clamped
state, the generator synchronizes the three axes so that the **next**
tick re-integrates from a consistent total-time horizon:

1. Compute $T_{\max} = \max_{j} T_{j}$, the duration of the longest axis.
2. For each axis with $T_{j} < T_{\max}$, recompute $T_{1,j}, T_{2,j},
   T_{3,j}$ subject to the additional constraint
   $T_{1,j} + T_{2,j} + T_{3,j} = T_{\max}$.

The constrained-time inversion uses the same polynomial relations as
Section 4 but with $T_{1,j}$ obtained from the larger quadratic that
incorporates $T_{\max}$ (PX4 `computeT1(T123, ...)` [1]). The other two
durations follow algebraically.

The net effect is that all three axes complete their respective
$\Delta v_{j}$ in exactly the same time, so the velocity vector
$\dot{\mathbf{r}}$ rotates only because of corner geometry (multi-waypoint
mode) and never because of unsynchronized axis dynamics. This step is
the last thing each tick does before returning the setpoints.

## 7. Output and timing

Each tick the generator emits

$$
(\mathbf{r}, \dot{\mathbf{r}}, \ddot{\mathbf{r}}, \mathbf{r}^{(3)})
\in \mathbb{R}^{3 \times 4},
$$

plus the unsmoothed velocity setpoint $\mathbf{v}_{\mathrm{sp}}$ used as a
diagnostic. Position is $C^{2}$ in nominal operation (post-clip
discontinuities aside), velocity is $C^{1}$, acceleration is $C^{0}$, and
jerk is piecewise constant with at most three transitions per axis per
tick.

The default operating frequency is 50 Hz ($\Delta t = 0.02$ s), matching
typical position-control loops; the generator is, however, unitless in
$\Delta t$ and any positive value works.

## 8. Conventions and numerical units

- World frame: ENU (x east, y north, z up). Climbing means Z getting more
  positive.
- All quantities in SI: position [m], velocity [m/s], acceleration
  [m/s²], jerk [m/s³], time [s], angle [rad].
- Disable semantics: any kino-dynamic limit $\le 0$ is treated as
  *disabled*. If $v_{\max}$, $a_{\max}$ and $j_{\max}$ are all disabled
  simultaneously the generator throws, since no feasible velocity command
  can be derived.
- The streaming `WaypointTrajectoryController` does not own a waypoint
  manager. The caller advances the active triplet on arrival; the
  recommended criterion is
  $\Vert \mathbf{p} - \mathbf{w}_{0} \Vert < \max(d, d_{\min})$, with the
  safety floor $d_{\min}$ exposed by the one-shot wrapper (see Section 9 below)
  as `SimulatorConfig::advance_radius_min`.

## 9. One-shot multi-waypoint driver

The `TrajectoryGenerator` class wraps the streaming controller into an
offline planner that accepts a full waypoint list, runs the integrator
forward at fixed step $\Delta t$, and emits the per-tick samples plus
their bookkeeping. Three knobs of `SimulatorConfig` and two of
`TrajectoryParameters` govern the outer loop; they are independent of
the per-tick mathematics of Sections 2–7.

**Active-waypoint advance.** While iterating, the loop maintains an
index `active` into the waypoint vector and feeds the streaming
controller the triplet $\{\mathbf{w}_{\text{active}-1},
\mathbf{w}_{\text{active}}, \mathbf{w}_{\text{active}+1}\}$. The active
index is incremented when

$$
\Vert \mathbf{p} - \mathbf{w}_{\text{active}} \Vert  < r_{\text{adv}},
\qquad r_{\text{adv}} = \max(d,\; r_{\text{floor}}),
$$

where $d$ is the user's `params.acceptance_radius` and $r_{\text{floor}}
=$ `simulator.advance_radius_min` (default 0.1 m, $\le 0$ disables the
floor entirely). Decoupling the floor from $d$ lets a caller use a
small acceptance radius for the Section 2.2(c) L1 lookahead while still
controlling, independently, how close the trajectory passes to each
intermediate waypoint at the moment of segment switch.

A tighter floor admits closer geometric tracking of the waypoint, but
the trajectory dynamics themselves bound how close the path can come
without slowing down. By centripetal feasibility the corner-speed cap
of Section 2.2(a) gives

$$
v_{\text{cross}} \le \sqrt{g_{\text{traj}}\, a_{\max}\, r_{\text{adv}}\, \tan(\alpha/2)},
$$

so reducing $r_{\text{floor}}$ at fixed $g_{\text{traj}}$ proportionally
reduces the speed at which the trajectory can sustain the corner; if
the caller wants to preserve $v_{\text{cross}} \approx v_{\max}$ at small
$r_{\text{floor}}$ the lateral acceleration budget must be raised by
increasing `trajectory_gain` (or $a_{\max}$).

**Termination.** The loop exits once the active waypoint is the last
one and the integrator has settled:

$$
\bigl(\text{active}+1 = n_{\text{wp}}\bigr)
\;\wedge\;
\Vert \mathbf{p} - \mathbf{w}_{n_{\text{wp}}-1} \Vert < r_{\text{end}}
\;\wedge\;
\Vert \dot{\mathbf{r}} \Vert < v_{\text{settle}},
$$

where $v_{\text{settle}} =$ `simulator.settle_velocity` and the
end-position tolerance is

$$
r_{\text{end}} =
\begin{cases}
\texttt{params.final\_acceptance\_radius} & \text{if } > 0 \\
r_{\text{adv}}                            & \text{otherwise}.
\end{cases}
$$

Decoupling $r_{\text{end}}$ from $r_{\text{adv}}$ is what lets the
caller keep a generous L1 corner-cut (large $d$) on intermediate
waypoints while still requiring a tight stop at the goal. The
fly-through-vs-stop discriminator of Section 2.2(e) is *already* automatic at
this point: with `next == target` (collapsed triplet on the last
segment) the multi-waypoint controller plans
$v_{\text{exit,target}} = 0$, so braking-to-rest at the final waypoint
is internal to the streaming controller and the outer loop only checks
the residual position/velocity.

**Hard horizon.** A simulation that does not satisfy the termination
predicate within `simulator.max_simulation_time` seconds (in trajectory
time, not wall-clock) returns failure. This is purely a fail-safe for
infeasible inputs (waypoint outside the kinodynamic envelope, mutually
inconsistent limits), not a design constraint of the algorithm.

**Spline post-fit.** After a successful run, the wrapper exposes a
piecewise-polynomial `Spline` obtained by least-squares fitting one
segment per pair of adjacent waypoints with order
`simulator.polynomial_order` (default 16). The spline is purely a
serialization artefact: callers that re-evaluate the trajectory should
prefer `evaluate(t)`, which interpolates the per-tick samples directly
and reproduces the integrator output exactly at sample times.

## 10. Summary of the algorithm

The algorithm can be stated as:

> Given the current state $(\mathbf{r}, \dot{\mathbf{r}}, \ddot{\mathbf{r}})$,
> a waypoint triplet $\{\mathbf{w}_{-1}, \mathbf{w}_{0}, \mathbf{w}_{+1}\}$,
> kino-dynamic limits $(v_{\max}, a_{\max}, j_{\max})$, an acceptance
> radius $d$ and a trajectory gain $g_{\mathrm{traj}}$, on every tick of
> period $\Delta t$:
>
> 1. **Velocity setpoint**: compute $\mathbf{v}_{\mathrm{sp}}$ as the
>    minimum of $v_{\max}$, the braking-distance cap and (if $d > 0$)
>    the corner cap from a backward sweep on the triplet, with the
>    direction taken from an L1 crossing point on the active segment
>    when the vehicle is turning.
> 2. **Time stretch**: compute the tracking-error gate
>    $\beta \in [0, 1]$ from the lag of the vehicle behind the
>    trajectory, so the trajectory waits for a slow follower.
> 3. **S-curve per axis**: derive the jerk-limited durations
>    $(T_{1}, T_{2}, T_{3})$ that drive each axis velocity to
>    $v_{\mathrm{sp},j}$ in minimum time and integrate one
>    $\beta \Delta t$ analytically.
> 4. **3D-norm post-clamp**: clamp the emitted velocity and acceleration
>    to the global 3D-norm bounds and re-inject the clamped state into
>    the per-axis profiles so the next tick is feasible.
> 5. **Axis synchronization**: extend the durations of the two shorter
>    axes to $T_{\max}$, preserving the straight-line path of the
>    vehicle in configuration space and preparing the consistent state
>    for the next tick.
>
> Setpoints from steps 3–4 are emitted to the controller and used as
> the reference for the actual flight-control loop. No optimization
> problem is solved; all phase durations are obtained in closed form.

## 11. Configurable parameters

All parameters share a uniform "disable on $\le 0$" semantics for the
kino-dynamic bounds. At least one of `max_speed`, `max_acceleration`
or `max_jerk` must be enabled, otherwise `WaypointTrajectoryController`
throws `std::invalid_argument` (the streaming API) and the one-shot
`TrajectoryGenerator::generate()` returns `false` after propagating
the same exception.

### 11.1 `TrajectoryParameters`

Defined in
[`parameters.hpp`](include/trajectory_generator_jerk_limited/parameters.hpp#L56-L85).
Used by both the streaming `WaypointTrajectoryController` and the
one-shot `TrajectoryGenerator`.

| Field | Type | Default | Units | Affects | Effect |
|---|---|---|---|---|---|
| `max_speed` | `double` | `0.0` | m/s | velocity setpoint (Section 2), post-clamp (Section 5) | 3D-norm cap on $\dot{\mathbf r}$. Disabled when `<= 0`. The one-shot `generate(waypoints, max_speed)` overrides this for that call. |
| `max_acceleration` | `double` | `0.0` | m/s² | T1 saturation (Section 4.2), brake cap (Section 2.1), corner cap (Section 2.2.a), post-clamp (Section 5) | 3D-norm cap on $\ddot{\mathbf r}$ and the acceleration plateau in phase T2 of the S-curve. Disabled when `<= 0`. |
| `max_jerk` | `double` | `0.0` | m/s³ | S-curve phases T1/T3 (Section 4.1), brake cap (Section 2.1) | Per-axis jerk used in phases 1 and 3. Defines how fast the acceleration ramps up and down. Disabled when `<= 0`. |
| `max_tracking_error` | `double` | `0.0` | m | time-stretch (Section 3) | Threshold above which the trajectory slows down to wait for a lagging vehicle. Disabled when `<= 0`. |
| `acceptance_radius` | `double` | `0.0` | m | L1 cut (Section 2.2.c), corner cap (Section 2.2.a), backward sweep (Section 2.2.b), outer-loop advance (Section 9) | Multi-waypoint corner radius. `<= 0` collapses the multi-waypoint branch entirely (Section 2.3, regression-safe). |
| `final_acceptance_radius` | `double` | `0.0` | m | outer-loop termination (Section 9) | Tight position tolerance for declaring arrival at the **last** waypoint, decoupled from the L1 corner-cut radius. `<= 0` reuses $\max(d,\, r_{\text{floor}})$. |
| `trajectory_gain` | `double` | `1.0` | — | corner cap (Section 2.2.a), backward sweep (Section 2.2.b) | Dimensionless scale on `max_acceleration` inside the corner-speed formula $\sqrt{g\, a\, d\, \tan(\alpha/2)}$. Values $> 1$ allow tighter corners at higher speed; values $< 1$ widen them. Clamped to $\ge 0$ at setter time. Has no effect on single-waypoint calls. |

### 11.2 `SimulatorConfig`

Defined in
[`trajectory_generator.hpp`](include/trajectory_generator_jerk_limited/trajectory_generator.hpp#L71-L105).
Used **only** by the one-shot `TrajectoryGenerator`. The streaming API
ignores these fields.

| Field | Type | Default | Units | Affects | Effect |
|---|---|---|---|---|---|
| `step_dt` | `double` | `0.01` | s | integration (Section 4.3), outer-loop tick (Section 9) | Internal simulation step. Must be `> 0`. |
| `max_simulation_time` | `double` | `600.0` | s | outer-loop termination (Section 9) | Hard horizon. Failsafe against infeasible inputs (waypoint outside the kino-dynamic envelope). Exceeding it makes `generate()` return `false`. |
| `settle_velocity` | `double` | `0.05` | m/s | outer-loop termination (Section 9) | "At-rest" threshold once the last waypoint is reached. Stops the simulation early. |
| `advance_radius_min` | `double` | `0.1` | m | outer-loop active-waypoint advance (Section 9) | Floor applied to $\max(d, \cdot)$ when deciding to advance the active index. `<= 0` disables the floor (advance happens strictly within `acceptance_radius`); values below `step_dt · max_speed` are ill-conditioned (the simulated drone may overshoot before the advance check fires). |
| `polynomial_order` | `int` | `16` | coefs. | `spline()` post-fit only (Section 9) | Polynomial degree is `polynomial_order - 1`. The spline is a **serialisation artefact**: callers re-evaluating the trajectory should prefer `evaluate(t)`, which interpolates the per-tick samples and reproduces the integrator output exactly at sample times. |

### 11.3 Tuning intuitions

- **Smoother trajectories**: lower `max_jerk` (longer T1/T3 ramps) or
  `max_acceleration` (longer cruise plateau in T2).
- **Tighter corners**: raise `trajectory_gain` (allows higher
  centripetal acceleration before the corner cap bites). Equivalent to
  raising the lateral-acceleration budget.
- **Pass close to intermediate waypoints**: lower `acceptance_radius`
  and, in the one-shot API, lower `simulator.advance_radius_min`. By
  the centripetal feasibility relation of Section 9, a smaller radius
  proportionally reduces the maximum speed at which the trajectory can
  sustain the corner — compensate by raising `trajectory_gain` if
  necessary.
- **Stop tightly at the final goal regardless of intermediate
  corner-cut radius**: set `final_acceptance_radius` to a small
  positive value while keeping `acceptance_radius` generous.
- **Wait for a lagging vehicle**: enable
  `max_tracking_error > 0`. The trajectory time slows down as
  $\Vert \mathbf{r} - \mathbf{p} \Vert$ approaches the threshold and
  pauses entirely at it.

## 12. Subtle behaviours worth knowing

These are easy to overlook but matter when tuning or debugging:

- **Seeded acceleration is not emitted on the first tick.** The
  streaming API accepts `Waypoint::velocity` and
  `Waypoint::acceleration` fields (or the `reset(...)` overload) to
  chain a trajectory onto a vehicle already in flight. The seeded
  velocity is honoured on the first emitted sample; the seeded
  acceleration is recorded internally but the first tick emits
  $\ddot{\mathbf r} = \mathbf 0$ because the integrator restarts the
  local phase clock. Subsequent ticks reflect the seed normally.

- **`max_speed` argument to one-shot `generate()` overrides
  `params.max_speed`.** This is intentional: the one-shot API treats
  `max_speed` as a per-call cruise speed, while the rest of the
  config persists across calls.

- **`acceptance_radius <= 0` disables every multi-waypoint
  refinement.** Not just the L1 corner-cut, but also the corner-speed
  cap, the backward sweep, the in-turn speed lock and the fly-through
  discriminator. The trajectory falls back to the single-waypoint
  formula and stops at every waypoint (Section 2.3).

- **L1 lookahead has a hard 5 m floor.** Even if `acceptance_radius`
  is set to 0.1 m, the L1 crossing point uses
  $L_1 = \max(d, 5\text{ m})$. This decouples the geometric corner
  radius (small, used for the corner-speed cap of Section 2.2.a) from the L1
  guidance radius (always at least 5 m, used to keep the trajectory
  pulled along the segment, Section 2.2.c).

- **The 3D-norm post-clamp can momentarily break $C^2$ continuity.**
  PX4-style axis synchronisation is axis-aligned, so the 3D norm of
  the integrated state can briefly exceed `max_speed` or
  `max_acceleration` during an axis transition. The post-clamp scales
  the state back into the envelope and **re-injects** the scaled
  values into the per-axis profiles so the next tick is feasible. The
  cost is a one-tick discontinuity in $\mathbf{r}^{(3)}$ exactly when
  the clamp activates. This trade-off is deliberate (Section 5).

- **The tracking-error stretch uses the trajectory state, not the
  vehicle.** The lag direction $(\mathbf r - \mathbf p) \cdot
  \dot{\mathbf r}$ is computed against the internally integrated
  position $\mathbf r$, so the gate is causal: even if the vehicle is
  wildly off, the command does not depend on the vehicle's future
  trajectory.

- **`isTurning` requires misaligned velocity, not triplet geometry.**
  On the very first segment (velocity aligned with the direction to
  the target by construction) the turn detector returns false, so the
  corner cap is not visible until the caller rotates the triplet and
  the new direction misaligns from the current velocity. Same
  semantics as PX4 `Navigator` over `PositionSmoothing`.

- **To stop at an intermediate waypoint, split the path.** The
  fly-through-vs-stop discriminator of Section 2.2(e) is automatic, so the
  public API does not expose a per-waypoint `stop` flag. Callers
  wanting an intermediate stop should issue two `generate()` calls.

## References

[1] PX4 Autopilot,
*VelocitySmoothing*: jerk-limited S-curve velocity profile with
analytical phase-duration inversion.
Source: [`src/lib/motion_planning/VelocitySmoothing.{hpp,cpp}`](https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/motion_planning).

[2] PX4 Autopilot,
*PositionSmoothing*: 3D extension of `VelocitySmoothing` with
braking-distance cap, L1 corner-cut, backward-sweep corner-speed and
tracking-error time stretch.
Source: [`src/lib/motion_planning/PositionSmoothing.{hpp,cpp}`](https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/motion_planning).

[3] Kröger, T., & Wahl, F. M. (2009). Online trajectory generation: Basic concepts for instantaneous
reactions to unforeseen events. IEEE Transactions on Robotics, 26(1), 94-111.

[4] PX4 Autopilot,
*TrajMath* helpers (`computeMaxSpeedFromDistance`,
`computeMaxSpeedInWaypoint`, `computeBrakingDistanceFromVelocity`).
Source: [`src/lib/mathlib/math/TrajMath.hpp`](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/mathlib/math/TrajMath.hpp).
