"""ORCA (optimal reciprocal collision avoidance) for drones sharing positions over the mesh.

The safety shield of the avoidance layer: whatever velocity the route (or the learned policy)
asks for, ORCA returns the closest velocity that keeps every neighbour outside the combined
separation radius for the next `horizon` seconds, assuming the neighbour does its half. Pure
Python (no numpy) so the same code runs on the Pi 5 and in the simulator. Horizontal (x, y) only:
height is kept by the route.

Port of the 2-D agent part of RVO2 (van den Berg, Guy, Snape, Lin, Manocha; Apache-2.0), with two
additions for the rescue fleet:
  * neighbour states arrive late over the mesh: positions are extrapolated by the message age and
    the separation radius grows with it (`neighbour_from_message`);
  * right-hand passing (`right_hand_preference`): on a collision course the preferred velocity is
    biased to the right, so two drones meeting head-on on the same path both pass on their right,
    as pilots and pedestrians do, instead of picking a side at random.
"""
import math

EPS = 1e-5


def _det(ax, ay, bx, by):
    return ax * by - ay * bx


class Line:
    __slots__ = ('px', 'py', 'dx', 'dy')

    def __init__(self, px, py, dx, dy):
        self.px, self.py, self.dx, self.dy = px, py, dx, dy


def neighbour_from_message(position, velocity, age_s, own_sigma_m, peer_sigma_m, min_separation_m,
                           buffer_m=2.0):
    """Neighbour for `orca_velocity` from a mesh message `age_s` old: extrapolated position and a
    combined radius that covers both GNSS errors, how far the peer may have moved meanwhile and a
    buffer for what the plain model leaves out (acceleration limits, GNSS drift between messages)."""
    age = max(0.0, age_s)
    speed = math.hypot(velocity[0], velocity[1])
    return dict(pos=(position[0] + velocity[0] * age, position[1] + velocity[1] * age),
                vel=(velocity[0], velocity[1]),
                radius=min_separation_m + own_sigma_m + peer_sigma_m + 0.5 * speed * age + buffer_m)


# The onboard separation gate (as2_rescue_runtime.swarm.pair_margin / avoidance.GATE_BUFFER_M): kept
# here so the simulator trains against exactly the radius the drones fly with (a test checks both agree).
LINK_LATENCY_S = 0.2
GATE_BUFFER_M = 1.0


def gate_margin(min_separation_m, own_sigma_m, own_speed, peer_sigma_m, peer_speed, age_s):
    """Distance the separation gate demands for a pair: the separation, both GNSS sigmas, and how far
    both may have moved since the peer's message was sent (its age + link latency)."""
    return (min_separation_m + own_sigma_m + peer_sigma_m
            + (own_speed + peer_speed) * (max(0.0, age_s) + LINK_LATENCY_S))


def neighbour_for_gate(position, velocity, age_s, own_sigma_m, own_speed, peer_sigma_m, min_separation_m,
                       buffer_m=GATE_BUFFER_M):
    """Neighbour as the onboard layer avoids it: position extrapolated by the message age, radius =
    the gate margin + a buffer, so avoidance keeps the pair outside the gate (the gate stays the last
    resort). With RTK (3 m, sigmas 0.1 m) over the base station at 1.5 m/s each: about 5 m."""
    age = max(0.0, age_s)
    speed = math.hypot(velocity[0], velocity[1])
    return dict(pos=(position[0] + velocity[0] * age, position[1] + velocity[1] * age),
                vel=(velocity[0], velocity[1]),
                radius=gate_margin(min_separation_m, own_sigma_m, own_speed, peer_sigma_m, speed, age) + buffer_m)


def time_to_closest(rel_pos, rel_vel):
    """Seconds until two drones are closest (0 if already separating) and the miss distance."""
    vv = rel_vel[0] ** 2 + rel_vel[1] ** 2
    t = 0.0 if vv < EPS else max(0.0, (rel_pos[0] * rel_vel[0] + rel_pos[1] * rel_vel[1]) / vv)
    return t, math.hypot(rel_pos[0] - rel_vel[0] * t, rel_pos[1] - rel_vel[1] * t)


def right_hand_preference(own_pos, own_vel, pref_vel, neighbours, horizon, strength=0.5):
    """Bias the preferred velocity to the right while a neighbour is on a collision course.

    A neighbour counts when it is ahead and, at the current velocities, would come within 1.5x its
    radius inside twice the horizon. The bias is applied in full from that moment (a gradual one
    let the geometry pick the side and half the head-on encounters passed on the left in the
    simulator), so ORCA's chosen side is the right one for both drones."""
    speed = math.hypot(pref_vel[0], pref_vel[1])
    if speed < EPS:
        return pref_vel
    for n in neighbours:
        rel_pos = (n['pos'][0] - own_pos[0], n['pos'][1] - own_pos[1])
        rel_vel = (own_vel[0] - n['vel'][0], own_vel[1] - n['vel'][1])
        t, miss = time_to_closest(rel_pos, rel_vel)
        ahead = rel_pos[0] * pref_vel[0] + rel_pos[1] * pref_vel[1] > 0
        if ahead and 0.0 < t < 2.0 * horizon and miss < 1.5 * n['radius']:
            # Right of the direction of travel (ENU: x east, y north) is (dy, -dx).
            ux, uy = pref_vel[0] / speed, pref_vel[1] / speed
            k = strength * speed
            return (pref_vel[0] + k * uy, pref_vel[1] - k * ux)
    return pref_vel


AREA_MARGIN_M = 1.5       # keep this far inside the planning area's edge (sim: overshoot ~0.8 m at 1.5 m/s)
AREA_HORIZON_S = 2.0      # an edge limits the velocity towards it so the margin is reached in this time


def area_walls(own_pos, shell, max_speed, margin=AREA_MARGIN_M, horizon=AREA_HORIZON_S):
    """Hard ORCA lines that keep the drone inside the planning area (polygon `shell`, any winding).

    For every edge within reach: the velocity towards it is limited to (distance - margin) /
    horizon, so the drone never crosses the margin. Nothing when the drone is outside the area
    (a transit into it from the launch point)."""
    pts = [(float(p[0]), float(p[1])) for p in shell or []]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3 or not _inside(own_pos, pts):
        return []
    walls = []
    reach = max_speed * horizon + margin
    for a, b in zip(pts, pts[1:] + pts[:1]):
        lx, ly = b[0] - a[0], b[1] - a[1]
        l2 = lx * lx + ly * ly
        t = 0.0 if l2 < EPS else max(0.0, min(1.0, ((own_pos[0] - a[0]) * lx + (own_pos[1] - a[1]) * ly) / l2))
        qx, qy = a[0] + t * lx - own_pos[0], a[1] + t * ly - own_pos[1]
        d = math.hypot(qx, qy)
        if d > reach or d < EPS:
            continue
        nx, ny = qx / d, qy / d                       # towards the edge (outwards)
        c = (d - margin) / horizon                    # allowed speed towards the edge
        walls.append(Line(c * nx, c * ny, -ny, nx))   # valid: v . n <= c
    return walls


def _inside(point, pts):
    x, y = point
    inside = False
    for (ax, ay), (bx, by) in zip(pts, pts[1:] + pts[:1]):
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def orca_velocity(own_pos, own_vel, pref_vel, neighbours, max_speed, horizon, dt, responsibility=0.5, walls=()):
    """New velocity (vx, vy) closest to `pref_vel` that ORCA allows.

    neighbours: dicts with pos (x, y), vel (vx, vy) and radius (combined separation radius, m).
    responsibility: share of the avoidance this drone takes (0.5 = reciprocal; 1.0 when the
    peer cannot be relied on to react, e.g. it is not running ORCA).
    walls: hard lines (area_walls): never relaxed; when drones and walls cannot all be satisfied
    only the drone lines are relaxed (RVO2's obstacle lines)."""
    lines = list(walls)
    hard = len(lines)
    inv_h = 1.0 / horizon
    for n in neighbours:
        rpx, rpy = n['pos'][0] - own_pos[0], n['pos'][1] - own_pos[1]
        rvx, rvy = own_vel[0] - n['vel'][0], own_vel[1] - n['vel'][1]
        dist_sq = rpx * rpx + rpy * rpy
        r = n['radius']
        r_sq = r * r
        if dist_sq > r_sq:
            wx, wy = rvx - inv_h * rpx, rvy - inv_h * rpy
            w_len_sq = wx * wx + wy * wy
            dot1 = wx * rpx + wy * rpy
            if dot1 < 0.0 and dot1 * dot1 > r_sq * w_len_sq:
                # Project on the cut-off circle.
                w_len = math.sqrt(w_len_sq)
                ux_, uy_ = wx / w_len, wy / w_len
                dx, dy = uy_, -ux_
                ux, uy = (r * inv_h - w_len) * ux_, (r * inv_h - w_len) * uy_
            else:
                # Project on the legs of the velocity obstacle.
                leg = math.sqrt(dist_sq - r_sq)
                if _det(rpx, rpy, wx, wy) > 0.0:
                    dx, dy = (rpx * leg - rpy * r) / dist_sq, (rpx * r + rpy * leg) / dist_sq
                else:
                    dx, dy = -(rpx * leg + rpy * r) / dist_sq, -(-rpx * r + rpy * leg) / dist_sq
                dot2 = rvx * dx + rvy * dy
                ux, uy = dot2 * dx - rvx, dot2 * dy - rvy
        else:
            # Already inside the radius: get out within one step.
            inv_t = 1.0 / dt
            wx, wy = rvx - inv_t * rpx, rvy - inv_t * rpy
            w_len = math.hypot(wx, wy)
            if w_len < EPS:   # same position and velocity: any separating direction will do
                wx, wy, w_len = 1.0, 0.0, 1.0
            ux_, uy_ = wx / w_len, wy / w_len
            dx, dy = uy_, -ux_
            ux, uy = (r * inv_t - w_len) * ux_, (r * inv_t - w_len) * uy_
        lines.append(Line(own_vel[0] + responsibility * ux, own_vel[1] + responsibility * uy, dx, dy))
    result = [0.0, 0.0]
    fail = _lp2(lines, max_speed, pref_vel, False, result)
    if fail < len(lines):
        _lp3(lines, hard, fail, max_speed, result)
    return result[0], result[1]


def _lp1(lines, no, radius, opt, direction_opt, result):
    line = lines[no]
    dot = line.px * line.dx + line.py * line.dy
    disc = dot * dot + radius * radius - (line.px * line.px + line.py * line.py)
    if disc < 0.0:
        return False
    sq = math.sqrt(disc)
    t_left, t_right = -dot - sq, -dot + sq
    for i in range(no):
        other = lines[i]
        den = _det(line.dx, line.dy, other.dx, other.dy)
        num = _det(other.dx, other.dy, line.px - other.px, line.py - other.py)
        if abs(den) <= EPS:
            if num < 0.0:
                return False
            continue
        t = num / den
        if den >= 0.0:
            t_right = min(t_right, t)
        else:
            t_left = max(t_left, t)
        if t_left > t_right:
            return False
    if direction_opt:
        t = t_right if opt[0] * line.dx + opt[1] * line.dy > 0.0 else t_left
    else:
        t = line.dx * (opt[0] - line.px) + line.dy * (opt[1] - line.py)
        t = min(max(t, t_left), t_right)
    result[0], result[1] = line.px + t * line.dx, line.py + t * line.dy
    return True


def _lp2(lines, radius, opt, direction_opt, result):
    if direction_opt:
        result[0], result[1] = opt[0] * radius, opt[1] * radius
    else:
        speed = math.hypot(opt[0], opt[1])
        if speed > radius:
            result[0], result[1] = opt[0] / speed * radius, opt[1] / speed * radius
        else:
            result[0], result[1] = opt[0], opt[1]
    for i, line in enumerate(lines):
        if _det(line.dx, line.dy, line.px - result[0], line.py - result[1]) > 0.0:
            saved = result[:]
            if not _lp1(lines, i, radius, opt, direction_opt, result):
                result[0], result[1] = saved
                return i
    return len(lines)


def _lp3(lines, hard, begin, radius, result):
    distance = 0.0
    for i in range(max(begin, hard), len(lines)):
        li = lines[i]
        if _det(li.dx, li.dy, li.px - result[0], li.py - result[1]) > distance:
            projected = list(lines[:hard])           # hard lines stay as they are
            for j in range(hard, i):
                lj = lines[j]
                den = _det(li.dx, li.dy, lj.dx, lj.dy)
                if abs(den) <= EPS:
                    if li.dx * lj.dx + li.dy * lj.dy > 0.0:
                        continue
                    px, py = 0.5 * (li.px + lj.px), 0.5 * (li.py + lj.py)
                else:
                    t = _det(lj.dx, lj.dy, li.px - lj.px, li.py - lj.py) / den
                    px, py = li.px + t * li.dx, li.py + t * li.dy
                dx, dy = lj.dx - li.dx, lj.dy - li.dy
                n = math.hypot(dx, dy)
                if n < EPS:
                    continue
                projected.append(Line(px, py, dx / n, dy / n))
            saved = result[:]
            if _lp2(projected, radius, (-li.dy, li.dx), True, result) < len(projected):
                result[0], result[1] = saved
            distance = _det(li.dx, li.dy, li.px - result[0], li.py - result[1])
