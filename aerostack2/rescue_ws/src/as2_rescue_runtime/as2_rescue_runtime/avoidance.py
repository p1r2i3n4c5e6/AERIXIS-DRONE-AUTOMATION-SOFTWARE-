"""In-flight avoidance layer ("AI avoidance"): flies route legs around other drones.

OFF (the default, and whenever the GCS switch is off) nothing here runs and every leg is the
usual AS2 GoTo. ON, a leg (SurveyWaypoint / ResumeCoverage) is flown with velocity commands: the
route's preferred velocity (a point LOOKAHEAD m ahead on the leg) goes through the learned brain
and the ORCA shield (as2_rescue_avoidance) 10 times a second, so drones that meet bend their paths,
pass on the right and rejoin their line. Everything else is unchanged: route approval of the leg,
the separation gate (the separation + both GNSS sigmas + speeds) as the last layer, holds, returns.

Each neighbour's radius is the separation gate's own margin plus GATE_BUFFER_M, so avoidance always
keeps the drones outside the gate and the gate stays a last resort. Without the trained model the
layer flies plain ORCA (the shield on the route velocity).
"""
import math
import os

from .swarm import pair_margin

LOOKAHEAD_M = 6.0
REACHED_M = 1.5
REACHED_Z_M = 0.8
CORRIDOR_M = 20.0          # never further than this from the planned leg line
GATE_BUFFER_M = 1.0
# The velocity command changes by at most this (m/s per s). SITL 2026-10-23: the AI return reversed
# 1.35 m/s in one step (and turned the nose 180 deg) and tripped the stability monitor (13 deg tilt,
# 67 deg/s). The simulator the brain was trained in flies x500 at 2.5 m/s^2.
COMMAND_ACCEL_MS2 = 1.5
MAX_PEER_AGE_S = 1.0       # the separation gate's max_peer_age_s (swarm_agent)
MIN_SEPARATION_M = 10.0    # the profile's minimum_separation_m (3 m only with RTK on every drone)
SEE_AGE_S = 5.0            # peers heard longer ago than this are not used
# Only peers within this height band are steered around: further above/below, the 3-D distance
# already exceeds the separation (SITL 2026-10-23: a flying drone swerved around drones still
# standing on the ground 12 m below). A peer climbing into the band is avoided from then on.


def default_model():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('as2_rescue_avoidance'), 'models', 'brain.npz')
    except Exception:
        return ''


class AvoidanceLayer:
    def __init__(self, model_path=None, min_separation_m=MIN_SEPARATION_M, max_peer_age_s=MAX_PEER_AGE_S):
        self.enabled = False
        self.min_separation_m, self.max_peer_age_s = min_separation_m, max_peer_age_s
        self.policy, self.mode, self.error = None, 'orca', ''
        try:
            from as2_rescue_avoidance import brain, orca
            self.brain, self.orca = brain, orca
            path = default_model() if model_path is None else model_path
            self.network, self.error = None, 'no trained model: plain ORCA'
            if path and os.path.exists(path):
                try:
                    self.network, self.mode, self.error = brain.Network(brain.load(path)), 'brain', ''
                except (ValueError, KeyError, OSError) as exc:     # wrong / damaged model file
                    self.error = 'model not usable (%s): plain ORCA' % exc
        except ImportError as exc:      # avoidance package missing: the layer cannot be switched on
            self.brain = self.orca = self.network = None
            self.mode, self.error = 'unavailable', str(exc)
        self.status = dict(enabled=False, mode=self.mode, active=False, note=self.error)

    @property
    def available(self):
        return self.brain is not None

    def set_enabled(self, enabled):
        self.enabled = bool(enabled) and self.available
        self.status.update(enabled=self.enabled, active=False)
        return self.enabled

    def start_leg(self, cmd):
        """Called when a leg starts: a fresh decision memory per leg."""
        cmd['avoid'] = self.enabled
        if self.enabled and self.network is not None:
            cmd['avoid_policy'] = self.brain.LearnedPolicy(self.network)

    def neighbours(self, vehicle, swarm, own_sigma, now):
        if not swarm:
            return []
        age_status = max(0.0, now - swarm.get('stamp', now))
        own_speed = math.hypot(*vehicle['velocity'][:2])
        out = []
        for n in swarm.get('neighbours') or []:
            age = age_status + float(n.get('age_s') or 0.0)
            if age > SEE_AGE_S or abs(float(n.get('z', vehicle['position'][2])) - vehicle['position'][2]) > self.min_separation_m:
                continue
            vel = (float(n['velocity'][0]), float(n['velocity'][1]))
            sigma = float(n.get('sigma_m', 2.5))
            # Position extrapolated by the message age; radius = the separation gate's own margin for
            # this pair (swarm.pair_margin) plus a buffer: the rule the brain was trained with
            # (as2_rescue_avoidance.orca.neighbour_for_gate).
            gate = pair_margin(self.min_separation_m, own_sigma, own_speed, sigma, math.hypot(*vel), age)
            out.append(dict(pos=(float(n['xy'][0]) + vel[0] * age, float(n['xy'][1]) + vel[1] * age), vel=vel,
                            sigma=sigma, age=age, radius=gate + GATE_BUFFER_M))
        return out

    def leg_command(self, cmd, start, goal, speed, vehicle, swarm, own_sigma, now, keep_heading=False, area=None):
        """(velocity [vx, vy, vz], heading, reached) for this tick of an avoidance leg.
        keep_heading: hold the current yaw instead of facing the direction of flight (returns).
        area: the planning area's outline (route approval): the shield never bends a detour out of
        it (SITL 2026-10-23: lanes along the edge of a 26 x 28 m area failed "detour left the area")."""
        p = vehicle['position']
        ax, ay = start[0], start[1]
        lx, ly = goal[0] - ax, goal[1] - ay
        length = math.hypot(lx, ly)
        if length < 0.5:
            ux, uy = (math.cos(vehicle['yaw']), math.sin(vehicle['yaw']))
        else:
            ux, uy = lx / length, ly / length
        along = (p[0] - ax) * ux + (p[1] - ay) * uy
        cross = ux * (p[1] - ay) - uy * (p[0] - ax)                 # + = left of the leg
        remaining = math.hypot(goal[0] - p[0], goal[1] - p[1])
        reached = remaining < REACHED_M and abs(goal[2] - p[2]) < REACHED_Z_M
        target_along = min(length, max(0.0, along) + LOOKAHEAD_M)
        tx, ty = ax + ux * target_along, ay + uy * target_along
        dx, dy = tx - p[0], ty - p[1]
        d = math.hypot(dx, dy)
        pref_speed = min(speed, max(0.3, 0.6 * remaining))
        pref = (dx / d * pref_speed, dy / d * pref_speed) if d > 1e-6 else (0.0, 0.0)
        neighbours = self.neighbours(vehicle, swarm, own_sigma, now)
        view = dict(id=0, t=now, pos=(p[0], p[1]), vel=tuple(vehicle['velocity'][:2]), pref=pref, speed=speed,
                    max_speed=min(3.0, max(1.0, 1.5 * speed)), sigma=own_sigma, min_separation=self.min_separation_m,
                    area=area, route_dir=(ux, uy),
                    cross_track=cross, remaining=remaining,
                    neighbours=[dict(pos=n['pos'], vel=n['vel'], sigma=n['sigma'], age=n['age']) for n in neighbours])
        shield = [dict(pos=n['pos'], vel=n['vel'], radius=n['radius']) for n in neighbours]
        if not neighbours:
            vx, vy = pref
        elif cmd.get('avoid_policy') is not None:
            action_view = view
            policy = cmd['avoid_policy']
            last = policy.memory.get(0)
            if last is None or now - last[0] >= self.brain.DECISION_S - 1e-6:
                policy.memory[0] = (now, policy.net(self.brain.observe(action_view, shield)))
            desired = self.brain.desired_velocity(view, policy.memory[0][1])
            vx, vy = self.brain.shield(view, desired, shield)
        else:
            vx, vy = self.brain.shield(view, pref, shield)
        # Stay within the corridor around the planned line: no further outward sideways motion.
        side = -uy * vx + ux * vy                                    # + = moving to the left
        if (cross > CORRIDOR_M and side > 0) or (cross < -CORRIDOR_M and side < 0):
            vx, vy = vx + uy * side, vy - ux * side
        # No step changes: from the last command (or the measured velocity at the start of the leg).
        last_t, lvx, lvy = cmd.get('last_velocity') or (now, vehicle['velocity'][0], vehicle['velocity'][1])
        step = COMMAND_ACCEL_MS2 * max(0.05, min(0.5, now - last_t))
        dvx, dvy = vx - lvx, vy - lvy
        dv = math.hypot(dvx, dvy)
        if dv > step:
            vx, vy = lvx + dvx / dv * step, lvy + dvy / dv * step
        cmd['last_velocity'] = (now, vx, vy)
        vz = max(-1.0, min(1.0, 0.8 * (goal[2] - p[2])))
        moving = math.hypot(vx, vy) > 0.3
        heading = math.atan2(vy, vx) if moving and not keep_heading else vehicle['yaw']
        deviating = abs(vx - pref[0]) + abs(vy - pref[1]) > 0.15
        self.status.update(active=bool(neighbours), deviating=deviating, cross_track=round(cross, 1),
                           nearest=round(min((math.dist(view['pos'], n['pos']) for n in neighbours), default=-1.0), 1),
                           note=('avoiding' if deviating else 'clear') + (' (%s)' % self.mode))
        return [vx, vy, vz], heading, reached
