"""Per-aircraft return altitude, so a fleet-wide return never puts two drones at the same height.

Without AI avoidance (and for PX4's emergency RTL when the Pi or its link is dead) every drone climbs
to its own height before flying home: base + step x slot (slot = MAVLink system ID - 1). The step is
the fleet separation, so returning drones keep it by height alone: 16 / 19 / 22 m with 3 m (RTK),
16 / 26 / 36 m with 10 m (ordinary GNSS). Gazebo SITL 2026-10-23 with a step smaller than the
separation had two returning drones cross 5.6 m apart directly above each other. With AI avoidance on,
the mission's return flies home at mission height through the avoidance layer instead (no climb). Both return paths use
it: AERIXIS core's ReturnToLaunch (mission end, recovery) and PX4's own RTL (FC guardian, GCS
RTL/emergency, offboard loss) through RTL_RETURN_ALT with RTL_CONE_ANG 0. The base must clear the
obstacles around the area and the mission altitude (the planner allows up to 50 m).
"""
import re

RETURN_BASE_M = 16.0
RETURN_STEP_M = 10.0
# PX4 v1.14 limit for RTL_RETURN_ALT is far higher; keep a sane ceiling for a rescue fleet.
RETURN_MAX_M = 120.0


def slot_for(system_id=0, drone_id=''):
    """0-based slot: MAVLink system ID - 1, else the number in 'droneN', else 0."""
    try:
        if int(system_id) >= 1:
            return int(system_id) - 1
    except (TypeError, ValueError):
        pass
    match = re.search(r'(\d+)$', str(drone_id or ''))
    return int(match.group(1)) if match else 0


def return_altitude(slot, base=RETURN_BASE_M, step=RETURN_STEP_M):
    """Return height above the take-off point for this slot (metres)."""
    if not (0 < base <= RETURN_MAX_M) or not (0 <= step <= 20):
        raise ValueError('Return altitude base must be 0..%g m and step 0..20 m' % RETURN_MAX_M)
    value = base + step * max(0, int(slot))
    if value > RETURN_MAX_M:
        raise ValueError('Return altitude %.0f m for slot %d is above %g m; lower the base or step'
                         % (value, slot, RETURN_MAX_M))
    return value
