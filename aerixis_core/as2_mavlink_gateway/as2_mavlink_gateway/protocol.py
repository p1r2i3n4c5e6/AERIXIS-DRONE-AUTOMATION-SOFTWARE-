"""Pure helpers for converting MAVLink coordinates, controls, and missions."""

import json
import math

EARTH_RADIUS_M = 6378137.0


def clamp(value, low, high):
    return max(low, min(high, value))


def enu_to_ned(position):
    east, north, up = position
    return north, east, -up


def ned_to_enu(position):
    north, east, down = position
    return east, north, -down


def global_to_enu(latitude, longitude, altitude, home_latitude, home_longitude,
                  home_altitude):
    """Small-area tangent-plane conversion for the play-field test area."""
    lat0 = math.radians(home_latitude)
    east = math.radians(longitude - home_longitude) * EARTH_RADIUS_M * math.cos(lat0)
    north = math.radians(latitude - home_latitude) * EARTH_RADIUS_M
    return east, north, altitude - home_altitude


def enu_to_global(east, north, up, home_latitude, home_longitude, home_altitude):
    lat0 = math.radians(home_latitude)
    latitude = home_latitude + math.degrees(north / EARTH_RADIUS_M)
    longitude = home_longitude + math.degrees(
        east / (EARTH_RADIUS_M * math.cos(lat0)))
    return latitude, longitude, home_altitude + up


def manual_control_to_enu(x, y, z, r, max_horizontal_speed=1.0,
                          max_vertical_speed=0.5, max_yaw_speed=0.5):
    """Map MAVLink MANUAL_CONTROL [-1000,1000] sticks into bounded FLU axes."""
    x = clamp(float(x), -1000.0, 1000.0)
    y = clamp(float(y), -1000.0, 1000.0)
    z = clamp(float(z), 0.0, 1000.0)
    r = clamp(float(r), -1000.0, 1000.0)
    # MAVLink x/y/r are body-relative. Return forward, left, up, counter-clockwise.
    return (
        x / 1000.0 * max_horizontal_speed,
        -y / 1000.0 * max_horizontal_speed,
        (z / 500.0 - 1.0) * max_vertical_speed,
        -r / 1000.0 * max_yaw_speed,
    )


def mission_to_as2_json(namespace, items, home, field_x=60.0, field_y=40.0,
                        default_speed=1.0, use_trajectory=True):
    """Translate supported MAVLink mission item dictionaries to AERIXIS core Mission JSON.

    Supported MAV_CMD items are NAV_TAKEOFF (22), NAV_WAYPOINT (16), and
    NAV_LAND (21). Items must explicitly begin with takeoff and end with land.
    ``frame`` may be GLOBAL[_INT], GLOBAL_RELATIVE_ALT[_INT], LOCAL_NED, or
    LOCAL_ENU; coordinates are x/y/z for local frames and lat/lon/alt globally.
    When use_trajectory is True, intermediate waypoints are composed into a
    single smooth polynomial trajectory behavior rather than stop-and-go points.
    """
    plan = []
    half_x, half_y = field_x / 2.0, field_y / 2.0
    home_lat, home_lon, home_alt = home
    waypoints_enu = []

    for item in items:
        command = int(item['command'])
        frame = int(item['frame'])
        z = float(item.get('z', 0.0))

        if frame in (0, 5, 3, 6):  # MAV_FRAME_GLOBAL[_INT], relative-alt variants
            scale = 1e7 if frame in (5, 6) else 1.0
            latitude = float(item.get('x', 0.0)) / scale
            longitude = float(item.get('y', 0.0)) / scale
            altitude = z + home_alt if frame in (3, 6) else z
            east, north, up = global_to_enu(
                latitude, longitude, altitude, home_lat, home_lon, home_alt)
        elif frame == 1:  # MAV_FRAME_LOCAL_NED
            north, east, down = float(item.get('x', 0.0)), float(item.get('y', 0.0)), z
            east, north, up = ned_to_enu((north, east, down))
        elif frame == 4:  # MAV_FRAME_LOCAL_ENU
            east, north, up = float(item.get('x', 0.0)), float(item.get('y', 0.0)), z
        else:
            raise ValueError(f'Unsupported MAVLink mission frame: {frame}')

        if command == 22:  # MAV_CMD_NAV_TAKEOFF
            if not 0.5 <= up <= 50.0:
                raise ValueError('Takeoff altitude must be between 0.5 m and 50 m')
            plan.append({
                'behavior': 'takeoff', 'method': '__call__',
                'args': {'height': up, 'speed': 0.5, 'wait': True},
            })
        elif command == 16:  # MAV_CMD_NAV_WAYPOINT
            if abs(east) > half_x or abs(north) > half_y or not 0.5 <= up <= 50.0:
                raise ValueError(f'Waypoint ({east:.1f}, {north:.1f} m, {up:.1f} m) lies outside field bounds (max {half_x:.0f}x{half_y:.0f} m) or altitude (0.5-50 m)')
            waypoints_enu.append([round(east, 3), round(north, 3), round(up, 3)])
            if not use_trajectory:
                plan.append({
                    'behavior': 'go_to', 'method': '__call__',
                    'args': {
                        'x': east, 'y': north, 'z': up,
                        'speed': clamp(default_speed, 0.1, 3.0),
                    },
                })
        elif command == 21:  # MAV_CMD_NAV_LAND
            if use_trajectory and waypoints_enu:
                plan.append({
                    'behavior': 'trajectory_generation', 'method': '__call__',
                    'args': {
                        'path': waypoints_enu,
                        'speed': float(clamp(default_speed, 0.5, 3.0)),
                        'yaw_mode': 1,  # YawMode.PATH_FACING
                        'yaw_angle': 0.0,
                        'wait': True,
                    },
                })
            plan.append({
                'behavior': 'land', 'method': '__call__',
                'args': {'speed': 0.5, 'wait': True},
            })
        else:
            raise ValueError(f'Unsupported MAVLink mission command: {command}')

    if len(plan) < 3 or plan[0]['behavior'] != 'takeoff' or plan[-1]['behavior'] != 'land':
        raise ValueError('Mission must contain takeoff first, at least one waypoint, and land last')
    has_waypoints = any(item['behavior'] in ('go_to', 'trajectory_generation') for item in plan)
    if not has_waypoints:
        raise ValueError('Mission needs at least one waypoint')
    return json.dumps({'target': namespace, 'plan': plan}, separators=(',', ':'))


def mission_progress(status_json):
    """Return the active/next mission sequence and total from AERIXIS core status JSON."""
    status = json.loads(status_json)
    done = max(0, int(status.get('done_items', 0)))
    pending = max(0, int(status.get('pending_items', 0)))
    current = status.get('current_item')
    total = done + pending + (1 if current is not None else 0)
    sequence = done if current is not None else total
    return sequence, total


# MAVLink ids used by the emergency path (stable across MAVLink common versions).
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_DO_SET_MODE = 176
MAV_COMP_ID_AUTOPILOT1 = 1
PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_RTL, PX4_SUB_MODE_AUTO_LAND = 4, 5, 6


def emergency_action(command, target_system, target_component, params, own_system):
    """GCS emergency RTL/LAND for the onboard FC guardian, or None for the normal AERIXIS core path.

    Emergency (sent straight to PX4 by fc_guardian, works with AERIXIS core down):
      COMMAND_LONG NAV_RETURN_TO_LAUNCH, any component            -> 'RTL'
      COMMAND_LONG NAV_LAND to the autopilot (component 1) or all (0)  -> 'LAND'
      COMMAND_LONG DO_SET_MODE custom AUTO/RTL or AUTO/LAND       -> 'RTL' / 'LAND'
    NAV_LAND to the AERIXIS core component (190) keeps its normal AERIXIS core land behaviour.
    target_system 0 addresses every aircraft on the radio; any other system only that one.
    """
    if int(target_system) not in (0, int(own_system)):
        return None
    command = int(command)
    if command == MAV_CMD_NAV_RETURN_TO_LAUNCH:
        return 'RTL'
    if command == MAV_CMD_NAV_LAND and int(target_component) in (0, MAV_COMP_ID_AUTOPILOT1):
        return 'LAND'
    if command == MAV_CMD_DO_SET_MODE and int(round(params[1])) == PX4_MAIN_MODE_AUTO:
        sub = int(round(params[2]))
        return {PX4_SUB_MODE_AUTO_RTL: 'RTL', PX4_SUB_MODE_AUTO_LAND: 'LAND'}.get(sub)
    return None


# Shared telemetry radio: every aircraft's gateway hears every GCS packet. Only the emergency
# RTL/LAND above is fleet-wide (target_system 0 = all aircraft); everything else (ARM, TAKEOFF,
# LAND via AS2, mode changes, pause/continue, mission upload/clear, manual sticks) must name
# this aircraft's system ID, or one GCS click would act on every drone in range.
ROUTE_EMERGENCY, ROUTE_AS2 = 'emergency', 'as2'


def addressed_to(target_system, own_system):
    """True only for packets naming this aircraft (never 0/broadcast)."""
    try:
        return int(target_system) == int(own_system)
    except (TypeError, ValueError):
        return False


def command_route(command, target_system, target_component, params, own_system):
    """Where a COMMAND_LONG/COMMAND_INT goes on this aircraft.

    (ROUTE_EMERGENCY, 'RTL'|'LAND')  emergency for this aircraft or for all (FC guardian)
    (ROUTE_AS2, None)                normal AERIXIS core command addressed to this system ID
    (None, None)                     not for this aircraft (another system ID, or a broadcast
                                     of a command that is not fleet-wide): ignored, no ACK
    """
    action = emergency_action(command, target_system, target_component, params, own_system)
    if action is not None:
        return ROUTE_EMERGENCY, action
    if addressed_to(target_system, own_system):
        return ROUTE_AS2, None
    return None, None


# ------------------------------------------------------------------ PX4 flight modes
# PX4 custom_mode = main << 16 | sub << 24 (px4_custom_mode.h), as QGroundControl reads it.
PX4_MAIN_MODE = {'MANUAL': 1, 'ALTCTL': 2, 'POSCTL': 3, 'AUTO': 4, 'ACRO': 5, 'OFFBOARD': 6,
                 'STABILIZED': 7}
# vehicle_status.nav_state (PX4 v1.16) -> (main, sub), as PX4's own MAVLink heartbeat maps them.
NAV_STATE_CUSTOM_MODE = {
    0: (1, 0),   # MANUAL
    1: (2, 0),   # ALTCTL
    2: (3, 0),   # POSCTL
    3: (4, 4),   # AUTO_MISSION
    4: (4, 3),   # AUTO_LOITER (Hold)
    5: (4, 5),   # AUTO_RTL
    10: (5, 0),  # ACRO
    12: (4, 6),  # DESCEND (shown as AUTO_LAND)
    13: (1, 0),  # TERMINATION
    14: (6, 0),  # OFFBOARD
    15: (7, 0),  # STABILIZED
    17: (4, 2),  # AUTO_TAKEOFF
    18: (4, 6),  # AUTO_LAND
    19: (4, 8),  # AUTO_FOLLOW_TARGET
    20: (4, 9),  # AUTO_PRECLAND
    21: (3, 1),  # ORBIT (POSCTL sub-mode)
}


def px4_custom_mode(nav_state):
    """MAVLink heartbeat custom_mode for a PX4 nav_state, or None when unknown."""
    main_sub = NAV_STATE_CUSTOM_MODE.get(int(nav_state))
    if main_sub is None:
        return None
    return (main_sub[0] << 16) | (main_sub[1] << 24)


def requested_mode(params):
    """DO_SET_MODE (param1 custom-mode flag, param2 main, param3 sub) -> mode name, or None.

    OFFBOARD goes to the AERIXIS core (it streams the setpoints PX4 needs); POSCTL / ALTCTL go
    straight to PX4. AUTO RTL / LAND never get here: they take the emergency path (FC guardian)."""
    main = int(round(params[1]))
    return {6: 'OFFBOARD', 3: 'POSCTL', 2: 'ALTCTL'}.get(main)
