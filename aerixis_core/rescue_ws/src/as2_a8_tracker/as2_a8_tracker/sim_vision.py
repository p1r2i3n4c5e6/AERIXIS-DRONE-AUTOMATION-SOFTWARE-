"""Geometry of a simulated A8 mini: people on flat ground, a gimbal and a center-lock tracker.

Conventions shared with the real bridge:
  * world frame ENU (x east, y north), aircraft yaw counter-clockwise from +x
  * gimbal yaw in degrees relative to the aircraft nose, positive = to the right
  * gimbal pitch in degrees, 0 = level, -90 = straight down
  * err_x > 0: person right of image centre, err_y > 0: person below image centre
"""
import math

HFOV_DEG = 65.0
VFOV_DEG = 42.0


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def camera_axis(aircraft_yaw, gimbal_yaw_deg):
    """World bearing (ENU, rad) the camera looks along."""
    return wrap(aircraft_yaw - math.radians(gimbal_yaw_deg))


def look_angles(aircraft, person):
    """Bearing (ENU rad), depression below horizon (rad) and slant range to a person."""
    dx, dy = person[0] - aircraft[0], person[1] - aircraft[1]
    height = aircraft[2] - person[2]
    ground = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx) if ground > 1e-6 else None
    return bearing, math.atan2(height, ground), math.hypot(ground, height)


def image_error(aircraft, aircraft_yaw, gimbal_yaw_deg, gimbal_pitch_deg, person):
    """Normalised image error of a person, or None when outside the field of view.

    Uses a small-angle pinhole approximation that stays well defined at nadir by working
    in the camera's ground-projected frame.
    """
    bearing, depression, _ = look_angles(aircraft, person)
    axis_bearing = camera_axis(aircraft_yaw, gimbal_yaw_deg)
    axis_depression = math.radians(-gimbal_pitch_deg)
    # Unit vector to the person and the camera axis in ENU.
    def unit(b, d):
        if b is None:
            return (0.0, 0.0, -1.0)
        return (math.cos(d) * math.cos(b), math.cos(d) * math.sin(b), -math.sin(d))
    to_person = unit(bearing, depression)
    axis = unit(axis_bearing, axis_depression)
    # Camera right = axis bearing rotated clockwise; camera down completes the frame.
    right = (math.sin(axis_bearing), -math.cos(axis_bearing), 0.0)
    # axis x right points to the image bottom (for a level camera facing east: world down).
    down = (axis[1] * right[2] - axis[2] * right[1],
            axis[2] * right[0] - axis[0] * right[2],
            axis[0] * right[1] - axis[1] * right[0])
    forward = sum(a * b for a, b in zip(to_person, axis))
    if forward <= 0.05:
        return None
    x = sum(a * b for a, b in zip(to_person, right)) / forward
    y = sum(a * b for a, b in zip(to_person, down)) / forward
    err_x = x / math.tan(math.radians(HFOV_DEG / 2))
    err_y = y / math.tan(math.radians(VFOV_DEG / 2))
    if abs(err_x) > 1.0 or abs(err_y) > 1.0:
        return None
    return err_x, err_y


def pointing_angles(aircraft, aircraft_yaw, person, current_yaw_deg):
    """Gimbal yaw/pitch that centres a person; yaw kept when the person is straight below."""
    bearing, depression, _ = look_angles(aircraft, person)
    pitch = -math.degrees(depression)
    if bearing is None or pitch < -89.0:
        return current_yaw_deg, max(-90.0, pitch)
    yaw = math.degrees(wrap(aircraft_yaw - bearing))
    return max(-135.0, min(135.0, yaw)), max(-90.0, min(25.0, pitch))


def step_towards(current, goal, max_step):
    return current + max(-max_step, min(max_step, goal - current))
