"""Flight-controller health summary for the GCS drone card (read-only; never commands PX4).

Publishes ``fc/health`` (std_msgs/String JSON, 1 Hz) with
  gps      fix type, satellites used, HDOP/VDOP, eph/epv, jamming/spoofing state
  status   arming / nav state, preflight checks passed, failsafe
  faults   short list of active PX4 failsafe flags (e.g. 'global position invalid')
  sensors  unhealthy sensors from MAVLink SYS_STATUS (e.g. 'MAG', 'GPS', 'ACCEL')
  messages last PX4 STATUSTEXT warnings ('Preflight Fail: Compass not calibrated', ...)

GPS/status/failsafe come over uXRCE-DDS. Sensor health and STATUSTEXT only exist on PX4's
MAVLink side, so ``mavlink_url`` optionally taps it: the FC's USB port on the Pi
('/dev/ttyACM0') or PX4 SITL's onboard stream ('udpin:0.0.0.0:14540' for instance 0).
Empty disables the tap; DDS data is still reported.
"""
from collections import deque
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from .node_main import run as run_node

try:
    from px4_msgs.msg import FailsafeFlags, SensorGps, VehicleStatus
except ImportError:  # bench machine without px4_msgs: MAVLink tap only
    FailsafeFlags = SensorGps = VehicleStatus = None

FIX = {0: 'none', 1: 'none', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'RTK float', 6: 'RTK fixed', 8: 'extrapolated'}
NAV = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'MISSION', 4: 'LOITER', 5: 'RTL', 10: 'ACRO',
       12: 'DESCEND', 13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB', 17: 'TAKEOFF', 18: 'LAND',
       19: 'FOLLOW', 20: 'PRECLAND', 21: 'ORBIT', 6: 'POSITION_SLOW', 23: 'EXTERNAL1'}
ARMING = {1: 'disarmed', 2: 'armed'}
# Failsafe flags worth an operator's attention on the ground or in flight.
FAULTS = ('angular_velocity_invalid', 'attitude_invalid', 'local_altitude_invalid', 'local_position_invalid',
          'local_velocity_invalid', 'global_position_invalid', 'home_position_invalid', 'battery_unhealthy',
          'battery_low_remaining_time', 'geofence_breached', 'navigator_failure', 'wind_limit_exceeded',
          'flight_time_limit_exceeded', 'local_position_accuracy_low', 'fd_critical_failure',
          'fd_esc_arming_failure', 'fd_imbalanced_prop', 'fd_motor_failure', 'offboard_control_signal_lost')
SENSOR_BITS = {0: 'GYRO', 1: 'ACCEL', 2: 'MAG', 3: 'BARO', 5: 'GPS', 16: 'RC', 17: 'GYRO2', 18: 'ACCEL2',
               19: 'MAG2', 21: 'AHRS', 22: 'TERRAIN', 24: 'LOGGING', 25: 'BATTERY', 28: 'PREARM'}
# PX4 has no GCS on this MAVLink port: these are its own link/RC complaints, not vehicle faults.
IGNORED_SENSORS = {'RC', 'TERRAIN', 'LOGGING'}


class FcHealth(Node):
    def __init__(self):
        super().__init__('fc_health')
        self.declare_parameter('fmu_prefix', '')
        self.declare_parameter('mavlink_url', '')
        self.declare_parameter('mavlink_baud', 115200)
        self.lock = threading.Lock()
        self.gps = self.status = self.failsafe = None
        self.gps_at = self.status_at = self.failsafe_at = self.mav_at = 0.0
        self.sensors = None
        self.mav_gps = None
        self.messages = deque(maxlen=6)
        self.mav_error = ''
        prefix = self.get_parameter('fmu_prefix').value.rstrip('/')
        if SensorGps is not None:
            self.create_subscription(SensorGps, f'{prefix}/fmu/out/vehicle_gps_position', self._gps, qos_profile_sensor_data)
            # PX4 v1.16: VehicleStatus is message version 1, bridged as vehicle_status_v1.
            self.create_subscription(VehicleStatus, f'{prefix}/fmu/out/vehicle_status_v1', self._status, qos_profile_sensor_data)
            self.create_subscription(FailsafeFlags, f'{prefix}/fmu/out/failsafe_flags', self._failsafe, qos_profile_sensor_data)
        self.publisher = self.create_publisher(String, 'fc/health', 10)
        self.create_timer(1.0, self.publish)
        url = self.get_parameter('mavlink_url').value
        self.stop = threading.Event()
        if url:
            threading.Thread(target=self._mavlink, args=(url,), daemon=True).start()

    # DDS callbacks only store the latest sample; all formatting happens once a second.
    def _gps(self, msg):
        self.gps, self.gps_at = msg, time.monotonic()

    def _status(self, msg):
        self.status, self.status_at = msg, time.monotonic()

    def _failsafe(self, msg):
        self.failsafe, self.failsafe_at = msg, time.monotonic()

    def _mavlink(self, url):
        try:
            from pymavlink import mavutil
        except ImportError:
            self.mav_error = 'pymavlink missing'
            return
        while not self.stop.is_set():
            try:
                link = mavutil.mavlink_connection(url, baud=int(self.get_parameter('mavlink_baud').value),
                                                  source_system=245, source_component=191)
                while not self.stop.is_set():
                    msg = link.recv_match(type=['SYS_STATUS', 'STATUSTEXT', 'GPS_RAW_INT'], blocking=True, timeout=2)
                    if msg is None:
                        continue
                    kind = msg.get_type()
                    with self.lock:
                        self.mav_at = time.monotonic()
                        self.mav_error = ''
                        if kind == 'SYS_STATUS':
                            present = msg.onboard_control_sensors_present
                            enabled = msg.onboard_control_sensors_enabled
                            health = msg.onboard_control_sensors_health
                            self.sensors = [name for bit, name in SENSOR_BITS.items()
                                            if present >> bit & 1 and enabled >> bit & 1 and not health >> bit & 1
                                            and name not in IGNORED_SENSORS]
                        elif kind == 'STATUSTEXT' and msg.severity <= 4:  # EMERGENCY..WARNING
                            text = msg.text.strip() if isinstance(msg.text, str) else ''
                            if text and not (self.messages and self.messages[-1]['text'] == text):
                                self.messages.append(dict(time=round(time.time(), 1), severity=int(msg.severity), text=text[:90]))
                        elif kind == 'GPS_RAW_INT':
                            self.mav_gps = msg
            except Exception as exc:  # serial unplugged, port busy: retry calmly
                self.mav_error = str(exc)[:80]
                self.stop.wait(3.0)

    def summary(self):
        now = time.monotonic()
        gps = None
        if self.gps is not None and now - self.gps_at < 3:
            g = self.gps
            gps = dict(fix=FIX.get(g.fix_type, str(g.fix_type)), fix_type=int(g.fix_type), sats=int(g.satellites_used),
                       hdop=round(float(g.hdop), 2), vdop=round(float(g.vdop), 2), eph=round(float(g.eph), 2),
                       epv=round(float(g.epv), 2), jamming=int(g.jamming_state), spoofing=int(getattr(g, 'spoofing_state', 0)))
        elif self.mav_gps is not None and now - self.mav_at < 3:
            g = self.mav_gps
            gps = dict(fix=FIX.get(g.fix_type, str(g.fix_type)), fix_type=int(g.fix_type), sats=int(g.satellites_visible),
                       hdop=None if g.eph == 65535 else round(g.eph / 100, 2), vdop=None if g.epv == 65535 else round(g.epv / 100, 2))
        status = None
        if self.status is not None and now - self.status_at < 3:
            s = self.status
            status = dict(arming=ARMING.get(s.arming_state, str(s.arming_state)), nav=NAV.get(s.nav_state, str(s.nav_state)),
                          preflight_ok=bool(s.pre_flight_checks_pass), failsafe=bool(s.failsafe))
        faults = None
        if self.failsafe is not None and now - self.failsafe_at < 3:
            armed = status is not None and status['arming'] == 'armed'
            # On the ground PX4 always reports the offboard stream as lost: not a fault until flying.
            faults = [name.replace('_', ' ') for name in FAULTS if getattr(self.failsafe, name, False)
                      and (armed or name != 'offboard_control_signal_lost')]
        with self.lock:
            mav_fresh = self.mav_at and now - self.mav_at < 5
            return dict(gps=gps, status=status, faults=faults,
                        sensors=list(self.sensors) if mav_fresh and self.sensors is not None else None,
                        messages=list(self.messages), mavlink='ok' if mav_fresh else (self.mav_error or ('off' if not self.get_parameter('mavlink_url').value else 'waiting')),
                        dds=bool(gps or status or faults is not None))

    def publish(self):
        self.publisher.publish(String(data=json.dumps(self.summary(), separators=(',', ':'))))

    def destroy_node(self):
        self.stop.set()
        super().destroy_node()


def main():
    run_node(FcHealth)

