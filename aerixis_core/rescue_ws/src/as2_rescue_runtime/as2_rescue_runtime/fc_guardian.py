"""Flight-controller guardian: emergency RTL/LAND straight to PX4 when AERIXIS core stops (one per drone).

Very light on purpose: rclpy, std_msgs, px4_msgs and the as2_msgs message definition only (no
AERIXIS core libraries, no other module of this package except the pure decision core guardian.py).
It talks to PX4 over the same Micro XRCE-DDS link as AERIXIS core, so it keeps working when every AERIXIS core
process is gone, and it is started before AERIXIS core and restarted on its own (systemd unit
as2-fc-guardian.service). Decision rules: see guardian.py.

  ros2 run as2_rescue_runtime fc_guardian --ros-args -r __ns:=/drone0 \
      -p target_system:=1 -p emergency_action:=RTL [-p fmu_prefix:=/drone0]

Topics (in the drone namespace):
  in   platform/info, rescue/state                AERIXIS core liveness
       <fmu_prefix>/fmu/out/vehicle_status_v1      armed, nav_state, takeoff_time (PX4 v1.16 name)
       <fmu_prefix>/fmu/out/vehicle_local_position height above the disarmed ground level
       <fmu_prefix>/fmu/out/vehicle_land_detected  bridged by PX4 v1.16
       <fmu_prefix>/fmu/out/vehicle_command_ack    bridged by PX4 v1.16
       fc_guardian/request     String JSON {"action":"RTL"|"LAND","source":"gcs","stamp":unix_s}
                               (the MAVLink gateway publishes GCS emergency commands here)
       fc_guardian/set_action  String "RTL"|"LAND" (latched): per-mission emergency choice
  out  <fmu_prefix>/fmu/in/vehicle_command         NAV_RETURN_TO_LAUNCH / NAV_LAND
       fc_guardian/state       String JSON, 2 Hz and on every change
Parameters: emergency_action (RTL|LAND, can be changed in flight with ros2 param set),
as2_timeout_s, px4_timeout_s, retry_period_s, rtl_fallback_after, min_airborne_height_m,
require_rescue_state, target_system, source_component, fmu_prefix.
"""
import json
import math
import os
import socket
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleLocalPosition, VehicleStatus

from .guardian import Config, GuardianCore, normalise_action
from .node_main import run as run_node

try:
    from px4_msgs.msg import VehicleLandDetected
except ImportError:
    VehicleLandDetected = None
try:  # message definition only; liveness still works from rescue/state without it
    from as2_msgs.msg import PlatformInfo
except ImportError:
    PlatformInfo = None

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
REQUEST_MAX_AGE_S = 5.0


class FcGuardian(Node):
    def __init__(self):
        super().__init__('fc_guardian')
        for name, default in (('fmu_prefix', ''), ('target_system', 1), ('source_component', 193),
                              ('emergency_action', 'RTL'), ('as2_timeout_s', 3.0), ('px4_timeout_s', 1.5),
                              ('retry_period_s', 1.0), ('rtl_fallback_after', 5),
                              ('min_airborne_height_m', 1.0), ('require_rescue_state', True),
                              ('state_rate_hz', 2.0)):
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        self.core = GuardianCore(Config(
            action=value('emergency_action'), as2_timeout_s=value('as2_timeout_s'),
            px4_timeout_s=value('px4_timeout_s'), retry_period_s=value('retry_period_s'),
            rtl_fallback_after=value('rtl_fallback_after'), min_airborne_height_m=value('min_airborne_height_m'),
            require_rescue_state=value('require_rescue_state')))
        self.target_system = int(value('target_system'))
        if not 1 <= self.target_system <= 254:
            raise ValueError('target_system must be the PX4 MAV_SYS_ID (1..254)')
        self.source_component = int(value('source_component'))
        self.add_on_set_parameters_callback(self._on_parameters)
        prefix = str(value('fmu_prefix')).rstrip('/')
        # Empty prefix = a real FC (PX4 publishes /fmu/..., not under the drone namespace). The old
        # relative 'fmu' became /drone0/fmu/... and never saw PX4 data (real Pi 5 + FC, 2026-10-25).
        fmu = (prefix + '/fmu') if prefix else '/fmu'

        self.command_pub = self.create_publisher(VehicleCommand, fmu + '/in/vehicle_command', qos_profile_sensor_data)
        self.state_pub = self.create_publisher(String, 'fc_guardian/state', 10)
        self.create_subscription(VehicleStatus, fmu + '/out/vehicle_status_v1', self._status, qos_profile_sensor_data)
        self.create_subscription(VehicleLocalPosition, fmu + '/out/vehicle_local_position', self._position,
                                 qos_profile_sensor_data)
        self.create_subscription(VehicleCommandAck, fmu + '/out/vehicle_command_ack', self._ack, qos_profile_sensor_data)
        if VehicleLandDetected is not None:
            self.create_subscription(VehicleLandDetected, fmu + '/out/vehicle_land_detected', self._land,
                                     qos_profile_sensor_data)
        # Best effort subscriptions also match AS2's reliable publishers.
        if PlatformInfo is not None:
            self.create_subscription(PlatformInfo, 'platform/info', self._platform, qos_profile_sensor_data)
        else:
            self.get_logger().warning('as2_msgs not found: AERIXIS core liveness from rescue/state only')
            self.core.platform.seen = True   # do not wait for a signal that can never arrive
            self.core.platform.at = float('inf')
        self.create_subscription(String, 'rescue/state', self._rescue, qos_profile_sensor_data)
        self.create_subscription(String, 'fc_guardian/request', self._request, 10)
        self.create_subscription(String, 'fc_guardian/set_action', self._set_action, LATCHED)

        self.last_state = None
        self.last_publish = 0.0
        self.state_period = 1.0 / max(0.2, float(value('state_rate_hz')))
        self.notify = _SystemdNotify()
        self.create_timer(0.1, self._tick)
        self.notify.send('READY=1')
        self.get_logger().info('FC guardian for {} (PX4 system {}): emergency action {}, AERIXIS core timeout {:.1f} s, '
                               'PX4 topics {}/...'.format(self.get_namespace(), self.target_system,
                                                         self.core.config.action, self.core.config.as2_timeout_s, fmu))

    # ------------------------------------------------------------ callbacks
    @staticmethod
    def now():
        return time.monotonic()

    def _status(self, msg):
        self.core.on_status(self.now(), msg.arming_state == VehicleStatus.ARMING_STATE_ARMED, msg.nav_state,
                            msg.takeoff_time)

    def _position(self, msg):
        self.core.on_local_position(self.now(), float(msg.z), bool(msg.z_valid))

    def _land(self, msg):
        self.core.on_land_detected(self.now(), msg.landed)

    def _ack(self, msg):
        if msg.target_system in (0, self.target_system) and msg.target_component in (0, self.source_component):
            self.core.on_ack(self.now(), msg.command, msg.result)

    def _platform(self, msg):
        self.core.on_platform(self.now(), msg.status.state)

    def _rescue(self, _msg):
        self.core.on_rescue(self.now())

    def _request(self, msg):
        try:
            data = json.loads(msg.data)
            stamp = float(data.get('stamp', 0))
            if not math.isfinite(stamp) or abs(time.time() - stamp) > REQUEST_MAX_AGE_S:
                raise ValueError('stale or unstamped request')
            accepted, reason = self.core.request(self.now(), data.get('action'), str(data.get('source', 'gcs'))[:32])
        except (ValueError, TypeError, AttributeError) as exc:
            accepted, reason = False, 'bad request: {}'.format(exc)
        self._log('warn' if accepted else 'error',
                  'Emergency request {}: {}'.format('accepted' if accepted else 'REJECTED', reason))
        self._tick()   # act now, not on the next timer tick

    def _set_action(self, msg):
        text = msg.data
        try:
            text = json.loads(text).get('action', '') if text.strip().startswith('{') else text
            action = normalise_action(text)
        except (ValueError, AttributeError) as exc:
            self.get_logger().error('Ignored emergency action {!r}: {}'.format(msg.data, exc))
            return
        if action != self.core.config.action:
            # Keep the parameter in step so 'ros2 param get' shows what the guardian will do.
            self.set_parameters([Parameter('emergency_action', value=action)])

    def _on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'emergency_action':
                try:
                    action = normalise_action(parameter.value)
                except ValueError as exc:
                    return SetParametersResult(successful=False, reason=str(exc))
                if self.core.set_action(action):
                    self.get_logger().warning('Emergency action set to {}'.format(action))
            elif parameter.name == 'as2_timeout_s':
                if not 0.5 <= float(parameter.value) <= 60:
                    return SetParametersResult(successful=False, reason='as2_timeout_s must be 0.5..60 s')
                self.core.config.as2_timeout_s = float(parameter.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ cycle
    def _tick(self):
        now = self.now()
        for command in self.core.step(now):
            self._send(command)
        for _, text, level in self.core.take_logs():
            self._log(level, text)
        if self.core.state != self.last_state or now - self.last_publish >= self.state_period:
            snapshot = self.core.snapshot(now)
            snapshot.update(stamp=round(time.time(), 2), drone=self.get_namespace().strip('/'),
                            target_system=self.target_system)
            self.state_pub.publish(String(data=json.dumps(snapshot)))
            self.last_state, self.last_publish = self.core.state, now
        self.notify.send('WATCHDOG=1')

    def _log(self, level, text):
        # rclpy binds one severity to each call site, so every level needs its own line.
        if level == 'error':
            self.get_logger().error(text)
        elif level == 'warn':
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def _send(self, command):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = int(command['command'])
        msg.param1 = msg.param2 = msg.param3 = msg.param4 = 0.0
        # NAV_LAND: NaN latitude/longitude/altitude = land where the aircraft is now.
        msg.param5 = msg.param6 = msg.param7 = float('nan')
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = self.target_system
        msg.source_component = self.source_component
        msg.confirmation = min(255, command['attempt'] - 1)
        msg.from_external = True
        self.command_pub.publish(msg)


class _SystemdNotify:
    """sd_notify without libsystemd: READY/WATCHDOG to $NOTIFY_SOCKET when systemd set one."""
    def __init__(self):
        address = os.environ.get('NOTIFY_SOCKET', '')
        self.address = ('\0' + address[1:]) if address.startswith('@') else address
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) if self.address else None

    def send(self, text):
        if self.socket is None:
            return
        try:
            self.socket.sendto(text.encode(), self.address)
        except OSError:
            pass


def main(args=None):
    run_node(FcGuardian, args=args)

