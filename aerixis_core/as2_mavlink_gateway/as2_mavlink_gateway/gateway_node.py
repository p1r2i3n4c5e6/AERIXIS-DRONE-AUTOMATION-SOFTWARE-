"""Standard MAVLink UDP gateway for AERIXIS core missions, controls, and telemetry."""

import math
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus
from as2_msgs.msg import MissionUpdate, PlatformInfo, PlatformStatus
from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from .protocol import (PX4_MAIN_MODE, ROUTE_EMERGENCY, addressed_to, command_route, enu_to_global,
                       global_to_enu, manual_control_to_enu, mission_to_as2_json, px4_custom_mode,
                       requested_mode)

MAV_CMD_DO_SET_MODE = 176
MODE_ACK_TIMEOUT_S = 3.0
OWN_COMPONENT = 190            # the AERIXIS core's MAVLink component; PX4 answers it in the ack


class MavlinkGateway(Node):
    """Translate a limited MAVLink common subset into AERIXIS core, without PX4 passthrough."""

    def __init__(self):
        super().__init__('mavlink_gateway')
        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('system_id', 1)
        self.declare_parameter('serial_device', '')
        self.declare_parameter('serial_baud', 57600)
        self.declare_parameter('fleet_token_file', '')
        self.declare_parameter('fleet_endpoint', 'http://127.0.0.1:8790')  # onboard mission manager
        # Position/GPS stream to the GCS. The radio is shared by every aircraft and also carries
        # mission traffic; the GCS map takes positions from the status poll, so 2 Hz is plenty.
        self.declare_parameter('telemetry_rate_hz', 2.0)
        self.declare_parameter('udp_port', 14550)
        self.declare_parameter('gcs_host', '')
        self.declare_parameter('gcs_port', 14550)
        self.declare_parameter('home_latitude', 47.397742)
        self.declare_parameter('home_longitude', 8.545594)
        self.declare_parameter('home_altitude', 488.0)
        self.declare_parameter('drone_namespace', '')
        # AS2's Mission Interpreter adapter defaults to root mission topics;
        # projects started with its --add_namespace flag use /<drone_namespace>.
        self.declare_parameter('mission_topic_prefix', '/')
        self.declare_parameter('manual_timeout_sec', 0.5)
        self.declare_parameter('field_width_m', 20000.0)
        self.declare_parameter('field_length_m', 20000.0)
        self.declare_parameter('use_trajectory', True)
        self.declare_parameter('default_speed', 1.5)

        self.drone_namespace = self.get_parameter('drone_namespace').value
        if not self.drone_namespace:
            self.drone_namespace = self.get_namespace().strip('/') or 'drone0'
        self.home = (
            float(self.get_parameter('home_latitude').value),
            float(self.get_parameter('home_longitude').value),
            float(self.get_parameter('home_altitude').value),
        )
        self.manual_timeout = float(self.get_parameter('manual_timeout_sec').value)
        self.field_width = float(self.get_parameter('field_width_m').value)
        self.field_length = float(self.get_parameter('field_length_m').value)
        self.declare_parameter(
            'px4_status_topic', f'/{self.drone_namespace}/fmu/out/vehicle_status_v1')

        # The AS2 interface runs its own ROS executor thread and exposes AS2 behaviors
        # and the supported motion-reference handler, rather than writing PX4 topics.
        self.drone = DroneInterfaceTeleop(
            drone_id=self.drone_namespace, spin_rate=50.0,
            use_sim_time=bool(self.get_parameter('use_sim_time').value))

        topic_prefix = f'/{self.drone_namespace}/'
        mission_prefix = str(self.get_parameter('mission_topic_prefix').value).strip('/')
        mission_prefix = f'/{mission_prefix}/' if mission_prefix else '/'
        self.mission_pub = self.create_publisher(
            MissionUpdate, mission_prefix + 'mission_update', 10)
        self.platform_sub = self.create_subscription(
            PlatformInfo, topic_prefix + 'platform/info', self.platform_callback, 10)
        # AS2 publishes localization as separate PoseStamped and TwistStamped topics.
        # There is no standard AS2 self_localization/odom topic.
        self.pose_sub = self.create_subscription(
            PoseStamped, topic_prefix + 'self_localization/pose',
            self.pose_callback, qos_profile_sensor_data)
        self.twist_sub = self.create_subscription(
            TwistStamped, topic_prefix + 'self_localization/twist',
            self.twist_callback, qos_profile_sensor_data)
        self.gps_sub = self.create_subscription(
            NavSatFix, topic_prefix + 'sensor_measurements/gps', self.gps_callback,
            qos_profile_sensor_data)
        self.mission_status_sub = self.create_subscription(
            String, mission_prefix + 'mission_status', self.mission_status_callback, 10)
        # PX4 link liveness from vehicle_status (PX4 v1.16 name, ~2 Hz): the 100 Hz odometry it used
        # before cost this Python node a large share of a Pi 5 core for a yes/no answer (2026-10-25).
        self.px4_status_sub = self.create_subscription(
            VehicleStatus, self.get_parameter('px4_status_topic').value,
            self.px4_status_callback, qos_profile_sensor_data)
        # Position / Altitude hold from the GCS go straight to PX4 (2026-10-25): AS2's manual()
        # did not change PX4's mode. PX4's own ack goes back to the GCS as COMMAND_ACK.
        status_topic = str(self.get_parameter('px4_status_topic').value)
        fmu = status_topic.rsplit('/out/', 1)[0] if '/out/' in status_topic else f'/{self.drone_namespace}/fmu'
        self.px4_command_pub = self.create_publisher(
            VehicleCommand, fmu + '/in/vehicle_command', qos_profile_sensor_data)
        self.px4_ack_sub = self.create_subscription(
            VehicleCommandAck, fmu + '/out/vehicle_command_ack', self.px4_ack_callback,
            qos_profile_sensor_data)
        self.px4_nav_state = None
        self.pending_mode = None
        self.rosout_sub = self.create_subscription(Log, '/rosout', self.rosout_callback, 100)
        # Emergency path that does not need AS2: GCS RTL/LAND -> onboard FC guardian -> PX4.
        self.guardian_pub = self.create_publisher(String, topic_prefix + 'fc_guardian/request', 10)
        self.guardian_sub = self.create_subscription(
            String, topic_prefix + 'fc_guardian/state', self.guardian_callback, 10)
        self.guardian = None
        self.last_guardian_time = 0.0
        self.guardian_report = (None, 0.0)

        self.platform = None
        self.pose = None
        self.twist = None
        self.last_pose_time = 0.0
        self.last_twist_time = 0.0
        self.gps = None
        self.last_odom_time = 0.0
        self.last_platform_time = 0.0
        self.last_px4_status_time = 0.0
        self.last_mission_status_time = 0.0
        self.last_health_time = 0.0
        self.last_health_signature = None
        self.status_text_id = 0
        self.gcs_last_heartbeat_time = 0.0
        self.recent_ros_logs = deque(maxlen=50)
        self.log_seen = {}
        self.log_tokens = 4.0
        self.log_refill = time.monotonic()
        self.last_manual_time = 0.0
        self.manual_active = False
        self.manual_timeout_applied = False
        self.mission_status = ''
        self.mission_sequence = 0
        self.mission_total = 0
        self.mission_running = False
        self.upload = None
        self.peer = None
        self.last_heartbeat_time = 0.0
        self.last_telemetry_time = 0.0
        self.telemetry_period = 1.0 / max(0.2, min(10.0, float(self.get_parameter('telemetry_rate_hz').value)))

        self._open_mavlink()
        self.fleet_relay = None
        import queue
        self.fleet_frames = queue.Queue(maxsize=512)
        token_file = self.get_parameter('fleet_token_file').value
        if token_file:
            from pathlib import Path
            from as2_rescue_runtime.radio_rpc import RadioRelay
            self.fleet_relay = RadioRelay(self.drone_namespace, Path(token_file).read_text().strip().encode(),
                lambda frame: self.fleet_frames.put(frame, timeout=2),
                self.get_parameter('fleet_endpoint').value)
        # 50 Hz: each poll drains every queued MAVLink message, and a 57600-baud radio delivers far
        # less than one message per 20 ms. At 100 Hz the empty polls alone took ~10 % of a Pi 5
        # core from the person detector (2026-10-23); an emergency RTL/LAND waits at most 10 ms more.
        self.poll_timer = self.create_timer(0.02, self.poll_mavlink)
        self.output_timer = self.create_timer(0.05, self.publish_telemetry)
        self.get_logger().info(
            f'MAVLink UDP listener on {self.get_parameter("bind_address").value}:'
            f'{self.get_parameter("udp_port").value}; AERIXIS core namespace=/{self.drone_namespace}')

    def _open_mavlink(self):
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise RuntimeError(
                'pymavlink is missing; install python3-pip, then run '
                'python3 -m pip install --user -r as2_mavlink_gateway/requirements.txt') from exc
        endpoint = 'udpin:{}:{}'.format(
            self.get_parameter('bind_address').value,
            int(self.get_parameter('udp_port').value))
        serial_device = self.get_parameter('serial_device').value
        if serial_device:
            endpoint = serial_device
        if not 1 <= int(self.get_parameter('system_id').value) <= 254:
            raise ValueError('system_id must be unique per aircraft, between 1 and 254')
        self.mavlink = mavutil.mavlink
        self.connection = mavutil.mavlink_connection(
            endpoint, baud=int(self.get_parameter('serial_baud').value), source_system=int(self.get_parameter('system_id').value), source_component=190, dialect='common')
        host = self.get_parameter('gcs_host').value
        if serial_device:
            # A telemetry radio is point to point: whatever is written reaches the GCS. There is no
            # sender address to learn (pymavlink serial links have none), so without this the
            # gateway never sent a heartbeat or telemetry over the radio (Pi 5, 2026-10-24).
            self.peer = ('serial', serial_device)
        elif host:
            self.peer = (host, int(self.get_parameter('gcs_port').value))
            # pymavlink's UDP listener uses the most recent sender as its return path.
            self.connection.address = self.peer
        self.connection.target_system = 255
        self.connection.target_component = 190

    def platform_callback(self, msg):
        self.platform = msg
        self.last_platform_time = time.monotonic()

    def guardian_callback(self, msg):
        try:
            import json
            self.guardian = json.loads(msg.data)
            self.last_guardian_time = time.monotonic()
        except ValueError:
            pass

    def emergency_command(self, action, command):
        """Hand a GCS emergency RTL/LAND to the FC guardian; works while AERIXIS core is down."""
        import json
        guardian = self.guardian if now_is_fresh(self.last_guardian_time, 2.0) else None
        if guardian is None:
            result, text = self.mavlink.MAV_RESULT_FAILED, 'FC guardian not running; use the RC or PX4 failsafe'
        elif not (guardian.get('px4') or {}).get('fresh'):
            result, text = self.mavlink.MAV_RESULT_FAILED, 'FC guardian has no PX4 link'
        elif not (guardian.get('px4') or {}).get('airborne'):
            result, text = self.mavlink.MAV_RESULT_DENIED, 'aircraft not airborne; nothing to do'
        else:
            self.guardian_pub.publish(String(data=json.dumps(
                dict(action=action, source='gcs', stamp=time.time()))))
            result, text = self.mavlink.MAV_RESULT_ACCEPTED, 'sent to FC guardian -> PX4'
        if result == self.mavlink.MAV_RESULT_ACCEPTED:
            self.get_logger().warning(f'GCS emergency {action}: {text}')
        else:
            self.get_logger().error(f'GCS emergency {action}: {text}')
        self.send_status_text(self.mavlink.MAV_SEVERITY_CRITICAL, f'EMERGENCY|{action}|{text}')
        self.connection.mav.command_ack_send(command, result)

    def publish_guardian_health(self):
        """'LINK|GUARDIAN|UP|<state> action=<RTL|LAND> ...' on change and every 10 s (radio budget)."""
        guardian = self.guardian if now_is_fresh(self.last_guardian_time, 2.0) else None
        if guardian is None:
            text = 'LINK|GUARDIAN|DOWN|fc_guardian not running'
        else:
            emergency = guardian.get('emergency') or {}
            text = 'LINK|GUARDIAN|UP|{} action={}{}'.format(
                guardian.get('state'), guardian.get('action'),
                ' emergency={} {}'.format(emergency.get('action'), emergency.get('state')) if emergency else '')
        previous, sent_at = self.guardian_report
        if text != previous or time.monotonic() - sent_at >= 10.0:
            urgent = guardian is None or bool((guardian or {}).get('emergency'))
            self.send_status_text(self.mavlink.MAV_SEVERITY_WARNING if urgent else self.mavlink.MAV_SEVERITY_INFO, text)
            self.guardian_report = (text, time.monotonic())

    def pose_callback(self, msg):
        self.pose = msg
        self.last_pose_time = time.monotonic()
        self._update_localization_freshness()

    def twist_callback(self, msg):
        self.twist = msg
        self.last_twist_time = time.monotonic()
        self._update_localization_freshness()

    def _update_localization_freshness(self):
        # Only advertise AS2 localization when both halves are arriving recently.
        if (now_is_fresh(self.last_pose_time, 1.5) and
                now_is_fresh(self.last_twist_time, 1.5)):
            self.last_odom_time = min(self.last_pose_time, self.last_twist_time)

    def gps_callback(self, msg):
        self.gps = msg

    def px4_status_callback(self, msg):
        """Fresh PX4 uXRCE-DDS output proves the PX4-to-ROS data path is live."""
        self.last_px4_status_time = time.monotonic()
        self.px4_nav_state = int(msg.nav_state)

    def set_px4_mode(self, mode):
        """POSCTL / ALTCTL: VehicleCommand DO_SET_MODE to PX4; the GCS gets PX4's own answer."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = MAV_CMD_DO_SET_MODE
        msg.param1 = 1.0                                  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.param2 = float(PX4_MAIN_MODE[mode])
        msg.param3 = 0.0
        msg.param4 = msg.param5 = msg.param6 = msg.param7 = 0.0
        system = int(self.get_parameter('system_id').value)
        msg.target_system = system
        msg.target_component = 1
        msg.source_system = system
        msg.source_component = OWN_COMPONENT
        msg.from_external = True
        self.px4_command_pub.publish(msg)
        self.pending_mode = dict(mode=mode, sent=time.monotonic())
        self.get_logger().info(f'GCS mode request {mode} sent to PX4')

    def px4_ack_callback(self, msg):
        pending = self.pending_mode
        if (pending is None or int(msg.command) != MAV_CMD_DO_SET_MODE
                or int(msg.target_component) not in (0, OWN_COMPONENT)):
            return
        result = int(msg.result)
        if result == 5:                                   # IN_PROGRESS: wait for the final answer
            return
        self.pending_mode = None
        self.connection.mav.command_ack_send(MAV_CMD_DO_SET_MODE, result)
        if result == 0:
            self.get_logger().info(f'PX4 switched to {pending["mode"]}')
        else:
            names = {1: 'temporarily rejected', 2: 'denied', 3: 'unsupported', 4: 'failed', 6: 'cancelled'}
            self.get_logger().warning(f'PX4 refused {pending["mode"]}: {names.get(result, result)} '
                                      '(manual modes need RC sticks or a joystick, and a valid position for POSCTL)')

    def check_mode_timeout(self):
        pending = self.pending_mode
        if pending is not None and time.monotonic() - pending['sent'] > MODE_ACK_TIMEOUT_S:
            self.pending_mode = None
            self.connection.mav.command_ack_send(MAV_CMD_DO_SET_MODE, 4)   # FAILED
            self.get_logger().warning(f'No answer from PX4 to {pending["mode"]} in {MODE_ACK_TIMEOUT_S:.0f} s')

    def rosout_callback(self, msg):
        """Forward ROS warnings, errors, and key operational info to GCS."""
        info_level = int.from_bytes(Log.INFO, byteorder='little') \
            if isinstance(Log.INFO, bytes) else int(Log.INFO)
        warning_level = int.from_bytes(Log.WARN, byteorder='little') \
            if isinstance(Log.WARN, bytes) else int(Log.WARN)
        error_level = int.from_bytes(Log.ERROR, byteorder='little') \
            if isinstance(Log.ERROR, bytes) else int(Log.ERROR)
        name_str = str(msg.name)
        name_lower = name_str.lower()
        msg_text = str(msg.msg)

        # Ignore external non-drone nodes (e.g. robot arm, CNC machines on same ROS network)
        if any(ign in name_lower for ign in ('cnc', 'arm_controller', 'move_group', 'robot_state', 'joint_state', 'adjustor', 'rviz', 'nextup')):
            return

        # Only accept logs from AS2 stack, drone namespace, or autopilot
        ns_filter = self.drone_namespace.lower().strip('/')
        if ns_filter:
            # Every aircraft's /rosout reaches every gateway on a shared ROS graph (SITL, or a
            # bench with several Pis): relay only this aircraft's own nodes over its radio.
            if not (name_lower.startswith(ns_filter + '.') or name_lower.endswith('_' + ns_filter)):
                return
        elif not any(k in name_lower for k in ('as2', 'drone', 'platform', 'behavior', 'estimator', 'controller', 'mission', 'mavlink', 'gateway', 'xrce', 'px4')):
            return

        level_value = int(msg.level)
        if level_value >= error_level:
            level = 'ERROR'
        elif level_value >= warning_level:
            level = 'WARN'
        elif level_value >= info_level:
            if 'control mode not established' in msg_text.lower():
                return
            if any(k in name_lower for k in ('platform', 'behavior', 'mission', 'adapter', 'gateway', 'takeoff', 'go_to', 'land')):
                level = 'INFO'
            else:
                return
        else:
            return
        event = (level, name_str, msg_text[:150])
        self.recent_ros_logs.append(event)
        if self.peer is not None and self._log_budget(event):
            self.send_ros_log(*event)

    def _log_budget(self, event):
        """Radio budget for log lines: a repeat waits 30 s, at most ~1 line/s (bursts of 4)."""
        now = time.monotonic()
        key = (event[0], event[1], event[2][:60])
        if now - self.log_seen.get(key, -1e9) < 30.0:
            return False
        self.log_tokens = min(4.0, self.log_tokens + (now - self.log_refill))
        self.log_refill = now
        if self.log_tokens < 1.0:
            return False
        self.log_tokens -= 1.0
        self.log_seen[key] = now
        if len(self.log_seen) > 256:
            self.log_seen = {k: t for k, t in self.log_seen.items() if now - t < 30.0}
        return True

    def send_ros_log(self, level, name, message):
        severity = (self.mavlink.MAV_SEVERITY_ERROR if level == 'ERROR'
                    else (self.mavlink.MAV_SEVERITY_WARNING if level == 'WARN'
                          else self.mavlink.MAV_SEVERITY_INFO))
        self.send_status_text(severity, f'ROSLOG|{level}|{name}|{message}')

    def send_status_text(self, severity, text):
        """Send MAVLink STATUSTEXT, chunking longer ROS diagnostics."""
        if self.peer is None:
            return
        encoded = text.encode('ascii', errors='replace')[:1000]
        chunks = [encoded[i:i + 50] for i in range(0, len(encoded), 50)] or [b'']
        chunk_id = 0
        if len(chunks) > 1:
            self.status_text_id = (self.status_text_id % 65535) + 1
            chunk_id = self.status_text_id
            if len(chunks[-1]) == 50:
                chunks.append(b'')
        for sequence, chunk in enumerate(chunks):
            try:
                self.connection.mav.statustext_send(
                    severity, chunk, chunk_id, sequence)
            except TypeError:
                self.connection.mav.statustext_send(severity, chunk[:50])

    def link_health(self):
        now = time.monotonic()
        platform_ok = now_is_fresh(self.last_platform_time, 2.0)
        as2_odom_ok = now_is_fresh(self.last_odom_time, 1.5)
        mission_ok = now_is_fresh(self.last_mission_status_time, 3.0)
        px4_odom_ok = now_is_fresh(self.last_px4_status_time, 1.5)
        missing_as2 = []
        if not platform_ok:
            missing_as2.append('platform/info')
        if not as2_odom_ok:
            missing_as2.append('estimator pose/twist')
        if not mission_ok:
            missing_as2.append('mission interpreter')
        return platform_ok and as2_odom_ok and mission_ok, px4_odom_ok, ','.join(missing_as2)

    def publish_link_health(self):
        as2_ok, px4_ok, missing_as2 = self.link_health()
        signature = (as2_ok, px4_ok, missing_as2)
        now = time.monotonic()
        as2_text = ('LINK|AS2|UP|platform, estimator, mission interpreter ready' if as2_ok else
                    f'LINK|AS2|DOWN|waiting for {missing_as2 or "ROS data"}')
        px4_text = ('LINK|PX4_DDS|UP|vehicle status fresh' if px4_ok else
                    'LINK|PX4_DDS|DOWN|no PX4 vehicle status; check the FC link and XRCE agent')
        previous = self.last_health_signature
        if previous is None or previous[0] != as2_ok:
            if as2_ok:
                self.get_logger().info('AERIXIS core platform, estimator, and mission interpreter connected')
            else:
                self.get_logger().warning(f'AERIXIS core stack not ready: {missing_as2}')
        if previous is None or previous[1] != px4_ok:
            if px4_ok:
                self.get_logger().info('PX4 uXRCE-DDS connected')
            else:
                self.get_logger().error(
                    f'PX4 uXRCE-DDS disconnected: no data on '
                    f'{self.get_parameter("px4_status_topic").value}; check SITL and agent')
        info = self.mavlink.MAV_SEVERITY_INFO
        error = self.mavlink.MAV_SEVERITY_ERROR
        self.send_status_text(info if as2_ok else error, as2_text)
        self.send_status_text(info if px4_ok else error, px4_text)
        self.last_health_signature = signature
        self.last_health_time = now

    def mission_status_callback(self, msg):
        self.mission_status = msg.data
        self.last_mission_status_time = time.monotonic()
        try:
            import json
            status = json.loads(msg.data)
            done = max(0, int(status.get('done_items', 0)))
            pending = max(0, int(status.get('pending_items', 0)))
            current = status.get('current_item')
            mav_count = getattr(self, 'upload_mavlink_count', None)

            if pending == 0 and current is None and done > 0:
                # Mission finished
                self.mission_total = mav_count if mav_count else done
                self.mission_sequence = self.mission_total
                self.mission_running = False
            else:
                as2_total = done + pending + (1 if current is not None else 0)
                if mav_count and as2_total > 0:
                    self.mission_total = mav_count
                    fraction = done / as2_total if current is not None else 1.0
                    self.mission_sequence = min(mav_count, int(round(fraction * mav_count)))
                else:
                    self.mission_total = as2_total
                    self.mission_sequence = done if current is not None else self.mission_total
                state = status.get('state', '')
                self.mission_running = str(state).upper() in ('RUNNING', 'PAUSED')
        except (TypeError, ValueError, AttributeError):
            self.get_logger().warning('Could not parse AERIXIS core mission_status JSON')

    def poll_mavlink(self):
        # Up to 2 queued GCS-reply frames per poll: the same 100 frames/s ceiling as at 100 Hz.
        for _ in range(2):
            if self.fleet_relay is None or self.fleet_frames.empty():
                break
            frame = self.fleet_frames.get_nowait()
            self.connection.mav.v2_extension_send(0, 255, 190, 42001, list(frame)+[0]*(249-len(frame)))
        """Read nonblocking and dispatch every waiting UDP MAVLink message."""
        try:
            while True:
                message = self.connection.recv_match(blocking=False)
                if message is None:
                    break
                address = getattr(self.connection, 'address', None)
                if address:
                    self.peer = address
                self.handle_message(message)
        except OSError as exc:
            self.get_logger().error(f'MAVLink UDP error: {exc}')

        if self.manual_active and not self.manual_timeout_applied:
            if time.monotonic() - self.last_manual_time > self.manual_timeout:
                self.drone.motion_ref_handler.hover()
                self.manual_timeout_applied = True
                self.manual_active = False
                self.get_logger().warning('MAVLink control timed out; AERIXIS core commanded hover')

    def handle_message(self, message):
        message_type = message.get_type()
        if message_type == 'V2_EXTENSION' and message.message_type == 42001:
            if self.fleet_relay and message.target_system == int(self.get_parameter('system_id').value) and message.target_component == 190:
                self.fleet_relay.receive(bytes(message.payload))
            return
        own = int(self.get_parameter('system_id').value)
        # Shared telemetry radio: drop every targeted packet meant for another aircraft before it
        # can change the reply path or touch AS2 (no ACK either: that aircraft answers itself).
        if message_type in ('MISSION_COUNT', 'MISSION_ITEM_INT', 'MISSION_ITEM', 'MISSION_CLEAR_ALL'):
            if not addressed_to(message.target_system, own):
                return
        elif message_type == 'MANUAL_CONTROL':
            if not addressed_to(message.target, own):
                return
        elif message_type in ('COMMAND_LONG', 'COMMAND_INT'):
            params = [message.param1, message.param2, message.param3, message.param4]
            params += ([message.param5, message.param6, message.param7] if message_type == 'COMMAND_LONG'
                       else [float(message.x), float(message.y), float(message.z)])
            route, action = command_route(message.command, message.target_system,
                                          message.target_component, params, own)
            if route is None:
                return
        source_system = int(message.get_srcSystem() or 255)
        source_component = int(message.get_srcComponent() or 190)

        # Requests and acknowledgements must be sent to the active GCS.
        self.connection.target_system = source_system
        self.connection.target_component = source_component
        address = getattr(self.connection, 'address', None)
        if address:
            self.peer = address

        if message_type == 'HEARTBEAT':
            if time.monotonic() - self.gcs_last_heartbeat_time > 3.0:
                for level, name, text in list(self.recent_ros_logs)[-10:]:  # radio: last 10 only
                    self.send_ros_log(level, name, text)
            self.gcs_last_heartbeat_time = time.monotonic()
        elif message_type == 'MISSION_COUNT':
            self.begin_mission_upload(message, source_system, source_component)
        elif message_type in ('MISSION_ITEM_INT', 'MISSION_ITEM'):
            self.receive_mission_item(message, source_system, source_component)
        elif message_type == 'MISSION_CLEAR_ALL':
            if self.mission_running:
                self.stop_mission()
            self._mission_ack(source_system, source_component,
                              self.mavlink.MAV_MISSION_ACCEPTED)
        elif message_type in ('COMMAND_LONG', 'COMMAND_INT'):
            if route == ROUTE_EMERGENCY:
                self.emergency_command(action, int(message.command))
            elif message_type == 'COMMAND_LONG':
                self.command_long(message, source_system, source_component)
            else:  # COMMAND_INT is accepted for the emergency path only
                self.connection.mav.command_ack_send(int(message.command),
                                                     self.mavlink.MAV_RESULT_UNSUPPORTED)
        elif message_type == 'MANUAL_CONTROL':
            self.manual_control(message)
        # Unknown messages are deliberately ignored; the gateway never forwards
        # arbitrary GCS packets or setpoints directly to PX4.

    def begin_mission_upload(self, msg, target_system, target_component):
        mission_type = int(getattr(msg, 'mission_type', self.mavlink.MAV_MISSION_TYPE_MISSION))
        count = int(msg.count)
        if mission_type != self.mavlink.MAV_MISSION_TYPE_MISSION or not 1 <= count <= 100:
            self._mission_ack(target_system, target_component,
                              self.mavlink.MAV_MISSION_UNSUPPORTED)
            return
        if self.mission_running or self.manual_active:
            self._mission_ack(target_system, target_component,
                              self.mavlink.MAV_MISSION_ERROR)
            return
        self.upload = {
            'count': count, 'items': [], 'target_system': target_system,
            'target_component': target_component,
        }
        self._mission_request(0)

    def receive_mission_item(self, msg, target_system, target_component):
        if self.upload is None:
            self._mission_ack(target_system, target_component,
                              self.mavlink.MAV_MISSION_ERROR)
            return
        sequence = int(msg.seq)
        expected = len(self.upload['items'])
        if sequence != expected or sequence >= self.upload['count']:
            self.upload = None
            self._mission_ack(target_system, target_component,
                              self.mavlink.MAV_MISSION_INVALID_SEQUENCE)
            return
        self.upload['items'].append({
            'seq': sequence, 'command': int(msg.command), 'frame': int(msg.frame),
            'x': float(msg.x), 'y': float(msg.y), 'z': float(msg.z),
        })
        if len(self.upload['items']) < self.upload['count']:
            self._mission_request(len(self.upload['items']))
            return

        upload = self.upload
        self.upload_mavlink_count = int(upload['count'])
        self.upload = None
        try:
            use_trajectory = bool(self.get_parameter('use_trajectory').value)
            default_speed = float(self.get_parameter('default_speed').value)
            mission_json = mission_to_as2_json(
                self.drone_namespace, upload['items'], self.home,
                self.field_width, self.field_length,
                default_speed=default_speed, use_trajectory=use_trajectory)
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'Rejected MAVLink mission: {exc}')
            self._mission_ack(
                upload['target_system'], upload['target_component'],
                getattr(self.mavlink, 'MAV_MISSION_INVALID_PARAM5_X',
                        self.mavlink.MAV_MISSION_INVALID))
            return

        update = MissionUpdate()
        update.drone_id = self.drone_namespace
        update.mission_id = 1
        update.item_id = 0
        update.action = MissionUpdate.EXECUTE
        update.mission = mission_json
        self.mission_pub.publish(update)
        self.mission_running = True
        self._mission_ack(upload['target_system'], upload['target_component'],
                          self.mavlink.MAV_MISSION_ACCEPTED)
        self.get_logger().info('Validated and submitted MAVLink mission to AERIXIS core')

    def command_long(self, msg, target_system, target_component):
        command = int(msg.command)
        result = self.mavlink.MAV_RESULT_UNSUPPORTED
        try:
            if command == self.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                armed = float(msg.param1) >= 0.5
                if armed:
                    offboard_ok = self.drone.offboard()
                    arm_ok = self.drone.arm()
                    result = self._command_result(arm_ok and offboard_ok)
                elif not self._on_ground():
                    # AERIXIS core disarms PX4 with the force flag (it would stop the motors in
                    # flight), so a GCS disarm is only passed on for a landed aircraft.
                    self.get_logger().warning('GCS disarm refused: aircraft not landed (land first)')
                    result = self.mavlink.MAV_RESULT_DENIED
                else:
                    result = self._command_result(self.drone.disarm())
            elif command == self.mavlink.MAV_CMD_NAV_TAKEOFF and not self.mission_running:
                height = max(0.5, min(10.0, float(msg.param7)))
                info = self.drone.info() if callable(self.drone.info) else self.drone.info
                if not info.get('offboard', False):
                    self.drone.offboard()
                result = self._command_result(self.drone.takeoff(height, 0.5, wait=False))
            elif command == self.mavlink.MAV_CMD_NAV_LAND and not self.mission_running:
                result = self._command_result(self.drone.land(speed=0.5, wait=False))
            elif command == self.mavlink.MAV_CMD_DO_SET_MODE:
                mode = requested_mode([msg.param1, msg.param2, msg.param3])
                if mode == 'OFFBOARD':        # AERIXIS core streams the setpoints PX4 needs
                    result = self._command_result(self.drone.offboard())
                elif mode in ('POSCTL', 'ALTCTL'):
                    if not now_is_fresh(self.last_px4_status_time, 1.5):
                        self.get_logger().warning(f'GCS mode {mode} refused: no PX4 link')
                        result = self.mavlink.MAV_RESULT_FAILED
                    else:
                        self.set_px4_mode(mode)
                        result = getattr(self.mavlink, 'MAV_RESULT_IN_PROGRESS', 5)   # PX4's answer follows
                # other modes stay UNSUPPORTED
            elif command == self.mavlink.MAV_CMD_DO_PAUSE_CONTINUE and self.mission_total:
                update = MissionUpdate()
                update.drone_id = self.drone_namespace
                update.mission_id = 1
                update.action = MissionUpdate.RESUME if float(msg.param1) else MissionUpdate.PAUSE
                self.mission_pub.publish(update)
                result = self.mavlink.MAV_RESULT_ACCEPTED
        except Exception as exc:  # report command failure to the GCS, keep node alive
            self.get_logger().error(f'AERIXIS core rejected command {command}: {exc}')
            result = self.mavlink.MAV_RESULT_FAILED
        self.connection.mav.command_ack_send(command, result)

    def _on_ground(self):
        """Fresh platform info says LANDED or DISARMED."""
        return (self.platform is not None and time.monotonic() - self.last_platform_time < 2.0 and
                self.platform.status.state in (PlatformStatus.LANDED, PlatformStatus.DISARMED))

    @staticmethod
    def _command_result(accepted):
        # MAV_RESULT_* numeric values are stable across MAVLink common versions.
        return 0 if accepted else 4  # ACCEPTED / FAILED

    def manual_control(self, msg):
        if self.mission_running:
            return
        vx, vy, vz, yaw_rate = manual_control_to_enu(msg.x, msg.y, msg.z, msg.r)
        try:
            self.drone.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
                [vx, vy, vz], twist_frame_id=self.drone.base_frame_id,
                yaw_speed=float(yaw_rate))
            self.last_manual_time = time.monotonic()
            self.manual_active = True
            self.manual_timeout_applied = False
        except Exception as exc:
            self.get_logger().error(f'Could not send AERIXIS core speed reference: {exc}')

    def stop_mission(self):
        update = MissionUpdate()
        update.drone_id = self.drone_namespace
        update.mission_id = 1
        update.action = MissionUpdate.STOP
        self.mission_pub.publish(update)
        self.mission_running = False
        self.mission_total = 0

    def _mission_request(self, sequence):
        self.connection.mav.mission_request_int_send(
            self.connection.target_system, self.connection.target_component,
            sequence, self.mavlink.MAV_MISSION_TYPE_MISSION)

    def _mission_ack(self, target_system, target_component, result):
        try:
            self.connection.mav.mission_ack_send(
                target_system, target_component, result,
                self.mavlink.MAV_MISSION_TYPE_MISSION)
        except TypeError:
            self.connection.mav.mission_ack_send(target_system, target_component, result)

    def publish_telemetry(self):
        now = time.monotonic()
        if self.peer is None:
            return
        self.check_mode_timeout()
        try:
            if now - self.last_heartbeat_time >= 1.0:
                self.send_heartbeat()
                self.last_heartbeat_time = now
            if now - self.last_health_time >= 1.0:
                self.publish_link_health()
                self.publish_guardian_health()
        except (AttributeError, OSError, TypeError, ValueError) as exc:
            self.get_logger().debug(f'Telemetry not ready: {exc}')
        if now - self.last_telemetry_time < self.telemetry_period:
            return
        # Advance first and isolate each sender: one failing field (e.g. a NaN GPS altitude)
        # used to skip this line, so positions went out on every 50 ms tick (20 Hz per aircraft).
        self.last_telemetry_time = now
        for sender in (self.send_position, self.send_gps, self.send_mission_progress):
            try:
                sender()
            except (AttributeError, OSError, TypeError, ValueError) as exc:
                self.get_logger().debug(f'{sender.__name__} skipped: {exc}', throttle_duration_sec=10.0)

    def send_heartbeat(self):
        armed = bool(self.platform and self.platform.armed)
        offboard = bool(self.platform and self.platform.offboard)
        base_mode = self.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if offboard:
            base_mode |= self.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        if armed:
            base_mode |= self.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        active = self.link_health()[0]
        state = self.mavlink.MAV_STATE_ACTIVE if active else self.mavlink.MAV_STATE_UNINIT
        # PX4's real mode (vehicle_status nav_state) when fresh, as QGroundControl expects.
        real = (px4_custom_mode(self.px4_nav_state)
                if self.px4_nav_state is not None and now_is_fresh(self.last_px4_status_time, 1.5) else None)
        custom_mode = real if real is not None else ((6 << 16) if offboard else (3 << 16))
        self.connection.mav.heartbeat_send(
            self.mavlink.MAV_TYPE_QUADROTOR, self.mavlink.MAV_AUTOPILOT_PX4,
            base_mode, custom_mode, state)

    def send_position(self):
        if (self.pose is None or self.twist is None or
                not now_is_fresh(self.last_odom_time, 1.0)):
            return
        pose = self.pose.pose.position
        twist = self.twist.twist.linear
        east, north, up = float(pose.x), float(pose.y), float(pose.z)
        vn, ve, vd = float(twist.y), float(twist.x), -float(twist.z)
        latitude, longitude, altitude = enu_to_global(
            east, north, up, *self.home)
        time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        self.connection.mav.global_position_int_send(
            time_boot_ms, int(latitude * 1e7), int(longitude * 1e7),
            int(altitude * 1000), int(up * 1000), int(vn * 100),
            int(ve * 100), int(vd * 100), 0xFFFF)
        self.connection.mav.local_position_ned_send(
            time_boot_ms, north, east, -up, float(twist.y), float(twist.x), -float(twist.z))

    def send_gps(self):
        if self.gps is None:
            return
        fix_type = 3 if self.gps.status.status >= NavSatFix.STATUS_FIX else 1
        altitude = self.gps.altitude if math.isfinite(self.gps.altitude) else 0.0
        if not (math.isfinite(self.gps.latitude) and math.isfinite(self.gps.longitude)):
            return
        self.connection.mav.gps_raw_int_send(
            int(time.time() * 1e6), fix_type,
            int(self.gps.latitude * 1e7), int(self.gps.longitude * 1e7),
            int(altitude * 1000), 65535, 65535, 0, 65535, 0)

    def send_mission_progress(self):
        if self.mission_total <= 0:
            return
        try:
            self.connection.mav.mission_current_send(
                min(self.mission_sequence, self.mission_total), self.mission_total)
        except TypeError:
            self.connection.mav.mission_current_send(
                min(self.mission_sequence, self.mission_total))

    def destroy_node(self):
        try:
            self.drone.shutdown()
        except Exception:
            pass
        try:
            if self.fleet_relay is not None:
                self.fleet_relay.close()
            self.connection.close()
        except Exception:
            pass
        super().destroy_node()


def now_is_fresh(timestamp, max_age):
    return timestamp > 0.0 and time.monotonic() - timestamp <= max_age


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MavlinkGateway()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        print(f'as2_mavlink_gateway: {exc}')
    finally:
        # SIGINT already shut ROS down on a link restart: destroying then raised (Pi log, 2026-10-25).
        if node is not None and rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()
