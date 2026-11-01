"""ROS 2 bridge for the A8 tracker process (hybrid_tracker.py, multilock build).

UDP 127.0.0.1:9090 (tracker telemetry JSON)  ->  vision/target, gimbal/state,
                                                  perception/tracks, camera/health
vision/command (JSON string)                  ->  UDP 127.0.0.1:9091 (tracker commands)

The tracker owns the gimbal. This node never sends a gimbal command of its own; it only
forwards mission requests the tracker validates itself (release, suppress, set_attitude).
Do not run the legacy a8_gimbal node at the same time.
"""
import json
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .vision_schema import as_string, frame_message, gimbal_state, target_message, tracks

ALLOWED_COMMANDS = ('release', 'suppress', 'resume', 'set_attitude')


class TrackerBridge(Node):
    def __init__(self):
        super().__init__('a8_tracker_bridge')
        for name, value in [('telemetry_port', 9090), ('control_port', 9091),
                            ('max_packet_age_s', 0.5), ('gimbal_yaw_sign', 1.0),
                            ('gimbal_pitch_sign', 1.0), ('gimbal_max_age_s', 0.5),
                            # Classes the rescue missions may act on (geotag, drop, peer claims).
                            # Any-class models (2026-10-24): a lock on another class (dog, car...) is
                            # tracked by the gimbal and shown, but reported to the runtime as no lock.
                            ('rescue_classes', 'person'),
                            # vision/target, gimbal/state and perception/tracks at most this often
                            # (0 = every frame). The runtime reads vision/frame, sent every frame;
                            # the others feed status, logs and the geolocator, which need ~10 Hz.
                            # Every frame on all four woke several Python nodes 23x/s each and
                            # took detector CPU (Pi 5, 2026-10-25). Lock changes go out at once.
                            ('aux_topics_hz', 10.0),
                            # Camera field of view reported to the GCS (search sweep planning);
                            # the tracker's own value wins when its telemetry carries one.
                            ('camera_hfov_deg', 65.0), ('camera_vfov_deg', 42.0)]:
            self.declare_parameter(name, value)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('127.0.0.1', int(self.get_parameter('telemetry_port').value)))
        self.sock.setblocking(False)
        self.control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_address = ('127.0.0.1', int(self.get_parameter('control_port').value))
        self.last_packet = None
        self.last_aux = 0.0
        self.last_aux_lock = None
        self.packets = 0
        self.target_pub = self.create_publisher(String, 'vision/target', 10)
        self.gimbal_pub = self.create_publisher(String, 'gimbal/state', 10)
        self.tracks_pub = self.create_publisher(String, 'perception/tracks', 10)
        self.frame_pub = self.create_publisher(String, 'vision/frame', 10)
        self.health_pub = self.create_publisher(String, 'camera/health', 10)
        self.create_subscription(String, 'vision/command', self.on_command, 10)
        self.create_timer(0.02, self.poll)
        self.create_timer(0.5, self.health)
        self.get_logger().info('A8 tracker bridge: telemetry UDP %d -> ROS, commands -> UDP %d'
                               % (self.get_parameter('telemetry_port').value,
                                  self.control_address[1]))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_command(self, msg):
        try:
            value = json.loads(msg.data)
            if value.get('cmd') not in ALLOWED_COMMANDS:
                raise ValueError('unsupported tracker command %r' % value.get('cmd'))
            self.control.sendto(json.dumps(value).encode(), self.control_address)
            self.get_logger().info('Tracker command: %s' % msg.data)
        except (ValueError, OSError, AttributeError) as exc:
            self.get_logger().warning('Rejected tracker command: %s' % exc)

    def poll(self):
        latest = None
        for _ in range(32):
            try:
                data, _ = self.sock.recvfrom(65536)
            except (BlockingIOError, OSError):
                break
            latest = data
        if latest is None:
            return
        try:
            packet = json.loads(latest.decode('utf-8'))
        except ValueError:
            return
        self.packets += 1
        self.last_packet = self.now()
        self.publish(packet)

    def publish(self, packet):
        stamp = self.now()
        locked = packet.get('locked_target') or None
        attitude = packet.get('gimbal_attitude')
        gimbal = None
        if attitude:
            gimbal = dict(attitude)
            gimbal['yaw'] = attitude['yaw'] * self.get_parameter('gimbal_yaw_sign').value
            gimbal['yaw_rate'] = attitude['yaw_rate'] * self.get_parameter('gimbal_yaw_sign').value
            gimbal['pitch'] = attitude['pitch'] * self.get_parameter('gimbal_pitch_sign').value
            gimbal['pitch_rate'] = attitude['pitch_rate'] * self.get_parameter('gimbal_pitch_sign').value
        label = None
        target_class = (locked or {}).get('label', 'person') if locked else None
        rescue = {c.strip().lower() for c in str(self.get_parameter('rescue_classes').value).split(',') if c.strip()}
        lock_state = packet.get('lock_state', '')
        if locked and str(target_class).lower() not in rescue:
            locked = None                       # not a rescue target: no mission may act on it
            lock_state = 'LOCKED_OTHER'
        if locked:
            label = str(packet.get('selected_label') or locked.get('id'))
        target = target_message(
            stamp, fresh=True, locked=locked is not None, label=label,
            err_x=(locked or {}).get('err_x', 0.0), err_y=(locked or {}).get('err_y', 0.0),
            dist_m=(locked or {}).get('dist_m'), conf=(locked or {}).get('conf', 0.0),
            lock_state=lock_state, suppressed=packet.get('suppressed', False),
            targets_count=packet.get('targets_count', 0), fps=packet.get('fps', 0.0),
            gimbal=gimbal, preview_only=packet.get('preview_only', False),
            build=packet.get('build', ''), target_class=target_class, model=packet.get('model'),
            hfov_deg=float(packet.get('hfov_deg') or self.get_parameter('camera_hfov_deg').value),
            vfov_deg=float(packet.get('vfov_deg') or self.get_parameter('camera_vfov_deg').value))
        gimbal_value = gimbal_state(target, self.get_parameter('gimbal_max_age_s').value)
        tracks_value = tracks(target, self.get_clock().now().nanoseconds)
        self.frame_pub.publish(as_string(frame_message(target, gimbal_value, tracks_value)))
        hz = float(self.get_parameter('aux_topics_hz').value)
        lock = (locked is not None, label, lock_state)
        if hz <= 0 or stamp - self.last_aux >= 1.0 / hz or lock != self.last_aux_lock:
            self.last_aux, self.last_aux_lock = stamp, lock
            self.target_pub.publish(as_string(target))
            self.gimbal_pub.publish(as_string(gimbal_value))
            self.tracks_pub.publish(as_string(tracks_value))

    def health(self):
        age = None if self.last_packet is None else self.now() - self.last_packet
        fresh = age is not None and age <= self.get_parameter('max_packet_age_s').value
        if not fresh:
            # Tell consumers explicitly that tracking is unavailable instead of going quiet.
            self.target_pub.publish(as_string(target_message(self.now(), fresh=False, locked=False,
                                                             lock_state='NO_TRACKER')))
        self.health_pub.publish(as_string(dict(stamp=self.now(), fresh=fresh, packet_age_s=age,
                                               packets=self.packets, source='a8_tracker')))


def main(args=None):
    rclpy.init(args=args)
    node = TrackerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # context shut down by SIGTERM from a launcher
        if rclpy.ok():
            raise
    finally:
        node.sock.close()
        node.control.close()
        node.destroy_node()
        rclpy.try_shutdown()
