"""Simulated A8 tracker for SITL. Publishes exactly what the real bridge publishes.

People stand at fixed ground positions (earth frame). While searching, the gimbal holds
its commanded attitude; a person inside the field of view and range is selected (labels
L1, L2, ... like the multilock tracker), and the gimbal then follows it with a bounded
slew rate. Release/suppress/resume/set_attitude behave like the real tracker's AERIXIS link.

Parameters:
  persons           "x,y;x,y" ground positions in the earth frame (metres)
  detect_range_m    maximum slant range for a detection
  lose_after_s      >0: a locked person becomes hidden after this many seconds
  hidden_s          how long a lost person stays hidden
"""
import json
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from .sim_vision import image_error, look_angles, pointing_angles, step_towards
from .vision_schema import as_string, frame_message, gimbal_state, target_message, tracks


def parse_persons(text):
    people = []
    for item in [p for p in str(text).split(';') if p.strip()]:
        x, y = [float(v) for v in item.split(',')[:2]]
        people.append([x, y, 0.0])
    return people


class SimVision(Node):
    def __init__(self):
        super().__init__('a8_sim_vision')
        for name, value in [('persons', '20.0,15.0'), ('detect_range_m', 60.0),
                            ('slew_dps', 90.0), ('hold_seconds', 1.5), ('lose_after_s', 0.0),
                            ('hidden_s', 30.0), ('rate_hz', 20.0), ('start_pitch_deg', 0.0),
                            ('noise', 0.01), ('ground_z', 0.0)]:
            self.declare_parameter(name, value)
        self.people = parse_persons(self.get_parameter('persons').value)
        ground = float(self.get_parameter('ground_z').value)
        for person in self.people:
            person[2] = ground
        self.hidden_until = [0.0] * len(self.people)
        self.yaw, self.pitch = 0.0, float(self.get_parameter('start_pitch_deg').value)
        self.goal = (self.yaw, self.pitch)
        self.pose = None
        self.selected = None
        self.label = None
        self.labels = 0
        self.locked_since = 0.0
        self.unseen_since = None
        self.suppressed_until = 0.0
        self.target_pub = self.create_publisher(String, 'vision/target', 10)
        self.gimbal_pub = self.create_publisher(String, 'gimbal/state', 10)
        self.tracks_pub = self.create_publisher(String, 'perception/tracks', 10)
        self.frame_pub = self.create_publisher(String, 'vision/frame', 10)
        self.health_pub = self.create_publisher(String, 'camera/health', 10)
        self.create_subscription(PoseStamped, 'self_localization/pose', self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(String, 'vision/command', self.on_command, 10)
        self.period = 1.0 / float(self.get_parameter('rate_hz').value)
        self.create_timer(self.period, self.tick)
        self.get_logger().info('Simulated A8 with %d person(s): %s' % (len(self.people), self.people))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_pose(self, msg):
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.position
        self.pose = ([p.x, p.y, p.z], yaw)

    def on_command(self, msg):
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        now = self.now()
        name = value.get('cmd')
        if name == 'release':
            self.suppressed_until = now + float(value.get('suppress_s', 8.0))
            self.drop('released by mission')
        elif name == 'suppress':
            self.suppressed_until = now + float(value.get('seconds', 5.0))
        elif name == 'resume':
            self.suppressed_until = 0.0
        elif name == 'set_attitude':
            if self.selected is None or now < self.suppressed_until:
                self.goal = (float(value.get('yaw', 0.0)), float(value.get('pitch', 0.0)))
        self.get_logger().info('Sim tracker command: %s' % msg.data)

    def drop(self, reason):
        if self.selected is not None:
            self.get_logger().info('Selection %s dropped: %s' % (self.label, reason))
        self.selected = None
        self.label = None
        self.unseen_since = None

    def visible(self, index, aircraft, yaw):
        if self.now() < self.hidden_until[index]:
            return None
        _, _, distance = look_angles(aircraft, self.people[index])
        if distance > self.get_parameter('detect_range_m').value:
            return None
        return image_error(aircraft, yaw, self.yaw, self.pitch, self.people[index])

    def tick(self):
        now = self.now()
        step = self.get_parameter('slew_dps').value * self.period
        locked = None
        count = 0
        if self.pose is not None:
            aircraft, yaw = self.pose
            seen = {i: self.visible(i, aircraft, yaw) for i in range(len(self.people))}
            seen = {i: e for i, e in seen.items() if e is not None}
            count = len(seen)
            suppressed = now < self.suppressed_until
            if self.selected is None and not suppressed and seen:
                self.selected = min(seen, key=lambda i: math.hypot(*seen[i]))
                self.labels += 1
                self.label = 'L%d' % self.labels
                self.locked_since = now
                self.get_logger().info('New selection %s (person %d)' % (self.label, self.selected))
            if self.selected is not None:
                lose_after = self.get_parameter('lose_after_s').value
                if lose_after > 0 and now - self.locked_since >= lose_after:
                    self.hidden_until[self.selected] = now + self.get_parameter('hidden_s').value
                if self.selected in seen and not suppressed:
                    self.unseen_since = None
                    self.goal = pointing_angles(aircraft, yaw, self.people[self.selected], self.yaw)
                    locked = seen[self.selected]
                else:
                    self.unseen_since = self.unseen_since or now
                    if now - self.unseen_since >= self.get_parameter('hold_seconds').value:
                        self.drop('lost')
            self.yaw = step_towards(self.yaw, self.goal[0], step)
            self.pitch = step_towards(self.pitch, self.goal[1], step)
        noise = self.get_parameter('noise').value
        err = [0.0, 0.0]
        dist = None
        if locked is not None:
            err = [locked[0] + random.gauss(0, noise), locked[1] + random.gauss(0, noise)]
            dist = round(look_angles(self.pose[0], self.people[self.selected])[2], 2)
        gimbal = dict(yaw=self.yaw, pitch=self.pitch, roll=0.0, yaw_rate=0.0, pitch_rate=0.0,
                      roll_rate=0.0, age_s=0.0)
        target = target_message(now, fresh=True, locked=locked is not None, label=self.label,
                                err_x=err[0], err_y=err[1], dist_m=dist, conf=0.9,
                                lock_state='LOCKED' if locked is not None else
                                'SUPPRESSED' if now < self.suppressed_until else 'SEARCHING',
                                suppressed=now < self.suppressed_until, targets_count=count,
                                fps=1.0 / self.period, gimbal=gimbal, source='sim_vision',
                                build='sim')
        gimbal_value = gimbal_state(target)
        tracks_value = tracks(target, self.get_clock().now().nanoseconds)
        self.target_pub.publish(as_string(target))
        self.gimbal_pub.publish(as_string(gimbal_value))
        self.tracks_pub.publish(as_string(tracks_value))
        self.frame_pub.publish(as_string(frame_message(target, gimbal_value, tracks_value)))
        self.health_pub.publish(as_string(dict(stamp=now, fresh=True, source='sim_vision')))


def main(args=None):
    rclpy.init(args=args)
    node = SimVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # context shut down by SIGTERM from a launcher
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
