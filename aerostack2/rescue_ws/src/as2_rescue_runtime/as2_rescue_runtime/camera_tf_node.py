"""Publishes the measured world -> camera_optical_frame transform.

The geolocator looks up the camera transform at the exact image timestamp and
refuses to produce a fix without it. Nothing in this workspace published that
transform, so every geolocation attempt failed.

The transform is composed from measured inputs, never from a guess:

* the aircraft pose in the world frame (``self_localization/pose``),
* the measured gimbal attitude (``gimbal/state``), and
* the fixed mechanical mounting offset between the aircraft body and the gimbal
  (a configuration value that must be measured on the aircraft).

Frames follow the ROS optical convention: +X right, +Y down, +Z along the
optical axis. A downward-looking gimbal therefore puts the world-relative
optical Z axis along the negative world Z, which is exactly what the
geolocation ray casting expects.

Transforms are stamped at the *image* or *measurement* timestamp so a lookup
resolves the attitude that actually applied when the frame was captured, not
the attitude now.
"""
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from .node_main import run as run_node

HISTORY = 60


def multiply(first, second):
    return [[sum(first[i][k]*second[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def quaternion_from_matrix(rotation):
    """Convert an orthonormal 3x3 rotation matrix to (x, y, z, w)."""
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2][1] - rotation[1][2]) / scale
        y = (rotation[0][2] - rotation[2][0]) / scale
        z = (rotation[1][0] - rotation[0][1]) / scale
    elif rotation[0][0] > rotation[1][1] and rotation[0][0] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2.0
        w = (rotation[2][1] - rotation[1][2]) / scale
        x = 0.25 * scale
        y = (rotation[0][1] + rotation[1][0]) / scale
        z = (rotation[0][2] + rotation[2][0]) / scale
    elif rotation[1][1] > rotation[2][2]:
        scale = math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2.0
        w = (rotation[0][2] - rotation[2][0]) / scale
        x = (rotation[0][1] + rotation[1][0]) / scale
        y = 0.25 * scale
        z = (rotation[1][2] + rotation[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2.0
        w = (rotation[1][0] - rotation[0][1]) / scale
        x = (rotation[0][2] + rotation[2][0]) / scale
        y = (rotation[1][2] + rotation[2][1]) / scale
        z = 0.25 * scale
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0:
        raise ValueError('Degenerate rotation')
    return (x / norm, y / norm, z / norm, w / norm)


def optical_rotation(yaw_deg, pitch_deg):
    """Body-to-optical rotation for a gimbal at the given measured angles.

    Convention: at yaw=0 and pitch=0 the camera looks along world +X, and pitch
    is positive upward, so a downward-looking gimbal has pitch = -90 and its
    optical axis is world -Z.

    Columns are the optical axes in the world frame: +X right, +Y down,
    +Z forward along the optical axis. The basis is built from an orthonormal
    set and the third axis is obtained by a cross product, so the rotation is
    orthonormal and right handed at every angle.

    Note: roll relative to the world vertical is deliberately not modelled here;
    a roll error belongs in the measured mounting transform, not in an
    assumption about the gimbal.
    """
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)

    forward = (cp * cy, cp * sy, sp)      # +Z optical: the optical axis
    right = (sy, -cy, 0.0)                # +X optical: image right, horizontal
    # +Y optical completes a right-handed frame: down = forward x right.
    down = (forward[1] * right[2] - forward[2] * right[1],
            forward[2] * right[0] - forward[0] * right[2],
            forward[0] * right[1] - forward[1] * right[0])

    return [[right[0], down[0], forward[0]],
            [right[1], down[1], forward[1]],
            [right[2], down[2], forward[2]]]


def euler_rotation(roll_deg, pitch_deg, yaw_deg):
    """Intrinsic Z-Y-X mount correction. Identity when every angle is zero.

    This is deliberately *not* ``optical_rotation``: that function encodes the
    absolute direction the camera points, whereas a mounting offset is a
    relative correction between the body frame and the gimbal. Using the former
    here rotated the optical axis even when the offset was configured as zero.
    """
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


class CameraTransform(Node):
    def __init__(self, args=None):
        super().__init__('a8_camera_transform', cli_args=args)
        for name, value in [
            ('world_frame', 'earth'),
            ('body_frame', 'base_link'),
            ('optical_frame', 'camera_optical_frame'),
            ('mount_x', 0.0), ('mount_y', 0.0), ('mount_z', 0.0),
            ('mount_roll_deg', 0.0), ('mount_pitch_deg', 0.0), ('mount_yaw_deg', 0.0),
            ('gimbal_yaw_sign', 1.0), ('gimbal_pitch_sign', 1.0),
            ('max_measurement_age_s', 0.5),
            ('require_gimbal', True),
            ('calibration_verified', False),
            ('attitude_reference', 'body'),
        ]:
            self.declare_parameter(name, value)

        self.max_age = float(self.get_parameter('max_measurement_age_s').value)
        if not 0 < self.max_age <= 5:
            raise ValueError('Maximum measurement age must be in (0, 5] seconds')
        self.require_gimbal = bool(self.get_parameter('require_gimbal').value)
        if self.get_parameter('attitude_reference').value not in ('body', 'world'):
            raise ValueError('Attitude reference must be calibrated as body or world')
        self.mount_offset = [float(self.get_parameter(name).value)
                             for name in ('mount_x', 'mount_y', 'mount_z')]
        if not all(math.isfinite(v) for v in self.mount_offset):
            raise ValueError('Mounting offset must be finite')

        # A zero mount offset must compose as the identity, so the measured
        # gimbal attitude alone determines the absolute optical orientation.
        self.mount_rotation = euler_rotation(
            float(self.get_parameter('mount_roll_deg').value),
            float(self.get_parameter('mount_pitch_deg').value),
            float(self.get_parameter('mount_yaw_deg').value))

        self.poses = deque(maxlen=HISTORY)
        self.gimbals = deque(maxlen=HISTORY)
        self.last_stamp = None
        self.published = 0
        self.error = ''

        self.broadcaster = TransformBroadcaster(self)
        self.health = self.create_publisher(String, 'camera/tf_health', 10)
        self.create_subscription(PoseStamped, 'self_localization/pose', self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(String, 'gimbal/state', self.on_gimbal, 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.create_timer(1.0, self.report)

    @staticmethod
    def _elapsed(stamp):
        return stamp.sec + stamp.nanosec / 1e9

    def on_pose(self, msg):
        stamp = self._elapsed(msg.header.stamp)
        if not math.isfinite(stamp) or not msg.header.frame_id:
            return
        q = msg.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if not math.isfinite(norm) or abs(norm - 1.0) > 0.01 or not all(
                math.isfinite(v) for v in (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)):
            self.error = 'Aircraft pose quaternion is not normalised'
            return
        self.poses.append((stamp, msg.header.frame_id,
                           (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                           (q.x, q.y, q.z, q.w),
                           (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)))

    def on_gimbal(self, msg):
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        stamp = value.get('stamp')
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return
        if not value.get('verified'):
            # Unverified gimbal feedback is not usable as a measured transform.
            self.gimbals.append((value.get('stamp'), None))
            return
        stamp = value.get('stamp')
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            self.error = 'Gimbal state has no usable timestamp'
            return
        if not all(isinstance(value.get(k), (int, float)) and math.isfinite(value[k])
                   for k in ('yaw', 'pitch', 'roll')):
            self.error = 'Gimbal state has no usable attitude'
            return
        self.gimbals.append((stamp, value))

    def _nearest(self, samples, stamp):
        """Newest sample at or before the stamp, plus its age."""
        best = None
        for item in samples:
            if item[0] is not None and item[0] <= stamp + 1e-6:
                if best is None or item[0] > best[0]:
                    best = item
        if best is None:
            return None, None
        return best, stamp - best[0]

    def on_image(self, msg):
        stamp = self._elapsed(msg.header.stamp)
        try:
            if not self.get_parameter('calibration_verified').value:
                raise ValueError('Camera mount and gimbal attitude convention are unverified')
            if not math.isfinite(stamp) or not 0 <= self.get_clock().now().nanoseconds/1e9-stamp <= self.max_age:
                raise ValueError('Image has no usable timestamp')
            if msg.header.frame_id != self.get_parameter('optical_frame').value:
                raise ValueError('Image optical frame does not match the calibrated transform')
            if self.last_stamp is not None and stamp <= self.last_stamp:
                return  # A repeated or out-of-order frame is not new geometry.
            self.last_stamp = stamp

            pose, pose_age = self._nearest(self.poses, stamp)
            if pose is None:
                raise ValueError('No aircraft pose at or before the image timestamp')
            if not 0 <= pose_age <= self.max_age:
                raise ValueError('Aircraft pose is %.2f s old' % pose_age)

            _, world_frame, position, quaternion, _ = pose
            expected = self.get_parameter('world_frame').value
            if world_frame != expected:
                raise ValueError('Aircraft pose is in %s, expected %s'
                                 % (world_frame, expected))

            gimbal, gimbal_age = self._nearest(self.gimbals, stamp)
            if gimbal is None or gimbal[1] is None:
                if self.require_gimbal:
                    raise ValueError('No verified gimbal attitude at the image timestamp')
                yaw_deg = pitch_deg = roll_deg = 0.0
            else:
                if not 0 <= gimbal_age <= self.max_age:
                    raise ValueError('Gimbal attitude is %.2f s old' % gimbal_age)
                yaw_deg = (gimbal[1]['yaw']
                           * float(self.get_parameter('gimbal_yaw_sign').value))
                pitch_deg = (gimbal[1]['pitch']
                             * float(self.get_parameter('gimbal_pitch_sign').value))
                roll_deg = gimbal[1]['roll']

            gimbal_rotation = optical_rotation(yaw_deg, pitch_deg)
            # Roll about the optical forward axis; calibrated positive direction.
            gimbal_rotation = multiply(gimbal_rotation, euler_rotation(0, 0, roll_deg))
            body = self._quaternion_matrix(quaternion)
            reference = body if self.get_parameter('attitude_reference').value == 'body' else [[1,0,0],[0,1,0],[0,0,1]]
            rotation = multiply(multiply(reference, self.mount_rotation), gimbal_rotation)
            qx, qy, qz, qw = quaternion_from_matrix(rotation)

            # Place the optical origin at the aircraft position plus the mount
            # offset rotated into the world by the aircraft's own attitude.
            body = self._quaternion_matrix(quaternion)
            offset = [sum(body[i][j] * self.mount_offset[j] for j in range(3))
                      for i in range(3)]
            translation = [position[i] + offset[i] for i in range(3)]

            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = expected
            transform.child_frame_id = self.get_parameter('optical_frame').value
            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])
            transform.transform.rotation.x = float(qx)
            transform.transform.rotation.y = float(qy)
            transform.transform.rotation.z = float(qz)
            transform.transform.rotation.w = float(qw)
            self.broadcaster.sendTransform(transform)
            self.published += 1
            self.error = ''
        except (ValueError, KeyError, TypeError) as exc:
            self.error = str(exc)

    @staticmethod
    def _quaternion_matrix(quaternion):
        x, y, z, w = quaternion
        return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]

    def report(self):
        now = self.get_clock().now().nanoseconds / 1e9
        pose = self.poses[-1] if self.poses else None
        gimbal = self.gimbals[-1] if self.gimbals else None
        self.health.publish(String(data=json.dumps(dict(
            stamp=now, published=self.published,
            pose_age_s=None if not pose else now - pose[0],
            gimbal_age_s=None if not gimbal or gimbal[0] is None else now - gimbal[0],
            require_gimbal=self.require_gimbal, error=self.error), allow_nan=False)))


def main(args=None):
    run_node(CameraTransform, args=args)

