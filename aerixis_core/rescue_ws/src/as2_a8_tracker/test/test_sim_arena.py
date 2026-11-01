"""Simulated tracker in the rescue arena: people given in the arena frame are converted to AS2's
earth frame from the first pose on the ground and the drone's spawn point (2026-10-31)."""
import rclpy
from geometry_msgs.msg import PoseStamped

from as2_a8_tracker.sim_vision_node import SimVision


def pose(x, y, z=0.0):
    msg = PoseStamped()
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    msg.pose.orientation.w = 1.0
    return msg


def make(*params):
    rclpy.init(args=['--ros-args', *sum((['-p', p] for p in params), [])])
    return SimVision()


def test_arena_persons_follow_the_measured_earth_offset():
    node = make('persons:=20.0,5.0;-30.0,40.0', 'persons_frame:=arena', 'spawn_xy:=10.0,0.0')
    try:
        assert node.pose is None
        node.on_pose(pose(12.0, -4.0, 1.9))          # not on the ground yet: wait
        assert node.pose is None and not node.calibrated
        node.on_pose(pose(7.65, -3.73, 0.02))         # on the ground: earth - arena = (-2.35, -3.73)
        assert node.calibrated
        assert [round(v, 2) for v in node.people[0][:2]] == [17.65, 1.27]
        assert [round(v, 2) for v in node.people[1][:2]] == [-32.35, 36.27]
        node.on_pose(pose(-6.0, 20.0, 0.0))            # estimator jumps while settling: follow it
        assert [round(v, 2) for v in node.people[0][:2]] == [4.0, 25.0]
        node.on_pose(pose(7.65, -3.73, 0.02))         # settled again
        node.on_pose(pose(9.0, -3.0, 4.0))             # first take-off: offset frozen
        node.on_pose(pose(30.0, 30.0, 0.5))            # landed elsewhere: never re-measured
        assert [round(v, 2) for v in node.people[0][:2]] == [17.65, 1.27]
        assert node.frozen
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_default_earth_frame_is_unchanged():
    node = make('persons:=20.0,5.0')
    try:
        node.on_pose(pose(3.0, 4.0, 0.0))
        assert node.calibrated and node.people[0][:2] == [20.0, 5.0]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_true_gazebo_pose_uses_arena_persons_directly():
    import time
    node = make('persons:=20.0,5.0', 'persons_frame:=arena', 'spawn_xy:=10.0,0.0')
    try:
        node.on_pose(pose(7.65, -3.73, 0.0))                       # estimate (calibrated, offset applied)
        node.truth = ([10.0, 0.0, 12.0], 0.0, time.monotonic())    # true pose from Gazebo
        aircraft, yaw, people = node.view()
        assert aircraft == [10.0, 0.0, 12.0] and people[0][:2] == [20.0, 5.0]   # arena frame, no offset
        node.truth = ([10.0, 0.0, 12.0], 0.0, time.monotonic() - 2.0)          # stale: fall back
        aircraft, _, people = node.view()
        assert aircraft == [7.65, -3.73, 0.0] and [round(v, 2) for v in people[0][:2]] == [17.65, 1.27]
    finally:
        node.destroy_node()
        rclpy.shutdown()
