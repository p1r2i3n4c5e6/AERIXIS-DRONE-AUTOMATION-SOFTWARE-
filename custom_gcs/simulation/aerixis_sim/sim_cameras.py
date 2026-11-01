#!/usr/bin/env python3
"""AERIXIS simulation: every drone's camera in one separate window, gimbals synced to the tracker.

For each simulated drone (Gazebo model x500_a8_I = ROS namespace droneI):
  * shows its gimbal camera (Gazebo image topic) in a grid tile;
  * points the Gazebo gimbal where the simulated A8 tracker (a8_sim_vision) points its gimbal
    (droneI/vision/frame: gimbal yaw right-positive, pitch down-negative, degrees, relative to the
    nose; Gazebo joints use the same signs in radians), so the picture shows what the tracker "sees";
  * overlays drone name, lock state, target label, distance, gimbal angles and the tracker's
    target position in the image (a cross), so a locked person should sit under the cross.

  sim_cameras.py [--world rescue_arena_300] [--drones N]     (q or closing the window quits)

Needs ROS 2 (sourced) for the tracker topics and Gazebo Harmonic's Python bindings for images.
Run with the system packages first: PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH.
"""
import argparse
import json
import math
import os
import signal
import threading
import time

import cv2
import numpy as np
from gz.msgs10.double_pb2 import Double
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node as GzNode

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

TILE_W, TILE_H = 640, 360
WINDOW = 'AERIXIS - simulated drone cameras'
LOCK_COLOURS = {'LOCKED': (60, 200, 80), 'COAST': (40, 170, 240), 'SEARCHING': (200, 200, 200)}


class Drone:
    def __init__(self, index):
        self.index, self.name, self.model = index, f'drone{index}', f'x500_a8_{index}'
        self.frame = None            # latest camera image (BGR)
        self.frame_t = 0.0
        self.frames = 0
        self.fps = 0.0
        self.vision = {}             # latest droneI/vision/frame 'vision' part
        self.gimbal = None           # (yaw_deg, pitch_deg) from the tracker
        self.vision_t = 0.0
        self.lock = threading.Lock()


class Bridge(Node):
    """ROS side: the simulated tracker's state per drone."""
    def __init__(self, drones):
        super().__init__('aerixis_sim_cameras')
        for d in drones:
            self.create_subscription(String, f'/{d.name}/vision/frame', lambda m, d=d: self.on_frame(d, m), 10)

    @staticmethod
    def on_frame(drone, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        vision = data.get('vision') or {}
        g = vision.get('gimbal') or {}
        with drone.lock:
            drone.vision = vision
            if 'yaw' in g and 'pitch' in g:
                drone.gimbal = (float(g['yaw']), float(g['pitch']))
            drone.vision_t = time.time()


def discover(world, timeout=60.0):
    """Drone models present in the Gazebo world (x500_a8_0, x500_a8_1, ...)."""
    node, found, start = GzNode(), set(), time.time()
    while time.time() - start < timeout:
        for topic in node.topic_list():
            prefix = f'/world/{world}/model/x500_a8_'
            if topic.startswith(prefix) and topic.endswith('/sensor/camera/image'):
                found.add(int(topic[len(prefix):].split('/')[0]))
        if found:
            time.sleep(2.0)     # let the other instances appear (spawned a few seconds apart)
            for topic in node.topic_list():
                prefix = f'/world/{world}/model/x500_a8_'
                if topic.startswith(prefix) and topic.endswith('/sensor/camera/image'):
                    found.add(int(topic[len(prefix):].split('/')[0]))
            return sorted(found)
        time.sleep(1.0)
    return []


def tile(drone):
    with drone.lock:
        img = None if drone.frame is None else drone.frame.copy()
        vision, gimbal, fresh = dict(drone.vision), drone.gimbal, time.time() - drone.vision_t < 2.0
        age = time.time() - drone.frame_t
    if img is None or age > 3.0:
        img = np.full((TILE_H, TILE_W, 3), 32, np.uint8)
        cv2.putText(img, 'waiting for camera...', (180, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2)
    elif img.shape[:2] != (TILE_H, TILE_W):
        img = cv2.resize(img, (TILE_W, TILE_H))
    h, w = img.shape[:2]
    # Centre reticle (where the tracker steers the person to).
    cv2.line(img, (w // 2 - 12, h // 2), (w // 2 + 12, h // 2), (255, 255, 255), 1)
    cv2.line(img, (w // 2, h // 2 - 12), (w // 2, h // 2 + 12), (255, 255, 255), 1)
    state = vision.get('lock_state', 'no tracker') if fresh else 'no tracker'
    colour = LOCK_COLOURS.get(state, (160, 160, 160))
    if fresh and vision.get('locked'):
        x = int(w / 2 + float(vision.get('err_x', 0.0)) * w / 2)
        y = int(h / 2 + float(vision.get('err_y', 0.0)) * h / 2)
        cv2.drawMarker(img, (x, y), colour, cv2.MARKER_TILTED_CROSS, 26, 2)
        cv2.circle(img, (x, y), 18, colour, 2)
    # Top banner.
    cv2.rectangle(img, (0, 0), (w, 30), (20, 24, 34), -1)
    cv2.putText(img, drone.name, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    label = vision.get('label') if vision.get('locked') else None
    text = state + (f'  {label}' if label else '')
    if vision.get('dist_m') is not None and vision.get('locked'):
        text += f'  {float(vision["dist_m"]):.0f} m'
    cv2.putText(img, text, (110, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
    info = f'{drone.fps:4.1f} fps'
    if gimbal:
        info = f'gimbal yaw {gimbal[0]:+.0f}  pitch {gimbal[1]:+.0f}   ' + info
    cv2.putText(img, info, (w - 330, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 210, 225), 1)
    if vision.get('suppressed'):
        cv2.putText(img, 'lock suppressed by mission', (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 170, 240), 1)
    return img


def grid(tiles):
    n = len(tiles)
    cols = 1 if n == 1 else 2
    rows = math.ceil(n / cols)
    blank = np.full((TILE_H, TILE_W, 3), 18, np.uint8)
    tiles = tiles + [blank] * (rows * cols - n)
    return np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols]) for r in range(rows)])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', default='rescue_arena_300')
    parser.add_argument('--drones', type=int, default=0, help='0 = find them in Gazebo')
    parser.add_argument('--snapshot', default='', help='write one grid image to this file and exit (tests)')
    args = parser.parse_args()

    indices = list(range(args.drones)) if args.drones else discover(args.world)
    if not indices:
        raise SystemExit(f'No simulated drone cameras found in Gazebo world {args.world}.')
    drones = [Drone(i) for i in indices]
    print(f'Camera window for {", ".join(d.name for d in drones)}', flush=True)

    gz = GzNode()
    pubs = {}
    for d in drones:
        def on_image(msg, d=d):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            now = time.time()
            with d.lock:
                d.frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                if d.frame_t:
                    d.fps = 0.9 * d.fps + 0.1 / max(1e-3, now - d.frame_t)
                d.frame_t = now
                d.frames += 1
        gz.subscribe(Image, f'/world/{args.world}/model/{d.model}/link/camera_link/sensor/camera/image', on_image)
        pubs[d.name] = (gz.advertise(f'/model/{d.model}/command/gimbal_yaw', Double),
                        gz.advertise(f'/model/{d.model}/command/gimbal_pitch', Double))

    rclpy.init()
    # Stopped by the launcher / GCS (INT or TERM): leave quietly. Set after rclpy.init(), which
    # installs its own SIGINT handler.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: os._exit(0))
    bridge = Bridge(drones)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()

    def sync_gimbals():
        msg = Double()
        while rclpy.ok():
            for d in drones:
                with d.lock:
                    gimbal = d.gimbal
                yaw, pitch = gimbal if gimbal else (0.0, -45.0)   # tracker not up yet: A8 search pose
                yaw_pub, pitch_pub = pubs[d.name]
                msg.data = math.radians(max(-160.0, min(160.0, yaw))); yaw_pub.publish(msg)
                msg.data = math.radians(max(-90.0, min(25.0, pitch))); pitch_pub.publish(msg)
            time.sleep(0.05)
    threading.Thread(target=sync_gimbals, daemon=True).start()

    if args.snapshot:
        deadline = time.time() + 20
        while time.time() < deadline and not all(d.frame is not None and d.vision_t for d in drones):
            time.sleep(0.2)
        time.sleep(3.0)     # let the gimbal reach the tracker's angles
        cv2.imwrite(args.snapshot, grid([tile(d) for d in drones]))
        print('snapshot', args.snapshot, flush=True)
    else:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cols = 1 if len(drones) == 1 else 2
        cv2.resizeWindow(WINDOW, TILE_W * cols, TILE_H * math.ceil(len(drones) / cols))
        # Closing the window: with GTK, WND_PROP_VISIBLE is always -1 (unsupported), so watch a
        # property that reads 0 while the window exists and -1 once the user has closed it.
        # Check BEFORE drawing: cv2.imshow re-creates a window the user has just closed with X, so
        # checking after it never saw the window gone (the X did nothing, 2026-10-31).
        seen_open = False
        next_world_check = time.time() + 15.0
        while True:
            if seen_open and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_AUTOSIZE) < 0:
                break                                    # closed with the window's X
            # The simulation stopped (Gazebo window closed, Stop pressed): close with it.
            if time.time() > next_world_check:
                next_world_check = time.time() + 2.0
                newest = max((d.frame_t for d in drones), default=0.0)
                if time.time() - newest > 6.0 and not any(t.startswith(f'/world/{args.world}/') for t in gz.topic_list()):
                    print('Simulation ended: closing the camera window', flush=True)
                    break
            cv2.imshow(WINDOW, grid([tile(d) for d in drones]))
            key = cv2.waitKey(50) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            seen_open = seen_open or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_AUTOSIZE) >= 0
        cv2.destroyAllWindows()
    # Gazebo's transport threads abort ("terminate called...") if torn down normally: exit directly.
    os._exit(0)


if __name__ == '__main__':
    main()
