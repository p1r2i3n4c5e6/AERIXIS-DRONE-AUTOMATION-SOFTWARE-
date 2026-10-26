#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
Raspberry Pi 5 + SIYI A8 Mini — Autonomous Vision & Gimbal Tracking System
===============================================================================
Architecture:
  - Detection              : YOLO26n NCNN in a separate process
  - Tracking               : Timestamped Kalman/IoU multi-person tracks
  - Locking                : Retain one target ID across brief missed detections;
                             stop command when its position becomes stale
  - Gimbal                 : Direct SIYI UDP velocity requests only in CLI
  - Preview                : Red box for locked target, latest-image JPEG stream
  - Health                 : Pi temperature stop and runtime logs
===============================================================================
"""

from __future__ import annotations

import os
import sys
import time
import math
import argparse
import logging
import threading
import socket
import json
import struct
import multiprocessing as mp
from collections import deque
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from dual_tracking import DualTracker
from siyi_startup import center_and_verify
from as2_link import MissionLink, CONTROL_PORT  # AERIXIS link: attitude + mission commands
from detector_models import REAL_HEIGHT_M

from runtime_support import (
    setup_logging,
    install_crash_hooks,
    pin_current_process,
    configure_thread_env,
    flush_and_fsync,
    ResourceMonitor,
)

# Detection models (2026-10-24): any classes, NCNN / ONNX / RKNN; each runtime is optional and a
# model whose runtime is missing is refused with a clear message before the gimbal moves.
import detector_models

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION & HARDWARE CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
BUILD_ID = "A8-20261024-dualtrack2+as2link+anyclass"

DEFAULT_A8_IP     = "192.168.144.25"
DEFAULT_A8_RTSP   = 8554
DEFAULT_SIYI_UDP  = 37260
A8_SUB_STREAM     = f"rtsp://{DEFAULT_A8_IP}:{DEFAULT_A8_RTSP}/sub.264"
A8_MAIN_STREAM    = f"rtsp://{DEFAULT_A8_IP}:{DEFAULT_A8_RTSP}/main.264"

# SIYI A8 Mini Optics (FOV & distance estimation)
FOCAL_LENGTH_MM      = 8.0
SENSOR_HEIGHT_MM     = 2.75
HUMAN_REAL_HEIGHT_MM = 1700.0   # 1.7m standard human height
HFOV_DEG             = 65.0     # Horizontal Field of View
VFOV_DEG             = 42.0     # Vertical Field of View

# Detection & inference defaults
INPUT_SIZE           = (640, 384)  # (width, height) of the shipped person model (YOLO26n, 640x384:
                                 # a 640x360 frame fills it). Other models: from their metadata.yaml
CONF_THRESH          = 0.50     # Strict human detection: prevents false locks on posters/furniture
IOU_THRESH           = 0.45
KEYFRAME_SKIP        = 4        # Legacy CLI compatibility; latest-frame scheduler ignores it

# OC-SORT tracker parameters
MAX_AGE              = 20       # Keep track alive for 20 frames during fast motion / blur
MIN_HITS             = 2        # Two independent detections before a new lock
IOU_ASSOC_THRESH     = 0.25     # IoU threshold for detection-track association

# Target Lock geometry gates (Eliminate wall posters, desk objects)
MIN_ASPECT_RATIO     = 0.85     # Human proportions (height >= 0.85 * width)
MAX_ASPECT_RATIO     = 5.0
MIN_HEIGHT_PX        = 50       # Reject tiny background objects (< 50 pixels)
CONF_INIT_TRACK      = 0.50     # Strict threshold to initiate a new track
LOCK_COAST_BUDGET    = 60       # Coast for ~2.0s if target temporarily occluded
OBSERVATION_TIMEOUT_S = 0.40   # Covers two measured ~180 ms inference periods
RECONNECT_WINDOW_S   = 3.0    # Short-gap recovery only, never stale motion
RECONNECT_HITS       = 3      # Distinct source observations, not display frames
LOCK_ID_HOLD_S        = 0.90   # Keep identity briefly, without commanding stale position

# Aerostac2 UDP telemetry
TELEMETRY_PORT       = 9090

# Runtime health & fault tolerance
HEALTH_INTERVAL_S         = 5.0    # temp / freq / throttle sampling period
MAX_CONSECUTIVE_ERRORS    = 200    # abort only if frames fail this many times in a row
ERROR_BACKOFF_S           = 0.05   # sleep after a failed frame (prevents spin-loop 100% CPU)


# ─────────────────────────────────────────────────────────────────────────────
#  CPU ISOLATION (cores from --cores, default 1,2,3 = same as run.sh)
# ─────────────────────────────────────────────────────────────────────────────
def set_cpu_affinity(cores=(2, 3)):
    """
    Pin vision process to the requested CPU cores.

    Uses os.sched_setaffinity on Linux (Raspberry Pi 5) and
    SetProcessAffinityMask on Windows, so the laptop run uses exactly the same
    core set as the Pi 5 run. Returns True when the mask was applied.
    """
    logger = logging.getLogger("cpu")
    ok = pin_current_process(cores, logger=logger)
    if ok:
        logger.info("   → Remaining cores stay free for Aerostac2 / ROS 2 flight control.")
    else:
        logger.warning("   → Vision is NOT core-pinned; timings will differ from the Pi 5 run.")
    return ok


# ─────────────────────────────────────────────────────────────────────────────
#  SIYI A8 MINI DIRECT ETHERNET UDP PROTOCOL (finite latency Control)
# ─────────────────────────────────────────────────────────────────────────────
def siyi_crc16(data: bytes) -> int:
    """CRC16-CCITT checksum for SIYI Gimbal SDK packets."""
    crc = 0
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_siyi_speed_packet(yaw_speed: int, pitch_speed: int, seq: int = 1) -> bytes:
    """
    Construct SIYI CMD 0x07 (Gimbal Rotation Speed).
    yaw_speed   : -100 to +100 (Negative=Left, Positive=Right)
    pitch_speed : -100 to +100 (Negative=Down, Positive=Up)
    """
    yaw_s   = max(-100, min(100, int(yaw_speed)))
    pitch_s = max(-100, min(100, int(pitch_speed)))

    stx       = b'\x55\x66'
    ctrl      = b'\x01'
    data_len  = struct.pack('<H', 2)
    seq_bytes = struct.pack('<H', seq & 0xFFFF)
    cmd_id    = b'\x07'
    payload   = struct.pack('<bb', yaw_s, pitch_s)

    head_body = stx + ctrl + data_len + seq_bytes + cmd_id + payload
    crc = siyi_crc16(head_body)
    return head_body + struct.pack('<H', crc)


def build_siyi_center_packet(seq: int = 1) -> bytes:
    """Construct SIYI CMD 0x08 (Center Gimbal)."""
    stx       = b'\x55\x66'
    ctrl      = b'\x01'
    data_len  = struct.pack('<H', 1)
    seq_bytes = struct.pack('<H', seq & 0xFFFF)
    cmd_id    = b'\x08'
    payload   = b'\x01'

    head_body = stx + ctrl + data_len + seq_bytes + cmd_id + payload
    crc = siyi_crc16(head_body)
    return head_body + struct.pack('<H', crc)


# ─────────────────────────────────────────────────────────────────────────────
#  UNIFIED CLOSED-LOOP GIMBAL CONTROLLER
# ─────────────────────────────────────────────────────────────────────────────
class UnifiedGimbalController:
    """
    Visual Servoing Gimbal Controller.
    Computes smooth, decisive PD velocity and angle commands to center target.
    Supports 4 actuation backends:
      1. 'auto'         : Dual-broadcast (SIYI UDP over Ethernet + MAVLink Rate)
      2. 'siyi_udp'     : Direct SIYI Ethernet UDP (192.168.144.25:37260)
      3. 'mavlink_rate' : Pure MAVLink Gimbal Manager v2 (MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW)
      4. 'mavlink_mount': Classic MAVLink Mount Control with FOV angle targeting
    """
    lead_s = 0.0          # look-ahead defaults (also for controllers built without __init__)
    _vel_prev = None
    est_vx = est_vy = 0.0

    def __init__(
        self,
        mode: str = "siyi_udp",
        mavlink_conn: str = "none",
        mavlink_baud: int = 115200,
        siyi_ip: str = DEFAULT_A8_IP,
        siyi_port: int = DEFAULT_SIYI_UDP,
        kp_yaw: float = 20.0,
        kp_pitch: float = 16.0,
        kd_yaw: float = 0.0,
        kd_pitch: float = 0.0,
        deadband: float = 0.035,
        max_rate: float = 30.0,
        invert_yaw: bool = False,
        invert_pitch: bool = True,
        scan_on_start: bool = False,
        scan_speed: float = 18.0,
        scan_period: float = 18.0,
        scan_mode: str = "sweep",
        siyi_command_cap: int = 35,
        defer_start: bool = False,
        lead_s: float = 0.0,
    ):
        if scan_on_start:
            raise ValueError("This tracking-only build does not scan without a target")
        self.mode = mode.lower()
        self.kp_yaw = kp_yaw
        self.kp_pitch = kp_pitch
        self.kd_yaw = kd_yaw
        self.kd_pitch = kd_pitch
        self.deadband = deadband
        self.max_rate = max_rate
        self.invert_yaw = invert_yaw
        self.invert_pitch = invert_pitch
        self.scan_active = scan_on_start
        self.scan_start_t = time.monotonic()
        self.scan_speed = scan_speed
        self.scan_period = scan_period
        self.scan_mode = scan_mode.lower()

        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_time = time.monotonic()

        self.smooth_yaw_rate = 0.0
        self.smooth_pitch_rate = 0.0
        self.alpha_smooth = 0.25  # Smooth cinematic low-pass filter (no jerky twitches)

        # FOV Angle Targeting accumulator for classic mount control
        self.target_pitch_deg = 0.0
        self.target_yaw_deg = 0.0

        self.last_cmd_time = 0.0
        self.cmd_interval = 0.02  # 50 Hz target; actual measured rate is shown in HUD
        self.seq = 0
        self.actual_hz = 0.0
        self.state_lock = threading.Lock()
        self.last_target = None
        self.running = True
        self.observation_timeout = OBSERVATION_TIMEOUT_S
        self.siyi_command_cap = max(1, min(100, int(siyi_command_cap)))
        # Look-ahead (s): aim where the target will be, from its speed measured in the image.
        # 0 = off (the tracker's own velocity, zero for dualtrack2, so no prediction: old behaviour).
        self.lead_s = max(0.0, min(0.5, float(lead_s)))
        self._vel_prev = None          # (target id, cx, cy, observed_t) of the last observation
        self.est_vx = self.est_vy = 0.0
        self.preview_only = os.environ.get("A8_PREVIEW_ONLY", "1") != "0"
        self.thermal_fault = False
        self._next_temp_check = 0.0

        # 1. Initialize Direct SIYI UDP Socket
        self.siyi_ip = siyi_ip
        self.siyi_port = siyi_port
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setblocking(False)
        self.siyi_active = True
        print(f"📡 [Gimbal Backend] SIYI Direct UDP socket ready → {siyi_ip}:{siyi_port}")

        # Rate-limited error logging
        self._tx_error_counts = {}
        self._tx_error_last_log = {}
        self._tx_log = logging.getLogger("gimbal")

        # Construct control worker but defer it until optional center verification.
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.started = False
        self.defer_start = defer_start

        # 2. Initialize PyMAVLink if requested
        self.mav_master = None
        self.mav_active = False
        if self.mode in ("auto", "mavlink_rate", "mavlink_mount"):
            if not HAS_PYMAVLINK:
                print("ℹ️  [MAVLink] 'pymavlink' not found. Falling back to Direct SIYI UDP.")
            elif mavlink_conn not in ("none", "NONE", "disable", "False", "0"):
                if mavlink_conn in ("auto", "AUTO", "", None):
                    mavlink_conn = self._auto_detect_serial_port()
                if mavlink_conn:
                    self._connect_mavlink(mavlink_conn, mavlink_baud)

        if not self.defer_start:
            self.start()

    def start(self):
        if not self.started:
            self.control_thread.start()
            self.started = True

    def _auto_detect_serial_port(self):
        if sys.platform == "win32":
            try:
                import serial.tools.list_ports
                for p in serial.tools.list_ports.comports():
                    desc = (p.description or "").lower()
                    if "bluetooth" not in desc and "standard serial" not in desc:
                        print(f"🔌 [MAVLink Auto-Detect] Found serial telemetry port: {p.device} ({p.description})")
                        return p.device
            except Exception:
                pass

        candidate_ports = [
            "/dev/ttyAMA0",   # Pi 5 GPIO UART (Telem to Flight Controller)
            "/dev/serial0",   # Default serial alias on Raspberry Pi OS
            "/dev/ttyUSB0",   # USB-to-UART telemetry cable
            "/dev/ttyACM0",   # Direct USB Pixhawk connection
            "/dev/ttyUSB1",
        ]
        for p in candidate_ports:
            if os.path.exists(p):
                print(f"🔌 [MAVLink Auto-Detect] Found serial telemetry port: {p}")
                return p
        return None

    def _connect_mavlink(self, conn_str, baud):
        try:
            print(f"📡 [MAVLink] Connecting to Flight Controller: {conn_str} ({baud} baud)...")
            if "udp" in conn_str or "tcp" in conn_str:
                self.mav_master = mavutil.mavlink_connection(conn_str)
            else:
                self.mav_master = mavutil.mavlink_connection(conn_str, baud=baud)

            hb = self.mav_master.wait_heartbeat(timeout=3.0)
            if hb is not None:
                sys_id = hb.get_srcSystem() if hb.get_srcSystem() > 0 else 1
                comp_id = hb.get_srcComponent() if hb.get_srcComponent() > 0 else 1
                self.mav_master.target_system = sys_id
                self.mav_master.target_component = comp_id
                print(f"✅ [MAVLink] Flight Controller Connected! System ID: {sys_id}, Component ID: {comp_id}")
            else:
                self.mav_master.target_system = 1
                self.mav_master.target_component = 1
                print("⚠️  [MAVLink] Port opened (heartbeat listening in background)...")
            self._acquire_gimbal_control(verbose=True)
            self.mav_active = True
        except Exception as e:
            print(f"⚠️  [MAVLink Warning] Could not open {conn_str}: {e}")
            self.mav_active = False

    def _acquire_gimbal_control(self, verbose: bool = False):
        """Acquire primary control lease from MAVLink Gimbal Manager v2."""
        if self.mav_master is None:
            return
        try:
            sys_id = getattr(self.mav_master, "target_system", 1) or 1
            comp_id = getattr(self.mav_master, "target_component", 1) or 1
            src_sys = getattr(self.mav_master, "source_system", 255)
            src_comp = getattr(self.mav_master, "source_component", 0)

            # MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE (1001)
            self.mav_master.mav.command_long_send(
                sys_id, comp_id,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                0,
                src_sys, src_comp,
                0, 0,
                0, 0, 0
            )
            if verbose:
                print(f"🎯 [MAVLink] Gimbal Manager control lease acquired (Sys:{src_sys}, Comp:{src_comp})")
        except Exception as e:
            self._log_tx_error("MAVLink gimbal configure", e)

    def update(self, locked_track, frame_w, frame_h):
        """Non-blocking target update: hands target vector off to the 50 Hz servo loop."""
        with self.state_lock:
            if locked_track is not None:
                if self.scan_active:
                    print(f"🎯 [Gimbal Scan] Target locked (ID:{locked_track['id']})! Exiting environment scan & engaging tracker.")
                    self.scan_active = False
                x1, y1, x2, y2 = locked_track["box"]
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                vel = locked_track.get("velocity", (0.0, 0.0))
                if self.lead_s > 0.0:
                    vel = self._estimate_velocity(locked_track.get("id"), cx, cy,
                                                  float(locked_track.get("observed_t", 0.0)))
                self.last_target = {
                    "cx": cx,
                    "cy": cy,
                    "vx": float(vel[0]) if hasattr(vel, '__getitem__') else 0.0,
                    "vy": float(vel[1]) if hasattr(vel, '__getitem__') else 0.0,
                    "w": frame_w,
                    "h": frame_h,
                    "t": locked_track.get("state_time", 0.0),
                    "observed_t": locked_track.get("observed_t", 0.0),
                }
            else:
                self.last_target = None
                self._vel_prev = None
                self.est_vx = self.est_vy = 0.0

    def _estimate_velocity(self, ident, cx, cy, observed_t):
        """Target speed in the image (px/s) from successive observations of the same target.
        Smoothed; restarts on a new target or after a gap, so a jump never becomes a speed."""
        prev = self._vel_prev
        if prev is None or prev[0] != ident:
            self.est_vx = self.est_vy = 0.0
            self._vel_prev = (ident, cx, cy, observed_t)
            return 0.0, 0.0
        dt = observed_t - prev[3]
        if dt <= 0.0:                      # same observation handed over again
            return self.est_vx, self.est_vy
        if dt > 0.3:
            self.est_vx = self.est_vy = 0.0
        else:
            alpha = 0.5
            self.est_vx = alpha * (cx - prev[1]) / dt + (1.0 - alpha) * self.est_vx
            self.est_vy = alpha * (cy - prev[2]) / dt + (1.0 - alpha) * self.est_vy
        self._vel_prev = (ident, cx, cy, observed_t)
        return self.est_vx, self.est_vy

    def _control_loop(self):
        """Dedicated high-frequency 50 Hz PD Servoing Thread with velocity forward-prediction."""
        last_t = time.monotonic()
        hz_count = 0
        hz_timer = time.monotonic()
        last_lease_refresh = time.monotonic()
        while self.running:
            now = time.monotonic()
            dt = max(1e-3, min(0.05, now - last_t))
            last_t = now

            # Maintain companion heartbeat & primary control lease every 1.0s
            if now - last_lease_refresh >= 1.0:
                last_lease_refresh = now
                if self.mav_master is not None and self.mav_active:
                    try:
                        self.mav_master.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0, 0, 0
                        )
                        self._acquire_gimbal_control()
                    except Exception:
                        pass

            with self.state_lock:
                target = self.last_target

            if now >= self._next_temp_check:
                self._next_temp_check = now + 1.0
                try:
                    if Path('/proc/device-tree/model').exists() and 'Raspberry Pi' in Path('/proc/device-tree/model').read_text():
                        temp = float(Path('/sys/class/thermal/thermal_zone0/temp').read_text()) / 1000.0
                        if temp >= 78.0 and not self.thermal_fault:
                            self.thermal_fault = True
                            self._tx_log.error("Thermal stop latched at %.1f C; cool before restarting", temp)
                except (OSError, ValueError):
                    pass
            stale = target is not None and now - target["observed_t"] > self.observation_timeout
            if stale or self.thermal_fault:
                target = None
                self.scan_active = False
                self.smooth_yaw_rate = self.smooth_pitch_rate = 0.0

            if target is not None:
                time_since = now - target["t"]
                horizon = min(0.25 + self.lead_s, max(0.0, time_since) + self.lead_s)
                cx = target["cx"] + target["vx"] * horizon
                cy = target["cy"] + target["vy"] * horizon
                frame_w, frame_h = target["w"], target["h"]

                err_x = (cx - (frame_w / 2.0)) / (frame_w / 2.0)
                err_y = (cy - (frame_h / 2.0)) / (frame_h / 2.0)

                # Deadband: prevent micro-twitches and sensor drift around center
                if abs(err_x) < self.deadband:
                    err_x = 0.0
                    d_err_x = 0.0
                    self.prev_err_x = 0.0
                    raw_yaw_rate = 0.0
                else:
                    d_err_x = (err_x - self.prev_err_x) / dt
                    self.prev_err_x = err_x
                    sign_x = 1.0 if err_x >= 0 else -1.0
                    curved_err_x = sign_x * (abs(err_x) ** 1.15)
                    raw_yaw_rate = (self.kp_yaw * curved_err_x) + (self.kd_yaw * d_err_x)

                if abs(err_y) < self.deadband:
                    err_y = 0.0
                    d_err_y = 0.0
                    self.prev_err_y = 0.0
                    raw_pitch_rate = 0.0
                else:
                    d_err_y = (err_y - self.prev_err_y) / dt
                    self.prev_err_y = err_y
                    sign_y = 1.0 if err_y >= 0 else -1.0
                    curved_err_y = sign_y * (abs(err_y) ** 1.15)
                    raw_pitch_rate = (self.kp_pitch * curved_err_y) + (self.kd_pitch * d_err_y)

                if self.invert_yaw:
                    raw_yaw_rate = -raw_yaw_rate
                if self.invert_pitch:
                    raw_pitch_rate = -raw_pitch_rate

                raw_yaw_rate = max(-self.max_rate, min(self.max_rate, raw_yaw_rate))
                raw_pitch_rate = max(-self.max_rate, min(self.max_rate, raw_pitch_rate))

                # Dynamic responsiveness: increase smoothing alpha up to 0.65 when moving to eliminate lag
                err_mag = math.sqrt(err_x * err_x + err_y * err_y)
                dyn_alpha = min(0.65, max(self.alpha_smooth, self.alpha_smooth + 0.45 * err_mag))
                self.smooth_yaw_rate = (dyn_alpha * raw_yaw_rate) + ((1.0 - dyn_alpha) * self.smooth_yaw_rate)
                self.smooth_pitch_rate = (dyn_alpha * raw_pitch_rate) + ((1.0 - dyn_alpha) * self.smooth_pitch_rate)

                angle_offset_x = (-err_x if self.invert_yaw else err_x) * (HFOV_DEG / 2.0)
                angle_offset_y = err_y * (VFOV_DEG / 2.0)
                step_gain = 0.15
                self.target_yaw_deg   = max(-160.0, min(160.0, self.target_yaw_deg + (angle_offset_x * step_gain * dt * 25.0)))
                self.target_pitch_deg = max(-90.0,  min(25.0,  self.target_pitch_deg - (angle_offset_y * step_gain * dt * 25.0)))
            elif self.scan_active:
                # ── Environment Scan Mode: Smooth panoramic sweep searching for humans ──
                scan_elapsed = now - self.scan_start_t
                ramp = min(1.0, max(0.05, scan_elapsed / 2.0))
                if self.scan_mode == "continuous":
                    scan_rate = ramp * self.scan_speed
                else:
                    # Sinusoidal oscillating sweep covering both left and right from center
                    scan_rate = ramp * self.scan_speed * math.cos(2.0 * math.pi * scan_elapsed / self.scan_period)
                raw_yaw_rate = -scan_rate if self.invert_yaw else scan_rate
                raw_pitch_rate = 0.0

                self.smooth_yaw_rate   = (0.25 * raw_yaw_rate) + (0.75 * self.smooth_yaw_rate)
                self.smooth_pitch_rate = (0.25 * raw_pitch_rate) + (0.75 * self.smooth_pitch_rate)
                self.prev_err_x = 0.0
                self.prev_err_y = 0.0
                self.target_yaw_deg = max(-135.0, min(135.0, self.target_yaw_deg + (self.smooth_yaw_rate * dt)))
            else:
                # SEARCHING: Immediately stop all gimbal actuation (no unwanted movement)
                self.prev_err_x = 0.0
                self.prev_err_y = 0.0
                self.smooth_yaw_rate = 0.0
                self.smooth_pitch_rate = 0.0

            self.seq = (self.seq + 1) & 0xFFFF
            t_tx = time.perf_counter()
            self._transmit(self.smooth_pitch_rate, self.smooth_yaw_rate,
                           self.target_pitch_deg, self.target_yaw_deg)
            self.last_tx_ms = (time.perf_counter() - t_tx) * 1000.0

            hz_count += 1
            if now - hz_timer >= 1.0:
                self.actual_hz = hz_count / (now - hz_timer)
                hz_count = 0
                hz_timer = now

            elapsed = time.monotonic() - now
            time.sleep(max(0.001, self.cmd_interval - elapsed))

    def _log_tx_error(self, where, exc, interval=10.0):
        """Log actuator errors at most once every `interval` seconds per backend."""
        count = self._tx_error_counts.get(where, 0) + 1
        self._tx_error_counts[where] = count
        now = time.monotonic()
        if now - self._tx_error_last_log.get(where, 0.0) >= interval:
            self._tx_error_last_log[where] = now
            self._tx_log.warning("[Gimbal] %s failed (%d times so far): %s",
                                 where, count, exc)
        else:
            self._tx_log.debug("[Gimbal] %s failed: %s", where, exc)

    def _transmit(self, pitch_rate, yaw_rate, pitch_deg, yaw_deg):
        if self.preview_only:
            return
        # AERIXIS link: an absolute angle move is in progress; zero-rate packets would cancel it.
        if time.monotonic() < getattr(self, "rate_pause_until", 0.0):
            return
        # ── 1. Direct SIYI UDP Command (Ethernet) ────────────────────────────
        if (self.mode in ("siyi_udp", "auto")) and self.siyi_active:
            try:
                yaw_speed   = int(np.clip(yaw_rate * (100.0 / self.max_rate), -self.siyi_command_cap, self.siyi_command_cap))
                pitch_speed = int(np.clip(pitch_rate * (100.0 / self.max_rate), -self.siyi_command_cap, self.siyi_command_cap))
                pkt = build_siyi_speed_packet(yaw_speed, pitch_speed, self.seq)
                self.udp_sock.sendto(pkt, (self.siyi_ip, self.siyi_port))
                now = time.monotonic()
                units = (yaw_speed, pitch_speed)
                previous = getattr(self, "_logged_speed", None)
                if (previous is None or (units == (0, 0) and previous != units) or
                        now - getattr(self, "_speed_log_t", 0.) >= 1.0):
                    self._tx_log.info("SIYI TX request cmd=0x07 yaw=%d pitch=%d seq=%d packet=%s (receipt unverified)",
                                      yaw_speed, pitch_speed, self.seq, pkt.hex())
                    self._logged_speed = units
                    self._speed_log_t = now
            except OSError as exc:
                self._log_tx_error("SIYI UDP send", exc)
            except Exception as exc:
                self._log_tx_error("SIYI UDP packet build", exc)

        # ── 2. MAVLink Actuation (Serial / UDP to Flight Controller) ─────────
        if self.mav_master is not None and self.mav_active:
            target_sys  = getattr(self.mav_master, "target_system", 1) or 1
            if target_sys <= 0:
                target_sys = 1
            target_comp = getattr(self.mav_master, "target_component", 1) or 1
            if target_comp <= 0:
                target_comp = 1

            # Mode A: Gimbal Manager v2 Rate Control (PX4 / ArduPilot 4.3+)
            if self.mode in ("auto", "mavlink_rate"):
                try:
                    if hasattr(self.mav_master.mav, "gimbal_manager_set_pitchyaw_send"):
                        self.mav_master.mav.gimbal_manager_set_pitchyaw_send(
                            target_sys,
                            target_comp,
                            0,                  # flags
                            0,                  # gimbal_device_id (0 = default/all)
                            float("nan"),       # pitch angle NaN = rate control
                            float("nan"),       # yaw angle NaN = rate control
                            math.radians(pitch_rate),  # rad/s
                            math.radians(yaw_rate)     # rad/s
                        )
                    else:
                        self.mav_master.mav.command_long_send(
                            target_sys,
                            target_comp,
                            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                            0,
                            float("nan"), float("nan"),
                            float(pitch_rate), float(yaw_rate),
                            0, 0, 0
                        )
                except Exception as exc:
                    self._log_tx_error("MAVLink gimbal_rate", exc)

            # Mode B: Classic Mount Control (ArduPilot MNT1_TYPE)
            elif self.mode == "mavlink_mount":
                try:
                    self.mav_master.mav.command_long_send(
                        target_sys,
                        target_comp,
                        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                        0,
                        float(pitch_deg), # degrees
                        0.0,
                        float(yaw_deg),   # degrees
                        0, 0, 0,
                        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING
                    )
                except Exception as exc:
                    self._log_tx_error("MAVLink mount_control", exc)

    def center_gimbal(self):
        """Do not issue a position move from the tracking application."""
        print("[Gimbal] Center shortcut disabled in this tracking-only build.")

    def get_status_str(self):
        backend_str = "PREVIEW: NO COMMANDS" if self.preview_only else "SIYI UDP REQUEST"
        if self.thermal_fault:
            return f"Gimbal [{backend_str} | THERMAL STOP]"
        with self.state_lock:
            target = self.last_target
        if target is None or time.monotonic() - target["observed_t"] > self.observation_timeout:
            return f"Gimbal [{backend_str} {self.actual_hz:.0f}Hz | NO FRESH TARGET]"
        if self.scan_active:
            return f"Gimbal [{backend_str} {self.actual_hz:.0f}Hz | SCANNING]: Yaw {self.smooth_yaw_rate:+.1f} deg/s"
        yaw_units = int(np.clip(self.smooth_yaw_rate * 100.0 / self.max_rate,
                                -self.siyi_command_cap, self.siyi_command_cap))
        pitch_units = int(np.clip(self.smooth_pitch_rate * 100.0 / self.max_rate,
                                  -self.siyi_command_cap, self.siyi_command_cap))
        return f"Gimbal [{backend_str} {self.actual_hz:.0f}Hz] SDK yaw:{yaw_units:+d} pitch:{pitch_units:+d}"

    def start_scan(self):
        print("[Gimbal] Scanning disabled: movement requires a fresh selected target.")

    def stop_scan(self):
        with self.state_lock:
            self.scan_active = False

    def toggle_scan(self):
        self.stop_scan()
        self.start_scan()

    def is_scanning(self) -> bool:
        return self.scan_active

    def toggle_invert_yaw(self):
        self.invert_yaw = not self.invert_yaw
        print(f"🔄 [Gimbal] Invert Yaw: {'ON' if self.invert_yaw else 'OFF'}")

    def toggle_invert_pitch(self):
        self.invert_pitch = not self.invert_pitch
        print(f"🔄 [Gimbal] Invert Pitch: {'ON' if self.invert_pitch else 'OFF'}")

    def stop(self):
        self.running = False
        self.control_thread.join(timeout=1.0)
        if not self.control_thread.is_alive():
            for _ in range(3):
                self._transmit(0.0, 0.0, self.target_pitch_deg, self.target_yaw_deg)
                time.sleep(0.02)
            self.udp_sock.close()



# ─────────────────────────────────────────────────────────────────────────────
#  LATENCY PROFILER (Microsecond-accurate per-step timing & diagnostics)
# ─────────────────────────────────────────────────────────────────────────────
class LatencyProfiler:
    """
    Step-by-step latency profiler for real-time vision and gimbal visual servoing.
    Measures elapsed time for each pipeline stage:
      1. RTSP Grab / Camera Ingestion
      2. Asynchronous YOLO Submission & Background Inference
      3. Multi-Object Tracking (OC-SORT Kalman)
      4. Center-Lock Decision
      5. Gimbal Control Vector Hand-off
      6. Telemetry Serialization & UDP Transmission
      7. Annotation & HUD Drawing
      8. GUI Presentation (cv2.imshow + cv2.waitKey)
      9. Total Frame Cycle
    """
    def __init__(self, ema_alpha: float = 0.15):
        self.alpha = ema_alpha
        self.cur: dict[str, float] = {}
        self.ema: dict[str, float] = {}

    def record(self, step: str, dur_ms: float):
        self.cur[step] = dur_ms
        if step not in self.ema:
            self.ema[step] = dur_ms
        else:
            self.ema[step] = (self.alpha * dur_ms) + ((1.0 - self.alpha) * self.ema[step])

    def get(self, step: str) -> float:
        return self.cur.get(step, 0.0)

    def get_ema(self, step: str) -> float:
        return self.ema.get(step, 0.0)

    def hud_str(self) -> str:
        grab   = self.get("grab")
        yolo   = self.get("yolo_worker")
        track  = self.get("track")
        draw   = self.get("draw")
        render = self.get("render")
        total  = self.get("total")
        return (f"Grab:{grab:.1f}ms | YOLO:{yolo:.0f}ms | Track:{track:.2f}ms | "
                f"Draw:{draw:.1f}ms | UI:{render:.1f}ms | Total:{total:.1f}ms")

    def log_line(self, frame_idx: int, fps: float) -> str:
        grab     = self.get("grab")
        grab_avg = self.get_ema("grab")
        yolo     = self.get("yolo_worker")
        track    = self.get("track")
        lock     = self.get("lock")
        gimbal   = self.get("gimbal")
        telem    = self.get("telem")
        draw     = self.get("draw")
        render   = self.get("render")
        total    = self.get("total")
        return (
            f"[LATENCY F{frame_idx:05d} {fps:4.1f}FPS] "
            f"Total:{total:4.1f}ms | Grab:{grab:4.1f}ms(avg:{grab_avg:4.1f}) | "
            f"YOLO-Worker:{yolo:4.0f}ms | Track:{track:4.2f}ms | Lock:{lock:4.2f}ms | "
            f"Gimbal:{gimbal:4.2f}ms | Telem:{telem:4.2f}ms | Draw:{draw:4.1f}ms | GUI:{render:4.1f}ms"
        )


# ─────────────────────────────────────────────────────────────────────────────
#  LOW-LATENCY SIYI A8 MINI RTSP FRAME GRABBER
# ─────────────────────────────────────────────────────────────────────────────
RTSP_OPEN_TIMEOUT_MS = 5000
# A read that waits longer than this ends the session and reconnects. Without it a stalled
# stream blocked cap.read() forever: the A8 mini stops a session whose reader falls behind its
# 25 fps, and a power blip kills it silently; the tracker then froze with the camera still up
# (Pi 5 + A8 mini, 2026-10-25: video stopped after ~30 s, every time, until a restart).
RTSP_READ_TIMEOUT_MS = 2000


def open_rtsp(url):
    """cv2.VideoCapture over FFmpeg with open/read timeouts (OpenCV >= 4.6), else without."""
    params = []
    for name, value in (('CAP_PROP_OPEN_TIMEOUT_MSEC', RTSP_OPEN_TIMEOUT_MS),
                        ('CAP_PROP_READ_TIMEOUT_MSEC', RTSP_READ_TIMEOUT_MS)):
        if hasattr(cv2, name):
            params += [getattr(cv2, name), value]
    return cv2.VideoCapture(url, cv2.CAP_FFMPEG, params) if params else cv2.VideoCapture(url, cv2.CAP_FFMPEG)


class LowLatencyA8Grabber:
    """
    Background-threaded low-latency RTSP grabber for the SIYI A8 Mini.
    Decodes at the source resolution, then resizes for processing.
    Notifies read() on each new decoded frame; never promises a particular FPS.
    A stalled stream times out after RTSP_READ_TIMEOUT_MS and is reconnected.
    """
    def __init__(self, stream_urls, target_w=640, target_h=360):
        self.stream_urls = stream_urls if isinstance(stream_urls, list) else [stream_urls]
        self.target_w    = target_w
        self.target_h    = target_h
        self.latest_frame = None
        self.latest_frame_t = self.consumed_frame_t = 0.0
        self.cond         = threading.Condition()
        self.has_new_frame = False
        self.running      = True
        self.connected    = False
        self.active_url   = self.stream_urls[0]
        self.cap          = None
        self.reconnects   = 0

        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            "rtsp_transport;tcp|fflags;nobuffer|max_delay;50000|flags;low_delay"
        )
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _try_connect(self):
        for url in self.stream_urls:
            print(f"📡 [RTSP] Connecting to {url} …")
            cap = open_rtsp(url)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"✅ [RTSP] Connected → {frame.shape[1]}×{frame.shape[0]}")
                    self.active_url = url
                    self.cap        = cap
                    self.connected  = True
                    return True
            cap.release()
        return False

    def _worker(self):
        while self.running:
            if not self.connected:
                if not self._try_connect():
                    time.sleep(1.0)
                    continue
            try:
                if not self.running or self.cap is None:
                    break
                ret, frame = self.cap.read()
            except Exception:
                break

            if ret and frame is not None:
                if self.target_w > 0 and self.target_h > 0:
                    if frame.shape[1] != self.target_w or frame.shape[0] != self.target_h:
                        frame = cv2.resize(frame, (self.target_w, self.target_h),
                                           interpolation=cv2.INTER_LINEAR)
                with self.cond:
                    self.latest_frame_t = time.monotonic()
                    self.latest_frame = frame
                    self.has_new_frame = True
                    self.cond.notify()
            else:
                if not self.running:
                    break
                self.reconnects += 1
                print(f"⚠️  [RTSP] Stream interrupted or stalled > {RTSP_READ_TIMEOUT_MS} ms "
                      f"(reconnect #{self.reconnects}). Reconnecting …")
                self.connected = False
                if self.cap:
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                time.sleep(0.5)

    def read(self, timeout=0.04):
        with self.cond:
            self.cond.wait_for(lambda: self.has_new_frame or not self.running, timeout)
            if self.running and self.has_new_frame and self.latest_frame is not None:
                self.has_new_frame = False
                self.consumed_frame_t = self.latest_frame_t
                frame = self.latest_frame.copy()
                self.latest_frame = None
                return True, frame
            return False, None

    def release(self):
        self.running = False
        with self.cond:
            self.cond.notify_all()
        if hasattr(self, "_thread") and self._thread.is_alive():
            try:
                self._thread.join(timeout=0.5)
            except Exception:
                pass
        if self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None


# ─────────────────────────────────────────────────────────────────────────────
#  THREADED HIGH-SPEED USB WEBCAM GRABBER (30+ FPS Smooth Capture)
# ─────────────────────────────────────────────────────────────────────────────
class ThreadedWebcamGrabber:
    """
    Dedicated high-speed threaded USB webcam grabber.
    Uses DirectShow (Windows) / V4L2 (Linux) with MJPG and buffer size 1.
    Decouples frame capture from vision processing; actual FPS depends on hardware.
    """
    def __init__(self, camera_id=0, target_w=640, target_h=360):
        self.camera_id = camera_id
        self.target_w = target_w
        self.target_h = target_h
        self.latest_frame = None
        self.latest_frame_t = self.consumed_frame_t = 0.0
        self.lock = threading.Lock()
        self.running = True

        if sys.platform == "win32":
            self.cap = cv2.VideoCapture(camera_id, cv2.CAP_DSHOW)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(camera_id, cv2.CAP_MSMF)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(camera_id)
        else:
            self.cap = cv2.VideoCapture(camera_id)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        if target_w > 0 and target_h > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_h)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        ret, frame = self.cap.read()
        if ret and frame is not None:
            if target_w > 0 and target_h > 0:
                if frame.shape[1] != target_w or frame.shape[0] != target_h:
                    frame = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
            self.latest_frame = frame
            self.latest_frame_t = time.monotonic()

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        print(f"📹 [Webcam Grabber] Dedicated capture worker online (Cam {camera_id})")

    def _worker(self):
        while self.running:
            try:
                if not self.running or self.cap is None:
                    break
                ret, frame = self.cap.read()
            except Exception:
                break
            if ret and frame is not None:
                if self.target_w > 0 and self.target_h > 0:
                    if frame.shape[1] != self.target_w or frame.shape[0] != self.target_h:
                        frame = cv2.resize(frame, (self.target_w, self.target_h), interpolation=cv2.INTER_LINEAR)
                with self.lock:
                    self.latest_frame_t = time.monotonic()
                    self.latest_frame = frame
            else:
                time.sleep(0.005)

    def read(self):
        with self.lock:
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()
                self.latest_frame = None
                self.consumed_frame_t = self.latest_frame_t
                return True, frame
            return False, None

    def release(self):
        self.running = False
        if hasattr(self, "_thread") and self._thread.is_alive():
            try:
                self._thread.join(timeout=0.5)
            except Exception:
                pass
        if hasattr(self, 'cap') and self.cap:
            try:
                self.cap.release()
            except Exception:
                pass
            self.cap = None



# ─────────────────────────────────────────────────────────────────────────────
#  CUSTOM KALMAN / IOU TRACKER (legacy OCSORT class names)
# ─────────────────────────────────────────────────────────────────────────────
class OCSORTTrack:
    _next_id = 0

    def __init__(self, bbox, conf, timestamp):
        OCSORTTrack._next_id += 1
        self.id   = OCSORTTrack._next_id

        w  = max(1.0, float(bbox[2] - bbox[0]))
        h  = max(1.0, float(bbox[3] - bbox[1]))
        cx = float(bbox[0]) + w / 2.0
        cy = float(bbox[1]) + h / 2.0
        s  = w * h
        r  = w / h

        # State vector: [cx, cy, s, r, vx, vy, vs]
        self.x = np.array([cx, cy, s, r, 0., 0., 0.], dtype=np.float32)
        self.P = np.diag([10., 10., 50., 1., 100., 100., 100.]).astype(np.float32)
        self.Q = np.diag([1.,  1.,  2.,  1e-2, 1e-1, 1e-1, 1e-2]).astype(np.float32)
        self.R = np.diag([2.,  2.,  20., 1e-1]).astype(np.float32)

        self.state_time = self.observed_t = timestamp
        self.time_since_update = 0
        self.hits              = 1
        self.hit_streak        = 1
        self.age               = 0
        self.conf              = float(conf)
        self.history           = deque([(int(cx), int(cy))], maxlen=30)
        self.last_observed_bbox = list(bbox[:4])
        self.velocity = np.array([0.0, 0.0])

    def predict(self, timestamp):
        dt = max(0.0, timestamp - self.state_time)
        F = np.eye(7, dtype=np.float32)
        F[0, 4] = F[1, 5] = F[2, 6] = dt
        self.x = F @ self.x
        self.x[2] = max(10.0, self.x[2])
        Q = np.zeros((7, 7), dtype=np.float32)
        for pos, vel, variance in ((0, 4, 400.0), (1, 5, 400.0), (2, 6, 2500.0)):
            Q[pos, pos] = variance * dt**3 / 3
            Q[pos, vel] = Q[vel, pos] = variance * dt**2 / 2
            Q[vel, vel] = variance * dt
        Q[3, 3] = 0.01 * dt
        self.P = F @ self.P @ F.T + Q
        self.state_time = timestamp
        self.age += 1
        self.time_since_update += 1

    def update(self, bbox, conf):
        self.observed_t = self.state_time
        self.time_since_update = 0
        self.hits        += 1
        self.hit_streak  += 1
        self.conf         = float(conf)

        w  = max(1.0, float(bbox[2] - bbox[0]))
        h  = max(1.0, float(bbox[3] - bbox[1]))
        cx = float(bbox[0]) + w / 2.0
        cy = float(bbox[1]) + h / 2.0
        s  = w * h
        r  = w / h
        z  = np.array([cx, cy, s, r], dtype=np.float32)

        self.last_observed_bbox = list(bbox[:4])

        y = z - self.x[:4]
        S = self.P[:4, :4] + self.R
        K = self.P[:, :4] @ np.linalg.inv(S)
        self.x += K @ y
        H = np.eye(4, 7, dtype=np.float32)
        A = np.eye(7, dtype=np.float32) - K @ H
        self.P = A @ self.P @ A.T + K @ self.R @ K.T

        cx_int = int(self.x[0])
        cy_int = int(self.x[1])
        self.history.append((cx_int, cy_int))

    def get_bbox(self):
        cx, cy, s, r = float(self.x[0]), float(self.x[1]), float(self.x[2]), float(self.x[3])
        s = max(10.0, s)
        r = max(0.1, min(10.0, r))
        w = np.sqrt(s * r)
        h = s / w
        return [cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0]


def _iou(b1, b2):
    x1 = max(b1[0], b2[0]);  y1 = max(b1[1], b2[1])
    x2 = min(b1[2], b2[2]);  y2 = min(b1[3], b2[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    a1 = max(1.0, (b1[2]-b1[0])*(b1[3]-b1[1]))
    a2 = max(1.0, (b2[2]-b2[0])*(b2[3]-b2[1]))
    union = a1 + a2 - inter
    return inter / union if union > 0 else 0.0


class OCSORTMultiTracker:
    def __init__(self,
                 max_age:    int   = MAX_AGE,
                 min_hits:   int   = MIN_HITS,
                 iou_thresh: float = IOU_ASSOC_THRESH,
                 conf_init:  float = CONF_INIT_TRACK,
                 min_aspect: float = MIN_ASPECT_RATIO,
                 max_aspect: float = MAX_ASPECT_RATIO,
                 min_h_px:   int   = MIN_HEIGHT_PX):
        self.max_age    = max_age
        self.min_hits   = min_hits
        self.iou_thresh = iou_thresh
        self.conf_init  = conf_init
        self.min_aspect = min_aspect
        self.max_aspect = max_aspect
        self.min_h_px   = min_h_px
        self.tracks: list[OCSORTTrack] = []

    def _geometry_ok(self, x1, y1, x2, y2):
        w = max(1.0, x2 - x1);  h = max(1.0, y2 - y1)
        asp = h / w
        return (self.min_aspect <= asp <= self.max_aspect) and (h >= self.min_h_px)

    def update(self, detections=None, timestamp=None):
        timestamp = time.monotonic() if timestamp is None else timestamp
        self.tracks = [t for t in self.tracks
                       if timestamp - t.observed_t <= LOCK_ID_HOLD_S]
        for trk in self.tracks:
            trk.predict(timestamp)

        if detections is not None:
            detections = [d for d in detections
                          if self._geometry_ok(d[0], d[1], d[2], d[3])]

            matched_trks = set()
            matched_dets = set()

            if self.tracks and detections:
                iou_mat = np.zeros((len(detections), len(self.tracks)), dtype=np.float32)
                for di, det in enumerate(detections):
                    for ti, trk in enumerate(self.tracks):
                        iou_mat[di, ti] = _iou(det[:4], trk.get_bbox())

                while True:
                    if iou_mat.size == 0:
                        break
                    best = np.max(iou_mat)
                    if best < self.iou_thresh:
                        break
                    di, ti = np.unravel_index(np.argmax(iou_mat), iou_mat.shape)
                    self.tracks[ti].update(detections[di][:4], detections[di][4])
                    matched_trks.add(ti)
                    matched_dets.add(di)
                    iou_mat[di, :] = -1.0
                    iou_mat[:, ti] = -1.0

            # Recheck real observed boxes when a velocity prediction drifted.
            # This is a conservative fallback, not the full OC-SORT algorithm.
            unmatched_dets = [i for i in range(len(detections)) if i not in matched_dets]
            unmatched_trks = [i for i in range(len(self.tracks)) if i not in matched_trks]
            pairs = [(di, ti) for di in unmatched_dets for ti in unmatched_trks
                     if _iou(detections[di][:4], self.tracks[ti].last_observed_bbox) >= 0.35]
            for di, ti in pairs:
                if sum(d == di for d, t in pairs) != 1 or sum(t == ti for d, t in pairs) != 1:
                    continue
                old = self.tracks[ti]
                # A recovery after prediction drift must not retain that drift.
                recovered = OCSORTTrack(detections[di][:4], detections[di][4], timestamp)
                recovered.id = old.id
                recovered.hits = old.hits + 1
                self.tracks[ti] = recovered
                matched_trks.add(ti)
                matched_dets.add(di)

            for di, det in enumerate(detections):
                if di not in matched_dets and float(det[4]) >= self.conf_init:
                    self.tracks.append(OCSORTTrack(det[:4], det[4], timestamp))

        surviving = []
        for trk in self.tracks:
            if timestamp - trk.observed_t > LOCK_ID_HOLD_S:
                continue
            surviving.append(trk)
        self.tracks = surviving
        return self.snapshot(timestamp)

    def snapshot(self, timestamp):
        """Predict for the current display without advancing measurement state."""
        active = []
        for trk in self.tracks:
            if timestamp - trk.observed_t > OBSERVATION_TIMEOUT_S:
                continue
            if trk.hits >= self.min_hits:
                dt = max(0.0, min(0.25, timestamp - trk.state_time))
                estimate = trk.x.copy()
                estimate[0] += estimate[4] * dt
                estimate[1] += estimate[5] * dt
                area = max(10.0, float(estimate[2]))
                ratio = max(0.1, min(10.0, float(estimate[3])))
                width = math.sqrt(area * ratio)
                height = area / width
                bbox = [int(estimate[0] - width / 2), int(estimate[1] - height / 2),
                        int(estimate[0] + width / 2), int(estimate[1] + height / 2)]
                active.append({
                    "id":       trk.id,
                    "box":      bbox,
                    "conf":     trk.conf,
                    "history":  list(trk.history),
                    "coast":    trk.observed_t != timestamp,
                    "state_time": timestamp,
                    "observed_t": trk.observed_t,
                    "observed_box": list(trk.last_observed_bbox),
                    "age":      trk.age,
                    "velocity": (float(trk.x[4]), float(trk.x[5])),
                })
        return active


# ─────────────────────────────────────────────────────────────────────────────
#  PERMANENT CENTER LOCK STATE MACHINE
# ─────────────────────────────────────────────────────────────────────────────
class CenterLockStateMachine:
    UNLOCKED    = "UNLOCKED"
    ACQUIRING   = "ACQUIRING"
    LOCKED      = "LOCKED"
    COAST       = "COAST"
    REACQUIRE   = "REACQUIRING"

    def __init__(self, frame_w: int, frame_h: int, coast_budget: int = LOCK_COAST_BUDGET):
        self.frame_w      = frame_w
        self.frame_h      = frame_h
        self.coast_budget = coast_budget

        self.state        = self.UNLOCKED
        self.locked_id    = None
        self.coast_count  = 0
        self.last_box     = None
        self.lock_conf    = 0.0
        self.last_seen_t  = 0.0
        self.last_observed_box = None
        self._clear_recovery()

    def _center_dist(self, box):
        cx = (box[0] + box[2]) / 2.0
        cy = (box[1] + box[3]) / 2.0
        dx = cx - self.frame_w / 2.0
        dy = cy - self.frame_h / 2.0
        return np.sqrt(dx * dx + dy * dy)

    def _is_out_of_frame(self, box, margin=0):
        x1, y1, x2, y2 = box
        return (x2 <= margin or x1 >= self.frame_w - margin or
                y2 <= margin or y1 >= self.frame_h - margin)

    def update(self, active_tracks: list) -> dict | None:
        track_by_id = {t["id"]: t for t in active_tracks}

        if self.state in (self.LOCKED, self.COAST):
            if self.locked_id in track_by_id:
                trk = track_by_id[self.locked_id]
                box = trk["box"]

                # Prediction can cross an edge even while the person remains
                # visible. Only an observed full exit can release this lock.
                observed_box = trk.get("observed_box")
                if observed_box is not None and self._is_out_of_frame(observed_box):
                    print(f"🔓 [Lock] ID {self.locked_id} exited frame → REACQUIRING")
                    self.state     = self.REACQUIRE
                    self.locked_id = None
                    self.last_box  = None
                    return self._reacquire([t for t in active_tracks if t["id"] != trk["id"]])

                self.coast_count = 0
                self.state       = self.LOCKED
                self.last_box    = box
                self.lock_conf   = trk["conf"]
                self.last_seen_t = trk["observed_t"]
                self.last_observed_box = list(trk.get("observed_box", box))
                self._clear_recovery()
                trk["lock_state"] = self.LOCKED
                return trk
            else:
                # A missed detection is not evidence that the person left.
                # Hold the selection without commanding stale coordinates.
                self.state = self.COAST
                return self._recover_short_gap(active_tracks)

        return self._reacquire(active_tracks)

    def _clear_recovery(self):
        self.recovery_id = None
        self.recovery_hits = 0
        self.recovery_observed_t = float("-inf")

    def _recover_short_gap(self, active_tracks):
        """Conservative geometric recovery; does not prove personal identity."""
        now = time.monotonic()
        # A lone, spatially consistent candidate is required. In a crowd or
        # after a long absence, keep stopped and require explicit selection.
        if (self.last_observed_box is None or len(active_tracks) != 1 or
                now - self.last_seen_t > RECONNECT_WINDOW_S):
            self._clear_recovery()
            return None
        candidate = active_tracks[0]
        observed_t = candidate["observed_t"]
        box = candidate.get("observed_box", candidate["box"])
        old = self.last_observed_box
        ow, oh = max(1., old[2]-old[0]), max(1., old[3]-old[1])
        nw, nh = max(1., box[2]-box[0]), max(1., box[3]-box[1])
        dx = abs((box[0]+box[2]-old[0]-old[2])/2.)
        dy = abs((box[1]+box[3]-old[1]-old[3])/2.)
        if (not 0 <= now-observed_t <= OBSERVATION_TIMEOUT_S or
                observed_t <= self.last_seen_t or self._is_out_of_frame(box) or
                not .5 <= nw/ow <= 2. or not .5 <= nh/oh <= 2. or
                dx > .75*ow or dy > .5*oh):
            self._clear_recovery()
            return None
        if candidate["id"] != self.recovery_id:
            self._clear_recovery()
            self.recovery_id = candidate["id"]
        if observed_t <= self.recovery_observed_t:
            return None  # Repeated display of one detection is not confirmation.
        self.recovery_observed_t = observed_t
        self.recovery_hits += 1
        if self.recovery_hits < RECONNECT_HITS:
            return None
        previous = self.locked_id
        self.locked_id = candidate["id"]
        print(f"[Lock] Short-gap recovery: ID {previous} -> {self.locked_id}; "
              f"{self.recovery_hits} fresh observations (geometric match, not re-identification)")
        return self.update([candidate])

    def reset(self):
        """Explicitly release a selection whose identity can no longer be verified."""
        self.state = self.UNLOCKED
        self.locked_id = None
        self.last_box = None
        self.last_seen_t = 0.0
        self.last_observed_box = None
        self._clear_recovery()

    def _reacquire(self, active_tracks):
        active_tracks = [t for t in active_tracks
                         if not self._is_out_of_frame(t.get("observed_box", t["box"]))]
        if not active_tracks:
            self.state = self.UNLOCKED
            return None

        # Lock onto nearest person to optical center
        best = min(active_tracks, key=lambda t: self._center_dist(t["box"]))
        self.locked_id   = best["id"]
        self.last_box    = best["box"]
        self.lock_conf   = best["conf"]
        self.last_seen_t = best["observed_t"]
        self.last_observed_box = list(best.get("observed_box", best["box"]))
        self._clear_recovery()
        self.coast_count = 0
        self.state       = self.LOCKED
        best["lock_state"] = self.LOCKED
        print(f"🎯 [Center-Lock] LOCKED target ID {self.locked_id} "
              f"(Offset={self._center_dist(best['box']):.1f}px)!")
        return best


# ─────────────────────────────────────────────────────────────────────────────
#  YOLO26n NCNN INFERENCER (FP16 Enabled)
# ─────────────────────────────────────────────────────────────────────────────
def _letterbox(img, size=INPUT_SIZE):
    """Fit img into the model input: size is (width, height), or one number for a square."""
    return detector_models.letterbox(img, (size, size) if isinstance(size, (int, float)) else tuple(size))


def real_height_mm(track):
    """Typical real height of the track's class for the distance estimate (None: unknown class)."""
    metres = REAL_HEIGHT_M.get(str(track.get("label", "person")).lower())
    return None if metres is None else metres * 1000.0


def distance_m(track, frame_h, ndigits=2):
    height_mm = real_height_mm(track)
    if height_mm is None:
        return None
    x1, y1, x2, y2 = track["box"]
    h_px = max(1.0, float(y2 - y1))
    dist = round((FOCAL_LENGTH_MM * height_mm * frame_h) / (SENSOR_HEIGHT_MM * h_px * 1000.0), ndigits)
    return max(1.0, min(50.0, dist))


# Scheduling priority (2026-10-25): with the flight software, video and AI on one Pi 5 the CPU is
# full, and the detector (1 thread, its own cores 2-3) got ~25 % of a core from the AS2 Python
# nodes that also run there: one detection waited ~140 ms for ~25 ms of work. A lower nice value
# makes the scheduler serve it first; the tracker loop (video decode) gets a smaller boost so
# frames never queue up. Needs CAP_SYS_NICE (docker compose cap_add); without it, nothing changes.
DETECTOR_NICE = -5
TRACKER_NICE = -2


def raise_priority(nice, who):
    try:
        os.nice(nice)        # threads started afterwards (NCNN, video) inherit it
        return True
    except (PermissionError, OSError) as exc:
        print(f"[priority] {who}: could not set nice {nice} ({exc}); running at normal priority")
        return False


def _inference_process(conn, model_dir, threads, min_person_height=MIN_HEIGHT_PX, cores=None, classes=None):
    """Own NCNN in a separate process so its CPU work cannot stall servo timing.
    cores: pin this process to its own cores (--detector-cores). Pi 5, 2026-10-23: sharing cores 2-3
    with video decoding and the tracker loop made one inference 450 ms instead of 240 ms."""
    try:
        if cores:
            os.sched_setaffinity(0, set(cores))
        raise_priority(DETECTOR_NICE, 'detector')
        import signal
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # Parent owns Ctrl+C cleanup.
        cv2.setNumThreads(1)
        inferencer = detector_models.make_detector(model_dir, threads=threads,
                                                   min_person_height=min_person_height, classes=classes)
        conn.send(("ready", inferencer.info))
        while True:
            task = conn.recv()
            if task is None:
                break
            frame_id, frame_t, frame, conf, _size = task
            dets, labels, model_ms = inferencer.detect(frame, conf_thresh=conf)
            conn.send(("result", frame_id, frame_t, dets, model_ms,
                       time.monotonic(), inferencer.last_profile, labels))
    except (EOFError, KeyboardInterrupt):
        pass
    except Exception as exc:
        try:
            conn.send(("error", f"{type(exc).__name__}: {exc}"))
        except (OSError, EOFError):
            pass
    finally:
        conn.close()


class AsyncYOLOInferencer:
    """One inference in flight; subsequent input always comes from latest capture."""
    def __init__(self, model_dir, threads, min_person_height=MIN_HEIGHT_PX, cores=None, classes=None):
        ctx = mp.get_context("spawn")
        self.conn, child = ctx.Pipe()
        self.process = ctx.Process(
            target=_inference_process,
            args=(child, model_dir, threads, min_person_height, cores, classes), daemon=True)
        self.process.start()
        child.close()
        self.busy = False
        if not self.conn.poll(60.0):    # NPU runtimes (RKNN/TensorRT) load slower than NCNN
            self.stop()
            raise RuntimeError("Detector process did not become ready")
        reply = self.conn.recv()
        if reply[0] != "ready":
            self.stop()
            raise RuntimeError(f"Detector startup failed: {reply}")
        self.info = reply[1]

    def submit(self, frame_id, frame_t, frame, conf_thresh, imgsz):
        if self.busy:
            return False
        self.conn.send((frame_id, frame_t, frame, conf_thresh, imgsz))
        self.busy = True
        return True

    def get_latest(self):
        if not self.conn.poll():
            if not self.process.is_alive():
                raise RuntimeError("NCNN inference process exited")
            return None
        reply = self.conn.recv()
        self.busy = False
        if reply[0] == "error":
            raise RuntimeError(f"NCNN inference failed: {reply[1]}")
        if reply[0] != "result":
            raise RuntimeError(f"Unexpected NCNN message: {reply[0]}")
        return reply[1:]

    def stop(self):
        try:
            if self.process.is_alive() and not self.busy:
                try:
                    self.conn.send(None)
                except (OSError, EOFError, ValueError):
                    # Windows may close the child pipe while Ctrl+C propagates.
                    # Still join/terminate the worker and finish parent cleanup.
                    pass
            self.process.join(timeout=1.0)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=1.0)
        finally:
            self.conn.close()


# ─────────────────────────────────────────────────────────────────────────────
#  TELEMETRY & PACKETS (UDP to Aerostac2)
# ─────────────────────────────────────────────────────────────────────────────
def build_telemetry_packet(timestamp, fps, all_tracks, locked_track,
                           lock_state, frame_w, frame_h, gimbal_status="", extra=None):
    locked_payload = None
    if locked_track is not None:
        x1, y1, x2, y2 = locked_track["box"]
        cx   = (x1 + x2) / 2.0
        cy   = (y1 + y2) / 2.0
        h_px = max(1.0, float(y2 - y1))

        err_x  = round((cx - frame_w / 2.0) / (frame_w / 2.0), 4)
        err_y  = round((cy - frame_h / 2.0) / (frame_h / 2.0), 4)

        locked_payload = {
            "id":     locked_track["id"],
            "err_x":  err_x,
            "err_y":  err_y,
            "dist_m": distance_m(locked_track, frame_h),
            "conf":   round(locked_track["conf"], 2),
            "state":  lock_state,
            "label":  locked_track.get("label", "person"),
            "box":    [round(float(v), 1) for v in locked_track["box"]],
        }

    all_targets = []
    for t in all_tracks:
        x1, y1, x2, y2 = t["box"]
        cx   = (x1 + x2) / 2.0
        cy   = (y1 + y2) / 2.0
        h_px = max(1.0, float(y2 - y1))
        err_x  = round((cx - frame_w / 2.0) / (frame_w / 2.0), 4)
        err_y  = round((cy - frame_h / 2.0) / (frame_h / 2.0), 4)
        all_targets.append({
            "id": t["id"], "err_x": err_x, "err_y": err_y,
            "dist_m": distance_m(t, frame_h), "conf": round(t["conf"], 2),
            "label": t.get("label", "person"),
            "box": [round(float(v), 1) for v in t["box"]],
        })

    packet = {
        "timestamp":       round(timestamp, 3),
        "fps":             round(fps, 1),
        "lock_state":      lock_state,
        "gimbal_status":   gimbal_status,
        "locked_target":   locked_payload,
        "all_targets":     all_targets,
        "targets_count":   len(all_targets),
    }
    if extra:  # AERIXIS link: build id, frame size, gimbal attitude and suppression state
        packet.update(extra)
    return json.dumps(packet)


# ─────────────────────────────────────────────────────────────────────────────
#  MJPEG WEB STREAMER
# ─────────────────────────────────────────────────────────────────────────────
class _MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, *args): pass
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            self.wfile.write(
                b"<html><head><title>Pi5 SIYI A8 Mini Tracker</title></head>"
                b"<body style='margin:0;background:#111;display:flex;justify-content:center;align-items:center;height:100vh;'>"
                b"<img src='/stream.mjpg' style='max-width:100%;max-height:100vh;border:2px solid #333;border-radius:6px;'>"
                b"</body></html>"
            )
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.end_headers()
            with self.server.viewer_lock:
                self.server.viewers += 1
            try:
                last_sent = None
                while True:
                    frame = getattr(self.server, "latest_jpeg", None)
                    if frame is not None and frame is not last_sent:
                        self.wfile.write(b"--FRAME\r\n")
                        self.send_header("Content-Type",  "image/jpeg")
                        self.send_header("Content-Length", str(len(frame)))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        last_sent = frame
                    time.sleep(0.035)
            except Exception:
                pass
            finally:
                with self.server.viewer_lock:
                    self.server.viewers -= 1


def _start_mjpeg_server(port=8080):
    srv = ThreadingHTTPServer(("0.0.0.0", port), _MJPEGHandler)
    srv.daemon_threads = True
    srv.latest_jpeg = None
    # Open /stream.mjpg connections. The HUD is drawn and JPEG-encoded only while someone watches
    # (Pi 5, 2026-10-23: drawing + 15 fps encoding with no viewer took CPU from the detector).
    srv.viewers = 0
    srv.viewer_lock = threading.Lock()
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"🌐 [Web Stream] Live HUD View: http://<pi_ip>:{port}")
    return srv


class LatestJPEGEncoder:
    """Encode only the newest preview image off the control/display loop."""
    def __init__(self, server):
        self.server = server
        self.lock = threading.Lock()
        self.event = threading.Event()
        self.latest = None
        self.running = True
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def submit(self, frame):
        with self.lock:
            self.latest = frame
            self.event.set()

    def _worker(self):
        while self.running:
            if not self.event.wait(timeout=0.1):
                continue
            with self.lock:
                frame = self.latest
                self.latest = None
                self.event.clear()
            if frame is not None:
                ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    self.server.latest_jpeg = jpg.tobytes()
            # Limit preview encode cost while always replacing queued work.
            time.sleep(1.0 / 15.0)

    def stop(self):
        self.running = False
        self.event.set()
        self.thread.join(timeout=0.5)


# ─────────────────────────────────────────────────────────────────────────────
#  VISUALIZATION & HIGH-VISIBILITY HUD
# ─────────────────────────────────────────────────────────────────────────────
_COLOR_CACHE = {}
_LOCK_COLORS = {
    CenterLockStateMachine.LOCKED:    (0, 0, 255),     # Red in OpenCV BGR
    CenterLockStateMachine.COAST:     (0, 215, 255),   # Golden Cyan
    CenterLockStateMachine.ACQUIRING: (0, 215, 255),   # Amber
    CenterLockStateMachine.REACQUIRE: (0, 100, 255),   # Orange
    CenterLockStateMachine.UNLOCKED:  (160,160,160),   # Grey
}


def _track_color(tid):
    if tid not in _COLOR_CACHE:
        np.random.seed(tid * 37)
        b, g, r = (int(c) for c in np.random.randint(80, 240, 3))
        # Never red-dominant (BGR): red marks the locked target only.
        _COLOR_CACHE[tid] = (r, g, b) if r > max(b, g) else (b, g, r)
    return _COLOR_CACHE[tid]


def draw_corner_brackets(img, box, color, length=14, thickness=2):
    x1, y1, x2, y2 = box
    # Top-Left
    cv2.line(img, (x1, y1), (x1 + length, y1), color, thickness)
    cv2.line(img, (x1, y1), (x1, y1 + length), color, thickness)
    # Top-Right
    cv2.line(img, (x2, y1), (x2 - length, y1), color, thickness)
    cv2.line(img, (x2, y1), (x2, y1 + length), color, thickness)
    # Bottom-Left
    cv2.line(img, (x1, y2), (x1 + length, y2), color, thickness)
    cv2.line(img, (x1, y2), (x1, y2 - length), color, thickness)
    # Bottom-Right
    cv2.line(img, (x2, y2), (x2 - length, y2), color, thickness)
    cv2.line(img, (x2, y2), (x2, y2 - length), color, thickness)


def draw_frame(frame, all_tracks, locked_track, lock_state,
               fps, ncnn_latency_ms, is_keyframe, frame_w, frame_h,
               gimbal_status="", cpu_pct=None,
               inv_yaw=False, inv_pitch=True, flip_h=False,
               latency_str="", is_scanning=False, held_lock_id=None):
    LOCKED_COLOR = _LOCK_COLORS.get(lock_state, (180, 180, 180))
    locked_id = locked_track["id"] if locked_track else None
    ocx, ocy = frame_w // 2, frame_h // 2

    # 1. Optical Center & Deadband Crosshairs
    db_w = int(frame_w * 0.020)
    db_h = int(frame_h * 0.020)
    cv2.rectangle(frame, (ocx - db_w, ocy - db_h), (ocx + db_w, ocy + db_h), (80, 80, 80), 1)
    cv2.drawMarker(frame, (ocx, ocy), (200, 200, 200), cv2.MARKER_CROSS, 16, 1, cv2.LINE_AA)

    # 2. Draw Tracks
    for trk in all_tracks:
        tid = trk["id"]
        x1, y1, x2, y2 = trk["box"]
        history = trk.get("history", [])
        is_locked = (tid == locked_id)

        color     = LOCKED_COLOR if is_locked else _track_color(tid)
        thickness = 2            if is_locked else 1

        # Draw bounding box & corner brackets
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        draw_corner_brackets(frame, (x1, y1, x2, y2), color, length=12, thickness=thickness+1)

        dist_m = distance_m(trk, frame_h, 1)
        label  = trk.get("label", "person")

        if is_locked:
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            err_x = (cx - ocx) / (frame_w / 2.0)
            err_y = (cy - ocy) / (frame_h / 2.0)
            score_text = (f"Nano:{trk['conf']:.2f}" if trk.get("source") == "OC-SORT + NanoTrack"
                          else f"{int(trk['conf']*100)}%")
            tag = f"[LOCKED] {label} ID:{tid} {score_text} [{lock_state}]"
        else:
            tag = f"{label} ID:{tid} {int(trk['conf']*100)}%" + (f" {dist_m}m" if dist_m else "")

        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.44, 1)
        cv2.rectangle(frame, (x1, y1 - 20), (x1 + tw + 6, y1), color, -1)
        cv2.putText(frame, tag, (x1 + 3, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 0, 0), 1, cv2.LINE_AA)

    # 3. Only the red box marks the locked target.
    # 4. Top Telemetry HUD Banner
    banner_h = 104 if latency_str else 86
    cv2.rectangle(frame, (0, 0), (frame_w, banner_h), (10, 10, 10), -1)
    mode_tag = f"YOLO result ({ncnn_latency_ms:.0f}ms)" if is_keyframe else f"YOLO last ({ncnn_latency_ms:.0f}ms)"
    cpu_str = f" | CPU:{cpu_pct:.0f}%" if cpu_pct is not None else ""

    cv2.putText(frame,
                f"A8 dualtrack2  |  FPS: {fps:.1f}  |  {mode_tag}  |  Targets: {len(all_tracks)}{cpu_str}",
                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)

    if locked_track is not None:
        x1, y1, x2, y2 = locked_track["box"]
        tcx, tcy = (x1 + x2) // 2, (y1 + y2) // 2
        ex = (tcx - ocx) / (frame_w / 2.0)
        ey = (tcy - ocy) / (frame_h / 2.0)
        lock_info = f"TARGET LOCKED ID:{locked_id}  Err:({ex:+.2f}, {ey:+.2f})"
        cv2.putText(frame, lock_info, (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1, cv2.LINE_AA)
    elif lock_state == CenterLockStateMachine.COAST and held_lock_id is not None:
        cv2.putText(frame,
                    f"TARGET ID:{held_lock_id} HELD | POSITION UNCERTAIN: STOP | R:RESELECT",
                    (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 165, 255), 1, cv2.LINE_AA)
    elif is_scanning:
        cv2.putText(frame,
                    "SCANNING FOR HUMAN",
                    (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 240, 255), 1, cv2.LINE_AA)
        sweep_phase = (math.sin(time.monotonic() * 2.5) + 1.0) / 2.0
        sx = int(10 + sweep_phase * (frame_w - 20))
        cv2.circle(frame, (sx, banner_h - 2), 3, (0, 240, 255), -1)
    else:
        cv2.putText(frame,
                    "SEARCHING: NO TARGET LOCKED",
                    (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 180, 255), 1, cv2.LINE_AA)

    cv2.putText(frame, gimbal_status, (8, 54), cv2.FONT_HERSHEY_SIMPLEX,
                0.38, (220, 220, 220), 1, cv2.LINE_AA)

    if latency_str:
        cv2.putText(frame,
                    f"⚡ LATENCY: {latency_str}",
                    (8, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 240, 255), 1, cv2.LINE_AA)
        key_y = 90
    else:
        key_y = 72

    s_tag = "ON" if is_scanning else "OFF"
    y_tag = "ON" if inv_yaw else "OFF"
    p_tag = "ON" if inv_pitch else "OFF"
    h_tag = "ON" if flip_h else "OFF"
    hud_keys = f"[Y] Yaw:{y_tag} [P] Pitch:{p_tag} [H] Flip:{h_tag} [R] Reselect [Q] Exit"
    hud_color = (0, 240, 255) if is_scanning else ((120, 255, 120) if inv_yaw else (100, 180, 255))
    cv2.putText(frame, hud_keys, (8, key_y), cv2.FONT_HERSHEY_SIMPLEX, 0.35, hud_color, 1, cv2.LINE_AA)

    if locked_track is not None:
        x1, y1, x2, y2 = locked_track["box"]
        # Keep the full red border visible even when its top crosses the HUD.
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

    return frame


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN PIPELINE
# ─────────────────────────────────────────────────────────────────────────────
def _cleanup_resources(actions, log):
    """Attempt every cleanup action in order, even if one resource has failed."""
    failures = 0
    for name, action in actions:
        try:
            action()
        except Exception:
            failures += 1
            log.exception("Cleanup failed for %s; continuing remaining cleanup", name)
    return failures


def run(
    model_dir:    str,
    target_w:     int   = 640,
    target_h:     int   = 360,
    rtsp_url:     str   = None,
    video_file:   str   = None,
    use_webcam:   bool  = False,
    camera_id:    int   = 0,
    flip_h:       bool  = False,
    flip_v:       bool  = False,
    cores:        tuple = (2, 3),
    threads:      int   = 2,
    skip:         int   = KEYFRAME_SKIP,
    conf_thresh:  float = CONF_THRESH,
    headless:     bool  = False,
    stream_port:  int   = 0,
    output_path:  str   = None,
    gimbal_mode:  str   = "siyi_udp",
    mavlink_conn: str   = "none",
    mavlink_baud: int   = 115200,
    siyi_ip:      str   = DEFAULT_A8_IP,
    gimbal_kp_yaw: float = 20.0,
    gimbal_kp_pitch: float = 16.0,
    gimbal_kd_yaw: float = 0.0,
    gimbal_kd_pitch: float = 0.0,
    gimbal_max_rate: float = 30.0,
    siyi_command_cap: int = 35,
    gimbal_lead_s: float = 0.0,
    gimbal_invert_yaw: bool = False,
    gimbal_invert_pitch: bool = True,
    scan_on_start: bool = False,
    scan_speed: float = 18.0,
    scan_period: float = 18.0,
    scan_mode: str = "sweep",
    latency_interval: int = 30,
    min_person_height: float = MIN_HEIGHT_PX,
    as2_link: bool = True,
    control_port: int = CONTROL_PORT,
    telemetry_port: int = TELEMETRY_PORT,
    attitude_in_preview: bool = False,
    detector_cores: tuple = None,
    classes: list = None,
    lock_classes: list = None,
    nano_dir: str = None,
):
    set_cpu_affinity(cores=cores)
    configure_thread_env(threads)
    raise_priority(TRACKER_NICE, 'tracker')

    log = logging.getLogger("tracker")
    log.info("Starting vision pipeline: cores=%s detector_cores=%s threads=%d skip=%d conf=%.2f res=%dx%d",
             list(cores), list(detector_cores or cores), threads, skip, conf_thresh, target_w, target_h)
    # Check the model (files, input size, classes, runtime on this board) before any centre command.
    model_info = detector_models.describe(model_dir)
    if not detector_models.runtime_available(model_info["format"]):
        raise RuntimeError("Model %s is %s; its runtime is not installed on this board"
                           % (model_info["name"], model_info["format"]))
    class_ids = detector_models.class_ids(model_info, classes)
    lock_ids = detector_models.class_ids(model_info, lock_classes)
    if class_ids is not None and lock_ids is not None and not set(lock_ids) <= set(class_ids):
        raise ValueError("Lock classes must be among the detected classes")
    names = model_info["names"]
    log.info("Model %s (%s) input %dx%d; detect %s; lock %s; person height cutoff=%.1f px",
             model_info["name"], model_info["format"], model_info["input"][0], model_info["input"][1],
             "all %d classes" % len(names) if class_ids is None else [names[c] for c in class_ids],
             "any detected" if lock_ids is None else [names[c] for c in lock_ids], min_person_height)

    # Load NanoTrack before any center command. If its models/runtime are
    # missing, fail without moving the gimbal.
    dual_tracker = DualTracker(nano_dir or model_dir, width=target_w, height=target_h,
                               names=names, lock_classes=lock_ids)

    monitor = ResourceMonitor(interval=HEALTH_INTERVAL_S)
    monitor.start()

    # Windows has no $DISPLAY but always has a GUI; Linux/Pi needs one.
    if os.name == "nt":
        has_display = not headless
    else:
        has_display = bool(os.environ.get("DISPLAY")) and not headless
    # No implicit JPEG workload in headless mode.

    # UDP telemetry socket for Aerostac2
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"📡 [Telemetry] Broadcasting to 127.0.0.1:{telemetry_port}")

    # Unified Gimbal Controller (Direct SIYI UDP + MAVLink)
    gimbal_controller = UnifiedGimbalController(
        mode         = gimbal_mode,
        mavlink_conn = mavlink_conn,
        mavlink_baud = mavlink_baud,
        siyi_ip      = siyi_ip,
        siyi_port    = DEFAULT_SIYI_UDP,
        kp_yaw       = gimbal_kp_yaw,
        kp_pitch     = gimbal_kp_pitch,
        kd_yaw       = gimbal_kd_yaw,
        kd_pitch     = gimbal_kd_pitch,
        max_rate     = gimbal_max_rate,
        siyi_command_cap = siyi_command_cap,
        lead_s       = gimbal_lead_s,
        defer_start = True,
        invert_yaw   = gimbal_invert_yaw,
        invert_pitch = gimbal_invert_pitch,
        scan_on_start = scan_on_start,
        scan_speed   = scan_speed,
        scan_period  = scan_period,
        scan_mode    = scan_mode,
    )

    # AERIXIS link: attitude feedback (read-only) and onboard mission commands.
    mission_link = None
    if as2_link:
        attitude_enabled = (not gimbal_controller.preview_only) or attitude_in_preview
        mission_link = MissionLink(siyi_ip, DEFAULT_SIYI_UDP, attitude_enabled, control_port, log)
        log.info("AERIXIS link: commands on 127.0.0.1:%d, attitude queries %s",
                 control_port, "on" if attitude_enabled else "off (preview)")

    # Center once before starting detection or camera capture, then verify SIYI
    # attitude feedback. In preview, no gimbal command is ever sent.
    if not gimbal_controller.preview_only:
        try:
            log.info("Startup: requesting SIYI center and checking attitude before video/detection")
            aligned = center_and_verify(gimbal_controller.udp_sock,
                                       (siyi_ip, DEFAULT_SIYI_UDP))
            gimbal_controller.seq = aligned['sequence']
            log.info("Startup center verified: yaw %.1f°, pitch %.1f°", aligned['yaw'], aligned['pitch'])
        except Exception:
            gimbal_controller.udp_sock.close()
            if mission_link:
                mission_link.close()
            monitor.stop();udp_sock.close()
            raise
    gimbal_controller.start()

    # NCNN starts only after verified centering; detection follows camera setup.
    try:
        async_inferencer = AsyncYOLOInferencer(model_dir, threads, min_person_height, cores=detector_cores,
                                               classes=class_ids)
    except Exception:
        gimbal_controller.stop()
        if mission_link:
            mission_link.close()
        monitor.stop()
        udp_sock.close()
        raise

    frame_history = deque(maxlen=48)
    frame_by_id = {}

    grabber = None
    cap     = None

    if video_file:
        print(f"🎬 [Source] Video file: {video_file}")
        cap = cv2.VideoCapture(video_file)
        # Play at the file's frame rate, like a camera: the tracker's freshness limits (0.4 s for
        # NanoTrack, 0.6 s for detections) assume real time, not 800 fps decoding.
        video_period = 1.0 / (cap.get(cv2.CAP_PROP_FPS) or 25.0)
        next_video_t = time.monotonic()
    elif use_webcam:
        print(f"📹 [Source] High-FPS Threaded USB Webcam (Device Index {camera_id})")
        grabber = ThreadedWebcamGrabber(camera_id=camera_id, target_w=target_w, target_h=target_h)
    else:
        urls    = [rtsp_url] if rtsp_url else [A8_MAIN_STREAM]
        print(f"📡 [Source] SIYI A8 Mini Ethernet RTSP ({urls[0]})")
        grabber = LowLatencyA8Grabber(urls, target_w=target_w, target_h=target_h)

    mjpeg_srv  = _start_mjpeg_server(stream_port) if stream_port > 0 else None
    preview_encoder = LatestJPEGEncoder(mjpeg_srv) if mjpeg_srv else None
    out_writer = None
    frame_idx  = 0
    frame_errors = 0
    prev_time  = time.monotonic()
    ncnn_lat   = 0.0
    cpu_pct    = 0.0
    last_cpu_sample = time.monotonic()
    lock_state = CenterLockStateMachine.UNLOCKED
    all_tracks: list = []
    locked_track     = None
    last_result_id = -1
    detector_results = stale_results = reset_results = 0
    selection_reset_t = 0.0
    profiler = LatencyProfiler()

    print("\n[Startup] Workers started; waiting for the first video frame. Tracking needs fresh detections.\n")

    try:
        while True:
            if gimbal_controller.thermal_fault:
                raise RuntimeError("Thermal stop: cool the Pi and inspect power/cooling before restart")
            if not gimbal_controller.control_thread.is_alive():
                raise RuntimeError("Gimbal control worker stopped")
            t_frame_start = time.perf_counter()

            # ── Step 1: Camera / RTSP Frame Ingestion ─────────────────────────
            t0 = time.perf_counter()
            if grabber is not None:
                ret, frame = grabber.read()
                if not ret or frame is None:
                    time.sleep(0.002)
                    continue
            else:
                ret, frame = cap.read()
                if not ret:
                    break
                next_video_t += video_period
                time.sleep(max(0.0, next_video_t - time.monotonic()))
                if target_w > 0 and target_h > 0:
                    if frame.shape[1] != target_w or frame.shape[0] != target_h:
                        frame = cv2.resize(frame, (target_w, target_h),
                                           interpolation=cv2.INTER_LINEAR)

            if flip_h:
                frame = cv2.flip(frame, 1)
            if flip_v:
                frame = cv2.flip(frame, 0)

            t_grab_ms = (time.perf_counter() - t0) * 1000.0
            profiler.record("grab", t_grab_ms)

            frame_idx += 1
            fh, fw = frame.shape[:2]

            if frame_idx == 1:
                print(f"📐 [Init] Frame size: {fw}×{fh} — upstream OC-SORT + NanoTrack ready.")

            if output_path and out_writer is None:
                os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
                out_writer = cv2.VideoWriter(
                    output_path, cv2.VideoWriter_fourcc(*"mp4v"), 25.0, (fw, fh))

            # Timestamp is decode completion, not exposure time.
            observation_t = getattr(grabber, "consumed_frame_t", time.monotonic()) if grabber else time.monotonic()
            history_item=(frame_idx,observation_t,frame.copy())
            frame_history.append(history_item);frame_by_id[frame_idx]=history_item
            oldest=frame_history[0][0]
            frame_by_id={fid:item for fid,item in frame_by_id.items() if fid>=oldest}
            t1 = time.perf_counter()
            result = async_inferencer.get_latest()
            has_new = is_keyframe = False
            if result is not None:
                result_id, result_t, dets, lat, completed_t, detector_profile, labels = result
                detector_results += 1
                if result_id <= last_result_id:
                    raise RuntimeError("Out-of-order inference result")
                last_result_id = result_id
                ncnn_lat = lat
                profiler.record("yolo_worker", (completed_t - result_t) * 1000.0)
                for name, value in detector_profile.items():
                    profiler.record(name, value)
                profiler.record("result_age", (time.monotonic()-result_t)*1000.)
                if result_t >= selection_reset_t and time.monotonic() - result_t <= OBSERVATION_TIMEOUT_S:
                    has_new = is_keyframe = True
                elif result_t < selection_reset_t:
                    reset_results += 1
                else:
                    stale_results += 1
            if not async_inferencer.busy:
                async_inferencer.submit(frame_idx, observation_t, frame,
                                        conf_thresh=conf_thresh, imgsz=async_inferencer.info["input"])
            profiler.record("yolo_sub", (time.perf_counter() - t1) * 1000.0)
            t2 = time.perf_counter()
            try:
                dual_result=None
                if has_new:
                    source_item=frame_by_id.get(result_id)
                    if source_item is not None:
                        dual_result=(result_id,result_t,dets,source_item[2],labels)
                all_tracks,locked_track,lock_state=dual_tracker.process(
                    frame,frame_idx,observation_t,dual_result,tuple(frame_history))
            except Exception:
                frame_errors+=1
                if frame_errors==1 or frame_errors%25==0:
                    log.exception("Dual tracker failed (consecutive=%d, frame=%d)",frame_errors,frame_idx)
                if frame_errors>=MAX_CONSECUTIVE_ERRORS:raise
                time.sleep(.005);continue
            else:frame_errors=0
            t_track_ms=(time.perf_counter()-t2)*1000.0
            profiler.record("track", t_track_ms)

            # AERIXIS link: release/suppress the selection on request of the onboard mission.
            if mission_link is not None:
                if mission_link.handle(gimbal_controller, locked_track):
                    selection_reset_t = time.monotonic()
                    dual_tracker.reset()
                    all_tracks = []
                    locked_track = None
                if mission_link.suppressed:
                    locked_track = None

            # ── Step 5: Autonomous 50 Hz Gimbal Servoing Hand-off ──────────────
            t4 = time.perf_counter()
            if gimbal_controller is not None:
                try:
                    gimbal_controller.update(locked_track, fw, fh)
                except Exception:
                    log.exception("Gimbal target update failed at frame %d", frame_idx)
            gimbal_status_str = gimbal_controller.get_status_str() if gimbal_controller else ""
            t_gimbal_ms = (time.perf_counter() - t4) * 1000.0
            profiler.record("gimbal", t_gimbal_ms)

            # ── Step 6: Telemetry Broadcast ───────────────────────────────────
            now  = time.monotonic()
            fps  = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now

            if HAS_PSUTIL and (now - last_cpu_sample >= 1.0):
                cpu_pct = psutil.cpu_percent()
                last_cpu_sample = now

            t5 = time.perf_counter()
            try:
                extra = {"build": BUILD_ID, "frame_w": fw, "frame_h": fh,
                         "selected_label": dual_tracker.selected_id,
                         "selected_class": names.get(dual_tracker.selected_cls) if dual_tracker.selected_cls is not None else None,
                         "model": model_info["name"],
                         "preview_only": gimbal_controller.preview_only}
                if mission_link is not None:
                    extra.update(mission_link.telemetry())
                packet = build_telemetry_packet(now, fps, all_tracks, locked_track,
                                                lock_state if locked_track is not None or mission_link is None
                                                or not mission_link.suppressed else "SUPPRESSED",
                                                fw, fh, gimbal_status_str, extra)
                udp_sock.sendto(packet.encode("utf-8"), ("127.0.0.1", telemetry_port))
            except Exception:
                log.debug("Telemetry send failed", exc_info=True)
            t_telem_ms = (time.perf_counter() - t5) * 1000.0
            profiler.record("telem", t_telem_ms)

            # ── Step 7 & 8: Visual HUD Annotation & GUI Presentation ──────────
            preview_wanted = mjpeg_srv is not None and mjpeg_srv.viewers > 0
            if has_display or preview_wanted or out_writer:
                try:
                    t6 = time.perf_counter()
                    vis = draw_frame(
                        frame.copy(), all_tracks, locked_track,
                        lock_state, fps, ncnn_lat, is_keyframe, fw, fh,
                        gimbal_status_str, cpu_pct=cpu_pct,
                        inv_yaw=gimbal_controller.invert_yaw if gimbal_controller else False,
                        inv_pitch=gimbal_controller.invert_pitch if gimbal_controller else True,
                        flip_h=flip_h,
                        latency_str=profiler.hud_str(),
                        is_scanning=gimbal_controller.is_scanning() if gimbal_controller else False,
                        held_lock_id=dual_tracker.selected_id if dual_tracker else None,
                    )
                    t_draw_ms = (time.perf_counter() - t6) * 1000.0
                    profiler.record("draw", t_draw_ms)

                    if out_writer:
                        out_writer.write(vis)

                    if preview_encoder and preview_wanted:
                        preview_encoder.submit(vis)

                    t7 = time.perf_counter()
                    if has_display:
                        cv2.imshow("A8 Mini OC-SORT + NanoTrack", vis)
                        key = cv2.waitKey(1) & 0xFF
                        if key in (ord("q"), 27):
                            print("\n[INFO] Stopped by user.")
                            break
                        elif key == ord("s"):
                            if gimbal_controller:
                                gimbal_controller.toggle_scan()
                        elif key == ord("r"):
                            selection_reset_t = time.monotonic()
                            dual_tracker.reset()
                            all_tracks = []
                            locked_track = None
                            gimbal_controller.update(None, fw, fh)
                        elif key in (ord("h"), ord("m")):
                            flip_h = not flip_h
                            print(f"[INFO] Horizontal Mirror (Flip): {'ON' if flip_h else 'OFF'}")
                        elif key == ord("v"):
                            flip_v = not flip_v
                            print(f"[INFO] Vertical Flip: {'ON' if flip_v else 'OFF'}")
                        elif key == ord("y"):
                            if gimbal_controller:
                                gimbal_controller.toggle_invert_yaw()
                        elif key == ord("p"):
                            if gimbal_controller:
                                gimbal_controller.toggle_invert_pitch()
                        elif key == ord("c"):
                            gimbal_controller.center_gimbal()
                    t_render_ms = (time.perf_counter() - t7) * 1000.0
                    profiler.record("render", t_render_ms)

                except Exception:
                    log.exception("Visualisation / web-stream failed at frame %d", frame_idx)

            profiler.record("observation_age", (time.monotonic() - observation_t) * 1000.0)
            if frame_idx % latency_interval == 0:
                log.info("Decoded-image age at loop end: %.1f ms (excludes camera/network/decode)", profiler.get("observation_age"))

            # ── Step 9: Total Loop Time ───────────────────────────────────────
            t_total_ms = (time.perf_counter() - t_frame_start) * 1000.0
            profiler.record("total", t_total_ms)

            # ── Step 10: Periodic Step-by-Step Latency Breakdown Log ──────────
            if frame_idx % latency_interval == 0:
                log.info(profiler.log_line(frame_idx, fps) + f" | Lock:{lock_state}(ID:{dual_tracker.selected_id}) | {gimbal_status_str}")
                log.info("Detector pre=%.2fms infer=%.2fms post=%.2fms total=%.2fms "
                         "result_age=%.1fms | Nano update=%.2fms last_seed=%.2fms "
                         "seeds=%d replay=%d score=%.3f | %s",
                         profiler.get("pre_ms"), profiler.get("infer_ms"),
                         profiler.get("post_ms"), profiler.get("detect_ms"),
                         profiler.get("result_age"), dual_tracker.nano_ms,
                         dual_tracker.seed_ms, dual_tracker.seed_count,
                         dual_tracker.replay_count, dual_tracker.score, dual_tracker.reason)
                log.info("Detector results=%d discarded_stale=%d discarded_before_reset=%d "
                         "last_confident_candidates=%d last_geometry_rejected=%d last_kept=%d",
                         detector_results, stale_results, reset_results,
                         profiler.get("confident_candidates"), profiler.get("geometry_rejected"),
                         profiler.get("kept_detections"))
                log.info("Size filter: top_score=%.3f threshold=%.1fpx confident_candidate_height=%.1f..%.1fpx "
                         "height_rejected=%d aspect_rejected=%d",
                         profiler.get("top_score"), min_person_height, profiler.get("candidate_h_min"),
                         profiler.get("candidate_h_max"), profiler.get("height_rejected"),
                         profiler.get("aspect_rejected"))

    except KeyboardInterrupt:
        log.warning("KeyboardInterrupt — stopping pipeline.")
    except Exception:
        log.critical("Fatal error in vision pipeline", exc_info=True)
        raise
    finally:
        # Stop the actuator first. A broken inference pipe or failed resource
        # cleanup must not skip this stop or the remaining cleanup actions.
        actions = [
            ("gimbal", gimbal_controller.stop),
            ("inference", async_inferencer.stop),
            ("monitor", monitor.stop),
        ]
        if grabber: actions.append(("capture", grabber.release))
        if cap: actions.append(("video", cap.release))
        if out_writer: actions.append(("recording", out_writer.release))
        if mission_link is not None:
            actions.append(("AERIXIS link", mission_link.close))
        if preview_encoder: actions.append(("preview encoder", preview_encoder.stop))
        if mjpeg_srv:
            actions.extend([("web stream", mjpeg_srv.shutdown),
                            ("web socket", mjpeg_srv.server_close)])
        if has_display: actions.append(("display", cv2.destroyAllWindows))
        actions.append(("telemetry", udp_sock.close))
        failures = _cleanup_resources(actions, log)
        log.info("Vision pipeline shutdown finished (%d cleanup errors).", failures)
        flush_and_fsync()



# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(
        description="Pi5 + SIYI A8 Mini — Official OC-SORT + NanoTrack person following",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--version", action="version", version=BUILD_ID)
    p.add_argument("--model",   default="model",
                   help="Path to NCNN model directory")
    p.add_argument("--rtsp",    default=None,
                   help="Override RTSP URL for SIYI A8 Mini")
    p.add_argument("--res",     default="640x360",
                   choices=["640x360","960x540","1280x720","512x512","720p","1080p"],
                   help="Capture & processing resolution")
    p.add_argument("--video",   default=None,    help="Test with a local video file")
    p.add_argument("--webcam",  action="store_true", help="Use USB webcam")
    p.add_argument("--camera-id", type=int, default=0, help="Webcam device index (default: 0)")
    p.add_argument("--flip-h",  action="store_true", help="Mirror/flip image horizontally (fixes left-right inversion)")
    p.add_argument("--flip-v",  action="store_true", help="Flip image vertically (fixes upside-down camera)")
    p.add_argument("--cores",   nargs="+", type=int, default=[1, 2, 3],
                   help="CPU cores to isolate vision (default matches run.sh: 1 2 3)")
    p.add_argument("--threads", type=int,  default=3,   help="NCNN inference threads")
    p.add_argument("--skip",    type=int,  default=KEYFRAME_SKIP,
                   help="Compatibility option; inference uses newest frame when worker is free")
    p.add_argument("--conf",    type=float, default=CONF_THRESH, help="Detection confidence threshold")
    p.add_argument("--headless",action="store_true", help="No GUI (for flight)")
    p.add_argument("--stream",  type=int,  default=0,
                   help="Enable MJPEG web stream on port (e.g. 8080)")
    p.add_argument("--output",  default=None,    help="Save annotated video to file")

    # Gimbal Mode & Connections
    p.add_argument("--gimbal-mode", default="siyi_udp",
                   choices=["siyi_udp"],
                   help="Verified launch path: direct SIYI Ethernet UDP only")
    p.add_argument("--siyi-ip", default=DEFAULT_A8_IP,
                   help="SIYI A8 Mini IP address")
    p.add_argument("--mavlink", default="none",
                   help="MAVLink connection: 'auto', '/dev/ttyAMA0', '/dev/ttyUSB0', 'udpin:127.0.0.1:14550', or 'none'")
    p.add_argument("--baud", type=int, default=115200,
                   help="Baud rate for serial MAVLink connection")
    p.add_argument("--invert-yaw", action=argparse.BooleanOptionalAction, default=False,
                   help="Reverse yaw direction; default sends positive yaw for a target right of center")
    p.add_argument("--invert-pitch", action=argparse.BooleanOptionalAction, default=True,
                   help="Invert gimbal pitch steering direction (default: True, follow target; use --no-invert-pitch to disable)")
    p.add_argument("--kp-yaw", type=float, default=20.0, help="Gimbal Yaw Proportional Gain")
    p.add_argument("--kp-pitch", type=float, default=16.0, help="Gimbal Pitch Proportional Gain")
    p.add_argument("--kd-yaw", type=float, default=0.0, help="Gimbal Yaw Derivative Gain (damping: 6-10)")
    p.add_argument("--kd-pitch", type=float, default=0.0, help="Gimbal Pitch Derivative Gain (damping: 4-8)")
    p.add_argument("--max-rate", type=float, default=30.0, help="Internal controller normalization; not measured physical degrees/s")
    p.add_argument("--siyi-command-cap", type=int, default=35,
                   help="Maximum SIYI SDK speed magnitude (1..100), not physical deg/s; default 35")
    p.add_argument("--lead-s", type=float, default=0.0,
                   help="Gimbal look-ahead in seconds (0..0.5): aim where the target will be, from its "
                        "measured image speed. 0 = off (default)")
    p.add_argument("--scan-on-start", action=argparse.BooleanOptionalAction, default=False,
                   help="Deprecated; automatic scanning is disabled")
    p.add_argument("--scan-speed", type=float, default=18.0,
                   help="Gimbal scan sweep rate in deg/s (default: 18.0; smooth 12-25 deg/s)")
    p.add_argument("--scan-period", type=float, default=18.0,
                   help="Gimbal scan sweep period in seconds (default: 18.0s)")
    p.add_argument("--scan-mode", default="sweep", choices=["sweep", "continuous"],
                   help="Scan motion mode: 'sweep' (panoramic sinusoidal oscillating arc) or 'continuous' (360 pan)")
    p.add_argument("--latency-interval", type=int, default=30,
                   help="Log detailed step-by-step latency breakdown every N frames (default: 30; 1 = every frame)")

    # Logging & diagnostics
    p.add_argument("--log-dir", default="logs",
                   help="Directory for rotating log + crash log files")
    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
                   help="Console/log verbosity")
    p.add_argument("--no-pin", action="store_true",
                   help="Skip CPU affinity pinning (run on all cores)")
    # AERIXIS (onboard) integration
    p.add_argument("--min-person-height", type=float, default=MIN_HEIGHT_PX,
                   help="Minimum detected height in processing-image pixels (at least 12)")
    p.add_argument("--detector-cores", nargs="+", type=int, default=None,
                   help="Own cores for the NCNN detector process (default: same as --cores)")
    p.add_argument("--as2-link", action=argparse.BooleanOptionalAction, default=True,
                   help="AERIXIS link: gimbal attitude + onboard mission commands")
    p.add_argument("--control-port", type=int, default=CONTROL_PORT,
                   help="UDP port for onboard mission commands (localhost)")
    p.add_argument("--telemetry-port", type=int, default=TELEMETRY_PORT,
                   help="UDP port the tracker telemetry is sent to (localhost)")
    p.add_argument("--classes", nargs="*", default=None,
                   help="Classes to detect, by name or id (default: every class of the model)")
    p.add_argument("--lock-classes", nargs="*", default=None,
                   help="Classes the gimbal may lock on (default: every detected class)")
    p.add_argument("--nano-dir", default=None,
                   help="Folder with nano/ (NanoTrack models); default: the --model folder")
    p.add_argument("--attitude-in-preview", action="store_true",
                   help="Query gimbal attitude in preview mode too (read-only)")

    args = p.parse_args()
    if args.scan_on_start:
        p.error("Scanning is disabled in this tracking-only build")

    # Logging first: every later failure must land in the file, not just stdout.
    level = getattr(logging, args.log_level)
    log_file = setup_logging(log_dir=args.log_dir, log_name="hybrid_tracker", level=level)
    crash_file = install_crash_hooks(log_dir=args.log_dir, log_name="hybrid_tracker")
    log = logging.getLogger("tracker")
    log.info("BUILD      : %s", BUILD_ID)
    log.info("SCRIPT     : %s", Path(__file__).resolve())
    log.info("Log file   : %s", log_file)
    log.info("Crash file : %s", crash_file)

    cores = [] if args.no_pin else list(args.cores)
    if not math.isfinite(args.min_person_height) or args.min_person_height < 12:
        p.error("--min-person-height must be finite and at least 12")

    res_map = {
        "640x360": (640, 360),
        "960x540": (960, 540),   # for 960x544 models (higher altitude): see model_library.capture_size
        "1280x720": (1280, 720),
        "512x512": (512, 512),
        "720p":    (1280, 720),
        "1080p":   (1920, 1080),
    }
    tw, th = res_map.get(args.res, (640, 360))

    try:
        run(
            model_dir       = args.model,
            target_w        = tw,
            target_h        = th,
            rtsp_url        = args.rtsp,
            video_file      = args.video,
            use_webcam      = args.webcam,
            camera_id       = args.camera_id,
            flip_h          = args.flip_h,
            flip_v          = args.flip_v,
            cores           = tuple(cores),
            threads         = args.threads,
            skip            = args.skip,
            conf_thresh     = args.conf,
            headless        = args.headless,
            stream_port     = args.stream,
            output_path     = args.output,
            gimbal_mode     = args.gimbal_mode,
            mavlink_conn    = args.mavlink,
            mavlink_baud    = args.baud,
            siyi_ip         = args.siyi_ip,
            gimbal_kp_yaw   = args.kp_yaw,
            gimbal_kp_pitch = args.kp_pitch,
            gimbal_kd_yaw   = args.kd_yaw,
            gimbal_kd_pitch = args.kd_pitch,
            gimbal_max_rate = args.max_rate,
            siyi_command_cap = args.siyi_command_cap,
            gimbal_lead_s   = args.lead_s,
            gimbal_invert_yaw = args.invert_yaw,
            gimbal_invert_pitch = args.invert_pitch,
            scan_on_start   = args.scan_on_start,
            scan_speed      = args.scan_speed,
            scan_period     = args.scan_period,
            scan_mode       = args.scan_mode,
            latency_interval = args.latency_interval,
            min_person_height = args.min_person_height,
            as2_link        = args.as2_link,
            control_port    = args.control_port,
            telemetry_port  = args.telemetry_port,
            attitude_in_preview = args.attitude_in_preview,
            detector_cores  = tuple(args.detector_cores) if args.detector_cores and not args.no_pin else None,
            classes         = args.classes or None,
            lock_classes    = args.lock_classes or None,
            nano_dir        = args.nano_dir,
        )
    except SystemExit:
        raise
    except Exception:
        log.critical("Fatal startup/runtime failure — see log above.", exc_info=True)
        flush_and_fsync()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
