#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===============================================================================
Raspberry Pi 5 + SIYI A8 Mini — Hardware Gimbal Test Utility
===============================================================================
Quickly verifies that the SIYI A8 Mini gimbal moves correctly before flight.
Tests:
  1. Direct SIYI Ethernet UDP (192.168.144.25:37260)
  2. MAVLink Gimbal Manager v2 over serial telemetry
===============================================================================
"""

import sys
import time
import socket
import struct
import argparse
import os

if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

DEFAULT_A8_IP = "192.168.144.25"
SIYI_UDP_PORT = 37260

def siyi_crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def send_siyi_speed(sock, ip, port, yaw_speed, pitch_speed, seq=1):
    stx       = b'\x55\x66'
    ctrl      = b'\x01'
    data_len  = struct.pack('<H', 2)
    seq_bytes = struct.pack('<H', seq & 0xFFFF)
    cmd_id    = b'\x07'
    payload   = struct.pack('<bb', int(yaw_speed), int(pitch_speed))
    head_body = stx + ctrl + data_len + seq_bytes + cmd_id + payload
    crc = siyi_crc16(head_body)
    packet = head_body + struct.pack('<H', crc)
    sock.sendto(packet, (ip, port))

def send_siyi_center(sock, ip, port, seq=1):
    stx       = b'\x55\x66'
    ctrl      = b'\x01'
    data_len  = struct.pack('<H', 1)
    seq_bytes = struct.pack('<H', seq & 0xFFFF)
    cmd_id    = b'\x08'
    payload   = b'\x01'
    head_body = stx + ctrl + data_len + seq_bytes + cmd_id + payload
    crc = siyi_crc16(head_body)
    packet = head_body + struct.pack('<H', crc)
    sock.sendto(packet, (ip, port))


def test_siyi_udp(ip=DEFAULT_A8_IP, port=SIYI_UDP_PORT):
    print("\n" + "="*60)
    print("  🧪 TESTING DIRECT SIYI UDP GIMBAL CONTROL (Ethernet)")
    print(f"  Target: {ip}:{port}")
    print("="*60)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    try:
        # Step 1: Center
        print("1️⃣  Centering gimbal...")
        send_siyi_center(sock, ip, port, seq=1)
        time.sleep(2.0)

        # Step 2: Pan Right (+30 speed)
        print("2️⃣  Rotating RIGHT (Yaw +30%) for 1.5 seconds...")
        for i in range(15):
            send_siyi_speed(sock, ip, port, yaw_speed=30, pitch_speed=0, seq=i+2)
            time.sleep(0.1)

        # Step 3: Stop
        send_siyi_speed(sock, ip, port, yaw_speed=0, pitch_speed=0, seq=20)
        time.sleep(0.5)

        # Step 4: Pan Left (-30 speed)
        print("3️⃣  Rotating LEFT (Yaw -30%) for 1.5 seconds...")
        for i in range(15):
            send_siyi_speed(sock, ip, port, yaw_speed=-30, pitch_speed=0, seq=i+25)
            time.sleep(0.1)

        # Step 5: Stop
        send_siyi_speed(sock, ip, port, yaw_speed=0, pitch_speed=0, seq=45)
        time.sleep(0.5)

        # Step 6: Tilt Down (-25 speed)
        print("4️⃣  Tilting DOWN (Pitch -25%) for 1.5 seconds...")
        for i in range(15):
            send_siyi_speed(sock, ip, port, yaw_speed=0, pitch_speed=-25, seq=i+50)
            time.sleep(0.1)

        # Step 7: Return Center
        print("5️⃣  Returning to CENTER...")
        send_siyi_center(sock, ip, port, seq=70)
        time.sleep(2.0)

        print("\n✅ [SIYI UDP Test Complete] If the gimbal moved, Direct UDP control is 100% verified!")
    except Exception as e:
        print(f"❌ Error during SIYI UDP test: {e}")
    finally:
        sock.close()


def test_mavlink(port="auto", baud=115200):
    print("\n" + "="*60)
    print("  🧪 TESTING MAVLINK GIMBAL CONTROL (Flight Controller)")
    print("="*60)
    try:
        from pymavlink import mavutil
    except ImportError:
        print("⚠️  'pymavlink' is not installed.")
        return

    candidate_ports = ["/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0", "/dev/ttyACM0"]
    if port == "auto":
        if sys.platform == "win32":
            try:
                import serial.tools.list_ports
                for p in serial.tools.list_ports.comports():
                    desc = (p.description or "").lower()
                    if "bluetooth" not in desc and "standard serial" not in desc:
                        port = p.device
                        break
            except Exception:
                pass
        if port == "auto":
            for p in candidate_ports:
                if os.path.exists(p):
                    port = p
                    break

    if port == "auto" or not port:
        print("⚠️  No serial telemetry port found. (Plug telemetry cable into USB or UART).")
        return

    try:
        print(f"📡 Connecting to {port} at {baud} baud...")
        master = mavutil.mavlink_connection(port, baud=baud)
        print("⏳ Waiting for heartbeat...")
        hb = master.wait_heartbeat(timeout=3.0)
        if hb:
            sys_id = master.target_system if master.target_system > 0 else (hb.get_srcSystem() if hb and hb.get_srcSystem() > 0 else 1)
            comp_id = master.target_component if master.target_component > 0 else (hb.get_srcComponent() if hb and hb.get_srcComponent() > 0 else 1)
            print(f"✅ Flight controller detected! SysID: {sys_id}, CompID: {comp_id}")

            # Acquire Gimbal Manager primary control
            print("🔑 Acquiring Gimbal Manager primary control...")
            master.mav.command_long_send(
                sys_id, comp_id,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
                0,
                master.source_system, master.source_component,
                0, 0, 0, 0, 0
            )
            time.sleep(0.2)

            print("🚀 Panning RIGHT (+25 deg/s yaw) for 2.0 seconds...")
            for _ in range(40):
                master.mav.gimbal_manager_set_pitchyaw_send(
                    sys_id, comp_id,
                    0, 0,
                    float("nan"), float("nan"),
                    0.0, 25.0
                )
                time.sleep(0.05)

            print("🛑 Stopping / recentering gimbal...")
            for _ in range(10):
                master.mav.gimbal_manager_set_pitchyaw_send(
                    sys_id, comp_id,
                    0, 0,
                    float("nan"), float("nan"),
                    0.0, 0.0
                )
                time.sleep(0.05)
            master.mav.command_long_send(
                sys_id, comp_id,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                0, 0.0, 0.0, 0.0, 0.0, 0, 0, 0
            )
            print("✅ MAVLink test complete.")
        else:
            print("⚠️  No heartbeat from flight controller within 3 seconds.")
    except Exception as e:
        print(f"❌ MAVLink test error: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test SIYI A8 Mini Gimbal Movement")
    parser.add_argument("--mode", default="all", choices=["siyi", "mavlink", "all"])
    parser.add_argument("--ip", default=DEFAULT_A8_IP)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    if args.mode in ("siyi", "all"):
        test_siyi_udp(ip=args.ip)
    if args.mode in ("mavlink", "all"):
        test_mavlink(port=args.port, baud=args.baud)

