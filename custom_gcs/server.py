"""AERIXIS Ground Control Station Server (UDP + Serial Telemetry, Flight Modes, Dynamic Behavior Tree)."""

import argparse

from bt_execution import LocalTreeRun
from fleet import Fleet
import planner_bridge
import onboard_discovery
from mission_store import MissionStore
from radio_rpc import RadioClient, MESSAGE_TYPE as FLEET_MESSAGE_TYPE
import glob
import json
import re
import math
import os
import select
import socket
import subprocess
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    from pymavlink.dialects.v20 import common as mavlink
except ImportError as exc:
    raise SystemExit(
        'PyMAVLink is missing. Install python3-pip, then run '
        'python3 -m pip install --user -r as2_ground_control/requirements.txt') from exc

HOME_LATITUDE = 47.397742
HOME_LONGITUDE = 8.545594
HOME_ALTITUDE = 488.0
EARTH_RADIUS_M = 6378137.0
LOCAL_WEB = Path(__file__).resolve().parent / 'web'
if LOCAL_WEB.is_dir():
    WEB_ROOT = LOCAL_WEB
elif (Path(__file__).resolve().parent.parent / 'web').is_dir():
    WEB_ROOT = Path(__file__).resolve().parent.parent / 'web'
else:
    WEB_ROOT = Path('/home/prince/custom_gcs/web')


def offset_meters(latitude, longitude, home_latitude, home_longitude):
    """Return east/north offsets in the local tangent plane at home."""
    east = math.radians(longitude - home_longitude) * EARTH_RADIUS_M * math.cos(
        math.radians(home_latitude))
    north = math.radians(latitude - home_latitude) * EARTH_RADIUS_M
    return east, north


def point_in_polygon(point, polygon):
    """Ray-casting test for a (latitude, longitude) point in a KML ring."""
    latitude, longitude = point
    inside = False
    for index, (lat1, lon1) in enumerate(polygon):
        lat2, lon2 = polygon[(index + 1) % len(polygon)]
        if (lon1 > longitude) != (lon2 > longitude):
            crossing_lat = (lat2 - lat1) * (longitude - lon1) / (lon2 - lon1) + lat1
            if latitude < crossing_lat:
                inside = not inside
    return inside


def list_available_ports():
    """Scan and list available serial ports (USB, ACM, COM), prioritizing telemetry devices."""
    results = []
    seen = set()

    # 1. Pyserial detection
    if HAS_SERIAL:
        try:
            for p in serial.tools.list_ports.comports():
                dev = p.device
                seen.add(dev)
                desc = p.description or 'Serial Port'
                is_usb = any(k in dev.lower() for k in ('usb', 'acm')) or 'usb' in desc.lower()
                results.append({
                    'device': dev,
                    'description': desc,
                    'hwid': p.hwid or '',
                    'is_usb': is_usb
                })
        except Exception:
            pass

    # 2. Linux /dev search fallback
    for dev in sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/serial/by-id/*')):
        if dev not in seen:
            seen.add(dev)
            results.append({
                'device': dev,
                'description': 'USB Telemetry / Flight Controller',
                'hwid': '',
                'is_usb': True
            })

    # Sort USB/telemetry ports first
    results.sort(key=lambda x: (not x['is_usb'], x['device']))
    return results


class MavlinkBridge:
    """Translates GCS browser requests into MAVLink packets over UDP or Serial Telemetry."""

    def __init__(self, gateway_host, gateway_port, listen_host, listen_port,
                 home_latitude, home_longitude, home_altitude,
                 diagnostics_host='127.0.0.1', diagnostics_port=14552):
        self.home = (home_latitude, home_longitude, home_altitude)
        self.lock = threading.RLock()
        self.state_lock = threading.RLock()
        self.bt_run_lock = threading.Lock()
        self.bt_active_run = None
        self.bt_cancel_pending = False

        # Connection config
        self.conn_type = 'udp'  # 'udp' | 'serial'
        self.udp_gateway = (gateway_host, gateway_port)
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.serial_port = '/dev/ttyUSB0'
        self.serial_baud = 57600

        self.socket = None
        self.serial_conn = None

        # Diagnostics socket for local ROS logs relay
        self.diagnostics_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.diagnostics_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.diagnostics_socket.bind((diagnostics_host, diagnostics_port))

        # Data Transfer / Telemetry Traffic Tracking
        self.tx_bytes = 0
        self.rx_bytes = 0
        self.tx_packets = 0
        self.rx_packets = 0
        self.bt_xml_upload_bytes = 0
        self.last_traffic_calc = time.monotonic()
        self.last_tx_bytes = 0
        self.last_rx_bytes = 0
        self.tx_rate = 0.0
        self.rx_rate = 0.0

        self.encoder = mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
        self.fleet_radio = RadioClient(self._send_fleet_frame)
        # Per-aircraft link accounting, keyed by MAVLink system ID (every aircraft shares one radio).
        self.systems = {}
        # Emergency RTL/LAND commands per MAVLink system ID (see emergency()).
        self.emergency_cmds = {}
        self.mode_cmds = {}     # system id -> last flight-mode request and PX4's answer
        # GCS safety switch: SAFE (default at every GCS start) refuses mission Start from every
        # browser window. RTL / LAND / DISARM are never blocked by it.
        self.safety_safe = True
        # FC guardian as reported by each gateway over STATUSTEXT (works with AERIXIS down):
        # 'LINK|GUARDIAN|UP|<state> action=<A>[ emergency=<A> <state>]' / 'LINK|GUARDIAN|DOWN|...'.
        self.guardian_links = {}

        self.state = {
            'connected': False,
            'armed': False,
            'mode': 'Waiting for AERIXIS gateway',
            'flight_mode': 'OFFBOARD',
            'is_offboard': True,
            'as2_active': True,
            'latitude': home_latitude,
            'longitude': home_longitude,
            'altitude': 0.0,
            'absolute_altitude': home_altitude,
            'mission_sequence': 0,
            'mission_total': 0,
            'mission_active': False,
            'mission_message': 'No mission uploaded',
            'trajectory': [],
            'mission_steps': [],
            'active_step_index': -1,
            'last_command': '',
            'last_seen': 0.0,
            'gateway_heartbeat': 0.0,
            'connection': {
                'type': 'udp',
                'target': f'{gateway_host}:{gateway_port}',
                'baud': None,
                'listen_port': listen_port
            },
            'as2_link': {'connected': None, 'detail': 'Waiting for gateway diagnostics', 'updated': 0.0},
            'px4_link': {'connected': None, 'detail': 'Waiting for PX4 DDS diagnostics', 'updated': 0.0},
            'logs': [{
                'time': datetime.now().astimezone().strftime('%H:%M:%S'),
                'severity': 'INFO',
                'message': 'GCS started. Default mode: OFFBOARD (AERIXIS Autonomy active)',
            }],
            'home_latitude': home_latitude,
            'home_longitude': home_longitude,
            'home_altitude': home_altitude,
        }

        self.upload = None
        self.stop_pending = False
        self.text_chunks = {}
        self.gateway_was_online = False
        self.running = True
        self.heartbeat_timestamp = 0.0
        self.heartbeat_times = deque(maxlen=10)

        # Initialize UDP socket by default
        self._init_udp(gateway_host, gateway_port, listen_host, listen_port)

        self.worker = threading.Thread(target=self._loop, name='mavlink-transport', daemon=True)
        self.worker.start()

    def _init_udp(self, host, port, l_host, l_port):
        with self.lock:
            if self.socket:
                try:
                    self.socket.close()
                except Exception:
                    pass
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((l_host, l_port))
            self.udp_gateway = (host, port)
            self.conn_type = 'udp'
            with self.state_lock:
                self.state['connection'] = {
                    'type': 'udp',
                    'target': f'{host}:{port}',
                    'baud': None,
                    'listen_port': l_port
                }
            self._append_log('INFO', f'MAVLink transport: UDP listening on {l_host}:{l_port}, targeting {host}:{port}')

    def _init_serial(self, port, baud):
        if not HAS_SERIAL:
            raise RuntimeError('pyserial is not installed on this system')
        with self.lock:
            if self.serial_conn:
                try:
                    self.serial_conn.close()
                except Exception:
                    pass
            self.serial_conn = serial.Serial(port, baudrate=baud, timeout=0.05)
            self.conn_type = 'serial'
            self.serial_port = port
            self.serial_baud = baud
            with self.state_lock:
                self.state['connection'] = {
                    'type': 'serial',
                    'target': port,
                    'baud': baud,
                    'listen_port': None
                }
            self._append_log('INFO', f'MAVLink transport: Serial Telemetry connected on {port} @ {baud} baud')

    def connect(self, config):
        """Switch transport dynamically between UDP and Serial Telemetry."""
        target_type = config.get('type', 'udp').lower()
        if target_type == 'serial':
            port = config.get('port', '/dev/ttyUSB0')
            baud = int(config.get('baud', 57600))
            self._init_serial(port, baud)
            return {'ok': True, 'type': 'serial', 'port': port, 'baud': baud}
        else:
            host = config.get('host', '127.0.0.1')
            port = int(config.get('port', 14550))
            l_port = int(config.get('listen_port', 14551))
            # The diagnostics relay owns 127.0.0.1:<diagnostics port>; a link bound to the same
            # port silently loses every packet to it ("waiting for heartbeat" forever).
            diag_port = self.diagnostics_socket.getsockname()[1]
            if not (1024 <= port <= 65535 and 1024 <= l_port <= 65535):
                raise ValueError('UDP ports must be between 1024 and 65535')
            if l_port == diag_port:
                raise ValueError(f'Listen port {l_port} is reserved for GCS diagnostics; use 14551')
            if l_port == port and host in ('127.0.0.1', 'localhost', '0.0.0.0'):
                raise ValueError('Listen port and aircraft port must differ on the same computer')
            self._init_udp(host, port, '0.0.0.0', l_port)
            return {'ok': True, 'type': 'udp', 'host': host, 'port': port, 'listen_port': l_port}

    def disconnect(self):
        """Disconnect current link."""
        with self.lock:
            if self.serial_conn:
                try:
                    self.serial_conn.close()
                except Exception:
                    pass
                self.serial_conn = None
            if self.socket:
                try:
                    self.socket.close()
                except Exception:
                    pass
                self.socket = None
            with self.state_lock:
                self.state['connected'] = False
                self.state['connection']['target'] = 'Disconnected'
            self._append_log('WARN', 'MAVLink transport disconnected by user')
            return {'ok': True}

    def state_snapshot(self):
        now = time.monotonic()
        with self.state_lock:
            # 7.0s window prevents transient packet drops from flagging false offline
            gateway_online = (now - self.state['gateway_heartbeat'] < 7.0) and (self.state['gateway_heartbeat'] > 0.0)
            hb_age = round(now - self.state['gateway_heartbeat'], 1) if self.state['gateway_heartbeat'] > 0.0 else 999.0

            # Traffic rate calculation
            dt = now - self.last_traffic_calc
            if dt >= 1.0:
                self.tx_rate = max(0.0, (self.tx_bytes - self.last_tx_bytes) / 1024.0 / dt)
                self.rx_rate = max(0.0, (self.rx_bytes - self.last_rx_bytes) / 1024.0 / dt)
                self.last_tx_bytes = self.tx_bytes
                self.last_rx_bytes = self.rx_bytes
                self.last_traffic_calc = now

            traffic_stats = {
                'tx_bytes': self.tx_bytes,
                'rx_bytes': self.rx_bytes,
                'tx_kb': round(self.tx_bytes / 1024.0, 1),
                'rx_kb': round(self.rx_bytes / 1024.0, 1),
                'tx_packets': self.tx_packets,
                'rx_packets': self.rx_packets,
                'tx_rate': round(self.tx_rate, 2),
                'rx_rate': round(self.rx_rate, 2),
                'bt_xml_bytes': self.bt_xml_upload_bytes,
            }

            # Calculate real-time heartbeat Hz
            if len(self.heartbeat_times) >= 2 and (now - self.heartbeat_times[-1] < 4.0):
                span = self.heartbeat_times[-1] - self.heartbeat_times[0]
                hb_hz = round((len(self.heartbeat_times) - 1) / span, 1) if span > 0.1 else 1.0
            else:
                hb_hz = 0.0 if not gateway_online else 1.0

            if gateway_online != self.gateway_was_online:
                message = ('MAVLink link active (Telemetry received from AERIXIS)' if gateway_online
                           else 'AERIXIS Gateway offline — listening for heartbeat...')
                self._append_log('INFO' if gateway_online else 'WARN', message)
                self.gateway_was_online = gateway_online
            state = dict(self.state)
            state['logs'] = list(state['logs'])
            state['as2_link'] = dict(state['as2_link'])
            state['px4_link'] = dict(state['px4_link'])
            state['connection'] = dict(state['connection'])

        systems = {}
        for sid, e in list(self.systems.items()):
            dt = now - e['last_calc']
            if dt >= 1.0:
                e['rx_rate'] = (e['rx_bytes'] - e['last_rxb']) / 1024.0 / dt
                e['tx_rate'] = (e['tx_bytes'] - e['last_txb']) / 1024.0 / dt
                e['last_rxb'], e['last_txb'], e['last_calc'] = e['rx_bytes'], e['tx_bytes'], now
            hb = list(e['hb'])
            hz = round((len(hb) - 1) / (hb[-1] - hb[0]), 1) if len(hb) >= 2 and hb[-1] - hb[0] > 0.1 and now - hb[-1] < 4 else 0.0
            systems[str(sid)] = dict(rx_kb=round(e['rx_bytes'] / 1024.0, 1), tx_kb=round(e['tx_bytes'] / 1024.0, 1),
                                     rx_packets=e['rx_packets'], tx_packets=e['tx_packets'],
                                     rx_rate=round(e['rx_rate'], 2), tx_rate=round(e['tx_rate'], 2), heartbeat_hz=hz,
                                     heartbeat_age=round(now - hb[-1], 1) if hb else None,
                                     last_rx_age=round(now - e['last_rx'], 1) if e['last_rx'] else None)
        state['systems'] = systems
        with self.state_lock:
            state['emergency'] = {str(k): dict(v) for k, v in self.emergency_cmds.items()}
            state['mode_cmds'] = {str(k): dict(v) for k, v in self.mode_cmds.items()}
            state['safety_safe'] = self.safety_safe
            state['guardian'] = {str(k): dict(v) for k, v in self.guardian_links.items()}
        state['connected'] = gateway_online
        state['heartbeat_age'] = hb_age
        state['traffic'] = traffic_stats
        state['heartbeat_hz'] = hb_hz
        conn_desc = (f"Serial {self.serial_port} ({self.serial_baud} baud)" if self.conn_type == 'serial'
                     else f"UDP {self.udp_gateway[0]}:{self.udp_gateway[1]}")
        state['gateway_detail'] = (f'MAVLink active via {conn_desc}' if state['connected']
                                   else f'Waiting for AERIXIS heartbeat via {conn_desc}')

        for link_key in ('as2_link', 'px4_link'):
            link = state[link_key]
            fresh = (now - link['updated'] < 5.5) and (link['updated'] > 0.0)
            if not state['connected']:
                link['connected'] = False
                link['detail'] = 'AERIXIS Gateway is offline'
            elif not fresh:
                link['connected'] = False
                link['detail'] = 'No recent link diagnostic from gateway'
            elif link['connected'] is None:
                link['connected'] = False

        # Build Decoupled 3-Tier Pipeline Architecture
        as2_conn = bool(state['connected'] and state['as2_link']['connected'])
        px4_conn = bool(as2_conn and state['px4_link']['connected'])

        state['pipeline'] = {
            'gcs': {
                'status': 'online',
                'label': 'GCS Studio (Laptop)',
                'listen': f"{self.listen_host}:{self.listen_port}" if self.conn_type == 'udp' else self.serial_port,
            },
            'link1_mavlink': {
                'status': 'connected' if state['connected'] else 'searching',
                'proto': 'MAVLink Telemetry',
                'target': f"{self.udp_gateway[0]}:{self.udp_gateway[1]}" if self.conn_type == 'udp' else self.serial_port,
                'heartbeat_hz': hb_hz,
                'heartbeat_age': hb_age,
                'detail': f'Heartbeat {hb_hz} Hz · Telemetry Active' if state['connected'] else f'Scanning for the AERIXIS core on port {self.udp_gateway[1]}...',
            },
            'as2_core': {
                'status': 'connected' if as2_conn else ('gateway_only' if state['connected'] else 'offline'),
                'label': 'AERIXIS Core',
                'sublabel': 'Platform / Estimator / Behaviors / Trajectory',
                'detail': state['as2_link'].get('detail', 'Waiting for AERIXIS nodes'),
            },
            'link2_dds': {
                'status': 'connected' if px4_conn else ('waiting' if as2_conn else 'offline'),
                'proto': 'Micro-XRCE-DDS Bridge',
                'channel': 'UDP 8888 (AERIXIS Middleware ➔ Autopilot)' if self.conn_type == 'udp' else 'Serial @ 921600 baud',
                'detail': state['px4_link'].get('detail', 'No odometry from PX4 DDS'),
            },
            'px4_autopilot': {
                'status': 'connected' if px4_conn else ('waiting' if as2_conn else 'offline'),
                'label': 'PX4 Flight Controller',
                'sublabel': 'Gazebo SITL / Pixhawk Hardware',
                'detail': 'Synchronized via Micro-XRCE-DDS only' if px4_conn else 'Waiting for PX4 client connection',
            }
        }

        state.pop('last_seen', None)
        state.pop('gateway_heartbeat', None)
        state['logs'] = state['logs'][-100:]
        return state

    def _append_log(self, severity, message):
        msg_str = str(message)
        msg_lower = msg_str.lower()
        # Strictly ignore non-drone external logs (e.g. robot arm / CNC machine)
        if any(ign in msg_lower for ign in ('cnc_', 'nextup_', 'arm_controller', 'adjustor', 'move_group', 'robot_state', 'joint_state')):
            return
        if self.state['logs'] and self.state['logs'][-1]['message'] == msg_str:
            return
        self.state['logs'].append({
            'time': datetime.now().astimezone().strftime('%H:%M:%S'),
            'severity': severity, 'message': msg_str,
        })
        self.state['logs'] = self.state['logs'][-100:]

    def _handle_external_log(self, packet):
        try:
            severity, source, message = packet.decode('utf-8', errors='replace').split('|', 2)
        except ValueError:
            return
        if severity not in ('ERROR', 'WARN', 'INFO'):
            return
        with self.state_lock:
            self._append_log(severity, f'{source}: {message}')

    @staticmethod
    def _message_text(message):
        value = message.text
        if isinstance(value, bytes):
            return value.decode('utf-8', errors='replace').rstrip(chr(0))
        return str(value).rstrip(chr(0))

    def _handle_status_text(self, message):
        text = self._message_text(message)
        text_id = int(getattr(message, 'id', 0))
        chunk_sequence = int(getattr(message, 'chunk_seq', 0))
        if text_id:
            chunks = self.text_chunks.setdefault(text_id, {})
            chunks[chunk_sequence] = text
            if len(text.encode('utf-8', errors='replace')) >= 50:
                return
            text = ''.join(chunks[index] for index in sorted(chunks))
            self.text_chunks.pop(text_id, None)

        with self.state_lock:
            source = int(message.get_srcSystem() or 0)
            if text.startswith('LINK|GUARDIAN|'):
                parts = text.split('|', 3)
                if len(parts) == 4:
                    words = parts[3].split()
                    fields = dict(w.split('=', 1) for w in words if '=' in w)
                    up = parts[2] == 'UP'
                    entry = dict(up=up, state=words[0] if up and words else None, action=fields.get('action'),
                                 emergency=None, detail=parts[3][:120], t=time.time())
                    if 'emergency' in fields:
                        rest = parts[3].split('emergency=', 1)[1].split()
                        entry['emergency'] = dict(action=rest[0], state=rest[1] if len(rest) > 1 else None)
                    previous = self.guardian_links.get(source)
                    self.guardian_links[source] = entry
                    if previous is None or previous['up'] != up or previous['state'] != entry['state']:
                        self._append_log('INFO' if up and entry['state'] not in ('NO_FC', 'PX4_FAILSAFE', 'COMMANDING', 'EMERGENCY_ACTIVE') else 'ERROR',
                                         f'System {source} FC guardian: {parts[2]} {parts[3]}')
                return
            if text.startswith('EMERGENCY|'):
                parts = text.split('|', 2)
                if len(parts) == 3:
                    entry = self.guardian_links.setdefault(source, dict(up=None, state=None, action=None, emergency=None, detail='', t=time.time()))
                    entry['reply'] = dict(action=parts[1], text=parts[2][:120], t=time.time())
                    self._append_log('INFO', f'System {source} emergency {parts[1]}: {parts[2]}')
                return
            if text.startswith('LINK|'):
                parts = text.split('|', 3)
                if len(parts) == 4:
                    _, link_name, status, detail = parts
                    key = {'AS2': 'as2_link', 'PX4_DDS': 'px4_link'}.get(link_name)
                    if key:
                        link = self.state[key]
                        connected = status == 'UP'
                        previous = link['connected']
                        link.update({'connected': connected, 'detail': detail, 'updated': time.monotonic()})
                        if previous is not connected:
                            severity = 'INFO' if connected else 'ERROR'
                            label = 'AERIXIS core' if key == 'as2_link' else 'PX4 uXRCE-DDS'
                            self._append_log(severity, f'{label}: {"connected" if connected else "disconnected"} — {detail}')
                    return
            if text.startswith('ROSLOG|'):
                parts = text.split('|', 3)
                if len(parts) == 4:
                    _, severity, name, detail = parts
                    self._append_log(severity, f'{name}: {detail}')
                    return
            severity_value = int(getattr(message, 'severity', 6))
            severity = 'ERROR' if severity_value <= 3 else ('WARN' if severity_value == 4 else 'INFO')
            if text:
                self._append_log(severity, text)

    def _send(self, message, system=None):
        with self.lock:
            packed = message.pack(self.encoder)
            self.tx_bytes += len(packed)
            self.tx_packets += 1
            if system:
                entry = self._system(int(system))
                entry['tx_bytes'] += len(packed); entry['tx_packets'] += 1
            if self.conn_type == 'serial' and self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(packed)
                except Exception as e:
                    self._append_log('ERROR', f'Serial write error: {e}')
            elif self.conn_type == 'udp' and self.socket:
                try:
                    self.socket.sendto(packed, self.udp_gateway)
                except Exception as e:
                    self._append_log('ERROR', f'UDP send error: {e}')

    def _send_fleet_frame(self, system, frame):
        self._send(self.encoder.v2_extension_encode(0,system,190,FLEET_MESSAGE_TYPE,list(frame)+[0]*(249-len(frame))),system)

    def _send_heartbeat(self):
        self._send(self.encoder.heartbeat_encode(
            mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
            mavlink.MAV_STATE_ACTIVE))

    # MAV_CMD_NAV_RETURN_TO_LAUNCH / MAV_CMD_NAV_LAND (FC guardian -> PX4) and
    # MAV_CMD_COMPONENT_ARM_DISARM with param1 0 = disarm (AERIXIS core -> PX4; PX4 refuses in flight).
    EMERGENCY_COMMANDS = {'RTL': 20, 'LAND': 21, 'DISARM': 400}
    EMERGENCY_TRIES = 5

    def emergency(self, systems, action, reason=''):
        """Operator emergency: COMMAND_LONG RTL or LAND to each aircraft over the telemetry link.

        One command per MAVLink system ID (never broadcast), target component 1 (autopilot): the
        onboard gateway hands these to the FC guardian, which commands PX4 directly and works with
        AERIXIS down (NAV_LAND to component 190 would be an ordinary AERIXIS land). Re-sent every second
        with an increasing confirmation count until that system answers COMMAND_ACK, at most
        EMERGENCY_TRIES times."""
        command = self.EMERGENCY_COMMANDS.get(action)
        if command is None:
            raise ValueError('Emergency action must be RTL, LAND or DISARM')
        ids = sorted({int(s) for s in systems})
        if not ids or any(not 1 <= s <= 254 for s in ids):
            raise ValueError('Choose at least one aircraft with a MAVLink system ID')
        if not (self.socket or (self.serial_conn and self.serial_conn.is_open)):
            raise ValueError('Telemetry link is closed; open it in Connect first')
        now = time.time()
        with self.state_lock:
            for sid in ids:
                self.emergency_cmds[sid] = dict(action=action, command=command, state='sending', attempts=0,
                                                result=None, reason=str(reason)[:120], t=now, acked=None)
            self._append_log('ERROR', f"EMERGENCY {action} sent to system(s) {', '.join(map(str, ids))}"
                                      + (f' — {reason}' if reason else ''))
        threading.Thread(target=self._emergency_sender, args=(ids, command), daemon=True).start()
        return {'ok': True, 'systems': ids, 'action': action}

    FLIGHT_MODES = {'OFFBOARD': 6, 'POSCTL': 3, 'ALTCTL': 2}
    MODE_TRIES = 3

    def set_modes(self, systems, mode):
        """Flight mode for each chosen aircraft (2026-10-25): DO_SET_MODE to its MAVLink system ID.

        OFFBOARD: the onboard gateway asks the AERIXIS core (it streams PX4's setpoints).
        POSCTL / ALTCTL: the gateway commands PX4 directly and forwards PX4's own COMMAND_ACK.
        RTL / LAND use emergency() (FC guardian). Re-sent each second until the aircraft answers."""
        mode = str(mode).upper().strip()
        mode = {'POSITION': 'POSCTL', 'ALTITUDE': 'ALTCTL'}.get(mode, mode)
        main = self.FLIGHT_MODES.get(mode)
        if main is None:
            raise ValueError('Mode must be OFFBOARD, POSCTL or ALTCTL (RTL / LAND use the emergency path)')
        ids = sorted({int(s) for s in systems})
        if not ids or any(not 1 <= s <= 254 for s in ids):
            raise ValueError('Choose at least one aircraft with a MAVLink system ID')
        if not (self.socket or (self.serial_conn and self.serial_conn.is_open)):
            raise ValueError('Telemetry link is closed; open it in Connect first')
        now = time.time()
        with self.state_lock:
            for sid in ids:
                self.mode_cmds[sid] = dict(mode=mode, state='sending', attempts=0, result=None, t=now, acked=None)
            self._append_log('INFO', f"Flight mode {mode} sent to system(s) {', '.join(map(str, ids))}")
        threading.Thread(target=self._mode_sender, args=(ids, mode, main), daemon=True).start()
        return {'ok': True, 'systems': ids, 'mode': mode}

    def _mode_sender(self, ids, mode, main):
        for attempt in range(self.MODE_TRIES):
            with self.state_lock:
                pending = [sid for sid in ids if (self.mode_cmds.get(sid) or {}).get('mode') == mode
                           and self.mode_cmds[sid]['state'] == 'sending']
                for sid in pending:
                    self.mode_cmds[sid]['attempts'] = attempt + 1
            if not pending:
                return
            for sid in pending:
                try:
                    self._send(self.encoder.command_long_encode(
                        sid, 190, mavlink.MAV_CMD_DO_SET_MODE, attempt, 1.0, float(main), 0.0, 0, 0, 0, 0), sid)
                except Exception as exc:
                    self._append_log('ERROR', f'Mode send to system {sid} failed: {exc}')
            time.sleep(1.0)
        with self.state_lock:
            for sid in ids:
                entry = self.mode_cmds.get(sid)
                if entry and entry['mode'] == mode and entry['state'] == 'sending':
                    entry['state'] = 'no-answer'
                    self._append_log('ERROR', f'Flight mode {mode} to system {sid}: no answer from the aircraft')

    def _emergency_sender(self, ids, command):
        for attempt in range(self.EMERGENCY_TRIES):
            pending = []
            with self.state_lock:
                for sid in ids:
                    entry = self.emergency_cmds.get(sid)
                    # A newer command for this aircraft replaces this one; an ACK ends the retries.
                    if entry and entry['command'] == command and entry['state'] == 'sending':
                        entry['attempts'] = attempt + 1
                        pending.append(sid)
            if not pending:
                return
            for sid in pending:
                try:
                    self._send(self.encoder.command_long_encode(sid, 1, command, attempt, 0, 0, 0, 0, 0, 0, 0), sid)
                except Exception as exc:
                    self._append_log('ERROR', f'Emergency send to system {sid} failed: {exc}')
            time.sleep(1.0)
        with self.state_lock:
            for sid in ids:
                entry = self.emergency_cmds.get(sid)
                if entry and entry['command'] == command and entry['state'] == 'sending':
                    entry['state'] = 'no-ack'
                    self._append_log('ERROR', f'Emergency command {command} to system {sid}: no acknowledgement')

    def set_flight_mode(self, mode_name):
        """Command drone to switch flight mode (OFFBOARD, POSCTL, ALTCTL, MANUAL, RTL)."""
        mode_upper = mode_name.upper().strip()
        custom_main_mode = 6  # default OFFBOARD
        custom_sub_mode = 0

        if mode_upper in ('OFFBOARD', 'OFF_BOARD'):
            custom_main_mode = 6
        elif mode_upper in ('POSCTL', 'POSITION', 'POS'):
            custom_main_mode = 3
        elif mode_upper in ('ALTCTL', 'ALTITUDE', 'ALT'):
            custom_main_mode = 2
        elif mode_upper in ('MANUAL', 'STABILIZED'):
            custom_main_mode = 1
        elif mode_upper in ('RTL', 'RETURN'):
            custom_main_mode = 4
            custom_sub_mode = 5  # AUTO_RTL
        elif mode_upper == 'LAND':
            custom_main_mode = 4
            custom_sub_mode = 6  # AUTO_LAND
        else:
            raise ValueError(f'Unsupported flight mode: {mode_name}')

        # Send MAV_CMD_DO_SET_MODE
        self._send(self.encoder.command_long_encode(
            1, 190, mavlink.MAV_CMD_DO_SET_MODE, 0,
            1.0,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            float(custom_main_mode),
            float(custom_sub_mode),
            0.0, 0.0, 0.0, 0.0
        ))

        is_offboard = (custom_main_mode == 6)
        with self.state_lock:
            self.state['flight_mode'] = mode_upper
            self.state['is_offboard'] = is_offboard
            self.state['as2_active'] = is_offboard
            status_msg = (f'Switched to OFFBOARD mode. AERIXIS Autonomous behaviors active.' if is_offboard
                          else f'Switched to {mode_upper} mode. AERIXIS Autonomous behaviors INACTIVE.')
            self._append_log('INFO', f'Flight Mode: {status_msg}')
            self.state['last_command'] = f'Mode -> {mode_upper}'

        return {'ok': True, 'mode': mode_upper, 'is_offboard': is_offboard}

    def command(self, action, altitude=None):
        """Basic flight commands: takeoff, arm, disarm, land."""
        with self.state_lock:
            is_off = self.state.get('is_offboard', True)

        if action in ('takeoff', 'land') and not is_off:
            raise ValueError(f'Cannot execute AERIXIS {action.upper()}: Vehicle is not in OFFBOARD mode! Switch mode to OFFBOARD first.')

        if action == 'takeoff':
            param7 = float(altitude or 2.0)
            if not 0.5 <= param7 <= 50.0:
                raise ValueError('Takeoff altitude must be between 0.5 m and 50 m')
            command, param1 = mavlink.MAV_CMD_NAV_TAKEOFF, 0.0
        elif action == 'arm':
            command, param1, param7 = mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0
        elif action == 'disarm':
            command, param1, param7 = mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0
        elif action == 'land':
            command, param1, param7 = mavlink.MAV_CMD_NAV_LAND, 0.0, 0.0
        else:
            raise ValueError(f'Unsupported flight command: {action}')

        self._send(self.encoder.command_long_encode(
            1, 190, command, 0, param1, 0, 0, 0, 0, 0, param7))
        with self.state_lock:
            self.state['last_command'] = f'{action.title()} requested via AERIXIS'
            self._append_log('INFO', f'Flight command: {action.upper()} sent via MAVLink')

    def upload_mission(self, request):
        if self.bt_run_lock.locked():
            raise ValueError('Cannot upload a MAVLink mission while a local behavior tree is active')
        polygon = [(float(p[0]), float(p[1])) for p in request.get('polygon', [])]
        waypoints = [(float(p[0]), float(p[1])) for p in request.get('waypoints', [])]
        altitude = float(request.get('altitude', 2.0))

        with self.state_lock:
            if not self.state.get('is_offboard', True):
                raise ValueError('Cannot execute AERIXIS Trajectory: Vehicle is not in OFFBOARD mode! Switch to OFFBOARD first.')

        if len(polygon) < 3:
            if len(waypoints) > 0:
                all_lats = [p[0] for p in waypoints] + [self.home[0]]
                all_lons = [p[1] for p in waypoints] + [self.home[1]]
                margin = 0.001
                polygon = [
                    (min(all_lats) - margin, min(all_lons) - margin),
                    (min(all_lats) - margin, max(all_lons) + margin),
                    (max(all_lats) + margin, max(all_lons) + margin),
                    (max(all_lats) + margin, min(all_lons) - margin),
                ]
            else:
                raise ValueError('Draw a play-area polygon or add waypoints first')
        if not 1 <= len(waypoints) <= 97:
            raise ValueError('Add between 1 and 97 route waypoints')
        if not 0.5 <= altitude <= 50.0:
            raise ValueError('Mission altitude must be between 0.5 m and 50 m')

        home_lat, home_lon, _ = self.home
        mission = [(mavlink.MAV_CMD_NAV_TAKEOFF, home_lat, home_lon, altitude)]
        mission.extend((mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, altitude) for lat, lon in waypoints)

        last_lat, last_lon = waypoints[-1]
        if offset_meters(last_lat, last_lon, home_lat, home_lon)[0] ** 2 +                 offset_meters(last_lat, last_lon, home_lat, home_lon)[1] ** 2 > 0.25:
            mission.append((mavlink.MAV_CMD_NAV_WAYPOINT, home_lat, home_lon, altitude))
        mission.append((mavlink.MAV_CMD_NAV_LAND, home_lat, home_lon, 0.0))

        with self.state_lock:
            if self.upload is not None:
                raise ValueError('A mission upload is already in progress')
            if self.state['mission_active']:
                raise ValueError('A mission is already active; land or stop it first')
            self.upload = mission
            self.state['mission_message'] = f'Uploading {len(mission)} MAVLink items to AERIXIS...'
            self._append_log('INFO', f'Mission: Submitting {len(mission)} route items to AERIXIS...')
            self.state['mission_sequence'] = 0
            self.state['mission_total'] = len(mission)
        self._send(self.encoder.mission_count_encode(1, 190, len(mission)))
        return len(mission)

    def _send_mission_item(self, sequence):
        with self.state_lock:
            if self.upload is None or not 0 <= sequence < len(self.upload):
                return
            command, latitude, longitude, altitude = self.upload[sequence]
            total = len(self.upload)
        self._send(self.encoder.mission_item_int_encode(
            1, 190, sequence, mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command, int(sequence == 0), 1, 0, 0, 0, 0,
            int(latitude * 1e7), int(longitude * 1e7), altitude))
        with self.state_lock:
            self.state['mission_message'] = f'Sending item {sequence + 1}/{total}'

    def run_behavior_tree(self, request):
        # Missions run onboard (Upload & start). Running the AERIXIS core tree on the GCS laptop
        # needs AS2 installed here, which a field GCS does not have: development only.
        if os.environ.get('AERIXIS_LOCAL_BT') != '1':
            raise ValueError('Running the tree on the GCS laptop is disabled; use Upload & start '
                             '(set AERIXIS_LOCAL_BT=1 only for development).')
        if not self.bt_run_lock.acquire(blocking=False):
            raise ValueError('A local behavior tree is already running; await its terminal result')
        try:
            with self.state_lock:
                if self.state.get('mission_active'):
                    raise ValueError('A mission is already active')
                self.bt_cancel_pending = False
            return self._prepare_behavior_tree(request)
        except Exception:
            self.bt_run_lock.release()
            raise

    def _prepare_behavior_tree(self, request):
        """Executes a dynamic Behavior Tree generated by the GCS visual builder."""
        now_check = time.monotonic()
        with self.state_lock:
            gw_online = (now_check - self.state.get('gateway_heartbeat', 0.0) < 7.0) and (self.state.get('gateway_heartbeat', 0.0) > 0.0)
            if not gw_online:
                raise ValueError('Cannot execute mission: AERIXIS is DISCONNECTED! Telemetry heartbeat is offline. Please launch ./2_launch_as2.sh first.')

        drone_id = str(request.get('drone_id', 'drone0'))
        custom_xml = request.get('tree_xml')
        if custom_xml is not None and (not isinstance(custom_xml, str) or not custom_xml.strip()):
            raise ValueError('tree_xml must be a nonempty XML string')
        waypoints_req = request.get('waypoints', [])
        home_lat, home_lon, home_alt = self.home

        # Calculate GPS trajectory points for GCS map rendering
        gps_trajectory = []
        steps = []
        
        # 1. Mandatory Offboard
        steps.append({"id": "step_offboard", "type": "Offboard", "name": "Set Offboard Mode", "icon": "⚡", "status": "pending"})
        # 2. Mandatory Arm
        steps.append({"id": "step_arm", "type": "Arm", "name": "Arm Drone Motors", "icon": "🛡️", "status": "pending"})

        if custom_xml is not None:
            bt_xml = custom_xml.strip()
            self.bt_xml_upload_bytes = len(bt_xml.encode('utf-8'))
            altitude = 2.5
            
            # Extract TakeOff height
            m_to = re.search(r'<Action ID="TakeOff"[^>]*height="([^"]+)"', bt_xml)
            if m_to:
                try: altitude = float(m_to.group(1))
                except Exception: pass
            steps.append({"id": "step_takeoff", "type": "TakeOff", "name": f"Autonomous Takeoff ({altitude}m)", "icon": "🛫", "status": "pending"})

            # Extract GoTo waypoints from XML
            goto_matches = re.findall(r'<Action ID="GoTo"[^>]*pose="([^"]+)"', bt_xml)
            for i, p_str in enumerate(goto_matches):
                coords = [float(v.strip()) for v in p_str.split(';') if v.strip()]
                if len(coords) >= 2:
                    east, north = coords[0], coords[1]
                    # convert local ENU meters to GPS
                    R = 6378137.0
                    lat = home_lat + (north / R) * (180.0 / math.pi)
                    lon = home_lon + (east / (R * math.cos(math.radians(home_lat)))) * (180.0 / math.pi)
                    gps_trajectory.append({"lat": round(lat, 7), "lon": round(lon, 7), "alt": coords[2] if len(coords)>2 else altitude, "name": f"WP {i+1}"})
                    steps.append({"id": f"step_wp_{i+1}", "type": "GoTo", "name": f"Scan WP {i+1} [{coords[0]}m, {coords[1]}m]", "icon": "📍", "status": "pending"})
            
            num_wps = len(goto_matches)
        else:
            waypoints = [(float(p[0]), float(p[1])) for p in waypoints_req]
            altitude = float(request.get('altitude', 2.5))
            steps.append({"id": "step_takeoff", "type": "TakeOff", "name": f"Autonomous Takeoff ({altitude}m)", "icon": "🛫", "status": "pending"})

            enu_wps = []
            for i, p in enumerate(waypoints):
                e, n = offset_meters(p[0], p[1], *self.home[:2])
                enu_wps.append((round(e, 3), round(n, 3), altitude))
                gps_trajectory.append({"lat": p[0], "lon": p[1], "alt": altitude, "name": f"WP {i+1}"})
                steps.append({"id": f"step_wp_{i+1}", "type": "GoTo", "name": f"Waypoint {i+1} [{round(e,1)}m, {round(n,1)}m]", "icon": "📍", "status": "pending"})

            if not enu_wps:
                enu_wps = [(5.0, 0.0, altitude), (5.0, 5.0, altitude), (0.0, 5.0, altitude), (0.0, 0.0, altitude)]
                for i, (e, n, a) in enumerate(enu_wps):
                    R = 6378137.0
                    lat = home_lat + (n / R) * (180.0 / math.pi)
                    lon = home_lon + (e / (R * math.cos(math.radians(home_lat)))) * (180.0 / math.pi)
                    gps_trajectory.append({"lat": round(lat, 7), "lon": round(lon, 7), "alt": a, "name": f"WP {i+1}"})
                    steps.append({"id": f"step_wp_{i+1}", "type": "GoTo", "name": f"Waypoint {i+1} [{e}m, {n}m]", "icon": "📍", "status": "pending"})

            goto_nodes = "\n".join([
                f'            <Action ID="GoTo" max_speed="1.5" pose="{x};{y};{z}" yaw_angle="0.0" yaw_mode="1" frame_id="earth"/>'
                for x, y, z in enu_wps
            ])

            bt_xml = f"""<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4" main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="FlightMissionSequence">
            <Action ID="Offboard" service_name="set_offboard_mode"/>
            <Action ID="Arm" service_name="set_arming_state"/>
            <Action ID="TakeOff" height="{altitude}" speed="0.5"/>
            <!-- AREA COVERAGE SCAN TRAJECTORY CHILD OF FLIGHT SEQUENCE -->
            <Sequence name="AreaCoverageScanTrajectory">
{goto_nodes}
            </Sequence>
            <Action ID="Land" speed="0.5"/>
            <Action ID="Disarm" service_name="set_arming_state"/>
        </Sequence>
    </BehaviorTree>
    <TreeNodesModel>
        <Action ID="Arm"><input_port name="service_name" default="set_arming_state"/></Action>
        <Action ID="Disarm"><input_port name="service_name" default="set_arming_state"/></Action>
        <Action ID="Offboard"><input_port name="service_name" default="set_offboard_mode"/></Action>
        <Action ID="TakeOff"><input_port name="height" default="2.0"/><input_port name="speed" default="0.5"/></Action>
        <Action ID="GoTo"><input_port name="pose"/><input_port name="max_speed" default="1.5"/><input_port name="yaw_angle" default="0.0"/><input_port name="yaw_mode" default="1"/><input_port name="frame_id" default="earth"/></Action>
        <Action ID="Land"><input_port name="speed" default="0.5"/></Action>
    </TreeNodesModel>
</root>"""
            num_wps = len(enu_wps)

        # 5. Land & Disarm
        steps.append({"id": "step_land", "type": "Land", "name": "Autonomous Land", "icon": "🛬", "status": "pending"})
        steps.append({"id": "step_disarm", "type": "Disarm", "name": "Disarm Motors", "icon": "🛑", "status": "pending"})

        run = LocalTreeRun(drone_id, bt_xml)
        with self.state_lock:
            self.bt_active_run = run
            if self.bt_cancel_pending:
                run.cancel()
            self.state['mission_id'] = run.mission_id
            self.state['mission_execution_location'] = 'local_gcs'
            self.state['mission_result'] = None
            self.state['mission_bt_nodes'] = []
            self.bt_xml_upload_bytes = len(bt_xml.encode('utf-8'))

        total_nodes = len(steps)
        with self.state_lock:
            self.state['mission_active'] = True
            self.state['mission_message'] = 'AERIXIS Behavior Tree running...'
            self.state['mission_sequence'] = 1
            self.state['mission_total'] = total_nodes
            self.state['mission_steps'] = steps
            self.state['active_step_index'] = 0
            if gps_trajectory:
                self.state['trajectory'] = gps_trajectory

        def update_execution_state():
            nodes = run.state.steps()
            # The full identified graph is exposed separately. Existing survey UI consumes actions.
            action_steps = [step for step in nodes
                            if run.state.nodes[step['node_uid']].get('node_category') == 'Action']
            with self.state_lock:
                self.state['mission_bt_nodes'] = nodes
                self.state['mission_steps'] = action_steps
                self.state['mission_total'] = len(action_steps)
                self.state['mission_sequence'] = sum(s['status'] == 'success' for s in action_steps)
                self.state['active_step_index'] = next(
                    (i for i, s in enumerate(action_steps) if s['status'] == 'running'), -1)
                if run.state.outcome:
                    self.state['mission_result'] = run.state.outcome
                    self.state['mission_message'] = f'{run.state.outcome}: {run.state.detail}'
                elif self.state['active_step_index'] >= 0:
                    self.state['mission_message'] = (
                        'AERIXIS BT: ' + action_steps[self.state['active_step_index']]['name'])

        def _exec_bt():
            try:
                self._append_log('INFO', f'Local BT execution {run.mission_id} for {drone_id}')
                proc = run.start()
                for line in proc.stdout:
                    if run.state.accept(line.strip()):
                        update_execution_state()
                    elif line.strip():
                        self._append_log('INFO', 'BT: ' + line.strip())
                run.state.finish_process(proc.wait())
                update_execution_state()
            except Exception as exc:
                # Never infer success from an exception, missing result, or zero launcher exit.
                run.cancel()
                with self.state_lock:
                    self.state['mission_result'] = 'ERROR'
                    self.state['mission_message'] = f'BT execution error: {exc}; hold not verified'
                self._append_log('ERROR', self.state['mission_message'])
                if run.process is not None:
                    # Retain exclusivity until the actual child exits, even if cancellation stalls.
                    run.process.wait()
            finally:
                with self.state_lock:
                    self.state['mission_active'] = False
                    self.bt_active_run = None
                self.bt_run_lock.release()

        t = threading.Thread(target=_exec_bt, daemon=True)
        t.start()
        return num_wps

    def stop_mission(self):
        with self.state_lock:
            if self.bt_run_lock.locked():
                self.bt_cancel_pending = True
                if self.bt_active_run is not None:
                    self.bt_active_run.cancel()
                self.state['mission_message'] = 'BT cancellation requested; awaiting result, hold not verified'
                return
            self.stop_pending = True
        self._send(self.encoder.mission_clear_all_encode(1, 190))
        with self.state_lock:
            self.state['mission_message'] = 'Stop requested through AERIXIS'

    def _system(self, sid):
        entry = self.systems.get(sid)
        if entry is None:
            entry = self.systems[sid] = dict(rx_bytes=0, rx_packets=0, tx_bytes=0, tx_packets=0, last_rx=0.0,
                                             hb=deque(maxlen=10), last_calc=time.monotonic(), last_rxb=0, last_txb=0,
                                             rx_rate=0.0, tx_rate=0.0)
        return entry

    def _handle(self, message):
        message_type = message.get_type()
        try:
            entry = self._system(int(message.get_srcSystem()))
            entry['rx_bytes'] += len(message.get_msgbuf()); entry['rx_packets'] += 1
            entry['last_rx'] = time.monotonic()
            if message_type == 'HEARTBEAT':
                entry['hb'].append(entry['last_rx'])
        except Exception:
            pass
        if message_type == 'V2_EXTENSION' and message.message_type == FLEET_MESSAGE_TYPE:
            if message.target_system == 255 and message.target_component == 190:
                self.fleet_radio.receive(message.get_srcSystem(),bytes(message.payload))
            return
        with self.state_lock:
            self.state['last_seen'] = time.monotonic()
            if message_type == 'HEARTBEAT':
                now_hb = time.monotonic()
                self.state['gateway_heartbeat'] = now_hb
                self.heartbeat_times.append(now_hb)
                armed = bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if self.state.get('armed') is not None and armed != self.state.get('armed'):
                    status_str = 'ARMED (Motors Live)' if armed else 'DISARMED'
                    self._append_log('INFO', f'Vehicle telemetry: Status changed to {status_str}')
                    if not armed and self.state.get('mission_active'):
                        self.state['mission_active'] = False
                        self.state['mission_message'] = 'Mission complete (Landed)'
                self.state['armed'] = armed

                # Parse PX4 / AERIXIS flight mode from heartbeat custom_mode
                custom_main = (message.custom_mode >> 16) & 0xFF
                if custom_main == 6:
                    detected_mode = 'OFFBOARD'
                elif custom_main == 3:
                    detected_mode = 'POSCTL'
                elif custom_main == 2:
                    detected_mode = 'ALTCTL'
                elif custom_main == 1:
                    detected_mode = 'MANUAL'
                elif custom_main == 4:
                    sub = (message.custom_mode >> 24) & 0xFF
                    sub_names = {1: 'READY', 2: 'TAKEOFF', 3: 'LOITER', 4: 'MISSION', 5: 'RTL', 6: 'LAND'}
                    detected_mode = f'AUTO_{sub_names.get(sub, "AUTO")}'
                elif bool(message.base_mode & mavlink.MAV_MODE_FLAG_GUIDED_ENABLED):
                    detected_mode = 'OFFBOARD'
                else:
                    detected_mode = self.state.get('flight_mode', 'OFFBOARD')

                if detected_mode != self.state.get('flight_mode'):
                    is_off = (detected_mode == 'OFFBOARD')
                    self.state['flight_mode'] = detected_mode
                    self.state['is_offboard'] = is_off
                    self.state['as2_active'] = is_off
                    self._append_log('INFO', f'Autopilot telemetry: Active flight mode is {detected_mode}')

                self.state['mode'] = f'AERIXIS / {detected_mode}'

            elif message_type == 'STATUSTEXT':
                self._handle_status_text(message)
            elif message_type == 'GLOBAL_POSITION_INT':
                self.state.update({
                    'latitude': message.lat / 1e7,
                    'longitude': message.lon / 1e7,
                    'altitude': message.relative_alt / 1000.0,
                    'absolute_altitude': message.alt / 1000.0,
                })
            elif message_type == 'MISSION_CURRENT':
                self.state['mission_sequence'] = int(message.seq)
                total = getattr(message, 'total', 0) or self.state.get('mission_total', 0)
                if total:
                    self.state['mission_total'] = total
                    complete = int(message.seq) >= total or (not self.state.get('armed', False) and int(message.seq) > 0)
                    self.state['mission_active'] = not complete
                    self.state['mission_message'] = ('Mission complete (Landed)' if complete
                                                     else f'AERIXIS executing item {message.seq + 1}/{total}')
            elif message_type in ('MISSION_REQUEST_INT', 'MISSION_REQUEST'):
                requested = int(message.seq)
                if self.upload is not None:
                    self._send_mission_item(requested)
            elif message_type == 'MISSION_ACK':
                was_uploading = self.upload is not None
                self.upload = None
                if int(message.type) == mavlink.MAV_MISSION_ACCEPTED:
                    if was_uploading:
                        self.state['mission_message'] = 'Mission submitted to AERIXIS'
                    elif self.stop_pending:
                        self.state['mission_message'] = 'Mission stop accepted by AERIXIS'
                    self.state['mission_active'] = was_uploading
                    self.stop_pending = False
                else:
                    self.state['mission_message'] = f'Mission rejected (MAVLink result {message.type})'
                    if was_uploading:
                        self.state['mission_active'] = False
            elif message_type == 'COMMAND_ACK':
                result = ('accepted' if message.result == mavlink.MAV_RESULT_ACCEPTED
                          else f'result {message.result}')
                self.state['last_command'] = f'Command {message.command}: {result}'
                mode_entry = self.mode_cmds.get(int(message.get_srcSystem()))
                if int(message.command) == mavlink.MAV_CMD_DO_SET_MODE and mode_entry and mode_entry['state'] in ('sending', 'sent'):
                    # IN_PROGRESS = handed to PX4 (its own answer follows); then PX4's result.
                    mode_entry['result'] = int(message.result)
                    mode_entry['acked'] = time.time()
                    names = {0: 'accepted', 1: 'temporarily rejected', 2: 'denied', 3: 'unsupported', 4: 'failed', 6: 'cancelled'}
                    if message.result == 5:
                        mode_entry['state'] = 'sent'
                    else:
                        mode_entry['state'] = 'accepted' if message.result == 0 else 'refused'
                        self._append_log('INFO' if message.result == 0 else 'ERROR',
                                         f"Flight mode {mode_entry['mode']} system {message.get_srcSystem()}: "
                                         f"{names.get(int(message.result), 'result %d' % message.result)}"
                                         + ('' if message.result == 0 else ' (PX4 needs RC sticks/joystick for POSCTL/ALTCTL, a valid position for POSCTL)'))
                entry = self.emergency_cmds.get(int(message.get_srcSystem()))
                if entry and entry['command'] == int(message.command) and entry['state'] == 'sending':
                    # ACCEPTED / IN_PROGRESS = handed to the FC guardian; DENIED = not airborne (nothing to
                    # do); FAILED / other = refused (guardian not running, no PX4 link) — operator must see it.
                    entry['result'] = int(message.result)
                    entry['state'] = ('accepted' if message.result in (0, 5) else
                                      'on-ground' if message.result == 2 and entry['action'] != 'DISARM' else 'refused')
                    entry['acked'] = time.time()
                    self._append_log('INFO' if entry['state'] == 'accepted' else 'ERROR',
                                     f"Emergency {entry['action']} system {message.get_srcSystem()}: {entry['state']} (MAV_RESULT {message.result})")

    def _loop(self):
        while self.running:
            now = time.monotonic()
            if now - self.heartbeat_timestamp >= 1.0:
                try:
                    self._send_heartbeat()
                    self.heartbeat_timestamp = now
                except Exception:
                    pass

            # 1. Check local diagnostics socket
            try:
                for _ in range(200):
                    try:
                        packet, _ = self.diagnostics_socket.recvfrom(4096, socket.MSG_DONTWAIT)
                    except (BlockingIOError, InterruptedError):
                        break
                    self.rx_bytes += len(packet)
                    self.rx_packets += 1
                    self._handle_external_log(packet)
            except Exception:
                pass

            # 2. Check main MAVLink transport (Serial or UDP)
            if self.conn_type == 'serial' and self.serial_conn and self.serial_conn.is_open:
                try:
                    raw = self.serial_conn.read(self.serial_conn.in_waiting or 1)
                    if raw:
                        with self.lock:
                            for msg in self.encoder.parse_buffer(raw) or []:
                                self._handle(msg)
                except Exception:
                    time.sleep(0.02)
            elif self.conn_type == 'udp' and self.socket:
                try:
                    readable, _, _ = select.select([self.socket], [], [], 0.05)
                    # Drain every queued datagram: reading one per loop pass fell behind the
                    # gateway's telemetry rate, and fleet replies then arrived seconds late.
                    for _ in range(500 if readable else 0):
                        try:
                            datagram, _ = self.socket.recvfrom(65535, socket.MSG_DONTWAIT)
                        except (BlockingIOError, InterruptedError):
                            break
                        self.rx_bytes += len(datagram)
                        self.rx_packets += 1
                        with self.lock:
                            for msg in self.encoder.parse_buffer(datagram) or []:
                                self._handle(msg)
                except Exception:
                    time.sleep(0.02)
            else:
                time.sleep(0.05)

    def close(self):
        self.running = False
        self.worker.join(timeout=1.0)
        with self.lock:
            if self.serial_conn:
                try: self.serial_conn.close()
                except Exception: pass
            if self.socket:
                try: self.socket.close()
                except Exception: pass
            try: self.diagnostics_socket.close()
            except Exception: pass



def get_ros_nodes_status():
    """Inspects genuine AERIXIS & PX4 nodes and controllers, returning green/blue/red status."""
    import subprocess
    
    # Strictly genuine AERIXIS and drone simulation/hardware nodes
    node_definitions = [
        {"id": "as2_platform_pixhawk_node", "name": "as2_platform_pixhawk_node", "pattern": "as2_platform_pixhawk_node", "type": "Platform Driver"},
        {"id": "as2_state_estimator_node", "name": "as2_state_estimator_node", "pattern": "as2_state_estimator_node", "type": "State Estimator"},
        {"id": "as2_motion_controller_node", "name": "as2_motion_controller_node", "pattern": "as2_motion_controller_node", "type": "Motion Controller"},
        {"id": "takeoff_behavior_node", "name": "takeoff_behavior_node", "pattern": "takeoff_behavior_node", "type": "Motion Behavior"},
        {"id": "land_behavior_node", "name": "land_behavior_node", "pattern": "land_behavior_node", "type": "Motion Behavior"},
        {"id": "go_to_behavior_node", "name": "go_to_behavior_node", "pattern": "go_to_behavior_node", "type": "Motion Behavior"},
        {"id": "follow_path_behavior_node", "name": "follow_path_behavior_node", "pattern": "follow_path_behavior_node", "type": "Motion Behavior"},
        {"id": "generate_polynomial_trajectory_behavior_node", "name": "trajectory_generator_node", "pattern": "generate_polynomial_trajectory_behavior_node", "type": "Trajectory Gen"},
        {"id": "mission_executor", "name": "mission_executor_node", "pattern": "mission_executor", "type": "Mission Planner"},
        {"id": "mavlink_gateway", "name": "mavlink_gateway_node", "pattern": "mavlink_gateway", "type": "Telemetry Gateway"},
        {"id": "behavior_trees_node", "name": "as2_behavior_tree_node", "pattern": "behavior_trees", "type": "Behavior Tree Engine"},
        {"id": "MicroXRCEAgent", "name": "MicroXRCEAgent", "pattern": "MicroXRCEAgent", "type": "XRCE-DDS Bridge"},
        {"id": "px4", "name": "px4_autopilot", "pattern": "/bin/px4", "type": "Autopilot"},
        {"id": "gz_sim", "name": "gazebo_simulator", "pattern": "gz sim", "type": "Physics Simulator"}
    ]

    # Strictly genuine AERIXIS controllers, plugins and behavior servers
    controller_definitions = [
        {"id": "pid_speed_controller", "name": "pid_speed_controller", "parent_proc": "as2_motion_controller_node", "default_active": True},
        {"id": "differential_flatness_controller", "name": "differential_flatness_controller", "parent_proc": "as2_motion_controller_node", "default_active": False},
        {"id": "raw_odometry_estimator", "name": "raw_odometry_estimator", "parent_proc": "as2_state_estimator_node", "default_active": True},
        {"id": "takeoff_behavior_plugin", "name": "takeoff_behavior_plugin", "parent_proc": "takeoff_behavior_node", "default_active": True},
        {"id": "land_behavior_plugin", "name": "land_behavior_plugin", "parent_proc": "land_behavior_node", "default_active": True},
        {"id": "goto_behavior_plugin", "name": "goto_behavior_plugin", "parent_proc": "go_to_behavior_node", "default_active": True},
        {"id": "follow_path_behavior_plugin", "name": "follow_path_behavior_plugin", "parent_proc": "follow_path_behavior_node", "default_active": True},
        {"id": "dynamic_mav_trajectory_generator", "name": "dynamic_mav_trajectory_generator", "parent_proc": "generate_polynomial_trajectory_behavior_node", "default_active": True}
    ]

    nodes_res = []
    for node in node_definitions:
        ret = subprocess.run(['pgrep', '-f', node['pattern']], stdout=subprocess.PIPE)
        is_running = (ret.returncode == 0)
        status = 'running' if is_running else 'offline'

        nodes_res.append({
            "id": node['id'],
            "name": node['name'],
            "type": node['type'],
            "status": status
        })

    ctrl_res = []
    for ctrl in controller_definitions:
        ret = subprocess.run(['pgrep', '-f', ctrl['parent_proc']], stdout=subprocess.PIPE)
        parent_running = (ret.returncode == 0)
        status = 'running' if (parent_running and ctrl.get('default_active')) else 'offline'
            
        ctrl_res.append({
            "id": ctrl['id'],
            "name": ctrl['name'],
            "status": status
        })

    return {
        "nodes": nodes_res,
        "controllers": ctrl_res,
        "summary": {
            "total_nodes": len(nodes_res),
            "running_nodes": sum(1 for n in nodes_res if n['status'] == 'running'),
            "error_nodes": sum(1 for n in nodes_res if n['status'] == 'error'),
            "offline_nodes": sum(1 for n in nodes_res if n['status'] == 'offline'),
            "total_controllers": len(ctrl_res),
            "running_controllers": sum(1 for c in ctrl_res if c['status'] == 'running')
        }
    }


def make_handler(bridge, fleet=None, missions=None):
    missions = missions or MissionStore(Path(__file__).parent / 'config/missions')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # suppress clutter in terminal

        def _json(self, value, status=200):
            body = json.dumps(value).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/api/fleet':
                self._json(fleet.snapshot() if fleet else {'drones': []})
                return
            if self.path == '/api/state':
                self._json(bridge.state_snapshot())
                return
            if self.path == '/api/ports':
                ports = list_available_ports()
                self._json({
                    'serial_ports': ports,
                    'current_connection': bridge.state['connection'],
                    'has_serial': HAS_SERIAL
                })
                return
            if self.path == '/api/ros/nodes':
                self._json(get_ros_nodes_status())
                return
            if self.path == '/api/bt/logs':
                self._json({'logs': bridge.state.get('logs', [])})
                return
            if self.path == '/api/missions':
                self._json(missions.catalog())
                return
            if self.path.startswith('/api/onboard/discover'):
                # Onboard panels (port 8090) on this computer's networks, for "Onboard calibration".
                from urllib.parse import urlparse, parse_qs
                query = parse_qs(urlparse(self.path).query)
                hints = [h for h in (query.get('hint') or [''])[0].split(',') if h]
                self._json({'panels': onboard_discovery.discover(fresh='fresh' in query, hints=hints)})
                return
            if self.path == '/api/plan/status':
                # Report planner availability so the UI can disable the KML
                # workflow honestly instead of failing at upload time.
                ok, reason = planner_bridge.available()
                self._json({'available': ok, 'reason': reason})
                return

            # Generic static file serving
            rel_path = self.path.split('?')[0].lstrip('/')
            if not rel_path or rel_path == 'index.html':
                rel_path = 'index.html'
            fpath = (WEB_ROOT / rel_path).resolve()
            if fpath.is_file() and (fpath == WEB_ROOT / rel_path or WEB_ROOT in fpath.parents):
                mimes = {
                    '.html': 'text/html; charset=utf-8',
                    '.css': 'text/css; charset=utf-8',
                    '.js': 'application/javascript; charset=utf-8',
                    '.json': 'application/json; charset=utf-8',
                    '.png': 'image/png',
                    '.svg': 'image/svg+xml',
                    '.webp': 'image/webp',
                    '.ico': 'image/x-icon'
                }
                ctype = mimes.get(fpath.suffix.lower(), 'application/octet-stream')
                data = fpath.read_bytes()
                self.send_response(200)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(data)
                return

            self._json({'error': 'Not found'}, 404)

        def do_POST(self):
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 1_100_000:
                    raise ValueError('Request too large')
                request = json.loads(self.rfile.read(length) or b'{}')
                if not isinstance(request, dict):
                    raise ValueError('Expected a JSON object')

                if self.path == '/api/fleet/remove':
                    if self.headers.get('X-AS2-GCS') != '1':
                        raise ValueError('Missing GCS request header')
                    if fleet is None:
                        raise ValueError('Fleet service is unavailable')
                    self._json(fleet.remove(str(request.get('drone_id', ''))))
                    return
                if self.path in ('/api/fleet/profile', '/api/fleet/command'):
                    if self.headers.get('X-AS2-GCS') != '1':
                        raise ValueError('Missing GCS request header')
                    if fleet is None:
                        raise ValueError('Fleet service is unavailable')
                    if self.path.endswith('/command') and request.get('action') == 'start' and bridge.safety_safe:
                        raise ValueError('Safety switch is SAFE: switch it to READY in the menu before Start')
                    result = fleet.configure(request) if self.path.endswith('/profile') else fleet.command(request)
                    self._json(result)
                    return
                if self.path in ('/api/missions/save', '/api/missions/delete', '/api/missions/describe'):
                    if self.headers.get('X-AS2-GCS') != '1':
                        raise ValueError('Missing GCS request header')
                    action = self.path.rsplit('/', 1)[1]
                    self._json(getattr(missions, action)(request))
                    return
                if self.path == '/api/avoidance':
                    # AI avoidance for the whole fleet (or the listed drones), each over its signed link.
                    if self.headers.get('X-AS2-GCS') != '1':
                        raise ValueError('Missing GCS request header')
                    if fleet is None:
                        raise ValueError('Fleet service is unavailable')
                    enabled = request.get('enabled')
                    if not isinstance(enabled, bool):
                        raise ValueError('enabled must be true or false')
                    wanted = request.get('drone_ids') or list(fleet.profiles)
                    results = {}
                    for drone_id in wanted:
                        try:
                            fleet.command(dict(drone_id=drone_id, action='avoidance', enabled=enabled))
                            results[drone_id] = 'ok'
                        except (ValueError, OSError, RuntimeError) as exc:
                            results[drone_id] = str(exc)
                    with bridge.state_lock:
                        bridge._append_log('WARN' if enabled else 'INFO', 'AI avoidance ' + ('ON' if enabled else 'off') +
                                           ': ' + ', '.join(f'{k} {v}' for k, v in results.items()))
                    self._json({'ok': all(v == 'ok' for v in results.values()), 'results': results})
                    return
                if self.path == '/api/safety':
                    if self.headers.get('X-AS2-GCS') != '1':
                        raise ValueError('Missing GCS request header')
                    with bridge.state_lock:
                        bridge.safety_safe = bool(request.get('safe', True))
                        bridge._append_log('WARN' if not bridge.safety_safe else 'INFO',
                                           'Safety switch ' + ('SAFE: mission start blocked' if bridge.safety_safe
                                                               else 'READY: mission start allowed'))
                    self._json({'ok': True, 'safe': bridge.safety_safe})
                    return
                if self.path == '/api/emergency':
                    if self.headers.get('X-AS2-GCS') != '1':
                        raise ValueError('Missing GCS request header')
                    # Drone IDs -> MAVLink system IDs from the fleet profiles (radio aircraft only).
                    profiles = dict(fleet.profiles) if fleet else {}
                    wanted = request.get('drone_ids')
                    if wanted == 'all':
                        wanted = list(profiles)
                    if not isinstance(wanted, list) or not wanted:
                        raise ValueError('Choose the aircraft for the emergency command')
                    unknown = [d for d in wanted if d not in profiles or profiles[d].get('transport') != 'mavlink']
                    if unknown:
                        raise ValueError('No telemetry-radio aircraft: ' + ', '.join(map(str, unknown)))
                    self._json(bridge.emergency([profiles[d]['system_id'] for d in wanted],
                                                str(request.get('action', '')), request.get('reason', '')))
                    return
                if self.path == '/api/plan/kml':
                    if self.headers.get('X-AS2-GCS') != '1':
                        raise ValueError('Missing GCS request header')
                    # Planning validates, partitions and produces preview XML.
                    # It never uploads or starts anything: upload stays a
                    # separate, individually acknowledged step per aircraft.
                    result = planner_bridge.plan_mission(request)
                    self._json(result)
                    return
                if self.path == '/api/mode' and request.get('drone_ids'):
                    # Per-aircraft flight mode (Flight mode dialog): drone IDs -> MAVLink system IDs.
                    if self.headers.get('X-AS2-GCS') != '1':
                        raise ValueError('Missing GCS request header')
                    profiles = dict(fleet.profiles) if fleet else {}
                    wanted = request.get('drone_ids')
                    if wanted == 'all':
                        wanted = list(profiles)
                    unknown = [d for d in wanted if d not in profiles or profiles[d].get('transport') != 'mavlink']
                    if unknown:
                        raise ValueError('No telemetry-radio aircraft: ' + ', '.join(map(str, unknown)))
                    self._json(bridge.set_modes([profiles[d]['system_id'] for d in wanted], request.get('mode', '')))
                    return
                if self.path == '/api/mode':
                    res = bridge.set_flight_mode(request.get('mode', 'offboard'))
                    self._json(res)
                elif self.path == '/api/connect':
                    res = bridge.connect(request)
                    self._json(res)
                elif self.path == '/api/disconnect':
                    res = bridge.disconnect()
                    self._json(res)
                elif self.path == '/api/command':
                    bridge.command(request.get('action', ''), request.get('altitude'))
                    self._json({'ok': True})
                elif self.path == '/api/mission':
                    count = bridge.upload_mission(request)
                    self._json({'ok': True, 'count': count})
                elif self.path == '/api/tree/run':
                    count = bridge.run_behavior_tree(request)
                    self._json({'ok': True, 'count': count})
                elif self.path == '/api/mission/stop':
                    bridge.stop_mission()
                    self._json({'ok': True})
                else:
                    self._json({'error': 'Not found'}, 404)
            except (ValueError, TypeError, KeyError, OSError, RuntimeError) as exc:
                self._json({'error': str(exc)}, 400)
            except Exception as exc:
                self._json({'error': f'Internal Server Error: {exc}'}, 500)

    return Handler


def main():
    parser = argparse.ArgumentParser(description='AERIXIS GCS Server')
    parser.add_argument('--gateway-host', default='127.0.0.1')
    parser.add_argument('--gateway-port', type=int, default=14550)
    parser.add_argument('--listen-host', default='0.0.0.0')
    parser.add_argument('--listen-port', type=int, default=14551)
    parser.add_argument('--diagnostics-host', default='127.0.0.1')
    parser.add_argument('--diagnostics-port', type=int, default=14552)
    parser.add_argument('--web-host', default='127.0.0.1')
    parser.add_argument('--fleet-config', default=str(Path(__file__).parent / 'config/fleet.json'))
    parser.add_argument('--web-port', type=int, default=8765)
    parser.add_argument('--home-latitude', type=float, default=HOME_LATITUDE)
    parser.add_argument('--home-longitude', type=float, default=HOME_LONGITUDE)
    parser.add_argument('--home-altitude', type=float, default=HOME_ALTITUDE)
    args = parser.parse_args()

    bridge = MavlinkBridge(
        args.gateway_host, args.gateway_port, args.listen_host, args.listen_port,
        args.home_latitude, args.home_longitude, args.home_altitude,
        args.diagnostics_host, args.diagnostics_port)

    fleet = Fleet(args.fleet_config, bridge.fleet_radio)
    missions = MissionStore(Path(args.fleet_config).parent / 'missions')
    server = ThreadingHTTPServer((args.web_host, args.web_port), make_handler(bridge, fleet, missions))
    print(f'AERIXIS Mission Control · GCS server running at http://{args.web_host}:{args.web_port}')
    try:
        server.serve_forever()
    finally:
        fleet.close()
        bridge.close()


if __name__ == '__main__':
    main()
