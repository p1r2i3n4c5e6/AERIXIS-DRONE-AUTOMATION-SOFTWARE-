"""Localhost web GCS that speaks MAVLink only to the AERIXIS gateway."""

import argparse
import json
import math
import select
import socket
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

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


class MavlinkBridge:
    """Owns one UDP socket and translates browser requests into MAVLink common."""

    def __init__(self, gateway_host, gateway_port, listen_host, listen_port,
                 home_latitude, home_longitude, home_altitude,
                 diagnostics_host='127.0.0.1', diagnostics_port=14552):
        self.gateway = (gateway_host, gateway_port)
        self.home = (home_latitude, home_longitude, home_altitude)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((listen_host, listen_port))
        self.diagnostics_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.diagnostics_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.diagnostics_socket.bind((diagnostics_host, diagnostics_port))
        self.encoder = mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
        self.lock = threading.RLock()
        self.state_lock = threading.RLock()
        self.state = {
            'connected': False, 'armed': False, 'mode': 'Waiting for AERIXIS gateway',
            'latitude': home_latitude, 'longitude': home_longitude,
            'altitude': 0.0, 'absolute_altitude': home_altitude,
            'mission_sequence': 0, 'mission_total': 0,
            'mission_active': False,
            'mission_message': 'No mission uploaded', 'last_command': '',
            'last_seen': 0.0, 'gateway_heartbeat': 0.0,
            'as2_link': {'connected': None, 'detail': 'Waiting for gateway diagnostics', 'updated': 0.0},
            'px4_link': {'connected': None, 'detail': 'Waiting for PX4 DDS diagnostics', 'updated': 0.0},
            'logs': [{
                'time': datetime.now().astimezone().strftime('%H:%M:%S'),
                'severity': 'INFO',
                'message': 'Waiting for AERIXIS MAVLink gateway heartbeat on UDP 14550',
            }], 'home_latitude': home_latitude,
            'home_longitude': home_longitude, 'home_altitude': home_altitude,
        }
        self._append_log(
            'INFO', f'Local process diagnostics listening on '
            f'{diagnostics_host}:{diagnostics_port}')
        self.upload = None
        self.stop_pending = False
        self.text_chunks = {}
        self.gateway_was_online = False
        self.running = True
        self.heartbeat_timestamp = 0.0
        self.worker = threading.Thread(target=self._loop, name='mavlink-udp', daemon=True)
        self.worker.start()

    def state_snapshot(self):
        now = time.monotonic()
        with self.state_lock:
            gateway_online = now - self.state['gateway_heartbeat'] < 3.0
            if gateway_online != self.gateway_was_online:
                message = ('MAVLink heartbeat received from AERIXIS gateway' if gateway_online
                           else 'MAVLink gateway heartbeat timed out')
                self._append_log('INFO' if gateway_online else 'ERROR', message)
                self.gateway_was_online = gateway_online
            state = dict(self.state)
            state['logs'] = list(state['logs'])
            state['as2_link'] = dict(state['as2_link'])
            state['px4_link'] = dict(state['px4_link'])
        state['connected'] = gateway_online
        state['gateway_detail'] = ('MAVLink heartbeat active' if state['connected']
                                   else 'No gateway heartbeat on UDP 14550')
        for link_key in ('as2_link', 'px4_link'):
            link = state[link_key]
            fresh = now - link['updated'] < 5.0
            if not state['connected']:
                link['connected'] = False
                link['detail'] = 'MAVLink gateway is offline'
            elif not fresh:
                link['connected'] = False
                link['detail'] = 'No recent link diagnostic from gateway'
            elif link['connected'] is None:
                link['connected'] = False
        state.pop('last_seen', None)
        state.pop('gateway_heartbeat', None)
        state['logs'] = state['logs'][-100:]
        return state

    def _append_log(self, severity, message):
        if self.state['logs'] and self.state['logs'][-1]['message'] == message:
            return
        self.state['logs'].append({
            'time': datetime.now().astimezone().strftime('%H:%M:%S'),
            'severity': severity, 'message': message,
        })
        self.state['logs'] = self.state['logs'][-100:]

    def _handle_external_log(self, packet):
        try:
            severity, source, message = packet.decode(
                'utf-8', errors='replace').split('|', 2)
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
            return value.decode('utf-8', errors='replace').rstrip('\x00')
        return str(value).rstrip('\x00')

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
            if text.startswith('LINK|'):
                parts = text.split('|', 3)
                if len(parts) == 4:
                    _, link_name, status, detail = parts
                    key = {'AS2': 'as2_link', 'PX4_DDS': 'px4_link'}.get(link_name)
                    if key:
                        link = self.state[key]
                        connected = status == 'UP'
                        previous = link['connected']
                        link.update({'connected': connected, 'detail': detail,
                                     'updated': time.monotonic()})
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

    def _send(self, message):
        with self.lock:
            self.socket.sendto(message.pack(self.encoder), self.gateway)

    def _send_heartbeat(self):
        self.encoder.heartbeat_send(
            mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
            mavlink.MAV_STATE_ACTIVE)

    def command(self, action, altitude=None):
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
            self._append_log('INFO', f'Flight command: {action.upper()} sent to AERIXIS gateway')

    def upload_mission(self, request):
        polygon = [(float(p[0]), float(p[1])) for p in request.get('polygon', [])]
        waypoints = [(float(p[0]), float(p[1])) for p in request.get('waypoints', [])]
        altitude = float(request.get('altitude', 2.0))
        if len(polygon) < 3:
            raise ValueError('Draw a play-area polygon with at least three points first')
        if not 1 <= len(waypoints) <= 97:
            raise ValueError('Add between 1 and 97 route waypoints')
        if not 0.5 <= altitude <= 10.0:
            raise ValueError('Mission altitude must be between 0.5 m and 10 m')
        for point in waypoints:
            if not point_in_polygon(point, polygon):
                raise ValueError('Every waypoint must be inside the drawn/loaded KML boundary')
            east, north = offset_meters(point[0], point[1], *self.home[:2])
            if abs(east) > 20000.0 or abs(north) > 20000.0:
                raise ValueError('Waypoint exceeds maximum allowed distance (20 km)')
        if not point_in_polygon(self.home[:2], polygon):
            raise ValueError('The play-area polygon must contain the home/landing point')

        home_lat, home_lon, _ = self.home
        mission = [(mavlink.MAV_CMD_NAV_TAKEOFF, home_lat, home_lon, altitude)]
        mission.extend((mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, altitude)
                       for lat, lon in waypoints)
        # AERIXIS's land behavior lands where it is called, so return to home first.
        last_lat, last_lon = waypoints[-1]
        if offset_meters(last_lat, last_lon, home_lat, home_lon)[0] ** 2 + \
                offset_meters(last_lat, last_lon, home_lat, home_lon)[1] ** 2 > 0.25:
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

    def stop_mission(self):
        with self.state_lock:
            self.stop_pending = True
        self._send(self.encoder.mission_clear_all_encode(1, 190))
        with self.state_lock:
            self.state['mission_message'] = 'Stop requested through AERIXIS'

    def _handle(self, message):
        message_type = message.get_type()
        with self.state_lock:
            self.state['last_seen'] = time.monotonic()
            if message_type == 'HEARTBEAT':
                self.state['gateway_heartbeat'] = time.monotonic()
                armed = bool(message.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if self.state.get('armed') is not None and armed != self.state.get('armed'):
                    status_str = 'ARMED (Motors Live)' if armed else 'DISARMED'
                    self._append_log('INFO', f'Vehicle telemetry: Status changed to {status_str}')
                self.state['armed'] = armed
                self.state['mode'] = 'AERIXIS / PX4 SITL'
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
                if getattr(message, 'total', 0):
                    self.state['mission_total'] = int(message.total)
                    complete = int(message.seq) >= int(message.total)
                    self.state['mission_active'] = not complete
                    self.state['mission_message'] = 'Mission complete' if complete else (
                        f'AERIXIS mission item {message.seq + 1}/{message.total}')
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
                result = 'accepted' if message.result == mavlink.MAV_RESULT_ACCEPTED else \
                    f'result {message.result}'
                self.state['last_command'] = f'Command {message.command}: {result}'

    def _loop(self):
        while self.running:
            now = time.monotonic()
            if now - self.heartbeat_timestamp >= 1.0:
                try:
                    self._send(self.encoder.heartbeat_encode(
                        mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                        mavlink.MAV_STATE_ACTIVE))
                    self.heartbeat_timestamp = now
                except OSError:
                    pass
            readable, _, _ = select.select(
                [self.socket, self.diagnostics_socket], [], [], 0.1)
            if not readable:
                continue
            if self.diagnostics_socket in readable:
                try:
                    packet, _ = self.diagnostics_socket.recvfrom(4096)
                    self._handle_external_log(packet)
                except OSError:
                    pass
            if self.socket not in readable:
                continue
            try:
                datagram, _ = self.socket.recvfrom(65535)
                with self.lock:
                    for byte in datagram:
                        message = self.encoder.parse_char(bytes((byte,)))
                        if message is not None:
                            self._handle(message)
            except (OSError, ValueError):
                continue

    def close(self):
        self.running = False
        self.worker.join(timeout=1.0)
        self.socket.close()
        self.diagnostics_socket.close()


def make_handler(bridge):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            print('GCS HTTP: ' + fmt % args)

        def _json(self, value, status=200):
            body = json.dumps(value).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/api/state':
                self._json(bridge.state_snapshot())
                return
            if self.path not in ('/', '/index.html'):
                self._json({'error': 'Not found'}, 404)
                return
            page = (WEB_ROOT / 'index.html').read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)

        def do_POST(self):
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if length > 1_000_000:
                    raise ValueError('Request too large')
                request = json.loads(self.rfile.read(length) or b'{}')
                if self.path == '/api/command':
                    bridge.command(request.get('action', ''), request.get('altitude'))
                    self._json({'ok': True})
                elif self.path == '/api/mission':
                    count = bridge.upload_mission(request)
                    self._json({'ok': True, 'count': count})
                elif self.path == '/api/mission/stop':
                    bridge.stop_mission()
                    self._json({'ok': True})
                else:
                    self._json({'error': 'Not found'}, 404)
            except (ValueError, TypeError, KeyError, OSError) as exc:
                self._json({'error': str(exc)}, 400)

    return Handler


def main():
    parser = argparse.ArgumentParser(description='AERIXIS MAVLink simulation GCS')
    parser.add_argument('--gateway-host', default='127.0.0.1')
    parser.add_argument('--gateway-port', type=int, default=14550)
    parser.add_argument('--listen-host', default='127.0.0.1')
    parser.add_argument('--listen-port', type=int, default=14551)
    parser.add_argument('--diagnostics-host', default='127.0.0.1')
    parser.add_argument('--diagnostics-port', type=int, default=14552)
    parser.add_argument('--web-host', default='127.0.0.1')
    parser.add_argument('--web-port', type=int, default=8765)
    parser.add_argument('--home-latitude', type=float, default=HOME_LATITUDE)
    parser.add_argument('--home-longitude', type=float, default=HOME_LONGITUDE)
    parser.add_argument('--home-altitude', type=float, default=HOME_ALTITUDE)
    args = parser.parse_args()

    bridge = MavlinkBridge(
        args.gateway_host, args.gateway_port, args.listen_host, args.listen_port,
        args.home_latitude, args.home_longitude, args.home_altitude,
        args.diagnostics_host, args.diagnostics_port)
    server = ThreadingHTTPServer((args.web_host, args.web_port), make_handler(bridge))
    print(f'AERIXIS SITL GCS: http://{args.web_host}:{args.web_port}')
    print(f'MAVLink gateway: {args.gateway_host}:{args.gateway_port}; '
          f'GCS UDP: {args.listen_host}:{args.listen_port}')
    print(f'Local process diagnostics UDP: '
          f'{args.diagnostics_host}:{args.diagnostics_port}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        bridge.close()


if __name__ == '__main__':
    main()
