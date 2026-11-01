#!/usr/bin/env python3
"""Onboard control panel: the Pi GUI as a web page, for headless onboard computers.

Started by as2_onboard_service.py as its LAST link (after everything else is up), it serves
http://<onboard-computer>:8090 to any browser on the drone's network (laptop, phone):

  * live state of every onboard link (FC link, FC guardian, AERIXIS core, mission manager,
    GCS telemetry gateway, mesh, camera) with restart buttons
  * settings for the GCS telemetry radio, the flight controller (UART or UDP), the A8 camera
    (and its live dashboard), the mesh, identity and safety switches
  * the GCS pairing token, plugged-in serial ports and the onboard log
  * Camera & AI models: choose the onboard computer, see its real hardware and the CPU AS2 leaves,
    upload / benchmark / delete detection models and choose the active one and its classes
    (models that this board cannot run fast enough, or at all, are locked)

Changes need the panel PIN (printed by `as2_onboard_service.py token`, stored next to the
settings). Restarting the FC link, guardian, AERIXIS core, mission manager or GCS link is refused
while the aircraft is armed or a mission runs. Settings go to the same file as the Pi GUI
(~/.config/as2/pi_launcher.json); values fixed by AS2_* environment variables are shown read-only.
Standard library only: no ROS needed (the model benchmark runs in the tracker's Python).
"""
import argparse
import hmac
import json
import os
import secrets
import signal
import socket
import subprocess
import sys
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import onboard_links as links  # noqa: E402


def _tracker_src():
    here = Path(__file__).resolve().parent
    workspace = Path(os.environ.get('AS2_RESCUE_WORKSPACE') or here.parent)
    for folder in (Path(os.environ['A8_TRACKER_HOME']) / 'src' if os.environ.get('A8_TRACKER_HOME') else None,
                   workspace / 'install/as2_a8_tracker/share/as2_a8_tracker/tracker/src',
                   here.parent / 'src/as2_a8_tracker/tracker/src'):
        if folder and (folder / 'model_library.py').exists():
            return folder
    return None


try:   # detection model library (Camera & AI models card); the panel works without it
    sys.path.insert(0, str(_tracker_src()))
    import model_library  # noqa: E402
except Exception as _exc:   # tracker not installed / older tracker
    model_library = None
    MODEL_LIBRARY_ERROR = str(_exc)

STATUS_FILE = Path(os.environ.get('AS2_STATUS_FILE', '/tmp/as2-onboard-status.json'))
CONTROL_FILE = Path(os.environ.get('AS2_CONTROL_FILE', '/tmp/as2-onboard-control.json'))
LOG_FILE = Path(os.environ.get('AS2_LOG_FILE', '/tmp/as2-onboard.log'))
PIN_FILE = links.CONFIG_DIR / 'panel.pin'
FLIGHT_LINKS = {'fc', 'guardian', 'as2', 'manager', 'gcs', 'mesh'}   # never restarted while armed / flying
EDITABLE = [k for k in links.DEFAULTS if k not in ('sim_persons',)] + ['sim_persons']
SECRET = ('mesh_passphrase',)


def panel_pin():
    """The panel PIN: AS2_PANEL_PIN (onboard.env) on first start, else a random one; changeable
    from the panel (Drone page)."""
    if not PIN_FILE.exists():
        PIN_FILE.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(str(PIN_FILE), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            stream.write((os.environ.get('AS2_PANEL_PIN') or secrets.token_urlsafe(6)) + '\n')
    return PIN_FILE.read_text().strip()


def set_panel_pin(new):
    new = str(new or '').strip()
    if not 4 <= len(new) <= 32 or not new.isprintable() or ' ' in new:
        raise ValueError('PIN: 4-32 characters, no spaces')
    tmp = PIN_FILE.with_suffix('.tmp')
    fd = os.open(str(tmp), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as stream:
        stream.write(new + '\n')
    os.replace(tmp, PIN_FILE)
    return new


GIMBAL_POSES = {'centre': (0.0, 0.0), 'down45': (0.0, -45.0), 'down': (0.0, -90.0)}
TRACKER_CONTROL = ('127.0.0.1', 9091)


def gimbal_command(pose):
    """Point the A8 gimbal for a ground check: drop any lock (10 s), then an absolute angle.
    The tracker applies it only in Track mode (Preview never moves the gimbal)."""
    if pose not in GIMBAL_POSES:
        raise ValueError('Unknown gimbal position')
    yaw, pitch = GIMBAL_POSES[pose]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(json.dumps(dict(cmd='release', suppress_s=10)).encode(), TRACKER_CONTROL)
        sock.sendto(json.dumps(dict(cmd='set_attitude', yaw=yaw, pitch=pitch)).encode(), TRACKER_CONTROL)
    finally:
        sock.close()
    return dict(yaw=yaw, pitch=pitch)


def addresses():
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2).stdout.split()
    except (OSError, subprocess.TimeoutExpired):
        out = []
    return [a for a in out if ':' not in a] or ['127.0.0.1']


def serial_ports():
    ports = []
    by_id = Path('/dev/serial/by-id')
    if by_id.is_dir():
        for p in sorted(by_id.iterdir()):
            ports.append(dict(path=str(p), device=os.path.realpath(p), name=p.name))
    seen = {p['device'] for p in ports}
    for pattern in ('ttyAMA*', 'ttyACM*', 'ttyUSB*', 'serial0'):
        for p in sorted(Path('/dev').glob(pattern)):
            if str(p) not in seen and os.path.realpath(p) not in seen:
                ports.append(dict(path=str(p), device=os.path.realpath(p), name=p.name))
    return ports


def vehicle(settings):
    """Armed / mission state from the onboard mission manager (loopback, pairing token)."""
    try:
        token = links.token_file(links.drone_id(settings.get)).read_text().strip()
        req = urllib.request.Request('http://127.0.0.1:8790/api/status?lite=1', headers={'Authorization': 'Bearer ' + token})
        with urllib.request.urlopen(req, timeout=2) as stream:
            data = json.load(stream)
    except Exception as exc:  # manager not up yet
        return dict(available=False, error=str(exc)[:120])
    rescue = ((data.get('ros') or {}).get('rescue') or {}).get('data') or {}
    v = rescue.get('vehicle') or {}
    links_ = (data.get('ros') or {}).get('links') or {}
    fc = links_.get('fc') or {}
    return dict(available=True, armed=bool(v.get('armed')), active=bool(data.get('active')),
                connected=bool(v.get('connected')), battery=v.get('battery'), position=v.get('position'),
                gps=rescue.get('gps'), gnss=fc.get('gps'), fc_status=fc.get('status'), faults=fc.get('faults'),
                sensors=fc.get('sensors'), messages=fc.get('messages'), vision=links_.get('vision'),
                mesh=links_.get('swarm'), radio=links_.get('mesh'), avoidance=rescue.get('avoidance'),
                execution=(data.get('execution') or {}).get('state'),
                ros_nodes=(data.get('ros') or {}).get('count'))


def link_states():
    try:
        states = json.loads(STATUS_FILE.read_text())
    except (OSError, ValueError):
        states = {}
    # FC link and guardian run as their own systemd units on a native Pi install.
    for name in ('fc', 'guardian'):
        if name not in states:
            pid = _pgrep('as2_onboard_service.py exec ' + name)
            states[name] = dict(state='running' if pid else 'not running', restarts=0, detail='systemd unit' if pid else '', pid=pid)
    return states


def _pgrep(pattern):
    try:
        out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True, timeout=2).stdout.split()
    except (OSError, subprocess.TimeoutExpired):
        return None
    own = {str(os.getpid())}
    pids = [p for p in out if p not in own]
    return int(pids[0]) if pids else None


def restart(names, states):
    """Restart links: the supervisor (control file) or, for systemd-run links, stop the process."""
    requests = []
    if 'all' in names:   # every supervised link, plus FC link / guardian when they are systemd units
        names = ['all'] + [n for n, i in states.items() if i.get('detail') == 'systemd unit']
    for name in names:
        item = states.get(name) or {}
        if item.get('detail') == 'systemd unit' and item.get('pid'):
            os.kill(item['pid'], signal.SIGTERM)   # Restart=always brings it back with the new settings
        else:
            requests.append(name)
    if requests:
        CONTROL_FILE.write_text(json.dumps(dict(id=secrets.token_hex(4), restart=requests, t=time.time())))
    return requests


def log_tail(lines=250):
    try:
        with open(LOG_FILE, 'rb') as stream:
            stream.seek(0, 2)
            size = stream.tell()
            stream.seek(max(0, size - 64000))
            return stream.read().decode(errors='replace').splitlines()[-lines:]
    except OSError:
        return []


_CACHE = {}


def _cached(key, seconds, make):
    hit = _CACHE.get(key)
    if hit and time.time() - hit[0] < seconds:
        return hit[1]
    value = make()
    _CACHE[key] = (time.time(), value)
    return value


def vision_state(settings):
    """Camera & AI models card: board (chosen and detected), CPU budget, models with lock state."""
    if model_library is None:
        return dict(available=False, error='Model library not installed: ' + MODEL_LIBRARY_ERROR)
    real = _cached('board', 60, model_library.detect_board)
    budget = _cached('budget', 10, lambda: model_library.cpu_budget(0.5))
    available = _cached('runtimes', 3600, model_library.runtimes)
    active = model_library.load_active()
    auto = active['board'] not in model_library.BOARDS       # '' = auto-detect (default)
    board = real['guess'] if auto else active['board']
    detector_cores = len(str(settings.get('detector_cores') or '').split()) or budget['cores']
    threads = max(1, min(budget['detector_threads'], detector_cores))
    return dict(available=True, board=board, board_auto=auto, detected=real['guess'], real=real,
                warnings=[] if auto else model_library.board_warnings(board, real), budget=budget, threads=threads,
                runtimes=available, active=active, policy=active['policy'],
                need_hz=model_library.required_detect_hz(active['policy']),
                boards={k: dict(label=v['label'], formats=v['formats'], tops=v['tops']) for k, v in model_library.BOARDS.items()},
                models=model_library.list_models(board, threads, available, active['policy']),
                sources=model_library.ESTIMATE_SOURCES, library=str(model_library.library_dir()))


def locked_keys():
    return sorted(k for k in links.DEFAULTS if os.environ.get('AS2_' + k.upper()) is not None)


def make_handler(port_info):
    pin_box = [panel_pin()]

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, value, status=200, ctype='application/json'):
            body = value if isinstance(value, bytes) else json.dumps(value).encode()
            self.send_response(status)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def authorised(self):
            given = self.headers.get('X-Panel-Pin', '')
            if hmac.compare_digest(given.encode(), pin_box[0].encode()):
                return True
            time.sleep(1.0)   # slow down guessing
            return False

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                page = PAGE if links.sim_camera_available() else PAGE.replace(',sim:Simulated', '')
                return self.reply(page.encode(), ctype='text/html; charset=utf-8')
            if self.path == '/api/state':
                settings = links.load_settings()
                masked = {k: ('' if k in SECRET else v) for k, v in settings.items()}
                masked['mesh_passphrase_set'] = bool(settings.get('mesh_passphrase'))
                return self.reply(dict(
                    hostname=socket.gethostname(), addresses=addresses(), web_port=port_info,
                    drone_id=settings.get('drone_id'), system_id=settings.get('system_id'),
                    settings=masked, locked=locked_keys(), links=link_states(), vehicle=vehicle(settings),
                    ports=serial_ports(), log=log_tail(), time=time.time()))
            if self.path == '/api/vision':
                try:
                    return self.reply(vision_state(links.load_settings()))
                except Exception as exc:
                    return self.reply(dict(available=False, error=str(exc)[:300]))
            return self.reply({'error': 'not found'}, 404)

        def upload(self):
            """Raw model upload (not JSON: models are megabytes). PIN checked before reading."""
            from urllib.parse import parse_qs, urlparse
            if not self.authorised():
                return self.reply({'error': 'Wrong panel PIN'}, 403)
            if model_library is None:
                return self.reply({'error': 'Model library not installed'}, 501)
            veh = vehicle(links.load_settings())
            if veh.get('armed') or veh.get('active'):
                return self.reply({'error': 'Aircraft is armed or a mission is running: upload after landing.'}, 409)
            query = {k: v[0] for k, v in parse_qs(urlparse(self.path).query).items()}
            size = int(self.headers.get('Content-Length', '0'))
            if not 0 < size <= model_library.MAX_UPLOAD_MB * 1000000:
                return self.reply({'error': 'Upload must be 1 byte .. %d MB' % model_library.MAX_UPLOAD_MB}, 413)
            data = bytearray()
            while len(data) < size:
                chunk = self.rfile.read(min(1 << 20, size - len(data)))
                if not chunk:
                    break
                data.extend(chunk)
            if len(data) != size:
                return self.reply({'error': 'Upload interrupted'}, 400)
            names = [n.strip() for n in query.get('classes', '').split(',') if n.strip()]
            imgsz = [int(v) for v in query.get('imgsz', '').replace('x', ',').split(',') if v.strip()] or None
            if imgsz and len(imgsz) != 2:
                return self.reply({'error': 'Input size: height,width (e.g. 384,640)'}, 400)
            try:
                folder = model_library.add_model(query.get('name', ''), query.get('filename', ''), bytes(data),
                                                 names or None, imgsz)
            except (ValueError, OSError) as exc:
                return self.reply({'error': str(exc)}, 400)
            _CACHE.pop('runtimes', None)
            return self.reply(dict(ok=True, model=folder.name))

        def do_POST(self):
            if self.path.startswith('/api/vision/upload'):
                return self.upload()
            try:
                size = int(self.headers.get('Content-Length', '0'))
                data = json.loads(self.rfile.read(min(size, 100000)) or b'{}')
            except ValueError:
                return self.reply({'error': 'bad request'}, 400)
            if not self.authorised():
                return self.reply({'error': 'Wrong panel PIN'}, 403)
            settings = links.load_settings()
            veh = vehicle(settings)
            flying = veh.get('armed') or veh.get('active')
            if self.path == '/api/token':
                drone = links.drone_id(settings.get)
                return self.reply(dict(drone_id=drone, system_id=settings.get('system_id'),
                                       token=links.ensure_token(drone).read_text().strip()))
            if self.path == '/api/restart':
                names = data.get('links') or []
                states = link_states()
                if names == 'all':
                    names = ['all']
                wanted = set(FLIGHT_LINKS) if 'all' in names else set(names)
                blocked = sorted(wanted & FLIGHT_LINKS) if flying else []
                if blocked:
                    return self.reply({'error': 'Aircraft is armed or a mission is running: will not restart ' + ', '.join(blocked)}, 409)
                return self.reply(dict(ok=True, restarting=restart(names, states)))
            if self.path.startswith('/api/vision/'):
                return self.vision_post(data, settings, flying)
            if self.path == '/api/pin':
                try:
                    pin_box[0] = set_panel_pin(data.get('new_pin'))
                except (ValueError, OSError) as exc:
                    return self.reply({'error': str(exc)}, 400)
                return self.reply(dict(ok=True))
            if self.path == '/api/gimbal':
                if flying:
                    return self.reply({'error': 'Aircraft is armed or a mission is running: gimbal checks on the ground only.'}, 409)
                if settings.get('camera_mode') != 'track':
                    return self.reply({'error': 'Set the camera to Track mode first (Preview never moves the gimbal).'}, 409)
                try:
                    return self.reply(dict(ok=True, **gimbal_command(str(data.get('pose', '')))))
                except (ValueError, OSError) as exc:
                    return self.reply({'error': str(exc)}, 400)
            if self.path == '/api/settings':
                changes = data.get('changes') or {}
                locked = set(locked_keys())
                bad = [k for k in changes if k not in links.DEFAULTS]
                if bad:
                    return self.reply({'error': 'Unknown setting(s): ' + ', '.join(bad)}, 400)
                fixed = [k for k in changes if k in locked]
                if fixed:
                    return self.reply({'error': 'Fixed by the environment (onboard.env / /etc/default): ' + ', '.join(fixed)}, 400)
                apply = data.get('apply') or []
                if flying and ('all' in apply or set(apply) & FLIGHT_LINKS):
                    return self.reply({'error': 'Aircraft is armed or a mission is running: save is allowed only for the camera.'}, 409)
                try:
                    stored = json.loads(links.CONFIG_FILE.read_text())
                except (OSError, ValueError):
                    stored = {}
                try:
                    if 'payload_servos' in changes or 'aux_set' in changes:
                        links.payload_servos(changes.get('payload_servos', ''), changes.get('aux_set', 0) or 0)
                    for key in ('payload_hold', 'aux_value'):
                        if key in changes and not -1 <= float(changes[key]) <= 1:
                            raise ValueError('{} must be -1..1'.format(key))
                except (ValueError, TypeError) as exc:
                    return self.reply({'error': str(exc)}, 400)
                for key, value in changes.items():
                    if isinstance(links.DEFAULTS[key], bool):
                        value = bool(value)
                    elif key in SECRET and value == '':
                        continue   # empty passphrase field = keep the stored one
                    else:
                        value = str(value).strip()
                    stored[key] = value
                merged = dict(links.DEFAULTS, **stored)
                try:
                    links.drone_id(merged.get)
                    if not 1 <= int(merged['system_id']) <= 254:
                        raise ValueError('System ID must be 1..254')
                    for key in ('gcs_baud', 'fc_baud', 'mesh_baud', 'gcs_port', 'fc_udp_port', 'mesh_port', 'gcs_bind_port'):
                        int(merged[key])
                    if merged.get('gimbal_speed') not in links.GIMBAL_SPEEDS:
                        raise ValueError('Gimbal speed must be one of: ' + ', '.join(links.GIMBAL_SPEEDS))
                except (ValueError, TypeError) as exc:
                    return self.reply({'error': str(exc)}, 400)
                links.CONFIG_DIR.mkdir(parents=True, exist_ok=True)
                tmp = links.CONFIG_FILE.with_suffix('.tmp')
                tmp.write_text(json.dumps(stored, indent=2))
                os.replace(tmp, links.CONFIG_FILE)
                restarted = restart(apply, link_states()) if apply else []
                return self.reply(dict(ok=True, saved=sorted(changes), restarting=restarted))
            return self.reply({'error': 'not found'}, 404)

        def vision_post(self, data, settings, flying):
            if model_library is None:
                return self.reply({'error': 'Model library not installed'}, 501)
            action = self.path.rsplit('/', 1)[-1]
            if flying and action in ('activate', 'benchmark', 'delete'):
                return self.reply({'error': 'Aircraft is armed or a mission is running: change models after landing.'}, 409)
            state = vision_state(settings)
            try:
                if action == 'board':
                    result = model_library.set_board(str(data.get('board', '')), data.get('policy'))
                    return self.reply(dict(ok=True, active=result))
                if action == 'activate':
                    result = model_library.set_active(
                        str(data.get('model', '')), [str(c) for c in data.get('classes') or []],
                        [str(c) for c in data.get('lock_classes') or []], state['board'], state['threads'],
                        state['runtimes'], state['policy'])
                    restarted = restart(['camera'], link_states()) if settings.get('camera_mode') in ('preview', 'track') else []
                    return self.reply(dict(ok=True, active=result, restarting=restarted))
                if action == 'benchmark':
                    cores = [int(c) for c in str(settings.get('detector_cores') or '').split()] or None
                    result = model_library.run_benchmark(str(data.get('model', '')), state['board'],
                                                         state['threads'], cores)
                    return self.reply(dict(ok=True, result=result))
                if action == 'delete':
                    model_library.delete_model(str(data.get('model', '')))
                    return self.reply(dict(ok=True))
            except (ValueError, RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
                return self.reply({'error': str(exc)[:400]}, 400)
            return self.reply({'error': 'not found'}, 404)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=int(links.load_settings().get('web_port') or 8090))
    parser.add_argument('--host', default='0.0.0.0')
    args = parser.parse_args()
    panel_pin()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(args.port))
    print('Onboard panel: ' + '  '.join('http://{}:{}'.format(a, args.port) for a in addresses()), flush=True)
    signal.signal(signal.SIGTERM, lambda *_: (server.shutdown(), sys.exit(0)))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AERIXIS onboard</title>
<style>
:root{--bg:#070d1c;--glass:rgba(20,30,58,.62);--g3:rgba(32,46,82,.85);--edge:rgba(150,180,255,.16);--line:rgba(150,180,255,.12);
--text:#e6ecf7;--muted:#a3b0c6;--faint:#717f98;--ok:#3fd18b;--warn:#f3b04b;--bad:#ff6b78;--blue:#4f8df7}
*{box-sizing:border-box}body{margin:0;font:13px/1.45 Inter,system-ui,sans-serif;color:var(--text);min-height:100vh;background:
radial-gradient(900px 600px at 0 0,rgba(70,110,255,.26),transparent 60%),radial-gradient(800px 600px at 100% 0,rgba(150,90,255,.18),transparent 60%),
radial-gradient(900px 700px at 100% 100%,rgba(20,190,170,.14),transparent 60%),#070d1c}
/* sidebar */
nav{position:fixed;left:0;top:0;bottom:0;width:76px;display:flex;flex-direction:column;align-items:center;gap:6px;padding:12px 0;
background:rgba(8,13,28,.82);border-right:1px solid var(--line);backdrop-filter:blur(14px);z-index:6}
.logo{width:40px;height:40px;border-radius:12px;display:grid;place-items:center;font-weight:800;color:#0b1530;margin-bottom:10px;
background:linear-gradient(140deg,#9fc0ff,#6b8cff 55%,#7a5cf0)}
nav button{width:62px;height:56px;border:0;border-radius:12px;background:transparent;color:var(--faint);cursor:pointer;display:flex;flex-direction:column;
align-items:center;justify-content:center;gap:3px;font:600 10px Inter,system-ui,sans-serif;position:relative}
nav button svg{width:22px;height:22px;stroke:currentColor;fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round}
nav button:hover{color:var(--text);background:rgba(255,255,255,.05)}
nav button.on{color:#fff;background:linear-gradient(160deg,color-mix(in srgb,var(--c) 45%,transparent),color-mix(in srgb,var(--c) 12%,transparent));box-shadow:inset 0 0 0 1px color-mix(in srgb,var(--c) 50%,transparent)}
nav button .badge{position:absolute;top:6px;right:12px;width:8px;height:8px;border-radius:50%;background:var(--bad);display:none}
/* top bar */
header{position:sticky;top:0;z-index:5;margin-left:76px;display:flex;flex-wrap:wrap;align-items:center;gap:10px;padding:12px 20px;
border-bottom:1px solid var(--line);background:rgba(10,16,32,.72);backdrop-filter:blur(14px)}
header b{font-size:15px}header small{display:block;color:var(--faint);font:500 10.5px monospace;letter-spacing:.12em;text-transform:uppercase}
.sp{flex:1}.pill{display:inline-flex;gap:7px;align-items:center;height:30px;padding:0 12px;border-radius:999px;background:var(--glass);border:1px solid var(--edge);font-weight:600;font-size:12px}
.pill input{height:22px;width:110px;border:0;background:transparent;color:var(--text);font:inherit;outline:none}
.dot{width:9px;height:9px;border-radius:50%;background:var(--faint);flex:none}.dot.ok{background:var(--ok);box-shadow:0 0 8px var(--ok)}.dot.warn{background:var(--warn)}.dot.bad{background:var(--bad)}
main{margin-left:76px;padding:18px 20px 60px}
.page{display:none;gap:14px;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));align-items:start}.page.on{display:grid}
.ptitle{grid-column:1/-1;display:flex;align-items:baseline;gap:10px;margin:2px 0 -2px}.ptitle h2{margin:0;font-size:19px}.ptitle span{color:var(--faint);font-size:12px}
/* cards */
.card{--c:var(--blue);position:relative;border-radius:16px;border:1px solid color-mix(in srgb,var(--c) 28%,var(--edge));
background:linear-gradient(165deg,color-mix(in srgb,var(--c) 15%,transparent),transparent 55%),var(--glass);overflow:hidden;backdrop-filter:blur(16px)}
.card:before{content:"";position:absolute;left:0;right:0;top:0;height:3px;background:linear-gradient(90deg,var(--c),transparent)}
.card h3{display:flex;align-items:center;gap:8px;margin:0;padding:13px 15px 4px;font-size:12px;letter-spacing:.07em;text-transform:uppercase;color:color-mix(in srgb,var(--c) 45%,#fff)}
.card h3 .dot{margin-left:auto}.body{padding:6px 15px 15px}.wide{grid-column:1/-1}
.prow{display:flex;gap:10px;align-items:flex-end;flex-wrap:wrap;margin:6px 0;padding:8px 10px;border-radius:10px;border:1px solid color-mix(in srgb,var(--c,#f0a12c) 30%,transparent)}.prow label{display:flex;flex-direction:column;gap:4px;font-size:12px}.prow select,.prow input{min-width:90px}
.c-violet{--c:#9b6cf6}.c-amber{--c:#f0a12c}.c-pink{--c:#ec5fa0}.c-teal{--c:#16a596}.c-green{--c:#22b07d}.c-cyan{--c:#22b5c9}.c-coral{--c:#f27a5e}.c-lime{--c:#8bd450}.c-blue{--c:#4f8df7}
/* widgets */
label{display:flex;flex-direction:column;gap:4px;font-size:11.5px;color:var(--muted);font-weight:600}
input,select{height:34px;border-radius:9px;border:1px solid var(--line);background:var(--g3);color:var(--text);padding:0 10px;font:inherit}
input:focus,select:focus{outline:2px solid color-mix(in srgb,var(--c) 60%,transparent);outline-offset:0}
input:disabled,select:disabled{opacity:.5}
.fields{display:grid;gap:10px;grid-template-columns:repeat(auto-fill,minmax(140px,1fr));margin-top:6px}
.seg{display:flex;flex-wrap:wrap;gap:4px;padding:3px;border-radius:11px;background:rgba(0,0,0,.25);border:1px solid var(--line)}
.seg button{flex:1;min-width:max-content;height:30px;padding:0 12px;border:0;border-radius:8px;background:transparent;color:var(--muted);font:600 12px Inter,system-ui,sans-serif;cursor:pointer}
.seg button.on{background:linear-gradient(135deg,color-mix(in srgb,var(--c) 80%,#fff),var(--c));color:#0b1530}
.seg.dis{opacity:.5;pointer-events:none}
.sw{display:flex;align-items:center;justify-content:space-between;gap:12px;padding:9px 0;border-bottom:1px solid var(--line);font-size:12.5px;color:var(--text);font-weight:600;flex-direction:row}
.sw small{display:block;color:var(--faint);font-weight:500;font-size:11px}
.toggle{position:relative;width:46px;height:26px;flex:none}.toggle input{opacity:0;width:0;height:0;position:absolute}
.toggle span{position:absolute;inset:0;border-radius:999px;background:rgba(255,255,255,.12);border:1px solid var(--edge);transition:.2s;cursor:pointer}
.toggle span:before{content:"";position:absolute;left:3px;top:3px;width:18px;height:18px;border-radius:50%;background:#cfd8ea;transition:.2s}
.toggle input:checked+span{background:var(--c)}.toggle input:checked+span:before{transform:translateX(20px);background:#fff}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:12px}
button.b{height:34px;padding:0 15px;border-radius:10px;border:1px solid var(--edge);background:var(--g3);color:var(--text);font:650 12.5px Inter,system-ui,sans-serif;cursor:pointer}
button.b.primary{background:linear-gradient(135deg,color-mix(in srgb,var(--c) 75%,#fff),var(--c));border-color:transparent;color:#0b1530}
button.sm{height:26px;padding:0 10px;font-size:11.5px;border-radius:8px}button:disabled{opacity:.45;cursor:not-allowed}
.tiles{display:grid;gap:10px;grid-template-columns:repeat(auto-fill,minmax(130px,1fr))}
.tile{border-radius:12px;padding:10px 12px;background:rgba(0,0,0,.22);border:1px solid var(--line)}
.tile small{display:block;color:var(--faint);font-size:10.5px;font-weight:600;text-transform:uppercase;letter-spacing:.06em}
.tile b{display:block;font-size:17px;margin-top:3px}.tile em{font-style:normal;font-size:11px;color:var(--muted)}
.bar{height:6px;border-radius:3px;background:rgba(255,255,255,.08);margin-top:7px;overflow:hidden}.bar i{display:block;height:100%;border-radius:3px;background:var(--ok)}
.link{display:grid;grid-template-columns:12px 1fr auto;gap:10px;align-items:center;padding:9px 0;border-bottom:1px solid var(--line)}
.link b{font-size:13px}.link small{display:block;color:var(--faint);font:11px monospace;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:46vw}
.kv{display:grid;grid-template-columns:auto 1fr;gap:5px 12px;margin:0}.kv dt{color:var(--muted)}.kv dd{margin:0;text-align:right;font:12px monospace}
.check{display:grid;grid-template-columns:22px 1fr auto;gap:8px;align-items:center;padding:8px 0;border-bottom:1px solid var(--line)}
.check i{font-style:normal;font-size:15px}.check small{color:var(--faint);font-size:11px}
.note{font-size:11.5px;color:var(--faint)}.bad{color:var(--bad)}.warnt{color:var(--warn)}.okt{color:var(--ok)}
details{margin-top:10px}summary{cursor:pointer;color:var(--muted);font-size:12px;font-weight:600}
pre{margin:0;max-height:420px;overflow:auto;font:11px/1.5 monospace;color:var(--muted);background:rgba(0,0,0,.25);border-radius:10px;padding:10px;white-space:pre-wrap}
iframe{width:100%;height:360px;border:0;border-radius:12px;background:#000}
.toast{position:fixed;right:16px;bottom:16px;max-width:380px;padding:11px 14px;border-radius:12px;background:var(--g3);border:1px solid var(--edge);border-left:4px solid var(--blue);display:none;z-index:9}
.step{font-weight:700;font-size:12px;color:color-mix(in srgb,var(--c) 55%,#fff);margin-bottom:6px}
.camgrid{display:grid;gap:16px;grid-template-columns:minmax(240px,330px) 1fr;align-items:start}@media(max-width:820px){.camgrid{grid-template-columns:1fr}}
.aigrid{display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(260px,1fr))}
.tbl{overflow:auto;margin-top:6px;border:1px solid var(--line);border-radius:10px}table{width:100%;border-collapse:collapse;font-size:12px}
th,td{padding:7px 8px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap}th{color:var(--muted);font-weight:600;background:rgba(0,0,0,.18)}
tr.lock td{opacity:.55}tr.act td{background:color-mix(in srgb,var(--c) 14%,transparent)}td .why{white-space:normal;max-width:300px;display:block;font-size:11px;color:var(--warn)}
.badge{display:inline-block;padding:1px 7px;border-radius:999px;font-size:10.5px;font-weight:700;border:1px solid var(--edge)}.badge.ok{color:var(--ok)}.badge.act{background:var(--c);color:#0b1530}
.meter{display:flex;height:14px;border-radius:7px;overflow:hidden;background:rgba(0,0,0,.3)}.meter span{display:block;height:100%}
.inl{flex-direction:row;align-items:center;gap:6px}.inl select{height:28px}.usebox{margin-top:10px;padding:10px;border-radius:10px;border:1px solid var(--line);background:rgba(0,0,0,.18)}
.chips{display:flex;flex-wrap:wrap;gap:5px;max-height:150px;overflow:auto;margin:6px 0}.chip{padding:3px 10px;border-radius:999px;border:1px solid var(--edge);cursor:pointer;font-size:11.5px;user-select:none}
.chip.on{background:var(--c);color:#0b1530;border-color:transparent}.help{font-size:11.5px;line-height:1.55;color:var(--muted)}.help b{color:var(--text)}
@media(max-width:720px){nav{top:auto;bottom:0;left:0;right:0;width:auto;height:64px;flex-direction:row;justify-content:space-around;padding:4px;border-right:0;border-top:1px solid var(--line)}
nav .logo{display:none}nav button{width:auto;flex:1;height:54px}header,main{margin-left:0}main{padding:14px 16px 90px}.page{grid-template-columns:1fr}}
</style></head><body>
<nav id="nav">
<div class="logo">A</div>
<button data-go="home" style="--c:#4f8df7" class="on"><svg viewBox="0 0 24 24"><path d="M3 11l9-7 9 7"/><path d="M5 10v10h14V10"/></svg>Home<i class="badge" id="b-home"></i></button>
<button data-go="connect" style="--c:#9b6cf6"><svg viewBox="0 0 24 24"><path d="M9 7V3M15 7V3M7 7h10v4a5 5 0 01-10 0z"/><path d="M12 16v5"/></svg>Connect<i class="badge" id="b-connect"></i></button>
<button data-go="calib" style="--c:#16a596"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="2.5"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/></svg>Calibrate<i class="badge" id="b-calib"></i></button>
<button data-go="camera" style="--c:#ec5fa0"><svg viewBox="0 0 24 24"><rect x="3" y="7" width="13" height="11" rx="2"/><path d="M16 11l5-3v9l-5-3"/></svg>Camera&nbsp;AI</button>
<button data-go="drone" style="--c:#f0a12c"><svg viewBox="0 0 24 24"><circle cx="5" cy="5" r="2.5"/><circle cx="19" cy="5" r="2.5"/><circle cx="5" cy="19" r="2.5"/><circle cx="19" cy="19" r="2.5"/><path d="M7 7l3 3M17 7l-3 3M7 17l3-3M17 17l-3-3"/><rect x="9.5" y="9.5" width="5" height="5" rx="1"/></svg>Drone</button>
<button data-go="log" style="--c:#22b5c9"><svg viewBox="0 0 24 24"><path d="M5 4h14v16H5z"/><path d="M8 8h8M8 12h8M8 16h5"/></svg>Log</button>
</nav>
<header><div><b id="title">AERIXIS onboard</b><small id="subtitle">Onboard computer</small></div><div class="sp"></div>
<span class="pill" id="veh"><span class="dot"></span>…</span>
<span class="pill"><span class="dot" id="pindot"></span><input id="pin" type="password" placeholder="Panel PIN" autocomplete="current-password"></span></header>
<main>

<!-- HOME -->
<section class="page on" data-page="home">
<div class="ptitle"><h2>Status</h2><span id="hometime"></span></div>
<div class="card wide c-blue"><h3>Vehicle</h3><div class="body"><div class="tiles" id="tiles"></div></div></div>
<div class="card wide c-blue"><h3>Onboard links</h3><div class="body"><div id="links"></div>
<div class="row"><button class="b sm" onclick="restartLinks('all')">Restart all</button><span class="note">Flight links never restart while armed or flying.</span></div></div></div>
</section>

<!-- CONNECT -->
<section class="page" data-page="connect">
<div class="ptitle"><h2>Connections</h2><span>radio · flight controller · mesh</span></div>
<div class="card c-violet"><h3>GCS telemetry radio<span class="dot" id="d-gcs"></span></h3><div class="body">
<div class="seg" data-k="gcs_mode" data-opts="serial:Serial radio,udp:Network (UDP)"></div>
<div class="fields"><label>Radio port<select data-k="gcs_device" class="ports"></select></label><label>Baud<select data-k="gcs_baud" class="bauds"></select></label>
<label data-show="gcs_mode=udp">Laptop IP<input data-k="gcs_host"></label><label data-show="gcs_mode=udp">Laptop port<input data-k="gcs_port"></label></div>
<div class="row"><button class="b primary" onclick="save(['gcs_mode','gcs_device','gcs_baud','gcs_host','gcs_port'],['gcs'])">Save &amp; reconnect</button></div></div></div>
<div class="card c-amber"><h3>Flight controller<span class="dot" id="d-fc"></span></h3><div class="body">
<div class="seg" data-k="fc_mode" data-opts="serial:UART / USB,udp:Ethernet / UDP"></div>
<div class="fields"><label data-show="fc_mode=serial">Port<select data-k="fc_device" class="ports"></select></label><label data-show="fc_mode=serial">Baud<select data-k="fc_baud" class="bauds"></select></label>
<label data-show="fc_mode=udp">UDP port<input data-k="fc_udp_port"></label>
<label>Health tap (FC USB)<select data-k="fc_health_url" class="ports" data-empty="off"></select></label></div>
<div class="row"><button class="b primary" onclick="save(['fc_mode','fc_device','fc_baud','fc_udp_port','fc_health_url'],['fc','guardian','as2'])">Save &amp; reconnect</button></div></div></div>
<div class="card c-cyan"><h3>Drone-to-drone mesh<span class="dot" id="d-mesh"></span></h3><div class="body">
<div class="seg" data-k="mesh_mode" data-opts="both:Base + LoRa,udp:IP only,serial:LoRa only,off:Off"></div>
<div class="fields"><label data-show="mesh_mode=both,serial">Modem port<select data-k="mesh_device" class="ports"></select></label><label data-show="mesh_mode=both,serial">Baud<select data-k="mesh_baud" class="bauds"></select></label>
<label>Fleet names<input data-k="mesh_names" placeholder="drone0 drone1"></label><label>Passphrase<input data-k="mesh_passphrase" type="password" placeholder="unchanged"></label>
<label data-show="mesh_mode=both,udp">Peer IPs<input data-k="mesh_peers_ip"></label></div>
<div class="row"><button class="b primary" onclick="save(['mesh_mode','mesh_device','mesh_baud','mesh_names','mesh_passphrase','mesh_peers_ip'],'all')">Save &amp; restart links</button></div>
<dl class="kv" id="meshinfo" style="margin-top:12px"></dl></div></div>
<div class="card c-green"><h3>Plugged-in ports</h3><div class="body"><div id="portlist"></div></div></div>
</section>

<!-- CALIBRATE -->
<section class="page" data-page="calib">
<div class="ptitle"><h2>Calibration &amp; checks</h2><span>before every flight</span></div>
<div class="card c-teal"><h3>Pre-flight checklist</h3><div class="body"><div id="checks"></div></div></div>
<div class="card c-amber"><h3>Flight controller sensors</h3><div class="body"><dl class="kv" id="fcinfo"></dl>
<details><summary>Sensor calibration</summary><div class="help" style="margin-top:6px">Compass, accelerometer, gyro and level calibration run on the
flight controller itself: connect QGroundControl to the FC USB and use Vehicle Setup → Sensors. Unhealthy sensors and PX4 warnings appear here.</div></details>
<pre id="fcmsg" style="margin-top:10px;max-height:150px"></pre></div></div>
<div class="card c-pink"><h3>Gimbal check</h3><div class="body">
<div class="note">Camera must be in <b>Track</b> mode. Ground only.</div>
<div class="row"><button class="b" onclick="gimbal('centre')">⊙ Centre</button><button class="b" onclick="gimbal('down45')">↘ 45° down</button><button class="b" onclick="gimbal('down')">↓ Straight down</button></div>
<div style="margin-top:12px"><iframe id="cam2" title="Camera"></iframe></div></div></div>
</section>

<!-- CAMERA & AI -->
<section class="page" data-page="camera">
<div class="ptitle"><h2>Camera &amp; AI</h2><span id="camsummary"></span></div>
<div class="card wide c-pink"><h3>A8 camera<span class="dot" id="d-cam"></span></h3><div class="body camgrid"><div>
<div class="seg" data-k="camera_mode" data-opts="off:Off,preview:Preview,track:Track,sim:Simulated"></div>
<div class="fields"><label>Camera IP<input data-k="camera_ip"></label><label>Tracker cores<input data-k="vision_cores"></label></div>
<div class="note" style="margin:8px 0 4px">Gimbal speed (Track mode)</div>
<div class="seg" data-k="gimbal_speed" data-opts="normal:Smooth,fast:Fast,max:Max"></div>
<div class="row"><button class="b primary" onclick="save(['camera_mode','camera_ip','vision_cores','gimbal_speed'],['camera'])">Save &amp; restart camera</button><a id="camlink" target="_blank" class="note" style="color:#9fc0ff">Full screen ↗</a></div>
<div class="note" style="margin-top:10px">Preview: detects, gimbal still. Track: gimbal follows the lock.
Gimbal speed: Smooth = original; Fast / Max turn quicker and aim ahead of a moving person (check on the ground, props off).</div></div>
<div><iframe id="cam" title="A8 tracker"></iframe><div class="note" id="camnote"></div></div></div></div>
<section class="card wide c-lime" id="ai"><h3>AI models</h3><div class="body">
<div class="aigrid">
<div><div class="step">1 · Onboard computer</div>
<div class="fields"><label>Board<select id="aiboard"></select></label>
<label title="Fastest the drone flies while the gimbal tracks a target">Drone speed while tracking (m/s)<input id="aispeed" type="number" min="0" max="25" step="0.5"></label>
<label title="Lowest AI scan rate allowed, even when hovering">Min detection (Hz)<input id="aimindet" type="number" min="1" max="120" step="1"></label>
<label title="How many video frames per second the gimbal must follow the locked target">Min tracking (fps)<input id="aimintrk" type="number" min="1" max="120" step="1"></label></div>
<div class="row" style="margin-top:6px"><span class="pill" id="aineed" style="height:24px"></span></div>
<details><summary>What do these numbers mean?</summary><div class="help" style="margin-top:6px"><b>Detection (Hz)</b> — how many times per second the AI model (YOLO) scans the whole frame
to find targets and re-check that the locked one is still the same. Needed rate = drone speed ÷ 0.5 m
(the drone may move at most about one person-width between two checks), never below <i>Min detection</i>:
2 m/s → 5 Hz, 5 m/s → 10 Hz, 8 m/s → 16 Hz.<br>
<b>Tracking (fps)</b> — how many video frames per second the gimbal follows the locked target (NanoTrack
on every frame between AI scans). Your standard: 19–25 fps (the A8 video is 25 fps).<br>
<b>Lock</b> — stays on the target until it leaves the picture or nothing has seen it for 5 s; if the AI
misses a small target, a confident NanoTrack keeps following for up to 4 s. A model too slow for these
numbers on this board is locked 🔒.</div></details>
<div class="row"><button class="b primary" onclick="saveBoard()">Save board &amp; speeds</button><span class="note" id="aiboardnote"></span></div>
<div id="aiwarn" class="warnt" style="margin-top:6px"></div></div>
<div><div class="step">Detected on this computer</div><dl class="kv" id="aireal"></dl></div>
<div><div class="step">CPU budget (measured now)</div><div class="meter" id="aimeter"></div><dl class="kv" id="aibudget" style="margin-top:8px"></dl></div>
</div>
<div class="step" style="margin-top:14px">2 · Models</div>
<div class="row" style="margin-top:4px"><label class="inl">Type<select id="aifmt" onchange="renderAI()"><option value="">all</option></select></label>
<label class="inl">Resolution<select id="aires" onchange="renderAI()"><option value="">all</option><option>320</option><option>512</option><option>640</option><option>960</option></select></label>
<label class="inl"><span><input type="checkbox" id="aiunl" onchange="renderAI()"> unlocked only</span></label></div>
<div class="tbl"><table><thead><tr><th>Model</th><th>Type</th><th>Input</th><th>Classes</th><th>Detection</th><th>Tracking</th><th>Status</th><th></th></tr></thead><tbody id="aimodels"></tbody></table></div>
<div id="aiuse" class="usebox" style="display:none"></div>
<div class="aigrid" style="margin-top:14px">
<div><div class="step">3 · Upload a model</div><div class="fields">
<label>File (.zip of export folder, .onnx, .rknn, .engine, .hef)<input type="file" id="aifile" accept=".zip,.onnx,.rknn,.engine,.hef"></label>
<label>Name<input id="ainame" placeholder="e.g. person-s-640x384"></label>
<label>Class names (only if missing)<input id="aiclasses" placeholder="person, dog, cat"></label>
<label>Input h,w (only if missing)<input id="aiimgsz" placeholder="384,640"></label></div>
<div class="row"><button class="b primary" onclick="uploadModel()">Upload</button><span class="note" id="aiup"></span></div></div>
<div><details><summary>How to export a model for each board</summary><pre class="howto" style="margin-top:6px">Pi 5 CPU (NCNN, fastest):  yolo export model=best.pt format=ncnn imgsz=384,640 half=True
                            → zip the best_ncnn_model folder and upload
Any CPU (ONNX):             yolo export model=best.pt format=onnx imgsz=384,640 opset=12
Pi 5 + AI HAT (HEF):        compile with the Hailo Dataflow Compiler on an x86 PC
Vicharak Axon (RKNN):       rknn-toolkit2 on a PC, target rk3588 (INT8)
Jetson (TensorRT):          build on the Jetson: trtexec --onnx=best.onnx --saveEngine=model.engine --fp16
Input 16:9 (e.g. 384,640) matches the 640x360 video: no padding, more pixels per person.
960x544 doubles the pixels on a person (for 20 m) but needs about 2x the compute.</pre></details></div></div>
<div class="note" id="aisrc" style="margin-top:8px"></div></div></section>
</section>

<!-- DRONE -->
<section class="page" data-page="drone">
<div class="ptitle"><h2>Drone</h2><span>identity · pairing · safety</span></div>
<div class="card c-teal"><h3>Identity &amp; GCS pairing</h3><div class="body"><div class="fields">
<label>Drone ID<input data-k="drone_id"></label><label>System ID<input data-k="system_id"></label></div>
<div class="row"><button class="b primary" onclick="save(['drone_id','system_id'],'all')">Save &amp; restart links</button><button class="b" onclick="showToken()">Show pairing token</button></div>
<pre id="token" style="display:none;margin-top:10px"></pre></div></div>
<div class="card c-coral"><h3>Safety switches</h3><div class="body">
<label class="sw">Allow flight<small>Off = observe only</small><span class="toggle"><input type="checkbox" data-k="allow_flight" data-bool><span></span></span></label>
<label class="sw">Paths verified clear<small>Required for automatic routes</small><span class="toggle"><input type="checkbox" data-k="path_clear" data-bool><span></span></span></label>
<div class="fields"><label>Emergency action<select data-k="emergency_action"><option>RTL</option><option>LAND</option></select></label>
<label>Min separation (m)<input data-k="separation_m"></label></div>
<div class="row"><button class="b primary" onclick="save(['allow_flight','path_clear','emergency_action','separation_m'],['as2','manager','guardian'])">Save &amp; apply</button></div></div></div>
<div class="card c-amber"><h3>Payload drop</h3><div class="body">
<div class="note">Each servo: the PX4 <b>Offboard Actuator Set</b> it is mapped to (1&ndash;6) and how many packages it drops, one per step. A servo with N drops moves evenly from the hold value to the release end, one step per drop; a servo with 1 drop goes straight to the release end. Drops go servo by servo. The GCS reads this set-up when the drone connects and plans deliveries with it.</div>
<input type="hidden" data-k="payload_servos" id="pservos"><input type="hidden" data-k="aux_set" id="paux">
<div id="pservolist" class="pservos"></div>
<div class="row"><button class="b" onclick="addServo()">+ Add servo</button><span id="pcap" class="note" style="margin:0"></span></div>
<div class="fields"><label>Hold value (-1..1)<input data-k="payload_hold"></label><label>Release end value (-1..1)<input data-k="aux_value"></label>
<label>Drop height min (m)<input data-k="drop_min_agl"></label><label>Drop height max (m)<input data-k="drop_max_agl"></label></div>
<label class="sw">Operator confirms the loaded packages<small>At upload in the GCS (no load sensor fitted)</small><span class="toggle"><input type="checkbox" data-k="operator_payload" data-bool><span></span></span></label>
<label class="sw">Overhead drop envelope verified<small>Drop height window tested with an inert package, props off</small><span class="toggle"><input type="checkbox" data-k="overhead_verified" data-bool><span></span></span></label>
<div class="row"><button class="b primary" onclick="save(['payload_servos','aux_set','payload_hold','aux_value','drop_min_agl','drop_max_agl','operator_payload','overhead_verified'],['as2','manager','guardian'])">Save &amp; apply</button></div></div></div>
<div class="card c-violet"><h3>Panel PIN</h3><div class="body"><div class="fields">
<label>New PIN<input id="newpin" type="password" autocomplete="new-password"></label><label>Repeat<input id="newpin2" type="password" autocomplete="new-password"></label></div>
<div class="row"><button class="b primary" onclick="changePin()">Change PIN</button></div></div></div>
</section>

<!-- LOG -->
<section class="page" data-page="log">
<div class="ptitle"><h2>Onboard log</h2></div>
<div class="card wide c-cyan"><div class="body" style="padding-top:14px"><pre id="log"></pre></div></div>
</section>
</main><div class="toast" id="toast"></div>
<script>
const $=s=>document.querySelector(s),$$=s=>[...document.querySelectorAll(s)];let S=null,dirty=new Set();
const BAUDS=['9600','19200','38400','57600','115200','230400','460800','921600'];
const pinEl=$('#pin');pinEl.value=sessionStorage.getItem('pin')||'';pinEl.oninput=()=>{sessionStorage.setItem('pin',pinEl.value);$('#pindot').className='dot '+(pinEl.value?'ok':'')};
function toast(t,bad){const e=$('#toast');e.textContent=t;e.style.borderLeftColor=bad?'#ff6b78':'#3fd18b';e.style.display='block';clearTimeout(e._t);e._t=setTimeout(()=>e.style.display='none',6000)}
async function post(path,body){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Panel-Pin':pinEl.value},body:JSON.stringify(body)});const d=await r.json().catch(()=>({}));if(!r.ok)throw new Error(d.error||r.status);return d}
function needPin(){if(!pinEl.value){toast('Enter the panel PIN (top right) first.',1);pinEl.focus();return true}return false}
// pages
function go(p){if(!$(`nav button[data-go="${p}"]`))p='home';$$('nav button').forEach(b=>b.classList.toggle('on',b.dataset.go===p));$$('.page').forEach(s=>s.classList.toggle('on',s.dataset.page===p));
 const c=$(`nav button[data-go="${p}"]`).style.getPropertyValue('--c');document.documentElement.style.setProperty('--c',c);location.hash=p;camFrames()}
$$('nav button').forEach(b=>b.onclick=()=>go(b.dataset.go));
// segmented controls
$$('.seg').forEach(seg=>{seg.innerHTML=seg.dataset.opts.split(',').map(o=>{const[v,t]=o.split(':');return `<button type="button" data-v="${v}">${t}</button>`}).join('');
 seg.onclick=e=>{const b=e.target.closest('button');if(!b)return;seg.value=b.dataset.v;dirty.add(seg.dataset.k);paintSeg(seg);showRules()}});
function paintSeg(seg){[...seg.querySelectorAll('button')].forEach(b=>b.classList.toggle('on',b.dataset.v===seg.value))}
function showRules(){$$('[data-show]').forEach(el=>{const[k,vs]=el.dataset.show.split('=');const seg=$(`.seg[data-k="${k}"]`);el.style.display=vs.split(',').includes(seg&&seg.value)?'':'none'})}
$$('input[data-k],select[data-k]').forEach(el=>el.addEventListener('input',()=>dirty.add(el.dataset.k)));
$$('select.bauds').forEach(sel=>sel.innerHTML=BAUDS.map(b=>`<option>${b}</option>`).join(''));
function val(el){if(el.classList.contains('seg'))return el.value;if(el.dataset.bool!==undefined)return el.type==='checkbox'?el.checked:el.value==='true';return el.value}
// Payload servos (2026-11-01): rows of actuator set + drops, kept in the hidden payload_servos.
function servoRows(){const spec=($('#pservos').value||'').trim();if(spec)return spec.split(',').filter(Boolean).map(p=>{const [a,n]=p.split(':');return {set:+a,drops:+(n||1)}});
 const aux=+($('#paux').value||0);return aux>=1&&aux<=6?[{set:aux,drops:1}]:[]}
function writeServos(rows){$('#pservos').value=rows.map(r=>r.set+':'+r.drops).join(',');$('#paux').value='0';dirty.add('payload_servos');dirty.add('aux_set');renderServos()}
function addServo(){const rows=servoRows();const free=[1,2,3,4,5,6].find(n=>!rows.some(r=>r.set===n));if(!free)return toast('All six actuator sets are in use',1);rows.push({set:free,drops:1});writeServos(rows)}
function renderServos(){const rows=servoRows(),box=$('#pservolist');if(!box)return;
 box.innerHTML=rows.length?rows.map((r,i)=>`<div class="prow"><label>Servo ${i+1} · actuator set<select data-i="${i}" class="pset">${[1,2,3,4,5,6].map(n=>`<option ${n===r.set?'selected':''}>${n}</option>`).join('')}</select></label><label>Drops<input data-i="${i}" class="pdrops" type="number" min="1" max="20" value="${r.drops}"></label><button class="b" data-i="${i}" onclick="delServo(${i})">Remove</button></div>`).join(''):'<div class="note">No payload servo: this drone carries no packages.</div>';
 const total=rows.reduce((a,r)=>a+r.drops,0);$('#pcap').textContent=total?`Capacity: ${total} package${total>1?'s':''} on ${rows.length} servo${rows.length>1?'s':''}`:'';
 box.querySelectorAll('.pset').forEach(el=>el.onchange=()=>{const rs=servoRows();rs[+el.dataset.i].set=+el.value;writeServos(rs)});
 box.querySelectorAll('.pdrops').forEach(el=>el.onchange=()=>{const rs=servoRows();rs[+el.dataset.i].drops=Math.max(1,Math.min(20,Math.round(+el.value||1)));writeServos(rs)})}
function delServo(i){const rows=servoRows();rows.splice(i,1);writeServos(rows)}
function fill(){const s=S.settings,locked=new Set(S.locked);
 $$('select.ports').forEach(sel=>{const k=sel.dataset.k,cur=s[k]||'';const opts=(sel.dataset.empty?[['',sel.dataset.empty]]:[]).concat(S.ports.map(p=>[p.path,p.name+(p.device!==p.path?' → '+p.device.replace('/dev/',''):'')]));
  if(cur&&!opts.some(o=>o[0]===cur))opts.push([cur,cur+' (not plugged in)']);if(dirty.has(k))return;
  sel.innerHTML=opts.map(([v,t])=>`<option value="${v}">${t}</option>`).join('')});
 $$('select.bauds').forEach(sel=>{const cur=String(s[sel.dataset.k]||'');if(cur&&![...sel.options].some(o=>o.value===cur))sel.insertAdjacentHTML('beforeend',`<option>${cur}</option>`)});
 $$('[data-k]').forEach(el=>{const k=el.dataset.k;const lock=locked.has(k);el.title=lock?'Fixed by AS2_'+k.toUpperCase()+' in onboard.env':'';
  if(el.classList.contains('seg')){el.classList.toggle('dis',lock);if(!dirty.has(k)){el.value=String(s[k]??'');paintSeg(el)}return}
  el.disabled=lock;if(dirty.has(k))return;let v=s[k];
  if(el.type==='checkbox')el.checked=!!v&&v!=='false';else{if(el.dataset.bool!==undefined)v=String(!!v&&v!=='false');el.value=v??''}});showRules();renderServos()}
const KIND={running:'ok',starting:'warn',waiting:'warn',restarting:'warn',off:''};
const NAMES={fc:'Flight controller',guardian:'FC guardian',as2:'AERIXIS core',manager:'Mission manager',gcs:'GCS radio',mesh:'Mesh',camera:'Camera',web:'Panel'};
function dot(id,state){const e=$(id);if(e)e.className='dot '+(state===undefined?'':(KIND[state]??'bad'))}
function tile(label,value,sub,pct,color){return `<div class="tile"><small>${label}</small><b>${value}</b><em>${sub||''}</em>${pct!=null?`<div class="bar"><i style="width:${Math.max(0,Math.min(100,pct))}%;background:${color||'var(--ok)'}"></i></div>`:''}</div>`}
function render(){const v=S.vehicle||{},L=S.links||{};$('#title').textContent='AERIXIS · '+S.drone_id;$('#subtitle').textContent='SYS '+S.system_id+' · '+S.addresses.map(a=>a+':'+S.web_port).join(' ');
 $('#veh').innerHTML=`<span class="dot ${v.available?(v.armed?'warn':'ok'):'bad'}"></span>${v.available?(v.armed?'ARMED':'Disarmed')+(v.active?' · mission':''):'Starting…'}`;
 $('#hometime').textContent=new Date(S.time*1000).toLocaleTimeString();
 const g=v.gnss||{},st=v.fc_status||{},vis=v.vision||{},bat=v.battery;
 $('#tiles').innerHTML=tile('Battery',bat!=null?Math.round(bat*100)+' %':'—','',bat!=null?bat*100:null,bat!=null&&bat<.3?'var(--bad)':'var(--ok)')
  +tile('GNSS',g.fix||'—',g.sats!=null?g.sats+' sat · HDOP '+g.hdop:'')+tile('Altitude',v.position?v.position[2].toFixed(1)+' m':'—','')
  +tile('Flight mode',st.nav||'—',st.arming||'')+tile('Camera',vis.fresh?Math.round(vis.fps||0)+' fps':'offline',vis.locked?'locked '+vis.label:'')
  +tile('Mission',v.execution||'—','ROS nodes '+(v.ros_nodes??'—'));
 const bad=Object.values(L).some(l=>!['running','off'].includes(l.state));$('#b-home').style.display=bad?'block':'none';
 $('#links').innerHTML=Object.entries(L).map(([n,l])=>`<div class="link"><span class="dot ${KIND[l.state]??'bad'}"></span><div><b>${NAMES[n]||n}</b><small title="${(l.detail||'').replace(/"/g,'')}">${l.state}${l.restarts?' · restarts '+l.restarts:''}${l.detail?' · '+l.detail:''}</small></div><button class="b sm" onclick="restartLinks(['${n}'])">Restart</button></div>`).join('');
 dot('#d-gcs',(L.gcs||{}).state);dot('#d-fc',(L.fc||{}).state);dot('#d-mesh',(L.mesh||{}).state);dot('#d-cam',(L.camera||{}).state);
 $('#b-connect').style.display=['gcs','fc'].some(n=>L[n]&&!['running','off'].includes(L[n].state))?'block':'none';
 const sensorsBad=(v.sensors||[]).length+(v.faults||[]).length;
 $('#fcinfo').innerHTML=`<dt>FC data</dt><dd class="${v.connected?'okt':'bad'}">${v.connected?'flowing':'none'}</dd><dt>Mode</dt><dd>${st.nav||'—'} · ${st.arming||'—'}</dd><dt>Preflight</dt><dd class="${st.preflight_ok===false?'bad':st.preflight_ok?'okt':''}">${st.preflight_ok===undefined?'—':st.preflight_ok?'pass':'FAIL'}</dd><dt>GNSS</dt><dd>${g.fix?g.fix+' · '+g.sats+' sat · HDOP '+g.hdop:'—'}</dd><dt>Sensors</dt><dd class="${sensorsBad?'bad':'okt'}">${[...(v.sensors||[]).map(x=>x+' unhealthy'),...(v.faults||[])].join(', ')||(v.connected?'all healthy':'—')}</dd>`;
 $('#fcmsg').textContent=(v.messages||[]).map(m=>typeof m==='string'?m:(m.text||JSON.stringify(m))).slice(-12).join('\n')||'No PX4 messages.';
 const checks=[['Flight controller connected',v.connected],['PX4 pre-flight checks pass',st.preflight_ok],['Sensors healthy',v.connected?!sensorsBad:undefined],
  ['GNSS 3D fix, ≥ 8 satellites',g.fix?(/3d|rtk/i.test(g.fix)&&g.sats>=8):undefined],['Battery above 30 %',bat!=null?bat>=.3:undefined],
  ['GCS radio link running',(L.gcs||{}).state==='running'],['Camera streaming',vis.fresh],['Allow flight switched on',!!S.settings.allow_flight&&S.settings.allow_flight!=='false']];
 $('#checks').innerHTML=checks.map(([t,ok])=>`<div class="check"><i>${ok===undefined?'⏳':ok?'✅':'❌'}</i><span>${t}</span><small>${ok===undefined?'waiting':ok?'ok':'check'}</small></div>`).join('');
 $('#b-calib').style.display=checks.some(c=>c[1]===false)?'block':'none';
 const m=v.mesh||{},rd=v.radio||{},ai=v.avoidance||{};
 const lk=Object.entries(rd.links||{}).map(([n,l])=>`<dt>${n==='udp'?'Base station':'LoRa'}</dt><dd class="${l.error?'bad':''}">${l.up===false?'down':'up'} · ${Object.entries(l.peers||{}).map(([p,a])=>p+' '+a.toFixed(1)+'s').join(' · ')||'no peer'}${l.error?' · '+l.error:''}</dd>`).join('');
 const aiText=ai.mode==='unavailable'?'not installed':ai.enabled?`ON (${ai.mode==='brain'?'brain + ORCA':'ORCA'})${ai.active?' · '+(ai.deviating?'avoiding':'watching'):''}`:'off (switch in GCS)';
 $('#meshinfo').innerHTML=`<dt>Peers</dt><dd>${m.required?(m.peers||[]).map(p=>p.owner+' '+(p.age_s!=null?p.age_s.toFixed(1)+'s':'—')).join(' · ')||'none heard':'single aircraft'}</dd>${lk}<dt>Separation</dt><dd>${m.required?(m.separation_safe?'safe':'TOO CLOSE'):'—'} · min ${S.settings.separation_m||'—'} m</dd><dt>AI avoidance</dt><dd>${aiText}</dd>`;
 $('#portlist').innerHTML=S.ports.length?S.ports.map(p=>`<div class="link"><span class="dot ok"></span><div><b>${p.name}</b><small>${p.device}</small></div><span></span></div>`).join(''):'<div class="note">No serial device plugged in.</div>';
 const mode=S.settings.camera_mode;$('#camsummary').textContent=(mode||'off')+(vis.fresh?' · '+Math.round(vis.fps||0)+' fps':'');
 const log=$('#log'),bottom=log.scrollTop+log.clientHeight>=log.scrollHeight-20;log.textContent=S.log.join('\n');if(bottom)log.scrollTop=log.scrollHeight;
 $('#pindot').className='dot '+(pinEl.value?'ok':'');camFrames()}
function camFrames(){if(!S)return;const url='http://'+location.hostname+':8080/',mode=S.settings.camera_mode,live=mode==='preview'||mode==='track';$('#camlink').href=url;
 [['#cam','camera'],['#cam2','calib']].forEach(([id,page])=>{const f=$(id),show=live&&$(`.page[data-page="${page}"]`).classList.contains('on');
  if(show&&f.dataset.src!==url){f.src=url;f.dataset.src=url}if(!show&&f.dataset.src){f.src='about:blank';f.dataset.src=''}f.style.display=live?'':'none'});
 $('#camnote').textContent=live?'':mode==='sim'?'Simulated camera: no video.':'Camera off.'}
async function poll(){try{S=await (await fetch('/api/state')).json();fill();render()}catch(e){$('#veh').innerHTML='<span class="dot bad"></span>Panel offline'}setTimeout(poll,2000)}
async function save(keys,apply){if(needPin())return;const changes={};
 keys.forEach(k=>{const el=$(`[data-k="${k}"]`);if(!el||el.disabled||el.classList.contains('dis'))return;changes[k]=val(el)});
 try{const d=await post('/api/settings',{changes,apply:apply==='all'?['all']:apply});keys.forEach(k=>dirty.delete(k));toast('Saved. Restarting: '+(d.restarting.join(', ')||'none'))}catch(e){toast(e.message,1)}}
async function restartLinks(names){if(needPin())return;try{const d=await post('/api/restart',{links:names});toast('Restarting '+(d.restarting.join(', ')||names))}catch(e){toast(e.message,1)}}
async function showToken(){if(needPin())return;try{const d=await post('/api/token',{});const t=$('#token');t.style.display='';t.textContent=`drone ID  ${d.drone_id}\nsystem ID ${d.system_id}\ntoken     ${d.token}`}catch(e){toast(e.message,1)}}
async function gimbal(pose){if(needPin())return;try{const d=await post('/api/gimbal',{pose});toast('Gimbal → yaw '+d.yaw+'°, pitch '+d.pitch+'°')}catch(e){toast(e.message,1)}}
async function changePin(){if(needPin())return;const a=$('#newpin').value,b=$('#newpin2').value;if(a!==b)return toast('The two PINs differ.',1);
 try{await post('/api/pin',{new_pin:a});pinEl.value=a;sessionStorage.setItem('pin',a);$('#newpin').value=$('#newpin2').value='';toast('PIN changed.')}catch(e){toast(e.message,1)}}
let V=null,useModel=null;
const ESC=t=>String(t??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function pollAI(){try{V=await (await fetch('/api/vision')).json();renderAI()}catch(e){}setTimeout(pollAI,10000)}
function renderAI(){if(!V)return;if(!V.available){$('#aimodels').innerHTML=`<tr><td colspan="8" class="bad">${ESC(V.error)}</td></tr>`;return}
 const sel=$('#aiboard');if(!sel.dataset.filled){const det=V.detected;
  sel.innerHTML=`<option value="auto">Auto-detect → ${ESC(V.boards[det].label)}</option>`+Object.entries(V.boards).map(([k,b])=>`<option value="${k}">${ESC(b.label)}${k===det?'  ✓ detected':''}</option>`).join('');sel.dataset.filled=1;
  sel.value=V.board_auto?'auto':V.board;$('#aispeed').value=V.policy.speed_ms;$('#aimindet').value=V.policy.min_detect_hz;$('#aimintrk').value=V.policy.min_track_fps;
  const f=$('#aifmt');f.innerHTML='<option value="">all</option>'+['ncnn','onnx','hef','rknn','tensorrt'].map(x=>`<option value="${x}">${x.toUpperCase()}</option>`).join('')}
 $('#aiboardnote').textContent=V.board_auto?'Auto: found '+V.boards[V.detected].label+' on this computer.':(V.board===V.detected?'Matches the detected board.':'Chosen by hand (detected: '+V.boards[V.detected].label+').');
 $('#aineed').textContent=`Needed now: detection ≥ ${V.need_hz} Hz · tracking ≥ ${V.policy.min_track_fps} fps (at ${V.policy.speed_ms} m/s)`;
 $('#aiwarn').innerHTML=(V.warnings||[]).map(w=>'⚠ '+ESC(w)).join('<br>');
 const r=V.real;$('#aireal').innerHTML=`<dt>Computer</dt><dd>${ESC(r.model)}</dd><dt>CPU</dt><dd>${ESC(r.cpu)}${r.max_mhz?' @ '+(r.max_mhz/1000).toFixed(1)+' GHz':''}</dd><dt>Cores / threads</dt><dd>${r.cores} / ${r.threads}</dd><dt>RAM</dt><dd>${(r.ram_mb/1024).toFixed(1)} GB</dd><dt>Accelerator</dt><dd>${r.accelerators.length?r.accelerators.map(a=>ESC(a.label)+(a.tops?' · '+a.tops+' TOPS':'')).join(', '):'none (CPU only)'}</dd><dt>Runtimes</dt><dd>${Object.entries(V.runtimes).filter(([k,v])=>v).map(([k])=>k.toUpperCase()).join(', ')||'none'}</dd><dt>Temperature</dt><dd>${r.temperature_c!=null?r.temperature_c+' °C':'—'}</dd>`;
 const b=V.budget,pc=x=>Math.max(0,100*x/b.cores);
 $('#aimeter').innerHTML=`<span style="width:${pc(b.as2_cores)}%;background:#4f8df7" title="AS2 / flight"></span><span style="width:${pc(b.tracker_cores)}%;background:#ec5fa0" title="camera tracker"></span><span style="width:${pc(b.other_cores)}%;background:#a3b0c6" title="other"></span>`;
 $('#aibudget').innerHTML=`<dt><span class="dot" style="background:#4f8df7"></span> AS2 / flight software</dt><dd>${b.as2_cores} cores (${b.as2_percent} %)</dd><dt><span class="dot" style="background:#ec5fa0"></span> Camera tracker</dt><dd>${b.tracker_cores} cores</dd><dt><span class="dot" style="background:#a3b0c6"></span> Other</dt><dd>${b.other_cores} cores</dd><dt>Detector threads given</dt><dd>${V.threads} of ${b.cores} (camera pipeline + reserve kept)</dd>`;
 const fmt=$('#aifmt').value,res=$('#aires').value,unl=$('#aiunl').checked;
 const rows=V.models.filter(m=>(!fmt||m.format===fmt)&&(!res||(m.input&&String(m.input[0])===res))&&(!unl||!m.locked));
 $('#aimodels').innerHTML=rows.map(m=>{const sp=m.speed||{};const tag=sp.measured?'measured':'estimate';
  return `<tr class="${m.locked?'lock':''} ${m.active?'act':''}"><td><b>${ESC(m.name)}</b>${m.builtin?' <span class="badge">built-in</span>':''}${m.active?' <span class="badge act">ACTIVE</span>':''}<br><span class="note">${m.size_mb!=null?m.size_mb+' MB':''}</span></td>
  <td>${m.format?m.format.toUpperCase():'—'}</td><td>${m.input?m.input[0]+'×'+m.input[1]:'—'}</td><td title="${ESC(Object.values(m.names||{}).join(', '))}">${m.classes??'—'}${m.classes&&m.classes<=4?' · '+ESC(Object.values(m.names).join(', ')):''}</td>
  <td>${sp.detect_hz!=null?sp.detect_hz+' Hz':'—'}<br><span class="note">${sp.detect_ms!=null?sp.detect_ms+' ms · '+tag:''}</span></td><td>${sp.track_fps!=null?Math.round(sp.track_fps)+' fps':'—'}<br><span class="note">${sp.capture?sp.capture.join('×')+' video':''}</span></td>
  <td>${m.locked?'🔒 locked<span class="why">'+ESC((m.reasons||[]).join('; '))+'</span>':'<span class="badge ok">✓ unlocked</span>'}</td>
  <td><button class="b sm" onclick="openUse('${ESC(m.name)}')" ${m.locked||m.error?'disabled':''}>Use…</button> <button class="b sm" onclick="bench('${ESC(m.name)}')" ${m.error?'disabled':''}>Benchmark</button>${m.builtin?'':` <button class="b sm" onclick="delModel('${ESC(m.name)}')">Delete</button>`}</td></tr>`}).join('')||'<tr><td colspan="8" class="note">No model matches the filter.</td></tr>';
 $('#aisrc').textContent='Estimates (until you press Benchmark on this board): '+V.sources.join(' · ')+'. Models folder: '+V.library;}
function openUse(name){const m=V.models.find(x=>x.name===name);useModel=m;const act=V.active.model===name?V.active:{classes:[],lock_classes:[]};
 const names=Object.values(m.names),chip=(n,set,grp)=>`<span class="chip ${set.includes(n)?'on':''}" data-g="${grp}" data-n="${ESC(n)}" onclick="this.classList.toggle('on')">${ESC(n)}</span>`;
 const det=act.classes.length?act.classes:(names.includes('person')&&names.length>1?['person']:[]),lock=act.lock_classes;
 $('#aiuse').style.display='';$('#aiuse').innerHTML=`<b>Use ${ESC(name)}</b> <span class="note">— none selected = every class</span>
 <div class="step" style="margin-top:8px">Detect these classes</div><div class="chips">${names.map(n=>chip(n,det,'d')).join('')}</div>
 <div class="step">Gimbal may lock on</div><div class="chips">${names.map(n=>chip(n,lock,'l')).join('')}</div>
 <div class="note">Rescue missions (geotag, payload drop, peer claims) act only on <b>person</b> locks; other classes are tracked and shown only.</div>
 <div class="row"><button class="b primary" onclick="activate()">Make active &amp; restart camera</button><button class="b" onclick="$('#aiuse').style.display='none'">Cancel</button></div>`}
async function activate(){if(needPin())return;const pick=g=>$$(`#aiuse .chip.on[data-g="${g}"]`).map(e=>e.dataset.n);
 try{const d=await post('/api/vision/activate',{model:useModel.name,classes:pick('d'),lock_classes:pick('l')});$('#aiuse').style.display='none';toast('Active model: '+d.active.model+(d.restarting.length?' · camera restarting':''));V=null;pollAINow()}catch(e){toast(e.message,1)}}
async function saveBoard(){if(needPin())return;try{await post('/api/vision/board',{board:$('#aiboard').value,policy:{speed_ms:+$('#aispeed').value,min_detect_hz:+$('#aimindet').value,min_track_fps:+$('#aimintrk').value}});toast('Saved: lock state recomputed');$('#aiboard').dataset.filled='';pollAINow()}catch(e){toast(e.message,1)}}
async function bench(name){if(needPin())return;toast('Benchmarking '+name+' on this board (up to a minute)…');
 try{const d=await post('/api/vision/benchmark',{model:name});toast(name+': '+d.result.detect_hz+' Hz ('+d.result.detect_ms+' ms per detection, '+d.result.threads+' threads)');pollAINow()}catch(e){toast(e.message,1)}}
async function delModel(name){if(needPin())return;if(!confirm('Delete model '+name+' from the onboard computer?'))return;
 try{await post('/api/vision/delete',{model:name});toast('Deleted '+name);pollAINow()}catch(e){toast(e.message,1)}}
async function uploadModel(){if(needPin())return;const f=$('#aifile').files[0];if(!f)return toast('Choose a model file first.',1);
 const name=$('#ainame').value.trim()||f.name.replace(/\.(zip|onnx|rknn|engine|hef)$/i,'').replace(/[^A-Za-z0-9._-]/g,'-').slice(0,48);
 const q=new URLSearchParams({name,filename:f.name,classes:$('#aiclasses').value,imgsz:$('#aiimgsz').value});$('#aiup').textContent='Uploading '+(f.size/1e6).toFixed(1)+' MB…';
 try{const r=await fetch('/api/vision/upload?'+q,{method:'POST',headers:{'X-Panel-Pin':pinEl.value,'Content-Type':'application/octet-stream'},body:f});const d=await r.json().catch(()=>({}));if(!r.ok)throw new Error(d.error||r.status);
  $('#aiup').textContent='';toast('Uploaded '+d.model+'. Press Benchmark to measure it on this board.');pollAINow()}catch(e){$('#aiup').textContent='';toast(e.message,1)}}
async function pollAINow(){try{V=await (await fetch('/api/vision')).json();renderAI()}catch(e){}}
pollAI();

go((location.hash||'#home').slice(1).replace(/[^a-z]/g,'')||'home');poll();
</script></body></html>"""

if __name__ == '__main__':
    main()
