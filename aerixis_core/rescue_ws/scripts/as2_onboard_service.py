#!/usr/bin/env python3
"""Headless onboard service: the same links as the Pi GUI, supervised, for Docker / systemd.

  as2_onboard_service.py            start every configured link and keep them running
  as2_onboard_service.py run --only fc,guardian   /  run --skip fc,guardian
                                    supervise only some links (systemd splits them, see
                                    rescue_ws/systemd/)
  as2_onboard_service.py exec NAME  replace this process with one link (fc, guardian, ...), for a
                                    systemd unit that restarts that link by itself
  as2_onboard_service.py token      print drone ID, system ID and the GCS pairing token
  as2_onboard_service.py config     print the effective settings (file + AS2_* environment)
  as2_onboard_service.py status     print the last link states (used by the Docker healthcheck)

Order: flight controller (XRCE agent) -> FC guardian -> mesh (if a passphrase is set) -> AERIXIS core +
mission manager -> GCS telemetry gateway -> camera; links start a second or two apart, not in
long fixed steps. A link whose device is missing (radio not plugged in yet) is retried every
10 s; a link that exits is restarted with back-off (FC link and guardian: 1-2 s). Nothing here
arms, flies or releases: missions come from the laptop GCS and the onboard gates still apply.
The FC guardian is the exception that can command PX4 itself, only for an emergency RTL/LAND
while airborne (AERIXIS core stopped, or the GCS asked for it).
Settings: ~/.config/as2/pi_launcher.json (written by the Pi GUI) or AS2_<KEY> variables.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import onboard_links as links  # noqa: E402

STATUS_FILE = Path(os.environ.get('AS2_STATUS_FILE', '/tmp/as2-onboard-status.json'))
REQUIRED = ('fc', 'guardian', 'as2', 'manager', 'gcs')   # the healthcheck needs these running


class Supervised:
    def __init__(self, name, build, delay=0.0, backoff=5.0, max_backoff=60.0, stable_s=5.0):
        # stable_s: running this long counts as a good start (backoff back to the minimum). The
        # camera uses 60 s: its tracker exits on purpose after an 8 s gimbal-centre check, which
        # with 5 s kept retrying every 5 s for ever (Pi 5, 2026-10-23, camera unpowered).
        self.name, self.build, self.delay, self.stable_s = name, build, delay, stable_s
        self.min_backoff, self.max_backoff = backoff, max_backoff
        self.process = None
        self.state = 'waiting'
        self.detail = ''
        self.restarts = 0
        self.next_try = time.monotonic() + delay
        self.backoff = backoff

    def poll(self, stopping):
        now = time.monotonic()
        if self.process is not None and self.process.poll() is None:
            if self.state == 'starting' and now - self.started > 5:
                self.state = 'running'
            if now - self.started > self.stable_s:
                self.backoff = self.min_backoff
            return
        if self.process is not None:
            code = self.process.returncode
            self.process = None
            if stopping:
                return
            self.state, self.detail = 'restarting', 'exited with code {}'.format(code)
            log(self.name, 'exited (code {}); restarting in {:.0f} s'.format(code, self.backoff))
            self.next_try = now + self.backoff
            self.backoff = min(self.max_backoff, self.backoff * 2)
            self.restarts += 1
            return
        if stopping or now < self.next_try:
            return
        try:
            command = self.build()
        except ValueError as exc:  # e.g. radio not plugged in yet: keep waiting, say why
            if self.detail != str(exc):
                log(self.name, 'waiting: {}'.format(exc))
            self.state, self.detail, self.next_try = 'waiting', str(exc), now + 10
            return
        if command is None:
            self.state, self.detail = 'off', 'disabled in settings'
            self.next_try = now + 3600
            return
        log(self.name, 'start: ' + (command[-1].split('exec ')[-1] if command[0] == 'bash' else ' '.join(command)))
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        stdin=subprocess.DEVNULL, text=True, bufsize=1, start_new_session=True)
        self.started, self.state, self.detail = now, 'starting', ''
        threading.Thread(target=self.pump, daemon=True).start()

    def pump(self):
        for line in iter(self.process.stdout.readline, ''):
            write_log('[{}] {}'.format(self.name, line))
            self.detail = line.strip()[-160:]

    def stop(self):
        if self.process is None or self.process.poll() is not None:
            return
        try:
            os.killpg(self.process.pid, signal.SIGINT)
            self.process.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


LOG_FILE = Path(os.environ.get('AS2_LOG_FILE', '/tmp/as2-onboard.log'))
CONTROL_FILE = Path(os.environ.get('AS2_CONTROL_FILE', '/tmp/as2-onboard-control.json'))
_log_lock = threading.Lock()


def write_log(text):
    """stdout (docker logs / journal) plus a small file the onboard web panel shows."""
    sys.stdout.write(text)
    sys.stdout.flush()
    with _log_lock:
        try:
            if LOG_FILE.exists() and LOG_FILE.stat().st_size > 1_000_000:
                LOG_FILE.write_text(LOG_FILE.read_text(errors='replace')[-200_000:])
            with open(LOG_FILE, 'a') as stream:
                stream.write(text)
        except OSError:
            pass


def log(name, message):
    write_log('[{}] {}\n'.format(name, message))


def banner(value):
    drone = links.drone_id(value)
    token = links.ensure_token(drone).read_text().strip()
    log('service', '=' * 64)
    log('service', 'AERIXIS onboard  ·  drone {}  ·  MAVLink system ID {}'.format(drone, value('system_id')))
    log('service', 'GCS link: {} {}'.format(value('gcs_mode'), value('gcs_device') if value('gcs_mode') == 'serial'
                                            else '{}:{}'.format(value('gcs_host'), value('gcs_port'))))
    log('service', 'Pairing token (enter in GCS → Connect → Add aircraft manually):')
    log('service', token)
    log('service', 'Flight allowed: {}'.format('YES' if value('allow_flight') else 'no (observation only)'))
    log('service', 'Emergency action if the AERIXIS core stops (FC guardian + PX4 failsafe): {}'.format(value('emergency_action')))
    if int(value('web_port') or 0):
        import onboard_web
        log('service', 'Onboard panel: {}  (PIN {})'.format(
            '  '.join('http://{}:{}'.format(a, value('web_port')) for a in onboard_web.addresses()), onboard_web.panel_pin()))
    log('service', '=' * 64)


def build_services(value):
    mesh_on = bool(value('mesh_passphrase')) and str(value('mesh_mode')) != 'off'

    def as2():
        return links.cmd_as2(value, links.write_profile(value, links.drone_id(value), mesh_on))

    services = [Supervised('fc', lambda: links.cmd_fc(value), 0, 1.0, 2.0),
                Supervised('guardian', lambda: links.cmd_guardian(value), 0.5, 1.0, 2.0)]
    if mesh_on:
        services.append(Supervised('mesh', lambda: links.cmd_mesh(value), 1))
    services += [Supervised('as2', as2, 1), Supervised('manager', lambda: links.cmd_manager(value), 2),
                 Supervised('gcs', lambda: links.cmd_gcs(value), 2),
                 Supervised('camera', lambda: links.cmd_camera(value), 4, stable_s=60.0),
                 # Onboard control panel (web page): last, once everything else is starting.
                 Supervised('web', lambda: links.cmd_web(value), 6)]
    return services


def select(services, argv):
    """'--only a,b' / '--skip a,b' from the run arguments."""
    names = lambda flag: set(argv[argv.index(flag) + 1].split(',')) if flag in argv[:-1] else None
    only, skip = names('--only'), names('--skip') or set()
    known = {s.name for s in services} | {'mesh'}
    unknown = ((only or set()) | skip) - known
    if unknown:
        raise SystemExit('unknown link(s): {} (known: {})'.format(', '.join(sorted(unknown)), ', '.join(sorted(known))))
    return [s for s in services if (only is None or s.name in only) and s.name not in skip]


def exec_link(name):
    """Become one link's process (systemd then supervises and restarts it directly)."""
    value = links.load_settings().get
    service = next((s for s in build_services(value) if s.name == name), None)
    if service is None:
        raise SystemExit('unknown link: ' + name)
    try:
        command = service.build()
    except ValueError as exc:
        log(name, 'cannot start: {}'.format(exc))
        sys.exit(1)
    if command is None:
        log(name, 'disabled in settings')
        sys.exit(0)
    log(name, 'exec: ' + (command[-1].split('exec ')[-1] if command[0] == 'bash' else ' '.join(command)))
    sys.stdout.flush()
    os.execvp(command[0], command)


class FreshSettings:
    """Settings re-read when the file changes, so a restarted link uses what the panel saved."""
    def __init__(self):
        self.mtime, self.settings = None, links.load_settings()

    def get(self, key, default=None):
        try:
            mtime = links.CONFIG_FILE.stat().st_mtime
        except OSError:
            mtime = None
        if mtime != self.mtime:
            self.mtime, self.settings = mtime, links.load_settings()
        return self.settings.get(key, default)


def handle_control(services, argv, seen):
    """Restart requests from the onboard web panel: {"id": ..., "restart": ["gcs", ...] | ["all"]}."""
    try:
        request = json.loads(CONTROL_FILE.read_text())
    except (OSError, ValueError):
        return services
    if request.get('id') in seen:
        return services
    seen.add(request.get('id'))
    names = set(request.get('restart') or [])
    if 'all' in names or {'drone_id', 'system_id'} & names:
        log('service', 'restart of all links requested by the onboard panel')
        for service in reversed(services):
            service.stop()
            service.poll(True)
        return select(build_services(FreshSettings().get), list(argv))
    for service in services:
        if service.name in names:
            log('service', 'restart of {} requested by the onboard panel'.format(service.name))
            service.stop()
            service.poll(True)
            service.next_try = time.monotonic() + 0.5
            service.backoff = service.min_backoff
    return services


def run(argv=()):
    value = FreshSettings().get
    banner(value)
    services = select(build_services(value), list(argv))
    log('service', 'links: ' + ', '.join(s.name for s in services))
    stopping = threading.Event()

    def stop(*_):
        stopping.set()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    seen = set()
    try:
        seen.add(json.loads(CONTROL_FILE.read_text()).get('id'))   # ignore a request from before this start
    except (OSError, ValueError):
        pass
    while not stopping.is_set():
        services = handle_control(services, argv, seen)
        for service in services:
            service.poll(False)
        STATUS_FILE.write_text(json.dumps({s.name: dict(state=s.state, restarts=s.restarts, detail=s.detail,
                                                        pid=s.process.pid if s.process is not None else None)
                                           for s in services}))
        stopping.wait(1.0)
    log('service', 'stopping all links')
    for service in reversed(services):
        service.stop()
        service.poll(True)


def main():
    action = sys.argv[1] if len(sys.argv) > 1 else 'run'
    settings = links.load_settings()
    if action == 'token':
        drone = links.drone_id(settings.get)
        print('drone_id={} system_id={}\ntoken={}'.format(drone, settings['system_id'],
                                                         links.ensure_token(drone).read_text().strip()))
        import onboard_web
        print('panel=http://{}:{}  panel_pin={}'.format(onboard_web.addresses()[0], settings.get('web_port'), onboard_web.panel_pin()))
    elif action == 'config':
        print(json.dumps({k: ('***' if 'passphrase' in k and v else v) for k, v in settings.items()}, indent=2))
    elif action == 'status':
        try:
            states = json.loads(STATUS_FILE.read_text())
        except (OSError, ValueError):
            print('no status yet'); sys.exit(1)
        for name, item in states.items():
            print('{:8} {:10} restarts={} {}'.format(name, item['state'], item['restarts'], item['detail'][:90]))
        required = [n for n in REQUIRED if n in states]   # a split service checks its own links
        sys.exit(0 if required and all(states[n].get('state') in ('running', 'starting') for n in required) else 1)
    elif action == 'run':
        run(sys.argv[2:])
    elif action == 'exec' and len(sys.argv) > 2:
        exec_link(sys.argv[2])
    else:
        raise SystemExit(__doc__)


if __name__ == '__main__':
    main()
