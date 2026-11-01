"""Simulation control for the GCS (laptop only): start/stop the PX4 SITL rescue arena, reopen the
drone camera window, and give the maps the arena layout (outline + ground-truth people).

Used only in Simulation mode, and only from this computer (the server refuses other hosts for
these routes). It runs the AERIXIS simulation launcher (aerostack2/launch/sim.sh --arena), which
starts PX4 + Gazebo with the arena placed at the chosen location, the onboard stack of every
simulated drone and the separate camera window. Field operation never touches this module.
"""
import json
import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

AS2_DIR = Path(os.environ.get('AERIXIS_AS2_DIR', Path.home() / 'aerostack2'))
SIM_SH = AS2_DIR / 'launch/sim.sh'
ARENA_ROOT = Path.home() / '.local/share/aerixis/arena'
RUN_ARENA = Path.home() / '.local/share/aerixis/run/arena.dir'
PX4_LOG = Path.home() / '.local/share/aerixis/logs/px4.log'
STATE_FILE = Path(__file__).parent / 'config/simulation.json'
PX4_116 = Path.home() / 'px4_sitl/PX4-Autopilot-1.16'


class SimControl:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = 'idle'          # idle | starting | running | stopping | failed
        self.message = ''
        self.log = deque(maxlen=60)
        self.process = None
        self.last_action = 'start'
        self.settings = self._load()

    # ---------- persistence (last location / drone count) ----------
    def _load(self):
        try:
            return json.loads(STATE_FILE.read_text())
        except (OSError, ValueError):
            return {}

    def _save(self):
        try:
            STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
            STATE_FILE.write_text(json.dumps(self.settings, indent=2) + '\n')
        except OSError:
            pass

    # ---------- launcher ----------
    def _run(self, *args, timeout=40):
        env = dict(os.environ, PX4_AUTOPILOT_DIR=os.environ.get('PX4_AUTOPILOT_DIR', str(PX4_116)))
        return subprocess.run(['bash', str(SIM_SH), *args], capture_output=True, text=True, timeout=timeout, env=env)

    def running_components(self):
        try:
            out = self._run('status', timeout=10).stdout
        except (OSError, subprocess.SubprocessError):
            return {}
        comps = {}
        for line in out.splitlines():
            parts = line.split()
            # Status lines only ("px4  running (pid N)"); skip extra lines such as "px4 pid N".
            if len(parts) >= 2 and parts[0] in ('px4', 'sitl-onboard', 'sim-cameras', 'gazebo-gui') and parts[1] != 'pid':
                comps[parts[0]] = 'running' in line
        return comps

    def status(self):
        comps = self.running_components()
        leftovers = not comps.get('px4') and (comps.get('sitl-onboard') or comps.get('sim-cameras'))
        with self.lock:
            idle = self.state in ('idle', 'running', 'failed') and self.process is None
        if idle and leftovers:
            # PX4 + Gazebo ended (Gazebo window closed): stop what is left (onboard stacks, camera window).
            self.stop()
            with self.lock:
                self.message = 'Gazebo window was closed: stopping the rest of the simulation…'
            comps = self.running_components()
        with self.lock:
            # A failure stays visible (with Retry) until the next start/add/stop.
            if self.state in ('idle', 'running') and self.process is None:
                self.state = 'running' if comps.get('px4') else 'idle'
            log = list(self.log)
            return dict(state=self.state, message=self.message, components=comps,
                        settings=self.settings, log=log[-12:], launcher=str(SIM_SH),
                        launcher_found=SIM_SH.is_file(), progress=self.progress(log, comps))

    def progress(self, log, comps):
        """Start-up stages for the GCS loading window, from the launcher output and the PX4 log."""
        text = '\n'.join(log)
        try:
            px4 = PX4_LOG.read_text(errors='replace')[-20000:] if PX4_LOG.exists() else ''
        except OSError:
            px4 = ''
        started = self.state in ('starting', 'adding')
        adding = self.state == 'adding'
        wanted = int(self.settings.get('drones') or 1)
        spawned = px4.count('(PX4 instance') if 'AERIXIS rescue arena' in px4 else 0
        running = bool(comps.get('px4')) and not started
        ready = running and bool(comps.get('sitl-onboard'))
        return dict(
            drones=wanted,
            arena=adding or running or 'AERIXIS simulation:' in text,
            gazebo=adding or running or spawned > 0,
            px4_spawned=wanted if running else (wanted if 'PX4 ready' in text else wanted - 1) if adding else min(spawned, wanted),
            adding=adding or (self.state == 'failed' and self.last_action == 'add'),
            px4_ready=running or 'PX4 ready' in text,
            onboard=ready or 'Simulation ready' in text,
            onboard_waiting='waiting for the onboard stacks' in text and 'Simulation ready' not in text,
            failed=self.state == 'failed')

    def start(self, drones, home, show3d=True, cameras=True):
        drones = int(drones)
        if not 1 <= drones <= 4:
            raise ValueError('The rescue arena fits 1 to 4 drones')
        lat, lon = float(home[0]), float(home[1])
        alt = float(home[2]) if len(home) > 2 and home[2] is not None else 0.0
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError('Invalid location')
        if not SIM_SH.is_file():
            raise RuntimeError(f'Simulation launcher not found: {SIM_SH}')
        with self.lock:
            if self.state in ('starting', 'stopping'):
                raise RuntimeError(f'Simulation is {self.state}')
            self.state, self.message, self.last_action = 'starting', 'Preparing the arena…', 'start'
            self.log.clear()
            self.settings.update(drones=drones, home=[lat, lon, alt], show3d=bool(show3d), cameras=bool(cameras))
            self._save()
        args = ['start', str(drones), '--arena', '--home', f'{lat:.7f},{lon:.7f},{alt:.1f}']
        if not show3d:
            args.append('--no-3d')
        if not cameras:
            args.append('--no-cameras')
        threading.Thread(target=self._start_thread, args=(args,), daemon=True).start()

    def _start_thread(self, args):
        env = dict(os.environ, PX4_AUTOPILOT_DIR=os.environ.get('PX4_AUTOPILOT_DIR', str(PX4_116)))
        try:
            subprocess.run(['bash', str(SIM_SH), 'stop'], capture_output=True, timeout=90, env=env)
            self.process = subprocess.Popen(['bash', str(SIM_SH), *args], stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, text=True, env=env)
            for line in self.process.stdout:
                line = line.rstrip()
                if not line:
                    continue
                clean = ''.join(ch for ch in line if ch.isprintable()).replace('[1m', '').replace('[0m', '').replace('[33m', '')
                if ' token ' in clean:      # pairing tokens stay on the simulation side (as with a Pi)
                    clean = clean.split(' token ')[0] + ' token (hidden — launch/sim.sh pairing)'
                with self.lock:
                    self.log.append(clean)
                    if 'waiting for PX4' in clean:
                        self.message = 'Starting PX4 and Gazebo…'
                    elif 'PX4 ready' in clean:
                        self.message = 'PX4 ready — starting the onboard software of each drone…'
                    elif 'Simulation ready' in clean:
                        self.message = 'Simulation ready'
            code = self.process.wait()
            with self.lock:
                ok = code == 0 and any('Simulation ready' in l or 'PX4 ready' in l for l in self.log)
                self.state = 'running' if ok else 'failed'
                if not ok:
                    self.message = f'Start failed (exit {code}) — see the log below'
                elif self.message != 'Simulation ready':
                    self.message = 'Running (onboard software still starting)'
        except (OSError, subprocess.SubprocessError) as exc:
            with self.lock:
                self.state, self.message = 'failed', str(exc)
        finally:
            self.process = None

    # ---------- one more drone in the running arena ----------
    def add(self):
        comps = self.running_components()
        if not comps.get('px4'):
            raise RuntimeError('Start the simulation first')
        n = len(list(Path('/tmp').glob('px4_instance_*')))
        try:
            onboard = len(json.loads((Path.home() / '.local/share/as2-sitl/sitl_fleet.json').read_text()))
        except (OSError, ValueError):
            onboard = n
        if onboard < n:
            n -= 1                 # finish the drone whose PX4 already runs (launcher resumes it)
        elif n >= 4:
            raise ValueError('The arena fits 4 drones')
        with self.lock:
            if self.state in ('starting', 'stopping', 'adding'):
                raise RuntimeError(f'Simulation is {self.state}')
            self.state, self.message, self.last_action = 'adding', f'Adding drone {n + 1} to the arena…', 'add'
            self.log.clear()
            self.settings['drones'] = n + 1
            self._save()
        threading.Thread(target=self._add_thread, daemon=True).start()
        return n + 1

    def _add_thread(self):
        env = dict(os.environ, PX4_AUTOPILOT_DIR=os.environ.get('PX4_AUTOPILOT_DIR', str(PX4_116)))
        try:
            self.process = subprocess.Popen(['bash', str(SIM_SH), 'add'], stdout=subprocess.PIPE,
                                            stderr=subprocess.STDOUT, text=True, env=env)
            for line in self.process.stdout:
                clean = ''.join(ch for ch in line.rstrip() if ch.isprintable()).replace('[1m', '').replace('[0m', '').replace('[33m', '')
                if not clean:
                    continue
                with self.lock:
                    self.log.append(clean)
                    if 'PX4 ready' in clean:
                        self.message = 'PX4 ready — restarting the onboard software for every drone…'
            code = self.process.wait()
            with self.lock:
                ok = code == 0 and any('Simulation ready' in l for l in self.log)
                self.state = 'running' if code == 0 else 'failed'
                self.message = 'Simulation ready' if ok else (f'Adding the drone failed (exit {code})' if code else 'Running (onboard software still starting)')
        except (OSError, subprocess.SubprocessError) as exc:
            with self.lock:
                self.state, self.message = 'failed', str(exc)
        finally:
            self.process = None

    def gui(self):
        if not self.running_components().get('px4'):
            raise RuntimeError('The simulation is not running')
        self._run('gui', timeout=20)

    def stop(self):
        with self.lock:
            if self.state == 'stopping':
                return
            self.state, self.message = 'stopping', 'Stopping the simulation…'
        def run():
            try:
                self._run('stop', timeout=120)
            except (OSError, subprocess.SubprocessError) as exc:
                with self.lock:
                    self.log.append(str(exc))
            with self.lock:
                self.state = 'idle'
                self.message = 'Simulation stopped (Gazebo window was closed)' if 'Gazebo' in self.message else 'Stopped'
        threading.Thread(target=run, daemon=True).start()

    def cameras(self):
        comps = self.running_components()
        if not comps.get('px4'):
            raise RuntimeError('The simulation is not running')
        if comps.get('sim-cameras'):
            return 'already open'
        self._run('cameras', timeout=20)
        return 'opened'

    # ---------- pairing (Simulation only) ----------
    def pairing(self):
        """Simulated drones the launcher knows: [(drone_id, system_id, token)], limited to the number
        of drones the last GCS start asked for. Read only on request (Add simulated drones); the
        tokens go straight into the simulation aircraft list and are never sent to the browser."""
        out = self._run('pairing', timeout=20).stdout
        found = []
        for line in out.splitlines():
            parts = line.split()
            # "  drone0   system ID 1   token XXXX"
            if len(parts) == 6 and parts[0].startswith('drone') and parts[1:3] == ['system', 'ID'] and parts[4] == 'token':
                found.append((parts[0], int(parts[3]), parts[5]))
        limit = self.settings.get('drones')
        if limit:
            found = [f for f in found if int(f[0][5:] or 0) < int(limit)]
        return found

    # ---------- arena for the maps ----------
    def arena(self):
        # The launcher records the arena it runs (also when started from a terminal).
        try:
            info = Path(RUN_ARENA.read_text().strip()) / 'sim_info.json'
        except OSError:
            home = self.settings.get('home')
            if not home:
                return None
            info = ARENA_ROOT / f'{home[0]:.6f}_{home[1]:.6f}' / 'sim_info.json'
        try:
            data = json.loads(info.read_text())
        except (OSError, ValueError):
            return None
        return dict(home=data['home'], size_m=data['size_m'], bounds_enu=data['bounds_enu'],
                    spawns=data['spawns'], humans=[dict(name=h['name'], latitude=h['latitude_deg'],
                                                        longitude=h['longitude_deg']) for h in data['humans']])
