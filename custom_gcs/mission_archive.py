"""Mission archive and mission time (2026-11-01).

Every mission run is saved on the GCS computer:

    ~/AERIXIS_Missions/Simulation/Mission_1/      (Simulation mode, fleet_sim.json)
    ~/AERIXIS_Missions/Real_Hardware/Mission_1/   (Field mode, real drones)
        summary.txt / summary.json      mission time, per-drone result, geotags, deliveries
        drone0/ drone1/ ...
            status.json                 last onboard execution record, times, counts
            track.csv                   1 row every 2 s: position, altitude, speed, battery, operation
            events.jsonl                every change of operation / execution state (GCS view)
            geotags.json / geotags.csv  every person geotagged by this drone
            tree.xml, route_bundle.json, payload_load.json   what the GCS uploaded
            onboard/                    Simulation only: the drone's own files from this laptop
                                        (BT event journal gzip, onboard logs of the mission window,
                                        rescue journal, payload journal, PX4 flight logs .ulg)

A mission opens when the first drone starts and closes once no drone has been running for
CLOSE_AFTER_S. Drones started in between belong to the same mission. Only data the GCS receives
anyway is used (fleet status over the link); on real hardware the full onboard logs stay on the
aircraft. Times: each drone's start/end are the onboard execution times; the mission time is
first start -> last end.
"""
import csv
import json
import math
import os
import re
import shutil
import subprocess
import threading
import time
from pathlib import Path

ROOT = Path(os.environ.get('AERIXIS_MISSIONS_DIR', Path.home() / 'AERIXIS_Missions'))
FOLDERS = {True: 'Simulation', False: 'Real_Hardware'}
CLOSE_AFTER_S = 30.0
TRACK_PERIOD_S = 1.0
SIM_STATE = Path.home() / '.local/share/as2-sitl'
PX4_TMP = Path('/tmp')
TERMINAL = ('SUCCESS', 'FAILURE', 'CANCELLED', 'ABORTED')
TRACK_FIELDS = ['time', 'elapsed_s', 'latitude', 'longitude', 'altitude_m', 'x_m', 'y_m', 'z_m',
                'speed_mps', 'battery', 'operation', 'armed', 'payload_remaining']


def duration_text(seconds):
    if seconds is None or not math.isfinite(seconds):
        return '—'
    seconds = int(round(max(0.0, seconds)))
    h, rest = divmod(seconds, 3600)
    m, s = divmod(rest, 60)
    return f'{h}:{m:02d}:{s:02d}' if h else f'{m}:{s:02d}'


def clock(t):
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t)) if t else '—'


def next_number(folder):
    numbers = [int(m.group(1)) for p in folder.glob('Mission_*') if (m := re.fullmatch(r'Mission_(\d+)', p.name))]
    return max(numbers, default=0) + 1


def _num(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


class DroneRecord:
    def __init__(self, folder, drone_id, label):
        self.drone_id, self.label = drone_id, label
        self.folder = folder / drone_id
        self.folder.mkdir(parents=True, exist_ok=True)
        self.mission_id = None
        self.mission_name = ''
        self.execution = {}
        self.started_at = self.finished_at = None
        self.geotags = {}
        self.payload = {}
        self.served = []
        self.last_track = 0.0
        self.last_xy = None
        self.distance_m = 0.0
        self.last_event = None
        self.staged = None
        self.track_file = open(self.folder / 'track.csv', 'w', newline='')
        self.track = csv.writer(self.track_file)
        self.track.writerow(TRACK_FIELDS)
        self.events_file = open(self.folder / 'events.jsonl', 'w')

    def close_files(self):
        for f in (self.track_file, self.events_file):
            try:
                f.close()
            except OSError:
                pass

    def update(self, data, now):
        execution = data.get('execution') or {}
        if execution.get('mission_id'):
            self.mission_id = execution['mission_id']
            self.execution = execution
            self.started_at = _num(execution.get('started_at')) or self.started_at
            if execution.get('state') in TERMINAL:
                self.finished_at = _num(execution.get('finished_at')) or self.finished_at or now
            else:
                self.finished_at = None
        ros = data.get('ros') or {}
        rescue = (ros.get('rescue') or {}).get('data') or {}
        for tag in rescue.get('geotags') or []:
            key = tag.get('id') or '%s:%s' % (tag.get('latitude'), tag.get('longitude'))
            if key not in self.geotags:
                self.geotags[key] = tag
        payload = (ros.get('links') or {}).get('payload')
        if isinstance(payload, dict) and payload:
            self.payload = payload
            # The drone resets its counts when the mission ends: keep the most seen during it.
            loaded, remaining = _num(payload.get('loaded')), _num(payload.get('remaining'))
            if loaded:
                self.max_loaded = max(getattr(self, 'max_loaded', 0), int(loaded))
                if remaining is not None:
                    self.max_delivered = max(getattr(self, 'max_delivered', 0), int(loaded - remaining))
        if rescue.get('served'):
            for xy in rescue['served']:
                if xy not in self.served:
                    self.served.append(xy)
        operation = rescue.get('active_operation') or ''
        event = (execution.get('state'), operation)
        if event != self.last_event:
            self.last_event = event
            self.events_file.write(json.dumps(dict(time=now, local_time=clock(now), execution=execution.get('state'),
                                                   detail=execution.get('detail'), operation=operation)) + '\n')
            self.events_file.flush()
        if now - self.last_track >= TRACK_PERIOD_S:
            self.last_track = now
            vehicle = rescue.get('vehicle') or {}
            gps = rescue.get('gps') or {}
            pos = vehicle.get('position') or [None, None, None]
            if pos[0] is not None and pos[1] is not None:
                if self.last_xy is not None:
                    step = math.hypot(pos[0] - self.last_xy[0], pos[1] - self.last_xy[1])
                    if step < 50.0:   # a jump (origin change) is not flown distance
                        self.distance_m += step
                self.last_xy = pos[:2]
            self.track.writerow([round(now, 2), round(now - (self.started_at or now), 1), gps.get('latitude'),
                                 gps.get('longitude'), gps.get('altitude'), *pos[:3], vehicle.get('speed'),
                                 vehicle.get('battery'), operation, vehicle.get('armed'), self.payload.get('remaining')])
            self.track_file.flush()

    def duration(self, now):
        if not self.started_at:
            return None
        return (self.finished_at or now) - self.started_at

    def delivered(self):
        if getattr(self, 'max_loaded', 0):
            return getattr(self, 'max_delivered', 0)
        loaded, remaining = _num(self.payload.get('loaded')), _num(self.payload.get('remaining'))
        return None if loaded is None or remaining is None else max(0, int(loaded - remaining))

    def summary(self, now):
        return dict(drone_id=self.drone_id, label=self.label, mission_id=self.mission_id, mission_name=self.mission_name,
                    result=self.execution.get('state'), detail=self.execution.get('detail'),
                    started_at=self.started_at, finished_at=self.finished_at,
                    started=clock(self.started_at), finished=clock(self.finished_at),
                    duration_s=None if self.duration(now) is None else round(self.duration(now), 1),
                    duration=duration_text(self.duration(now)), geotags=len(self.geotags),
                    packages_loaded=getattr(self, 'max_loaded', 0) or _num(self.payload.get('loaded')), packages_delivered=self.delivered(),
                    distance_flown_m=round(self.distance_m))


class MissionArchive:
    def __init__(self, root=ROOT, sim_state=SIM_STATE, px4_tmp=PX4_TMP, close_after_s=CLOSE_AFTER_S):
        self.root, self.sim_state, self.px4_tmp, self.close_after_s = Path(root), Path(sim_state), Path(px4_tmp), close_after_s
        self.lock = threading.Lock()
        self.current = None
        self.staged = {}
        self.copying = 0

    # ---------- inputs ----------
    def note_stage(self, request):
        """What the GCS uploaded to a drone (Upload & validate): saved with the next mission."""
        drone_id = str(request.get('drone_id') or '')
        if request.get('action') != 'stage' or not drone_id:
            return
        with self.lock:
            self.staged[drone_id] = dict(label=str(request.get('gcs_label') or '')[:200],
                                         xml=request.get('xml'), route_bundle=request.get('route_bundle'),
                                         operator_payload=request.get('operator_payload'),
                                         emergency_action=request.get('emergency_action'), at=time.time())

    def tick(self, drones, sim_mode, now=None):
        now = time.time() if now is None else now
        with self.lock:
            active = {d['drone_id'] for d in drones if (d.get('data') or {}).get('active')}
            cur = self.current
            if cur is None:
                if not active:
                    return None
                cur = self._open(sim_mode, now)
            elif cur['sim'] != bool(sim_mode):
                self._close(now)
                return None
            for d in drones:
                drone_id = d['drone_id']
                if drone_id in active and drone_id not in cur['drones']:
                    record = DroneRecord(cur['folder'], drone_id, d.get('label') or drone_id)
                    record.staged = self.staged.get(drone_id)
                    record.mission_name = (record.staged or {}).get('label') or ''
                    cur['drones'][drone_id] = record
                if drone_id in cur['drones'] and d.get('data'):
                    cur['drones'][drone_id].update(d['data'], now)
            if active:
                cur['idle_since'] = None
                return None
            cur['idle_since'] = cur['idle_since'] or now
            if now - cur['idle_since'] >= self.close_after_s:
                return self._close(now)
        return None

    # ---------- mission folder ----------
    def _open(self, sim_mode, now):
        base = self.root / FOLDERS[bool(sim_mode)]
        base.mkdir(parents=True, exist_ok=True)
        number = next_number(base)
        folder = base / f'Mission_{number}'
        folder.mkdir()
        self.current = dict(number=number, folder=folder, sim=bool(sim_mode), opened=now, idle_since=None, drones={})
        (folder / 'summary.txt').write_text(f'Mission {number} ({FOLDERS[bool(sim_mode)]}) started {clock(now)}; '
                                            'running — the summary is written when it ends.\n')
        return self.current

    def _times(self, cur, now):
        starts = [r.started_at for r in cur['drones'].values() if r.started_at]
        ends = [r.finished_at or now for r in cur['drones'].values() if r.started_at]
        start = min(starts) if starts else cur['opened']
        end = max(ends) if ends else now
        return start, end

    def _close(self, now):
        cur, self.current = self.current, None
        start, end = self._times(cur, now)
        drones = []
        for record in cur['drones'].values():
            record.close_files()
            summary = record.summary(now)
            drones.append(summary)
            (record.folder / 'status.json').write_text(json.dumps(dict(summary, execution=record.execution,
                                                                       payload=record.payload, served=record.served), indent=2))
            tags = list(record.geotags.values())
            (record.folder / 'geotags.json').write_text(json.dumps(tags, indent=2))
            with open(record.folder / 'geotags.csv', 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['id', 'label', 'latitude', 'longitude', 'altitude_msl', 'x_m', 'y_m', 'sigma_m', 'time'])
                for t in tags:
                    local = t.get('local') or [None, None]
                    w.writerow([t.get('id'), t.get('label'), t.get('latitude'), t.get('longitude'), t.get('altitude_msl'),
                                local[0], local[1], t.get('sigma_m'), clock(t.get('wall_time') or t.get('stamp'))])
            if record.staged:
                if record.staged.get('xml'):
                    (record.folder / 'tree.xml').write_text(record.staged['xml'])
                for key, name in (('route_bundle', 'route_bundle.json'), ('operator_payload', 'payload_load.json')):
                    if record.staged.get(key):
                        (record.folder / name).write_text(json.dumps(record.staged[key], indent=2))
        summary = dict(mission=cur['number'], mode=FOLDERS[cur['sim']], folder=str(cur['folder']),
                       started_at=start, finished_at=end, started=clock(start), finished=clock(end),
                       mission_time_s=round(end - start, 1), mission_time=duration_text(end - start),
                       geotags=sum(d['geotags'] for d in drones),
                       packages_delivered=sum(d['packages_delivered'] or 0 for d in drones), drones=drones,
                       onboard_logs='copied into each drone folder (onboard/)' if cur['sim'] else
                       'full onboard logs stay on each aircraft (mission folder on the Pi)')
        (cur['folder'] / 'summary.json').write_text(json.dumps(summary, indent=2))
        (cur['folder'] / 'summary.txt').write_text(self._text(summary))
        if cur['sim']:
            self.copying += 1
            threading.Thread(target=self._copy_sim, args=(cur, start, end), daemon=True).start()
        return summary

    @staticmethod
    def _text(s):
        lines = [f"AERIXIS mission {s['mission']} — {s['mode']}",
                 f"Started   {s['started']}", f"Finished  {s['finished']}",
                 f"Mission time  {s['mission_time']}  (first drone start -> last drone end)",
                 f"Geotags   {s['geotags']}", f"Packages delivered  {s['packages_delivered']}", '',
                 f"{'Drone':<10}{'Result':<11}{'Time':>9}{'Geotags':>9}{'Packages':>10}{'Flown':>9}  Detail"]
        for d in s['drones']:
            pk = '—' if d['packages_delivered'] is None else f"{d['packages_delivered']}/{d['packages_loaded']}"
            lines.append(f"{d['label']:<10}{str(d['result']):<11}{d['duration']:>9}{d['geotags']:>9}{pk:>10}"
                         f"{d['distance_flown_m']:>7} m  {d['detail'] or ''}")
        lines += ['', f"Onboard logs: {s['onboard_logs']}"]
        return '\n'.join(lines) + '\n'

    # ---------- Simulation: the drones' own files from this laptop ----------
    def _copy_sim(self, cur, start, end):
        try:
            for drone_id, record in cur['drones'].items():
                out = record.folder / 'onboard'
                out.mkdir(exist_ok=True)
                missions = self.sim_state / 'missions' / drone_id
                if record.mission_id:
                    for path in missions.glob(record.mission_id + '.*'):
                        if path.name.endswith('.events.jsonl'):
                            with open(out / (path.name + '.gz'), 'wb') as f:   # often hundreds of MB
                                subprocess.run(['nice', '-n', '15', 'gzip', '-c', str(path)], stdout=f, check=False)
                        else:
                            shutil.copy2(path, out / path.name)
                for src, name in ((self.sim_state / 'rescue' / drone_id / 'rescue.json', 'rescue_journal.json'),
                                  (self.sim_state / 'payload' / drone_id / 'actuator.json', 'payload_journal.json')):
                    if src.is_file():
                        shutil.copy2(src, out / name)
                for log in (self.sim_state / 'logs').glob(f'{drone_id}_*.log'):
                    self._slice_log(log, out / log.name, start - 30, end + 30)
                index = re.sub(r'\D', '', drone_id)
                for ulog in (self.px4_tmp / f'px4_instance_{index}' / 'log').glob('*/*.ulg') if index else []:
                    mtime = ulog.stat().st_mtime
                    if start - 120 <= mtime and ulog.stat().st_ctime <= end + 600:
                        with open(out / (ulog.name + '.gz'), 'wb') as f:      # ~270 MB each uncompressed
                            subprocess.run(['nice', '-n', '15', 'gzip', '-c', str(ulog)], stdout=f, check=False)
        except OSError as exc:
            (cur['folder'] / 'copy_error.txt').write_text(str(exc) + '\n')
        finally:
            self.copying -= 1

    @staticmethod
    def _slice_log(src, dst, start, end):
        """Lines of the mission window: ROS lines carry [<epoch>] stamps; unstamped lines follow the
        last stamp seen."""
        stamp = re.compile(r'\[(\d{3,})\.\d+\]')
        ansi = re.compile(r'\x1b\[[0-9;]*m')
        keep = False
        stamped = False
        with open(src, errors='replace') as f, open(dst, 'w') as out:
            for line in f:
                m = stamp.search(line)
                if m:
                    stamped = True
                    keep = start <= int(m.group(1)) <= end
                if keep:
                    out.write(ansi.sub('', line))
        if not stamped:
            # A log without time stamps (gateway, mesh, mission manager): its last 5 MB.
            with open(src, 'rb') as f:
                f.seek(max(0, os.path.getsize(src) - 5_000_000))
                data = f.read().decode(errors='replace')
            Path(dst).write_text(ansi.sub('', data))

    # ---------- outputs ----------
    def status(self, now=None):
        now = time.time() if now is None else now
        with self.lock:
            cur = self.current
            if cur is None:
                return dict(running=False, root=str(self.root), copying=self.copying)
            start, end = self._times(cur, now)
            return dict(running=True, mission=cur['number'], mode=FOLDERS[cur['sim']], folder=str(cur['folder']),
                        started_at=start, elapsed_s=round(end - start, 1), elapsed=duration_text(end - start),
                        ending_in_s=None if cur['idle_since'] is None else max(0, round(self.close_after_s - (now - cur['idle_since']))),
                        drones=[r.summary(now) for r in cur['drones'].values()], root=str(self.root), copying=self.copying)

    def history(self, limit=50):
        rows = []
        for mode in FOLDERS.values():
            for path in (self.root / mode).glob('Mission_*/summary.json'):
                try:
                    s = json.loads(path.read_text())
                    rows.append({k: s.get(k) for k in ('mission', 'mode', 'folder', 'started', 'started_at', 'mission_time',
                                                       'mission_time_s', 'geotags', 'packages_delivered')} |
                                dict(drones=[{k: d.get(k) for k in ('label', 'result', 'duration', 'geotags', 'packages_delivered')}
                                             for d in s.get('drones', [])]))
                except (OSError, ValueError):
                    continue
        rows.sort(key=lambda r: r.get('started_at') or 0, reverse=True)
        return rows[:limit]
