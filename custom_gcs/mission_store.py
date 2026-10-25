"""Operator missions saved on this GCS laptop (config/missions/<id>.json).

Built-in missions come from the planner (the five A8 missions) and can never be saved over or
deleted here; the operator edits a *copy*. A custom mission stores only operator choices: a
feature spec (validated by as2_rescue_planner.features), the parameter values and a name. The
behaviour tree is composed again by the planner every time the mission is planned, so a saved
mission can never carry hand-made or outdated tree XML.
"""
import json
import os
from pathlib import Path
import re
import threading
import time
import uuid

import planner_bridge

ID = re.compile(r'c_[0-9a-f]{12}')
MAX_MISSIONS = 200
# Names the GCS shows for the built-in missions (web/ui/js/page-plan.js MISSIONS).
BUILTIN_NAMES = {'track & target', 'orbit person', 'geotag & resume', 'geotag + drop', 'swarm coverage'}


class MissionStore:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.lock = threading.Lock()

    def _features(self):
        return planner_bridge.features()

    def builtin(self):
        f = self._features()
        from_planner = importlib_titles()
        return [dict(id=key, builtin=True, title=from_planner.get(key, key), features=spec)
                for key, spec in f.BUILTIN_FEATURES.items()]

    def _read(self, path):
        try:
            value = json.loads(path.read_text())
            return value if isinstance(value, dict) and ID.fullmatch(str(value.get('id', ''))) else None
        except (OSError, ValueError):
            return None

    def custom(self):
        if not self.directory.is_dir():
            return []
        rows = [self._read(p) for p in sorted(self.directory.glob('c_*.json'))]
        return sorted((r for r in rows if r), key=lambda r: r.get('updated', 0), reverse=True)

    def catalog(self):
        try:
            builtin, error = self.builtin(), ''
        except ImportError as exc:
            builtin, error = [], str(exc)
        return dict(builtin=builtin, custom=self.custom(), error=error)

    def describe(self, request):
        f = self._features()
        spec = f.feature_spec(request.get('features'))
        fleet = int(request.get('fleet_size') or 1)
        return dict(features=spec, does=f.describe(spec, fleet_size=fleet), requires=f.requirements(spec))

    def save(self, request):
        f = self._features()
        spec = f.feature_spec(request.get('features'))
        name = str(request.get('name') or '').strip()
        if not 1 <= len(name) <= 60:
            raise ValueError('Give the mission a name (1-60 characters)')
        if name.lower() in BUILTIN_NAMES | {b['title'].lower() for b in self.builtin()}:
            raise ValueError('That name belongs to a built-in mission; choose another')
        options = request.get('options') or {}
        if not isinstance(options, dict) or len(options) > 40 or any(
                not re.fullmatch(r'[a-z_]{1,32}', str(k)) or isinstance(v, bool) or not isinstance(v, (int, float))
                for k, v in options.items()):
            raise ValueError('Mission parameters must be numbers')
        notes = str(request.get('notes') or '')[:500]
        key = request.get('id') or ''
        with self.lock:
            existing = {r['id']: r for r in self.custom()}
            if key:
                if not ID.fullmatch(key) or key not in existing:
                    raise ValueError('Built-in missions cannot be changed; save your edit as a new mission')
            else:
                if len(existing) >= MAX_MISSIONS:
                    raise ValueError('Too many saved missions; delete some first')
                key = 'c_' + uuid.uuid4().hex[:12]
            if any(r['name'].lower() == name.lower() and r['id'] != key for r in existing.values()):
                raise ValueError('Another saved mission already has this name')
            record = dict(id=key, name=name, features=spec, options={k: float(v) for k, v in options.items()},
                          notes=notes, based_on=str(request.get('based_on') or '')[:40],
                          created=existing.get(key, {}).get('created', time.time()), updated=time.time())
            self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
            path = self.directory / (key + '.json')
            temp = path.with_suffix('.tmp')
            temp.write_text(json.dumps(record, indent=1))
            os.replace(temp, path)
        return record

    def delete(self, request):
        key = str(request.get('id') or '')
        if not ID.fullmatch(key):
            raise ValueError('Built-in missions cannot be deleted')
        with self.lock:
            path = self.directory / (key + '.json')
            if not path.is_file():
                raise ValueError('Mission not found')
            path.unlink()
        return dict(ok=True, id=key)


def importlib_titles():
    try:
        trees = planner_bridge.trees()
        return {k: v.split(' - ', 1)[-1] for k, v in trees.VISION_MODES.items()}
    except ImportError:
        return {}
