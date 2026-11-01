"""Offline AI mission planner for the GCS (2026-11-01).

The operator describes the mission in their own words (Hindi, Hinglish, English, ...) on the Plan
page. A local model (llama.cpp `llama-server`, Qwen3-4B-Instruct, on this laptop's GPU; nothing
leaves the computer) asks for what is missing and returns a filled mission FORM: mission type (or
the features of a custom mission), parameters, each drone's role, emergency action. The GCS checks
the form against the parameter limits and the drones' real abilities (camera, payload servos);
the Plan page applies it, the operator accepts the conditions, and the existing planner makes the
behaviour trees. The model never writes a tree and never commands a drone.

Files: ~/aerixis_llm/runtime/{cuda,vulkan}/... (llama.cpp builds), ~/aerixis_llm/models/*.gguf.
The server is started once and kept loaded (no cold start); its prompt cache makes follow-up
messages fast because the long instructions are processed only once.
"""
import glob
import json
import math
import os
import subprocess
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

LLM_DIR = Path(os.environ.get('AERIXIS_LLM_DIR', Path.home() / 'aerixis_llm'))
PORT = int(os.environ.get('AERIXIS_LLM_PORT', '8772'))
CONTEXT_TOKENS = 4096   # planning chats stay well under this; 8192 took 3.6 GB GPU memory
MAX_REPLY_TOKENS = 700
ROLES = ('search', 'delivery')
FEATURE_TYPES = ('coverage', 'watch', 'track')
ENDINGS = ('RTL', 'LAND')

# The answer the model must give (llama-server turns it into a grammar: always valid JSON).
ANSWER_SCHEMA = {
    'type': 'object',
    'properties': {
        'reply': {'type': 'string'},
        'status': {'type': 'string', 'enum': ['need_info', 'ready']},
        'plan': {
            'type': 'object',
            'properties': {
                'mission': {'type': 'string'},
                'custom_name': {'type': 'string'},
                'features': {
                    'type': 'object',
                    'properties': {
                        'type': {'type': 'string', 'enum': list(FEATURE_TYPES)},
                        'detect': {'type': 'boolean'}, 'geotag': {'type': 'boolean'}, 'orbit': {'type': 'boolean'},
                        'drop': {'type': 'boolean'}, 'resume': {'type': 'boolean'},
                        'finish': {'type': 'string', 'enum': list(ENDINGS)},
                        'delivery_drones': {'type': 'boolean'},
                    },
                    'required': ['type', 'detect', 'geotag', 'orbit', 'drop', 'resume', 'finish', 'delivery_drones'],
                },
                'params': {'type': 'object', 'additionalProperties': {'type': 'number'}},
                'roles': {'type': 'object', 'additionalProperties': {'type': 'string', 'enum': list(ROLES)}},
                'orbit_drones': {'type': 'array', 'items': {'type': 'string'}},
                'emergency': {'type': 'string', 'enum': list(ENDINGS)},
                'ai_drones': {'type': 'array', 'items': {'type': 'string'}},
            },
            'required': ['mission', 'params', 'roles', 'emergency', 'ai_drones'],
        },
    },
    'required': ['reply', 'status', 'plan'],
}

RULES = """You are AERIXIS Copilot, the mission planner of AERIXIS, a flood-rescue drone ground station.
You talk to the operator and fill in a mission form (the JSON "plan").

SCOPE: you ONLY help with AERIXIS drone missions, the drones, this ground station (GCS) and the
AERIXIS software (planning, parameters, roles, payload drops, geotags, safety, simulation). For any
other topic (general knowledge, coding, news, chit-chat beyond a greeting) reply in one short line
that you only help with drone missions and ask what mission to plan; keep status "need_info".

LANGUAGE: reply in the SAME language and script as the operator's last message. Hindi in Devanagari
-> Hindi in Devanagari. Hindi written in English letters (Hinglish, e.g. "mere paas 4 drone hai") ->
reply in Hinglish. English -> English. Short replies: 2-6 lines, no markdown tables.

DECIDE THE MISSION (first rule that matches):
1. Some drones must drop/deliver packages to people that other drones find -> "search_deliver".
   Set plan.roles for EVERY drone in FLEET: "search" (has a camera) or "delivery" (has packages > 0).
2. Drones should circle/orbit each person they find -> "search_orbit" (plan.orbit_drones = who orbits).
3. One drone geotags people and also drops a package itself -> "human_geotag_drop_resume".
4. Find people and record their positions (geotag) -> "human_geotag_resume".
5. Follow / track one person -> "human_track_rtl". Watch from a fixed place and orbit -> "human_orbit".
6. Just fly over the area, no camera -> "swarm_coverage".
7. Nothing fits -> "custom" with plan.custom_name and plan.features.
A saved custom mission listed below may be used by its id.

FILL THE REST:
- Use only drones, cameras and payloads listed in FLEET; never invent abilities.
- Put every parameter the operator gave into plan.params (use the PARAMETERS names). For the others
  choose good values and say which ones you chose. Stay inside each min..max.
- Fast complete search: lane spacing = the camera table value for the chosen height (no gaps); the
  operator's speed for speed_mps, delivery_speed and rtl_speed; approach_speed 1 (safe over people).
- Each geotag costs about 20-30 s. Warn if the work will not fit the drones' endurance or the time
  the operator wants.
- AREA must be loaded. If not, ask the operator to load a KML file, draw the area on the map or
  press "Use arena as search area" (simulation) - status "need_info".
- AI avoidance (plan.ai_drones): by default ONLY the drones that drop packages (role delivery, or
  the drop mission's drones); list others only if the operator asks for them.
- When values are missing, ask with concrete options, e.g. "Height: 15 m (default) / 20 m / 25 m?".
- Ask at most two questions, only for what you cannot decide. status "ready" when the area is loaded
  and the plan is complete, else "need_info". Always return the best plan so far.
- You only fill the form: the operator accepts the conditions, the planner makes the behaviour
  trees, the operator uploads and starts. Mention risks (landing on water with finish LAND,
  dropping near people, low endurance).

EXAMPLE (4 drones; drone2/drone3 carry packages; area loaded):
operator: "mere 4 drone hai, 2 search karein aur 2 packet drop karein, speed 2"
{"reply": "Theek hai! Mission: Search, geotag & deliver. drone0, drone1 search karenge (15 m height, 17 m lane gap), drone2, drone3 packet drop karenge. Speed 2 m/s, approach 1 m/s. Emergency mein RTL. Area loaded hai - plan ready.", "status": "ready", "plan": {"mission": "search_deliver", "params": {"altitude_m": 15, "spacing_m": 17, "speed_mps": 2, "approach_speed": 1, "delivery_speed": 2, "rtl_speed": 2}, "roles": {"drone0": "search", "drone1": "search", "drone2": "delivery", "drone3": "delivery"}, "emergency": "RTL", "ai_drones": ["drone2", "drone3"]}}"""


def find_server():
    """llama-server: a CUDA build if one runs here, else the Vulkan build. (2026-11-01: the
    published CUDA build needs glibc 2.38; Ubuntu 22.04 has 2.35, so this laptop uses Vulkan.)"""
    for kind in ('cuda', 'vulkan'):
        hits = sorted(glob.glob(str(LLM_DIR / 'runtime' / kind / '**' / 'llama-server'), recursive=True))
        if hits:
            return Path(hits[0]), kind
    return None, None


def gpu_device(server, env):
    """The NVIDIA GPU's device name for --device (the laptop also has an Intel GPU)."""
    try:
        out = subprocess.run([str(server), '--list-devices'], capture_output=True, text=True, timeout=30,
                             env=env, cwd=str(server.parent)).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    devices = [line.strip().split(':', 1)[0] for line in out.splitlines() if ':' in line and line.strip()[:1].isalpha()
               and not line.strip().startswith('Available')]
    for line in out.splitlines():
        if 'NVIDIA' in line and ':' in line:
            return line.strip().split(':', 1)[0]
    return devices[0] if devices else None


def find_model():
    hits = sorted(LLM_DIR.glob('models/*.gguf'))
    return hits[0] if hits else None


def cuda_library_path():
    """CUDA 13 runtime libraries that ship with PyTorch (~/.local/.../nvidia/cu13/lib): the CUDA
    build of llama.cpp needs libcudart/libcublas and this laptop has no CUDA toolkit."""
    hits = glob.glob(str(Path.home() / '.local/lib/python3*/site-packages/nvidia/cu13/lib'))
    return hits[0] if hits else ''


def footprint_table(fleet, altitudes=(10, 15, 20, 25, 30)):
    rows = []
    for d in fleet:
        cam = d.get('camera') or {}
        hfov = cam.get('hfov_deg')
        if not hfov:
            continue
        cells = ', '.join(f'{h} m -> {2 * h * math.tan(math.radians(hfov / 2)) / 1.1:.0f} m'
                          for h in altitudes)
        rows.append(f"{d.get('label')} (camera {hfov:g}x{cam.get('vfov_deg', 0):g} deg): max lane spacing without sweep at {cells}")
    return '\n'.join(rows) or 'no camera reported'


class LocalPlanner:
    def __init__(self):
        self.lock = threading.Lock()
        self.process = None
        self.kind = None
        self.started_at = None
        self.last = {}            # timing of the last answer
        self.error = ''

    # ---------- llama-server ----------
    def status(self):
        server, kind = find_server()
        model = find_model()
        with self.lock:
            healthy = self._healthy()
            running = healthy or (self.process is not None and self.process.poll() is None)
            return dict(installed=bool(server and model), server=str(server or ''), engine=kind, model=model.name if model else '',
                        running=running, ready=healthy, port=PORT, last=self.last, error=self.error)

    def _healthy(self):
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{PORT}/health', timeout=1.5) as r:
                return r.status == 200
        except (OSError, urllib.error.URLError):
            return False

    def ensure_running(self, wait_s=90):
        server, kind = find_server()
        model = find_model()
        if not (server and model):
            raise RuntimeError(f'The offline planner is not installed (llama.cpp and a .gguf model in {LLM_DIR})')
        with self.lock:
            adopt = (self.process is None or self.process.poll() is not None) and self._healthy()
            if adopt:
                # Still running from before a GCS restart: use it (one model in GPU memory, no reload).
                self.kind = self.kind or kind
            elif self.process is None or self.process.poll() is not None:
                env = dict(os.environ)
                lib = ':'.join(p for p in (str(server.parent), cuda_library_path(), env.get('LD_LIBRARY_PATH', '')) if p)
                env['LD_LIBRARY_PATH'] = lib
                device = gpu_device(server, env)
                log = open(LLM_DIR / 'llama-server.log', 'a')
                self.process = subprocess.Popen(
                    [str(server), '-m', str(model), '--host', '127.0.0.1', '--port', str(PORT),
                     '-ngl', '99', '-c', str(CONTEXT_TOKENS), '-np', '1', '--jinja', '-fa', 'on',
                     # Share the 6 GB GPU with Gazebo (2026-11-01: its 3D window crashed next to an
                     # 8k-context model): 8-bit attention cache, smaller work buffers.
                     '-ctk', 'q8_0', '-ctv', 'q8_0', '-ub', '256']
                    + (['--device', device] if device else []),
                    stdout=log, stderr=subprocess.STDOUT, env=env, cwd=str(server.parent))
                self.kind, self.started_at, self.error = kind, time.time(), ''
        if adopt:
            return
        deadline = time.time() + wait_s
        while time.time() < deadline:
            if self._healthy():
                self._warm_up()
                return
            if self.process.poll() is not None:
                self.error = f'llama-server stopped (exit {self.process.returncode}); see {LLM_DIR / "llama-server.log"}'
                raise RuntimeError(self.error)
            time.sleep(0.3)
        raise RuntimeError('The offline planner did not start in time')

    def _warm_up(self):
        """First requests after a start compile the GPU shaders (Vulkan): ~13 s for a long prompt
        (SITL 2026-11-01). Run a short and a long dummy prompt once, before the operator asks."""
        if getattr(self, 'warmed', None) == self.started_at:
            return
        for words in (8, 900):
            body = dict(prompt='drone mission plan ' * words, n_predict=4, cache_prompt=False)
            try:
                urllib.request.urlopen(urllib.request.Request(f'http://127.0.0.1:{PORT}/completion', data=json.dumps(body).encode(),
                                                              headers={'Content-Type': 'application/json'}), timeout=120).read()
            except (OSError, urllib.error.URLError):
                return
        self.warmed = self.started_at

    def stop(self):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()
            self.process = None

    # ---------- chat ----------
    @staticmethod
    def system_prompt(context):
        fleet = context.get('fleet') or []
        area = context.get('area') or {}
        lines = [RULES, '', f"MODE: {context.get('mode', 'unknown')} (simulation or real drones)", '', 'FLEET:']
        for d in fleet:
            cam = d.get('camera')
            pay = d.get('payload')
            lines.append(f"- {d.get('id')} \"{d.get('label')}\": camera {'%gx%g deg' % (cam.get('hfov_deg', 0), cam.get('vfov_deg', 0)) if cam else 'none'}, "
                         f"packages {pay.get('capacity', 0) if pay else 0}, endurance {d.get('endurance_min', '?')} min, "
                         f"{'online' if d.get('online') else 'offline'}")
        if not fleet:
            lines.append('- no drones registered yet (ask the operator to add drones on the Connect page)')
        lines += ['', 'AREA: ' + (f"loaded \"{area.get('name')}\", {area.get('area_ha', '?')} ha, about {area.get('width_m', '?')} x {area.get('height_m', '?')} m"
                                  if area.get('loaded') else 'not loaded'),
                  '', 'CAMERA (lane spacing):', footprint_table(fleet), '', 'BUILT-IN MISSIONS (id: what it does; parameters):']
        for m in context.get('missions') or []:
            lines.append(f"- {m.get('id')}: {m.get('title')} - {m.get('summary')}; params {', '.join(m.get('options') or [])}")
        saved = context.get('saved') or []
        if saved:
            lines.append('SAVED CUSTOM MISSIONS (use the id like a built-in): ' + '; '.join(f"{s.get('id')} \"{s.get('name')}\" ({s.get('summary')})" for s in saved))
        lines += ['', 'CUSTOM MISSION FEATURES: type coverage (survey lanes) | watch (stations) | track (follow one person); '
                  'detect (use camera), geotag, orbit, drop (release a package over the person), resume (continue after each person), '
                  'finish RTL|LAND, delivery_drones (search drones geotag, other drones deliver; needs coverage+geotag).',
                  '', 'PARAMETERS (name: meaning, default, min..max, current value):']
        for key, p in (context.get('params') or {}).items():
            lines.append(f"- {key}: {p.get('label')}, default {p.get('default')}, {p.get('min')}..{p.get('max')}, now {p.get('value')}")
        lines += ['', f"CURRENT FORM: mission {context.get('current_mission')}, roles {json.dumps(context.get('roles') or {})}, emergency {context.get('emergency', 'RTL')}",
                  '', 'Reply ONLY with the JSON object: {"reply": ..., "status": "need_info"|"ready", "plan": {...}}.']
        return '\n'.join(lines)

    def chat(self, messages, context):
        self.ensure_running()
        history = [dict(role=m['role'], content=m['content']) for m in messages
                   if m.get('role') in ('user', 'assistant') and isinstance(m.get('content'), str)][-8:]
        if history and history[-1]['role'] == 'user':
            # The facts that matter most, next to the question (a small model trusts the nearest text).
            history[-1]['content'] += '\n\n[GCS now: ' + state_note(context) + ']'
        body = dict(messages=[dict(role='system', content=self.system_prompt(context))] + history,
                    temperature=0.3, top_p=0.9, max_tokens=MAX_REPLY_TOKENS, cache_prompt=True,
                    response_format=dict(type='json_schema', json_schema=dict(name='mission_form', schema=ANSWER_SCHEMA)))
        t0 = time.time()
        req = urllib.request.Request(f'http://127.0.0.1:{PORT}/v1/chat/completions', data=json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=120) as r:
            out = json.loads(r.read())
        elapsed = time.time() - t0
        text = out['choices'][0]['message']['content']
        answer = json.loads(text)
        timings = out.get('timings') or {}
        self.last = dict(seconds=round(elapsed, 2), tokens=(out.get('usage') or {}).get('completion_tokens'),
                         tokens_per_s=round(timings.get('predicted_per_second') or 0, 1),
                         prompt_tokens_per_s=round(timings.get('prompt_per_second') or 0, 1),
                         cached_prompt_tokens=timings.get('cache_n'), engine=self.kind)
        plan, warnings = check_plan(answer.get('plan') or {}, context)
        status = answer.get('status') if answer.get('status') in ('ready', 'need_info') else 'need_info'
        if not (context.get('area') or {}).get('loaded'):
            status = 'need_info'
            warnings.append('No search area yet: load a KML file, draw the area on the map, or use the arena (simulation).')
        if not plan.get('mission'):
            status = 'need_info'
        return dict(reply=str(answer.get('reply') or ''), status=status, plan=plan, warnings=warnings,
                    raw=text, timing=self.last)


def state_note(context):
    area = context.get('area') or {}
    fleet = context.get('fleet') or []
    drones = ', '.join(f"{d.get('id')} camera {'yes' if d.get('camera') else 'no'} packages "
                       f"{(d.get('payload') or {}).get('capacity', 0)}" for d in fleet) or 'none'
    where = (f"search area LOADED ({area.get('name')}, {area.get('area_ha', '?')} ha)" if area.get('loaded')
             else 'search area NOT loaded yet')
    return f'{where}; drones: {drones}'


def lane_limit(context, altitude, roles):
    """Widest lane spacing every search camera covers without a sweep (10 % overlap), or None."""
    fleet = context.get('fleet') or []
    hfovs = [d['camera']['hfov_deg'] for d in fleet if d.get('camera') and d['camera'].get('hfov_deg')
             and roles.get(d.get('id'), 'search') == 'search']
    if not hfovs or not altitude:
        return None
    return 2 * altitude * math.tan(math.radians(min(hfovs) / 2)) / 1.1


def check_plan(plan, context):
    """Clamp and check the model's form against the real limits and drones. Returns (plan, warnings)."""
    warnings = []
    params_catalogue = context.get('params') or {}
    fleet = {d.get('id'): d for d in context.get('fleet') or []}
    missions = {m.get('id'): m for m in context.get('missions') or []} | {s.get('id'): s for s in context.get('saved') or []}
    mission = str(plan.get('mission') or '')
    out = dict(mission=mission, custom_name='', features=None, params={}, roles={}, orbit_drones=[],
               emergency=plan.get('emergency') if plan.get('emergency') in ENDINGS else 'RTL')
    if mission == 'custom':
        f = plan.get('features') or {}
        features = dict(camera=bool(f.get('detect')), payload=bool(f.get('drop')),
                        type=f.get('type') if f.get('type') in FEATURE_TYPES else 'coverage',
                        detect=bool(f.get('detect')), geotag=bool(f.get('geotag')), orbit=bool(f.get('orbit')),
                        drop=bool(f.get('drop')), resume=bool(f.get('resume')),
                        finish=f.get('finish') if f.get('finish') in ENDINGS else 'RTL', emergency=out['emergency'])
        if (features['geotag'] or features['orbit'] or features['drop']) and not features['detect']:
            features['detect'] = features['camera'] = True
            warnings.append('Geotag/orbit/drop need the camera: detection switched on.')
        delivery = bool(f.get('delivery_drones'))
        if delivery and not (features['type'] == 'coverage' and features['geotag']):
            delivery = False
            warnings.append('Delivery drones need a survey with geotags: delivery option removed.')
        out.update(features=features, custom_name=str(plan.get('custom_name') or 'AI mission')[:50], delivery_drones=delivery)
    elif mission and mission not in missions:
        warnings.append(f'Unknown mission "{mission}": choose one in the list.')
        out['mission'] = ''
    for key, value in (plan.get('params') or {}).items():
        spec = params_catalogue.get(key)
        if spec is None or not isinstance(value, (int, float)) or not math.isfinite(value):
            continue
        clamped = min(max(float(value), spec['min']), spec['max'])
        if clamped != value:
            warnings.append(f"{spec['label']}: {value:g} is outside {spec['min']:g}..{spec['max']:g}, set to {clamped:g}.")
        out['params'][key] = clamped
    for drone_id, role in (plan.get('roles') or {}).items():
        d = fleet.get(drone_id)
        if d is None or role not in ROLES:
            continue
        if role == 'delivery' and not ((d.get('payload') or {}).get('capacity')):
            warnings.append(f"{d.get('label')} has no packages: kept as a search drone.")
            role = 'search'
        if role == 'search' and not d.get('camera'):
            warnings.append(f"{d.get('label')} reports no camera.")
        out['roles'][drone_id] = role
    out['orbit_drones'] = [i for i in plan.get('orbit_drones') or [] if i in fleet]
    out['ai_drones'] = [i for i in plan.get('ai_drones') or [] if i in fleet]
    if 'ai_drones' not in plan:
        out['ai_drones'] = [i for i, r in out['roles'].items() if r == 'delivery']
    if out['mission'] == 'search_deliver' and 'delivery' not in out['roles'].values():
        out['mission'] = 'human_geotag_resume'
        warnings.append('No drone delivers packages: mission changed to Geotag & resume (every drone searches).')
    elif out['mission'] == 'search_deliver' and 'search' not in out['roles'].values():
        warnings.append('Search & deliver needs at least one search drone: choose one.')
    # Lane spacing against the camera footprint (exact, not left to the model's arithmetic).
    altitude = out['params'].get('altitude_m') or (params_catalogue.get('altitude_m') or {}).get('value')
    limit = lane_limit(context, altitude, out['roles'])
    spacing = out['params'].get('spacing_m')
    if limit and spacing:
        if spacing > limit + 0.5:
            warnings.append(f'Lanes {spacing:g} m apart are wider than the camera sees at {altitude:g} m '
                            f'({limit:.0f} m): the camera will sweep sideways to fill the gap.')
        elif spacing < 0.85 * limit:
            warnings.append(f'At {altitude:g} m the camera covers lanes up to {int(limit)} m apart: '
                            f'spacing {int(limit)} m would finish sooner with no gaps.')
    return out, warnings
