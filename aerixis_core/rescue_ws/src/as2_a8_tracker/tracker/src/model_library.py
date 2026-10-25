"""Detection model library on the onboard computer: board profile, models, speed, lock rules.

Used by the onboard panel (upload / delete / choose a model) and by the tracker launcher (which
model and classes to start with). Standard library only, plus detector_models.describe().

  board      what the onboard computer is: the operator picks it in the panel (Raspberry Pi 5, Pi 5
             + Hailo AI HAT+, Vicharak Axon, Jetson ...); the panel also reads the real hardware
             (CPU cores, clock, RAM, NPU / AI HAT present) and warns when the two disagree.
  budget     how much CPU AS2 takes right now, measured from /proc; the rest (minus the camera
             pipeline) is what the detector may use.
  speed      detection rate per model: measured on this board by the benchmark button, otherwise
             estimated from published figures (see ESTIMATE_SOURCES) scaled by input size, model
             size and detector threads.
  lock       a model is usable only when the board has its format's runtime and accelerator, and
             its detection rate and the tracking rate reach the panel minimums. Locked models
             cannot be made active.

Models live in LIBRARY (/data/models on the Pi image, one folder each); the shipped ones are in
the tracker's models/ folder and cannot be deleted. The active choice is LIBRARY/active.json.
"""
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import zipfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detector_models import FORMATS, IMPLEMENTED, describe  # noqa: E402

TRACKER_HOME = Path(__file__).resolve().parents[1]
BUILTIN_DIRS = [TRACKER_HOME / 'model', TRACKER_HOME / 'models']
DEFAULT_MODEL = 'person-n512-640x384'     # the tracker's model/ folder
NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,47}$')
MAX_UPLOAD_MB = 300
MAX_FILES = 64

# Board choices. base_ms: one inference of a ~2.6 M-parameter nano detector at 640x640, with 4 CPU
# threads (CPU formats) or the accelerator. Figures from published benchmarks (ESTIMATE_SOURCES),
# rounded towards slower; only a measurement on the board (benchmark) is a fact.
BOARDS = {
    'pi5': dict(label='Raspberry Pi 5 (CPU only)', accelerator=None, tops=0,
                formats=['ncnn', 'onnx'], base_ms=dict(ncnn=100, onnx=128),
                nano_ms=12, decode_ms_per_mpix=30),
    'pi5-hailo8l': dict(label='Raspberry Pi 5 + AI HAT+ 13 TOPS (Hailo-8L)', accelerator='hailo8l', tops=13,
                        formats=['hef', 'ncnn', 'onnx'], base_ms=dict(hef=14, ncnn=100, onnx=128),
                        nano_ms=12, decode_ms_per_mpix=30),
    'pi5-hailo8': dict(label='Raspberry Pi 5 + AI HAT+ 26 TOPS (Hailo-8)', accelerator='hailo8', tops=26,
                       formats=['hef', 'ncnn', 'onnx'], base_ms=dict(hef=8, ncnn=100, onnx=128),
                       nano_ms=12, decode_ms_per_mpix=30),
    'axon': dict(label='Vicharak Axon (RK3588, 6 TOPS NPU)', accelerator='rknpu', tops=6,
                 formats=['rknn', 'ncnn', 'onnx'], base_ms=dict(rknn=25, ncnn=85, onnx=110),
                 nano_ms=10, decode_ms_per_mpix=12),
    'jetson-nano': dict(label='NVIDIA Jetson Nano (Maxwell GPU, 0.5 TFLOPS)', accelerator='cuda', tops=0.5,
                        formats=['tensorrt', 'onnx', 'ncnn'], base_ms=dict(tensorrt=40, ncnn=400, onnx=450),
                        nano_ms=25, decode_ms_per_mpix=10),
    'jetson-orin-nano': dict(label='NVIDIA Jetson Orin Nano (40-67 TOPS)', accelerator='cuda', tops=40,
                             formats=['tensorrt', 'onnx', 'ncnn'], base_ms=dict(tensorrt=8, ncnn=90, onnx=110),
                             nano_ms=8, decode_ms_per_mpix=8),
    'generic': dict(label='Other Linux computer (CPU only)', accelerator=None, tops=0,
                    formats=['ncnn', 'onnx'], base_ms=dict(ncnn=100, onnx=128),
                    nano_ms=12, decode_ms_per_mpix=30),
}
ESTIMATE_SOURCES = [
    'Ultralytics Raspberry Pi guide: YOLO26n 640, Pi 5, ONNX ~128 ms (8.4.14)',
    'Raspberry Pi AI HAT+ tables: YOLOv8s 60-80 fps (13 TOPS), ~80 fps (26 TOPS); Pi 5 PCIe is x1',
    'airockchip rknn_model_zoo: yolo11n INT8 RK3588 single core; Python post-processing adds 10-30 ms',
    'Jetson Orin Nano Super: YOLOv8n TensorRT FP16 ~4.4 ms GPU only; end to end slower',
]
FORMAT_RUNTIME = dict(ncnn='ncnn', onnx='onnxruntime', rknn='rknn-toolkit-lite2', tensorrt='TensorRT',
                      hef='HailoRT')
ACCELERATOR_FORMAT = dict(hailo8l='hef', hailo8='hef', rknpu='rknn', cuda='tensorrt')
CAPTURE_SIZES = [(640, 360), (960, 540), (1280, 720)]
CAMERA_FPS = 25.0
# Speed minimums (2026-10-24, user's flight conditions): tracking 19-25 fps (the A8 stream gives 25),
# and a detection rate that grows with the drone's speed: between two detector checks the drone
# moves at most TARGET_MOVE_M (about one person width), so a fast pass cannot outrun the identity
# check. 5 m/s -> 10 Hz, 8 m/s -> 16 Hz; never below min_detect_hz.
DEFAULT_POLICY = dict(min_detect_hz=5.0, min_track_fps=19.0, speed_ms=5.0)
TARGET_MOVE_M = 0.5


def required_detect_hz(policy):
    policy = dict(DEFAULT_POLICY, **(policy or {}))
    return round(max(float(policy['min_detect_hz']), float(policy['speed_ms']) / TARGET_MOVE_M), 1)


def library_dir():
    return Path(os.environ.get('A8_MODEL_LIBRARY') or Path.home() / 'models')


# --------------------------------------------------------------------------- board
def _read(path, default=''):
    try:
        return Path(path).read_text(errors='replace').strip('\x00 \n')
    except OSError:
        return default


def _exists(path):
    try:
        return Path(path).exists()
    except OSError:          # e.g. debugfs without root
        return False


def detect_board():
    """What this computer really is (device tree, /proc, /dev), and the closest board choice."""
    model = _read('/proc/device-tree/model') or _read('/sys/firmware/devicetree/base/model')
    compatible = _read('/proc/device-tree/compatible').replace('\x00', ' ')
    cpuinfo = _read('/proc/cpuinfo')
    if not model:   # inside a container the device tree may be hidden; the Pi kernel also puts it here
        model = (re.search(r'^Model\s*:\s*(.+)$', cpuinfo, re.M) or [None, ''])[1].strip()
    meminfo = _read('/proc/meminfo')
    ram_kb = int((re.search(r'MemTotal:\s+(\d+)', meminfo) or [0, 0])[1])
    parts = sorted(set(re.findall(r'CPU part\s*:\s*(0x[0-9a-f]+)', cpuinfo)))
    part_names = {'0xd0b': 'Cortex-A76', '0xd05': 'Cortex-A55', '0xd08': 'Cortex-A72', '0xd03': 'Cortex-A53',
                  '0xd07': 'Cortex-A57', '0xd42': 'Cortex-A78AE', '0xd41': 'Cortex-A78'}
    max_khz = [int(v) for v in (_read(p) for p in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/cpuinfo_max_freq')) if v.isdigit()]
    name = (re.search(r'model name\s*:\s*(.+)', cpuinfo) or [None, ''])[1]
    temp = _read('/sys/class/thermal/thermal_zone0/temp')
    accelerators = []
    if _exists('/dev/hailo0'):
        arch = _hailo_arch()
        accelerators.append(dict(type=arch or 'hailo8l', label='Hailo AI HAT+ (%s)' % (arch or 'Hailo-8 family'),
                                 tops=26 if arch == 'hailo8' else 13, present=True, exact=bool(arch)))
    if _exists('/dev/rknpu') or _exists('/sys/kernel/debug/rknpu') or 'rk3588' in compatible:
        accelerators.append(dict(type='rknpu', label='Rockchip NPU (RK3588)', tops=6, present=True, exact=True))
    if _exists('/etc/nv_tegra_release') or 'nvidia,tegra' in compatible:
        accelerators.append(dict(type='cuda', label='NVIDIA GPU (Jetson)', tops=None, present=True, exact=True))
    lower = (model + ' ' + compatible).lower()
    if 'raspberry pi 5' in lower:
        guess = 'pi5-' + accelerators[0]['type'] if accelerators and accelerators[0]['type'].startswith('hailo') else 'pi5'
    elif 'rk3588' in lower or 'axon' in lower:
        guess = 'axon'
    elif 'orin' in lower:
        guess = 'jetson-orin-nano'
    elif 'jetson' in lower or 'tegra210' in lower:
        guess = 'jetson-nano'
    else:
        guess = 'generic'
    topology = {(_read(p / 'topology/physical_package_id'), _read(p / 'topology/core_id'))
                for p in Path('/sys/devices/system/cpu').glob('cpu[0-9]*')}
    return dict(model=model or name or os.uname().machine, compatible=compatible, guess=guess,
                cores=len(topology) if len(topology) > 1 else (os.cpu_count() or 1), threads=os.cpu_count() or 1,
                cpu=', '.join(part_names.get(p, p) for p in parts) or name or os.uname().machine,
                max_mhz=round(max(max_khz) / 1000) if max_khz else None,
                ram_mb=round(ram_kb / 1024), arch=os.uname().machine,
                temperature_c=round(int(temp) / 1000, 1) if temp.lstrip('-').isdigit() else None,
                accelerators=accelerators)


def _hailo_arch():
    try:
        out = subprocess.run(['hailortcli', 'fw-control', 'identify'], capture_output=True, text=True,
                             timeout=5).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    match = re.search(r'Device Architecture:\s*(HAILO8L|HAILO8)\b', out)
    return match.group(1).lower() if match else None


def board_warnings(choice, real):
    """Where the operator's board choice and the detected hardware disagree."""
    board = BOARDS[choice]
    found = {a['type'] for a in real['accelerators']}
    notes = []
    if board['accelerator'] and board['accelerator'] not in found:
        if board['accelerator'].startswith('hailo') and found & {'hailo8', 'hailo8l'}:
            notes.append('AI HAT found, but it reports %s' % '/'.join(sorted(found)))
        else:
            notes.append('%s not found on this computer: its models stay locked' % board['label'].split('(')[-1].rstrip(')'))
    if choice != real['guess'] and not (choice.startswith('pi5') and real['guess'].startswith('pi5')):
        notes.append('This computer looks like "%s"' % BOARDS[real['guess']]['label'])
    return notes


# --------------------------------------------------------------------------- CPU budget
def _jiffies():
    fields = _read('/proc/stat').splitlines()[0].split()[1:]
    values = [int(v) for v in fields]
    return sum(values), values[3] + (values[4] if len(values) > 4 else 0)


def _process_ticks():
    ticks = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / 'stat').read_text()
            cmd = (entry / 'cmdline').read_bytes().replace(b'\x00', b' ').decode(errors='replace')
        except OSError:
            continue
        fields = stat[stat.rfind(')') + 2:].split()
        ticks[entry.name] = (int(fields[11]) + int(fields[12]), cmd)
    return ticks


def cpu_budget(window_s=1.0):
    """Measure for window_s: total CPU, AS2/flight software, the camera tracker; cores left over."""
    hz = os.sysconf('SC_CLK_TCK')
    cores = os.cpu_count() or 1
    total0, idle0 = _jiffies()
    procs0 = _process_ticks()
    time.sleep(window_s)
    total1, idle1 = _jiffies()
    procs1 = _process_ticks()
    used_cores = cores * (1 - (idle1 - idle0) / max(1, total1 - total0))
    tracker = flight = 0.0
    for pid, (ticks, cmd) in procs1.items():
        if pid not in procs0:
            continue
        load = (ticks - procs0[pid][0]) / hz / window_s
        if 'hybrid_tracker' in cmd or 'a8_tracker' in cmd:
            tracker += load
        elif any(k in cmd for k in ('as2', 'ros2', 'rescue', 'MicroXRCE', 'mavlink', 'onboard_service', '/opt/ros')):
            flight += load
    # The detector may use what AS2 and the rest leave, minus the camera pipeline (video decode,
    # NanoTrack and the 50 Hz gimbal loop, about 0.6 of a core on a Pi 5) and a 0.25-core reserve.
    other = max(0.0, used_cores - flight - tracker)
    free = cores - flight - other - 0.6 - 0.25
    return dict(cores=cores, used_cores=round(used_cores, 2), as2_cores=round(flight, 2),
                tracker_cores=round(tracker, 2), other_cores=round(other, 2),
                # Measured on the Pi 5 with everything running (2026-10-25): 2 NCNN threads made
                # detection slower (median 177 ms, p90 590 ms: the threads spin-wait on each other on
                # busy cores); 1 thread at raised priority gave 71 ms / p90 86 ms at 25 fps. More
                # threads only when the budget shows whole free cores.
                detector_threads=int(max(1, min(4, math.floor(free + 1e-9)))),
                as2_percent=round(100 * flight / cores), window_s=window_s)


# --------------------------------------------------------------------------- models
def _model_dirs():
    seen = {}
    for root in BUILTIN_DIRS:
        if not root.is_dir():
            continue
        if (root / 'metadata.yaml').exists():
            seen[DEFAULT_MODEL] = (root, True)
            continue
        for folder in sorted(root.iterdir()):
            if folder.is_dir():
                seen.setdefault(folder.name, (folder, True))
    library = library_dir()
    if library.is_dir():
        for folder in sorted(library.iterdir()):
            if folder.is_dir() and not folder.name.startswith('.'):
                seen.setdefault(folder.name, (folder, False))
    return seen


def find_model(name):
    entry = _model_dirs().get(name)
    if entry is None:
        raise ValueError('No model named %s' % name)
    return entry


def _params_millions(info):
    bytes_per = dict(ncnn=2, onnx=4, rknn=1, hef=1, tensorrt=2)[info['format']]
    return max(0.3, info['size_mb'] * 1e6 / bytes_per / 1e6)


def capture_size(info):
    """Smallest camera frame size that fills the model input (no upscaling of the video)."""
    w, _ = info['input']
    return next(((cw, ch) for cw, ch in CAPTURE_SIZES if cw >= w * 0.98), CAPTURE_SIZES[-1])


def estimate(info, board, threads):
    """Estimated detector rate (Hz) and tracking rate (fps) of a model on a board."""
    b = BOARDS[board]
    fmt = info['format']
    base = b['base_ms'].get(fmt)
    if base is None:
        return None
    w, h = info['input']
    size = max(0.5, (_params_millions(info) / 2.6) ** 0.9)
    ms = base * (w * h / 409600.0) * size
    if fmt in ('ncnn', 'onnx'):
        ms *= (4.0 / max(1, threads)) ** 0.8
    cw, ch = capture_size(info)
    track_ms = b['nano_ms'] * (cw * ch / 230400.0) ** 0.5 + b['decode_ms_per_mpix'] * cw * ch / 1e6
    return dict(detect_ms=round(ms, 1), detect_hz=round(1000.0 / ms, 1),
                track_fps=round(min(CAMERA_FPS, 1000.0 / track_ms), 1), capture=[cw, ch], measured=False)


def _bench_file():
    return library_dir() / 'benchmarks.json'


def benchmarks():
    try:
        return json.loads(_bench_file().read_text())
    except (OSError, ValueError):
        return {}


def save_benchmark(name, board, result):
    data = benchmarks()
    data['%s@%s' % (name, board)] = dict(result, measured=True, t=time.time())
    library_dir().mkdir(parents=True, exist_ok=True)
    _atomic_write(_bench_file(), json.dumps(data, indent=1))


def runtimes(python=None):
    """Formats whose runtime is installed in the tracker's Python (it runs the detector)."""
    python = python or os.environ.get('A8_TRACKER_PYTHON') or sys.executable
    probe = ('import json,sys\nfrom pathlib import Path\nsys.path.insert(0,%r)\n'
             'from detector_models import FORMATS,runtime_available\n'
             'print(json.dumps({f:runtime_available(f) for f in FORMATS}))' % str(Path(__file__).resolve().parent))
    try:
        out = subprocess.run([python, '-c', probe], capture_output=True, text=True, timeout=60)
        return json.loads(out.stdout.strip().splitlines()[-1])
    except Exception:
        return {f: False for f in FORMATS}


def evaluate(info, board, threads, available, policy, measured=None):
    """Speed and lock decision for one model on the chosen board."""
    b = BOARDS[board]
    speed = measured or estimate(info, board, threads)
    reasons = []
    if info['format'] not in b['formats']:
        reasons.append('%s models do not run on %s' % (info['format'].upper(), b['label']))
    elif info['format'] not in IMPLEMENTED:
        reasons.append('%s inference is not in the AERIXIS tracker yet (needs this board to develop and test)'
                       % info['format'].upper())
    elif not available.get(info['format']):
        reasons.append('%s runtime (%s) is not installed on this computer'
                       % (info['format'].upper(), FORMAT_RUNTIME[info['format']]))
    if speed:
        need = required_detect_hz(policy)
        if speed['detect_hz'] < need:
            reasons.append('detection %.1f Hz is below the %.0f Hz needed at %.0f m/s'
                           % (speed['detect_hz'], need, float(policy['speed_ms'])))
        if speed['track_fps'] < policy['min_track_fps']:
            reasons.append('tracking %.0f fps is below %.0f fps' % (speed['track_fps'], policy['min_track_fps']))
    return dict(speed=speed, locked=bool(reasons), reasons=reasons)


def list_models(board, threads, available, policy=None):
    policy = dict(DEFAULT_POLICY, **(policy or {}))
    measured = benchmarks()
    active = load_active()
    models = []
    for name, (folder, builtin) in _model_dirs().items():
        try:
            info = describe(folder)
        except ValueError as exc:
            models.append(dict(name=name, builtin=builtin, error=str(exc), locked=True, reasons=[str(exc)]))
            continue
        info['name'] = name
        result = evaluate(info, board, threads, available, policy, measured.get('%s@%s' % (name, board)))
        models.append(dict(name=name, builtin=builtin, format=info['format'], input=list(info['input']),
                           names=info['names'], classes=len(info['names']), size_mb=info['size_mb'],
                           description=info['description'][:160], active=name == active['model'], **result))
    return models


# --------------------------------------------------------------------------- active model
def _active_file():
    return library_dir() / 'active.json'


def load_active():
    try:
        data = json.loads(_active_file().read_text())
    except (OSError, ValueError):
        data = {}
    out = dict(model=DEFAULT_MODEL, classes=[], lock_classes=[], board='', policy=dict(DEFAULT_POLICY), threads=0)
    out.update({k: v for k, v in data.items() if k in out})
    out['policy'] = dict(DEFAULT_POLICY, **(out['policy'] or {}))
    return out


def _check_policy(policy):
    policy = dict(DEFAULT_POLICY, **(policy or {}))
    for key in ('min_detect_hz', 'min_track_fps'):
        value = float(policy[key])
        if not 1 <= value <= 120:
            raise ValueError('%s must be 1..120' % key)
        policy[key] = value
    speed = float(policy['speed_ms'])
    if not 0 <= speed <= 25:
        raise ValueError('Drone speed while tracking must be 0..25 m/s')
    policy['speed_ms'] = speed
    return policy


def set_board(board, policy=None):
    """Store the operator's onboard-computer choice ('auto' = use the detected board) and the
    speed minimums (the model stays)."""
    if board == 'auto':
        board = ''
    if board and board not in BOARDS:
        raise ValueError('Unknown onboard computer: %s' % board)
    data = load_active()
    data.update(board=board, policy=_check_policy(policy or data['policy']), t=time.time())
    library_dir().mkdir(parents=True, exist_ok=True)
    _atomic_write(_active_file(), json.dumps(data, indent=1))
    return data


def set_active(name, classes, lock_classes, board, threads, available, policy=None):
    """Make a model active: only an unlocked model, with classes it really has."""
    if board not in BOARDS:
        raise ValueError('Choose the onboard computer first')
    policy = _check_policy(policy)
    folder, _ = find_model(name)
    info = describe(folder)
    info['name'] = name
    result = evaluate(info, board, threads, available, policy, benchmarks().get('%s@%s' % (name, board)))
    if result['locked']:
        raise ValueError('Model %s is locked on this board: %s' % (name, '; '.join(result['reasons'])))
    known = {v.lower() for v in info['names'].values()}
    for item in list(classes) + list(lock_classes):
        if str(item).lower() not in known:
            raise ValueError('Class "%s" is not in model %s' % (item, name))
    if classes and lock_classes and not {c.lower() for c in lock_classes} <= {c.lower() for c in classes}:
        raise ValueError('Lock classes must be among the detected classes')
    # board: evaluated on the given board, but the stored setting stays ('' = auto-detect).
    data = dict(model=name, classes=list(classes), lock_classes=list(lock_classes),
                board=load_active()['board'], policy=policy,
                threads=int(threads), t=time.time())
    library_dir().mkdir(parents=True, exist_ok=True)
    _atomic_write(_active_file(), json.dumps(data, indent=1))
    return data


def active_launch(fallback_dir):
    """(model folder, classes, lock classes, capture size, note) for the tracker launcher; the
    detector threads chosen with the model are in load_active()['threads'] (0 = launch default).
    An active model that is missing or broken falls back to the shipped person model, loudly."""
    active = load_active()
    try:
        folder, _ = find_model(active['model'])
        info = describe(folder)
        return folder, active['classes'], active['lock_classes'], capture_size(info), ''
    except ValueError as exc:
        return Path(fallback_dir), [], [], (640, 360), 'Active model %s unusable (%s): using the shipped person model' % (
            active['model'], exc)


# --------------------------------------------------------------------------- add / delete
def _atomic_write(path, text):
    tmp = Path(str(path) + '.tmp')
    tmp.write_text(text)
    os.replace(tmp, path)


def _yaml_names(names):
    return 'names:\n' + ''.join("  %d: %s\n" % (i, json.dumps(n)) for i, n in enumerate(names))


def add_model(name, filename, data, class_names=None, imgsz=None):
    """Store an uploaded model: a .zip of an export folder (NCNN folder with metadata.yaml), or one
    .onnx / .rknn / .engine / .hef file. class_names / imgsz fill a missing metadata.yaml."""
    if not NAME_RE.match(name or ''):
        raise ValueError('Name: letters, digits, . _ - only (max 48)')
    if name in _model_dirs():
        raise ValueError('A model named %s already exists' % name)
    if len(data) > MAX_UPLOAD_MB * 1e6:
        raise ValueError('Upload larger than %d MB' % MAX_UPLOAD_MB)
    library = library_dir()
    library.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(prefix='.incoming-', dir=str(library)))
    try:
        lower = filename.lower()
        if lower.endswith('.zip'):
            archive = work / 'upload.zip'
            archive.write_bytes(data)
            _safe_extract(archive, work / 'x')
            archive.unlink()
            folder = _model_folder(work / 'x')
        elif lower.endswith(('.onnx', '.rknn', '.engine', '.hef')):
            folder = work / 'x'
            folder.mkdir()
            (folder / ('model' + Path(lower).suffix)).write_bytes(data)
        elif lower.endswith(('.pt', '.pth')):
            raise ValueError('PyTorch .pt cannot run here: export it first, e.g. '
                             'yolo export model=best.pt format=ncnn imgsz=384,640 half=True, then zip the folder')
        else:
            raise ValueError('Upload a .zip of the exported model folder, or a .onnx / .rknn / .engine / .hef file')
        meta = folder / 'metadata.yaml'
        if class_names or imgsz:
            text = meta.read_text() if meta.exists() else 'task: detect\n'
            if imgsz:
                h, w = int(imgsz[0]), int(imgsz[1])
                if not (32 <= h <= 2048 and 32 <= w <= 2048 and h % 32 == 0 and w % 32 == 0):
                    raise ValueError('Input size must be multiples of 32 between 32 and 2048')
                text = re.sub(r'(?ms)^imgsz:.*?(?=^\S|\Z)', '', text) + 'imgsz:\n- %d\n- %d\n' % (h, w)
            if class_names:
                text = re.sub(r'(?ms)^names:.*?(?=^\S|\Z)', '', text) + _yaml_names(class_names)
            meta.write_text(text)
        describe(folder)                      # refuse anything the tracker could not load
        target = library / name
        shutil.move(str(folder), str(target))
        return target
    finally:
        shutil.rmtree(work, ignore_errors=True)


def _safe_extract(archive, target):
    with zipfile.ZipFile(archive) as z:
        members = [m for m in z.infolist() if not m.is_dir()]
        if len(members) > MAX_FILES:
            raise ValueError('Too many files in the zip (max %d)' % MAX_FILES)
        if sum(m.file_size for m in members) > MAX_UPLOAD_MB * 1e6:
            raise ValueError('Zip content larger than %d MB' % MAX_UPLOAD_MB)
        root = target.resolve()
        for m in members:
            path = (target / m.filename).resolve()
            if root not in path.parents or m.filename.startswith('/') or '..' in Path(m.filename).parts:
                raise ValueError('Unsafe path in zip: %s' % m.filename)
            if (m.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError('Links are not allowed in the zip')
        for m in members:
            if Path(m.filename).name.startswith('.') or '__MACOSX' in m.filename:
                continue
            z.extract(m, target)


def _model_folder(root):
    """The folder in an extracted zip that holds the model files (the export folder may be nested)."""
    marks = ('model.ncnn.param', '.onnx', '.rknn', '.engine', '.hef')
    candidates = sorted({p.parent for p in root.rglob('*') if p.is_file() and p.name.endswith(marks)})
    if not candidates:
        raise ValueError('No model file in the zip (NCNN param+bin, .onnx, .rknn, .engine or .hef)')
    if len(candidates) > 1:
        raise ValueError('The zip holds several model folders: upload one model at a time')
    return candidates[0]


def delete_model(name):
    folder, builtin = find_model(name)
    if builtin:
        raise ValueError('%s is shipped with the tracker and cannot be deleted' % name)
    if load_active()['model'] == name:
        raise ValueError('%s is the active model: choose another model first' % name)
    shutil.rmtree(folder)
    data = benchmarks()
    for key in [k for k in data if k.startswith(name + '@')]:
        del data[key]
    if _bench_file().exists():
        _atomic_write(_bench_file(), json.dumps(data, indent=1))


def run_benchmark(name, board, threads, cores=None, runs=20, python=None, timeout=240):
    """Time the model on this board in the tracker's Python (synthetic camera frames)."""
    folder, _ = find_model(name)
    python = python or os.environ.get('A8_TRACKER_PYTHON') or sys.executable
    command = [python, str(Path(__file__).resolve().parent / 'model_bench.py'), str(folder),
               '--threads', str(threads), '--runs', str(runs)]
    if cores:
        command += ['--cores'] + [str(c) for c in cores]
    out = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
    lines = [l for l in out.stdout.splitlines() if l.startswith('{')]
    if out.returncode != 0 or not lines:
        raise RuntimeError((out.stderr or out.stdout).strip().splitlines()[-1] if (out.stderr or out.stdout).strip()
                           else 'benchmark failed')
    result = json.loads(lines[-1])
    info = describe(folder)
    est = estimate(info, board, threads) or {}
    result.update(track_fps=est.get('track_fps', CAMERA_FPS), capture=list(capture_size(info)), threads=threads)
    save_benchmark(name, board, result)
    return result
