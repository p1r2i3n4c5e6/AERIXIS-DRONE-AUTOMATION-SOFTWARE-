"""Detection models for the A8 tracker: any YOLO-style detector, any classes, several formats.

A model is a folder. What it is comes from its files and an Ultralytics-style metadata.yaml
(imgsz, names, task):

  ncnn      model.ncnn.param + model.ncnn.bin           CPU (Pi 5, Axon, Jetson)
  onnx      *.onnx                                      CPU via onnxruntime
  rknn      *.rknn                                      Rockchip NPU (Vicharak Axon / RK3588), rknnlite
  tensorrt  *.engine                                    NVIDIA Jetson, TensorRT
  hef       *.hef                                       Hailo AI HAT (Pi 5), hailo_platform

Every format gives the same YOLO output, decoded here once: (4 + classes) x anchors, boxes as
centre x/y, width, height in model-input pixels, then one score per class (YOLO26/YOLO11/YOLOv8
detect heads, end2end off). The board-specific runtimes are loaded only when their library is
installed; without it the model cannot be selected (the onboard panel locks it).
"""
import math
import time
from pathlib import Path

import numpy as np

FORMATS = ('ncnn', 'onnx', 'rknn', 'tensorrt', 'hef')
IMPLEMENTED = ('ncnn', 'onnx', 'rknn')   # HEF / TensorRT: format recognised, inference still to write
# Rough real heights (m) for a distance estimate from the box height; other classes: no estimate.
REAL_HEIGHT_M = {'person': 1.7, 'dog': 0.6, 'cat': 0.3, 'horse': 1.6, 'cow': 1.5, 'sheep': 0.8,
                 'car': 1.5, 'truck': 3.0, 'bus': 3.0, 'motorcycle': 1.1, 'bicycle': 1.1, 'boat': 1.5}
PERSON_MIN_ASPECT, PERSON_MAX_ASPECT = 0.85, 5.0     # box height / width for a person
OTHER_MIN_SIDE_PX = 8.0                              # smallest box side kept for other classes


# --------------------------------------------------------------------------- model description
def _parse_metadata(text):
    """The few keys we need from an Ultralytics metadata.yaml (no YAML library needed)."""
    info, lines, i = {}, text.splitlines(), 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        if stripped.startswith('imgsz:'):
            rest = stripped[len('imgsz:'):].strip()
            if rest:
                info['imgsz'] = [int(v) for v in rest.strip('[]').split(',') if v.strip()]
            else:
                values = []
                while i + 1 < len(lines) and lines[i + 1].strip().startswith('-'):
                    i += 1
                    values.append(int(lines[i].strip().lstrip('-').strip()))
                info['imgsz'] = values
        elif stripped == 'names:':
            names = {}
            while i + 1 < len(lines) and lines[i + 1].startswith((' ', '\t')) and ':' in lines[i + 1]:
                i += 1
                key, _, value = lines[i].strip().partition(':')
                try:
                    names[int(key)] = value.strip().strip('\'"')
                except ValueError:
                    break
            info['names'] = names
        elif stripped.startswith(('task:', 'description:', 'date:', 'version:')):
            key, _, value = stripped.partition(':')
            info[key] = value.strip().strip('\'"')
        i += 1
    return info


def describe(model_dir):
    """What a model folder contains: dict(name, format, file, input=(w, h), names, task).
    Raises ValueError when the folder is not a usable model."""
    folder = Path(model_dir)
    if not folder.is_dir():
        raise ValueError('Model folder not found: %s' % folder)
    meta = {}
    if (folder / 'metadata.yaml').exists():
        meta = _parse_metadata((folder / 'metadata.yaml').read_text(errors='replace'))
    found = []
    if (folder / 'model.ncnn.param').exists() and (folder / 'model.ncnn.bin').exists():
        found.append(('ncnn', folder / 'model.ncnn.param'))
    for pattern, fmt in (('*.onnx', 'onnx'), ('*.rknn', 'rknn'), ('*.engine', 'tensorrt'), ('*.hef', 'hef')):
        found += [(fmt, f) for f in sorted(folder.glob(pattern))]
    if not found:
        raise ValueError('No model file in %s (expected NCNN param+bin, .onnx, .rknn, .engine or .hef)' % folder)
    if len(found) > 1:
        raise ValueError('More than one model file in %s: keep one model per folder' % folder)
    fmt, path = found[0]
    if fmt == 'onnx' and not (meta.get('imgsz') and meta.get('names')):
        meta = dict(_onnx_metadata(path), **meta)   # Ultralytics stores both inside the .onnx
    size = meta.get('imgsz') or []
    if len(size) == 1:
        size = [size[0], size[0]]
    if len(size) != 2:
            raise ValueError('Input size unknown: add metadata.yaml with imgsz (height, width) to %s' % folder)
    names = meta.get('names') or {}
    if not names:
        raise ValueError('Class names unknown: add metadata.yaml with names to %s' % folder)
    task = meta.get('task', 'detect')
    if task != 'detect':
        raise ValueError('Only detection models are supported (task: %s)' % task)
    return dict(name=folder.name, format=fmt, file=str(path), input=(int(size[1]), int(size[0])),
                names={int(k): str(v) for k, v in names.items()}, task=task,
                description=meta.get('description', ''),
                size_mb=round(_model_mb(fmt, path), 2))


def _model_mb(fmt, path):
    """Size of the weights alone (an NCNN folder may also hold other files, e.g. NanoTrack)."""
    path = Path(path)
    if fmt == 'ncnn':
        path = path.with_suffix('.bin')
    return path.stat().st_size / 1e6


def _onnx_metadata(path):
    """imgsz and names from an Ultralytics ONNX export (custom metadata), else the input shape."""
    info = {}
    try:
        import ast
        import onnxruntime
        session = onnxruntime.InferenceSession(str(path), providers=['CPUExecutionProvider'])
        custom = session.get_modelmeta().custom_metadata_map
        if 'names' in custom:
            info['names'] = {int(k): str(v) for k, v in ast.literal_eval(custom['names']).items()}
        if 'imgsz' in custom:
            info['imgsz'] = [int(v) for v in ast.literal_eval(custom['imgsz'])]
        shape = session.get_inputs()[0].shape
        if 'imgsz' not in info and isinstance(shape[2], int) and isinstance(shape[3], int):
            info['imgsz'] = [shape[2], shape[3]]
        if custom.get('task'):
            info['task'] = custom['task']
    except Exception:
        pass
    return info


def runtime_available(fmt):
    """True when this board can run the format (its runtime library is installed)."""
    modules = {'ncnn': 'ncnn', 'onnx': 'onnxruntime', 'rknn': 'rknnlite.api', 'tensorrt': 'tensorrt',
               'hef': 'hailo_platform'}
    try:
        __import__(modules[fmt])
        return True
    except Exception:
        return False


# --------------------------------------------------------------------------- shared decoding
def letterbox(img, size):
    """Fit img into the model input (width, height) with grey bars; returns padded, scale, dx, dy."""
    import cv2   # only the tracker needs OpenCV; the onboard panel imports describe() without it
    size_w, size_h = size
    h, w = img.shape[:2]
    r = min(size_h / h, size_w / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    dw, dh = (size_w - nw) // 2, (size_h - nh) // 2
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR) if (nw, nh) != (w, h) else img
    padded = cv2.copyMakeBorder(resized, dh, size_h - nh - dh, dw, size_w - nw - dw,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return padded, r, dw, dh


def nms(boxes, scores, iou_thresh):
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_thresh]
    return keep


class Detector:
    """Common part of every format: letterbox, run, decode, filter, class-aware NMS.

    detect() returns (dets, labels, latency_ms): dets = [[x1, y1, x2, y2, conf]] in frame pixels,
    labels = class ids. classes: the class ids to keep (None = all)."""

    def __init__(self, info, threads=2, min_person_height=24.0, classes=None, iou_thresh=0.45):
        if not math.isfinite(min_person_height) or min_person_height < 8:
            raise ValueError('Minimum person height must be at least 8 pixels')
        self.info = info
        self.input_size = tuple(info['input'])
        self.names = info['names']
        self.person_ids = {k for k, v in self.names.items() if v.lower() == 'person'}
        wanted = set(self.names) if classes is None else {int(c) for c in classes}
        unknown = wanted - set(self.names)
        if unknown or not wanted:
            raise ValueError('Classes %s are not in this model (%s)' % (sorted(unknown) or '[]', self.names))
        self.classes = wanted
        self.threads, self.min_person_height, self.iou_thresh = threads, float(min_person_height), iou_thresh
        self.last_profile = {}

    # Implemented per format: model input (BGR uint8, input_size) -> raw output array.
    def run(self, img):
        raise NotImplementedError

    def detect(self, img_bgr, conf_thresh=0.5):
        started = time.perf_counter()
        orig_h, orig_w = img_bgr.shape[:2]
        img, r, dw, dh = letterbox(img_bgr, self.input_size)
        t0 = time.perf_counter()
        out = np.asarray(self.run(img), dtype=np.float32)
        latency_ms = (time.perf_counter() - t0) * 1000.0
        dets, labels, stats = self.decode(out, r, dw, dh, orig_w, orig_h, conf_thresh)
        stats.update(pre_ms=(t0 - started) * 1000.0, infer_ms=latency_ms,
                     detect_ms=(time.perf_counter() - started) * 1000.0, kept_detections=len(dets))
        self.last_profile = stats
        return dets, labels, latency_ms

    def decode(self, out, r, dw, dh, orig_w, orig_h, conf_thresh):
        stats = dict(confident_candidates=0, height_rejected=0, aspect_rejected=0, geometry_rejected=0,
                     candidate_h_min=0.0, candidate_h_max=0.0, top_score=0.0)
        rows = 4 + len(self.names)
        out = np.squeeze(out)
        if out.ndim != 2 or out.size == 0:
            return [], [], stats
        if out.shape[0] != rows and out.shape[1] == rows:
            out = out.T
        if out.shape[0] != rows:
            raise RuntimeError('Model output %s does not match %d classes' % (out.shape, len(self.names)))
        scores = out[4:]
        cls = np.argmax(scores, axis=0)
        conf = scores[cls, np.arange(scores.shape[1])]
        finite = conf[np.isfinite(conf)]
        stats['top_score'] = float(finite.max()) if finite.size else 0.0
        keep = (conf >= conf_thresh) & np.isin(cls, list(self.classes))
        stats['confident_candidates'] = int(np.count_nonzero(keep))
        if not keep.any():
            return [], [], stats
        cx, cy, w, h = out[0][keep], out[1][keep], out[2][keep], out[3][keep]
        conf, cls = conf[keep], cls[keep]
        x1 = np.clip((cx - w / 2 - dw) / r, 0, orig_w)
        y1 = np.clip((cy - h / 2 - dh) / r, 0, orig_h)
        x2 = np.clip((cx + w / 2 - dw) / r, 0, orig_w)
        y2 = np.clip((cy + h / 2 - dh) / r, 0, orig_h)
        bw, bh = np.maximum(1.0, x2 - x1), np.maximum(1.0, y2 - y1)
        person = np.isin(cls, list(self.person_ids))
        aspect = bh / bw
        aspect_ok = ~person | ((aspect >= PERSON_MIN_ASPECT) & (aspect <= PERSON_MAX_ASPECT))
        height_ok = np.where(person, bh >= self.min_person_height, np.minimum(bw, bh) >= OTHER_MIN_SIDE_PX)
        stats.update(height_rejected=int(np.count_nonzero(~height_ok)), aspect_rejected=int(np.count_nonzero(~aspect_ok)),
                     candidate_h_min=float(bh.min()), candidate_h_max=float(bh.max()))
        valid = aspect_ok & height_ok
        stats['geometry_rejected'] = int(np.count_nonzero(~valid))
        if not valid.any():
            return [], [], stats
        x1, y1, x2, y2, conf, cls = x1[valid], y1[valid], x2[valid], y2[valid], conf[valid], cls[valid]
        # Class-aware NMS: boxes of different classes never suppress each other.
        offset = cls.astype(np.float32)[:, None] * 10000.0
        order = nms(np.stack([x1, y1, x2, y2], 1) + offset, conf, self.iou_thresh)
        dets = [[float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]), float(conf[i])] for i in order]
        return dets, [int(cls[i]) for i in order], stats


class NCNNDetector(Detector):
    def __init__(self, info, threads=2, **kw):
        super().__init__(info, threads, **kw)
        import ncnn
        self.ncnn = ncnn
        self.net = ncnn.Net()
        for option in ('use_fp16_packed', 'use_fp16_storage', 'use_fp16_arithmetic', 'use_packing_layout'):
            setattr(self.net.opt, option, True)
        self.net.opt.num_threads = threads
        folder = Path(info['file']).parent
        if self.net.load_param(str(folder / 'model.ncnn.param')) != 0:
            raise RuntimeError('NCNN could not load %s' % folder)
        if self.net.load_model(str(folder / 'model.ncnn.bin')) != 0:
            raise RuntimeError('NCNN could not load weights in %s' % folder)
        self.in_name, self.out_name = self.net.input_names()[0], self.net.output_names()[0]

    @staticmethod
    def make_tensor(img):
        """RGB planes scaled to 0..1, packed by NCNN straight from the BGR image (no float copy)."""
        import ncnn
        h, w = img.shape[:2]
        tensor = ncnn.Mat.from_pixels(img, ncnn.Mat.PixelType.PIXEL_BGR2RGB, w, h)
        tensor.substract_mean_normalize([], [1.0 / 255.0] * 3)
        return tensor

    def run(self, img):
        tensor = self.make_tensor(img)
        with self.net.create_extractor() as ex:
            if ex.input(self.in_name, tensor) != 0:
                raise RuntimeError('NCNN input failed')
            status, out = ex.extract(self.out_name)
            if status != 0:
                raise RuntimeError('NCNN extraction failed: %s' % status)
        return np.array(out)


class ONNXDetector(Detector):
    def __init__(self, info, threads=2, **kw):
        super().__init__(info, threads, **kw)
        import onnxruntime
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        self.session = onnxruntime.InferenceSession(info['file'], options, providers=['CPUExecutionProvider'])
        self.in_name = self.session.get_inputs()[0].name

    def run(self, img):
        blob = img[:, :, ::-1].astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        return self.session.run(None, {self.in_name: np.ascontiguousarray(blob)})[0]


class RKNNDetector(Detector):
    """Rockchip NPU (Vicharak Axon / RK3588) via rknn-toolkit-lite2."""

    def __init__(self, info, threads=2, **kw):
        super().__init__(info, threads, **kw)
        from rknnlite.api import RKNNLite
        self.rknn = RKNNLite()
        if self.rknn.load_rknn(info['file']) != 0 or self.rknn.init_runtime() != 0:
            raise RuntimeError('RKNN runtime could not load %s' % info['file'])

    def run(self, img):
        return self.rknn.inference(inputs=[img[:, :, ::-1][None]])[0]


def make_detector(model_dir, threads=2, min_person_height=24.0, classes=None):
    """Detector for a model folder, or a clear error why it cannot run on this board."""
    info = describe(model_dir)
    if not runtime_available(info['format']):
        raise RuntimeError('%s models need their runtime on this board (%s)' % (
            info['format'], {'onnx': 'onnxruntime', 'rknn': 'rknn-toolkit-lite2 (Rockchip NPU)',
                             'tensorrt': 'TensorRT (Jetson)', 'hef': 'HailoRT (Hailo AI HAT)',
                             'ncnn': 'ncnn'}[info['format']]))
    kinds = {'ncnn': NCNNDetector, 'onnx': ONNXDetector, 'rknn': RKNNDetector}
    if info['format'] not in kinds:
        raise RuntimeError('%s inference is not implemented yet in this tracker (needs the board to test)' % info['format'])
    return kinds[info['format']](info, threads, min_person_height=min_person_height, classes=classes)


def class_ids(info, names):
    """Class ids for a list of class names (case-insensitive) or ids; None = all classes."""
    if not names:
        return None
    by_name = {v.lower(): k for k, v in info['names'].items()}
    ids = []
    for item in names:
        item = str(item).strip()
        if not item:
            continue
        if item.isdigit():
            ids.append(int(item))
        elif item.lower() in by_name:
            ids.append(by_name[item.lower()])
        else:
            raise ValueError('Class "%s" is not in this model (%s)' % (item, ', '.join(info['names'].values())))
    return ids or None
