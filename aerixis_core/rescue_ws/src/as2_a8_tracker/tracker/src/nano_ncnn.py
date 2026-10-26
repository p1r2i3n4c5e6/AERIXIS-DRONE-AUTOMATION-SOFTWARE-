"""NanoTrack on ncnn: same algorithm and numbers as OpenCV 4.10's cv::TrackerNano, ~3x faster.

OpenCV runs the two NanoTrack networks on its generic DNN engine. On the Pi 5 with everything
running (2026-10-26) one update took ~43 ms there and ~15 ms on ncnn (fp16, 1 thread), which caps
the tracker at ~13-17 fps while a person is locked. This is a line-by-line port of
modules/video/src/tracking/tracker_nano.cpp (OpenCV 4.10.0) around the same ONNX models converted
with pnnx (outputs equal to 1e-5). Quirks are kept on purpose, because the lock thresholds
(score_min, hold_score) were tuned on OpenCV's scores:
  * the scale penalty divides by sizeCal(targetPos), the target's position, not its size;
  * the search crop is exactly 2x the template (integer 255 / 127);
  * the returned box is truncated to int like cv::Rect.
Same interface as cv2.TrackerNano: init(frame, (x, y, w, h)), update(frame) -> (ok, box),
getTrackingScore().
"""
import math
from pathlib import Path

import cv2
import numpy as np

EXEMPLAR = 127
INSTANCE = 255
STRIDE = 16
SCORE_SIZE = (INSTANCE - EXEMPLAR) // STRIDE + 8      # 16
WINDOW_INFLUENCE = 0.455
LR = 0.37
CONTEXT = 0.5
PENALTY_K = 0.055


def model_files(model_dir):
    nano = Path(model_dir) / 'nano'
    return [nano / ('%s.ncnn.%s' % (name, ext))
            for name in ('nanotrack_backbone_sim', 'nanotrack_head_sim') for ext in ('param', 'bin')]


def available(model_dir):
    try:
        import ncnn  # noqa: F401
    except ImportError:
        return False
    return all(path.is_file() for path in model_files(model_dir))


def _size(w, h):
    pad = (w + h) * 0.5
    return np.sqrt((w + pad) * (h + pad))


def _reciprocal_max(values):
    return np.maximum(values, 1.0 / values)


class NanoTrackNcnn:
    def __init__(self, model_dir, threads=1, fp16=False, head_fp16=False):
        import ncnn
        self._ncnn = ncnn
        backbone_param, backbone_bin, head_param, head_bin = map(str, model_files(model_dir))
        self.backbone = self._load(backbone_param, backbone_bin, threads, fp16)
        self.head = self._load(head_param, head_bin, threads, head_fp16)
        self.window = cv2.createHanningWindow((SCORE_SIZE, SCORE_SIZE), cv2.CV_32F)
        offsets = (np.arange(SCORE_SIZE, dtype=np.float32) - SCORE_SIZE // 2) * STRIDE + INSTANCE // 2
        self.grid_x, self.grid_y = np.meshgrid(offsets, offsets)
        self.zf = None
        self.target_pos = [0.0, 0.0]
        self.target_sz = [0.0, 0.0]
        self.image_size = (0, 0)
        self.tracking_score = 0.0
        self.last_ms = 0.0

    def _load(self, param, weights, threads, fp16):
        net = self._ncnn.Net()
        net.opt.num_threads = int(threads)
        net.opt.use_fp16_storage = net.opt.use_fp16_arithmetic = net.opt.use_fp16_packed = bool(fp16)
        if net.load_param(param) != 0 or net.load_model(weights) != 0:
            raise RuntimeError('NanoTrack ncnn model failed to load: ' + param)
        return net

    def _crop(self, image, original, resize):
        """getSubwindow: square crop around target_pos, padded with the image mean, resized."""
        h, w = image.shape[:2]
        c = (original + 1) // 2
        xmin = int(self.target_pos[0]) - c
        ymin = int(self.target_pos[1]) - c
        xmax, ymax = xmin + original - 1, ymin + original - 1
        left, top = max(0, -xmin), max(0, -ymin)
        right, bottom = max(0, xmax - w + 1), max(0, ymax - h + 1)
        if left or top or right or bottom:
            mean = cv2.mean(image)
            image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=mean)
            xmin, ymin, xmax, ymax = xmin + left, ymin + top, xmax + left, ymax + top
        crop = image[ymin:ymax + 1, xmin:xmax + 1]
        return cv2.resize(crop, (resize, resize))

    def _features(self, crop):
        # blobFromImage(crop, 1.0, swapRB=True): RGB, 0..255, CHW.
        mat = self._ncnn.Mat.from_pixels(np.ascontiguousarray(crop), self._ncnn.Mat.PixelType.PIXEL_BGR2RGB,
                                        crop.shape[1], crop.shape[0])
        extractor = self.backbone.create_extractor()
        extractor.input('in0', mat)
        return extractor.extract('out0')[1]

    def init(self, image, box):
        x, y, w, h = [int(v) for v in box]
        self.target_pos = [x + w * 0.5, y + h * 0.5]
        self.target_sz = [float(w), float(h)]
        self.image_size = (image.shape[1], image.shape[0])
        total = self.target_sz[0] + self.target_sz[1]
        side = int(math.sqrt((self.target_sz[0] + CONTEXT * total) * (self.target_sz[1] + CONTEXT * total)))
        self.zf = self._features(self._crop(image, side, EXEMPLAR))

    def update(self, image):
        if self.zf is None:
            return False, (0, 0, 0, 0)
        total = int(self.target_sz[0] + self.target_sz[1])
        side = math.sqrt((self.target_sz[0] + CONTEXT * total) * (self.target_sz[1] + CONTEXT * total))
        scale_z = EXEMPLAR / side
        search = side * (INSTANCE // EXEMPLAR)
        size_w, size_h = self.target_sz[0] * scale_z, self.target_sz[1] * scale_z
        xf = self._features(self._crop(image, int(search), INSTANCE))
        extractor = self.head.create_extractor()
        extractor.input('in0', self.zf)
        extractor.input('in1', xf)
        cls = np.array(extractor.extract('out0')[1], dtype=np.float32).reshape(2, SCORE_SIZE, SCORE_SIZE)
        bbox = np.array(extractor.extract('out1')[1], dtype=np.float32).reshape(4, SCORE_SIZE, SCORE_SIZE)
        top = np.maximum(cls[0], cls[1])
        e0, e1 = np.exp(cls[0] - top), np.exp(cls[1] - top)
        score = e1 / (e0 + e1)
        x1, y1 = self.grid_x - bbox[0], self.grid_y - bbox[1]
        x2, y2 = self.grid_x + bbox[2], self.grid_y + bbox[3]
        with np.errstate(divide='ignore', invalid='ignore'):
            sc = _reciprocal_max(_size(x2 - x1, y2 - y1) / _size(self.target_pos[0], self.target_pos[1]))
            rc = _reciprocal_max((size_w / size_h) / ((x2 - x1) / (y2 - y1)))
        penalty = np.exp(-(rc * sc - 1) * PENALTY_K)
        pscore = penalty * score * (1.0 - WINDOW_INFLUENCE) + self.window * WINDOW_INFLUENCE
        best = np.unravel_index(int(np.argmax(pscore)), pscore.shape)
        self.tracking_score = float(pscore[best])
        pred_x, pred_y = (x1[best] + x2[best]) / 2, (y1[best] + y2[best]) / 2
        pred_w, pred_h = (x2[best] - x1[best]) / scale_z, (y2[best] - y1[best]) / scale_z
        lr = float(penalty[best] * score[best] * LR)
        res_x = self.target_pos[0] + (pred_x - INSTANCE // 2) / scale_z
        res_y = self.target_pos[1] + (pred_y - INSTANCE // 2) / scale_z
        res_w = pred_w * lr + (1 - lr) * (size_w / scale_z)
        res_h = pred_h * lr + (1 - lr) * (size_h / scale_z)
        width, height = self.image_size
        res_x = max(0.0, min(float(width), float(res_x)))
        res_y = max(0.0, min(float(height), float(res_y)))
        res_w = max(10.0, min(float(width), float(res_w)))
        res_h = max(10.0, min(float(height), float(res_h)))
        self.target_pos = [res_x, res_y]
        self.target_sz = [res_w, res_h]
        return True, (int(res_x - res_w / 2), int(res_y - res_h / 2), int(res_w), int(res_h))

    def getTrackingScore(self):
        return self.tracking_score
