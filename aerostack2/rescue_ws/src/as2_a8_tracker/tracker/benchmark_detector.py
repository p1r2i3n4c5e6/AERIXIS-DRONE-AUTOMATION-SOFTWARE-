"""Offline, two-thread A/B benchmark. Does not connect to or move a camera."""
import os
os.environ.setdefault("OMP_NUM_THREADS", "2")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import argparse
import json
import platform
import sys
import time
from pathlib import Path

import cv2
import ncnn
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from hybrid_tracker import YOLONCNNInferencer, _letterbox


class LegacyPacking(YOLONCNNInferencer):
    def _make_tensor(self, img):
        # NCNN can wrap NumPy storage. Keep it alive through extraction, just
        # as the original detect() kept its local blob/img_chw variables alive.
        self._legacy_blob = cv2.dnn.blobFromImage(
            img, scalefactor=1./255., size=(512, 512), swapRB=True, crop=False)
        self._legacy_chw = np.ascontiguousarray(self._legacy_blob[0])
        return ncnn.Mat(self._legacy_chw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, help="Local, representative camera image")
    parser.add_argument("--iterations", type=int, default=40)
    parser.add_argument("--output", default="benchmark_detector.json")
    args = parser.parse_args()
    if args.iterations < 10:
        parser.error("Use at least 10 iterations")
    image = cv2.imread(args.image)
    if image is None:
        parser.error("Image could not be read")
    cv2.setNumThreads(1)  # Same as the production detector process.
    image = cv2.resize(image, (640, 360))
    model_dir = Path(__file__).resolve().parent / "model"
    variants = {"legacy": LegacyPacking(model_dir, 2),
                "native": YOLONCNNInferencer(model_dir, 2)}
    letterboxed = _letterbox(image)[0]
    tensors = [np.array(v._make_tensor(letterboxed)) for v in variants.values()]
    tensor_delta = float(np.max(np.abs(tensors[0]-tensors[1])))
    timings = {key: [] for key in variants}
    profiles = {key: [] for key in variants}
    max_box_delta = 0.
    mismatches = 0
    for i in range(args.iterations+4):
        outputs = {}
        # Alternate order to reduce warm-up and thermal ordering bias.
        keys = list(variants) if i % 2 == 0 else list(reversed(variants))
        for key in keys:
            detector = variants[key]
            start = time.perf_counter()
            outputs[key], _ = detector.detect(image)
            elapsed = (time.perf_counter()-start)*1000.
            if i >= 4:
                timings[key].append(elapsed)
                profiles[key].append(dict(detector.last_profile))
        a, b = np.array(outputs["legacy"]), np.array(outputs["native"])
        if a.shape != b.shape:
            mismatches += 1
        elif a.size:
            max_box_delta = max(max_box_delta, float(np.max(np.abs(a-b))))
    report = {
        "host": platform.platform(), "machine": platform.machine(),
        "opencv": cv2.__version__, "ncnn": ncnn.__version__,
        "threads": 2, "input_size": 512, "iterations": args.iterations,
        "tensor_max_abs_delta": tensor_delta,
        "detection_max_abs_delta": max_box_delta,
        "detection_count_mismatches": mismatches,
        "detections_in_last_image": len(outputs["native"]),
        "note": "One static image; not an accuracy, live FPS, or hardware-response test.",
    }
    for key in variants:
        report[key] = {
            "total_p50_ms": float(np.median(timings[key])),
            "total_p95_ms": float(np.percentile(timings[key], 95)),
            **{name+"_p50": float(np.median([p[name] for p in profiles[key]]))
               for name in ("pre_ms", "infer_ms", "post_ms")},
        }
    Path(args.output).write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
