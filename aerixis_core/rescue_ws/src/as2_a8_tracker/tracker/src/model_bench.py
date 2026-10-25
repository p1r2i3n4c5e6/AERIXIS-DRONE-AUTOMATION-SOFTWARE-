"""Time one detection model on this board, the way the tracker runs it (prints one JSON line).

  python model_bench.py <model folder> --threads 2 --cores 2 3 --runs 20

Synthetic camera frames at the model's capture size (no camera needed). Run by the onboard panel's
benchmark button in the tracker's Python; the result unlocks or locks the model for this board.
"""
import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import detector_models  # noqa: E402
from model_library import capture_size  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('model')
    parser.add_argument('--threads', type=int, default=2)
    parser.add_argument('--cores', type=int, nargs='*', default=None)
    parser.add_argument('--runs', type=int, default=20)
    args = parser.parse_args()
    if args.cores:
        os.sched_setaffinity(0, set(args.cores))
    if not 3 <= args.runs <= 200:
        parser.error('--runs must be 3..200')
    started = time.perf_counter()
    detector = detector_models.make_detector(args.model, threads=args.threads)
    load_s = time.perf_counter() - started
    cw, ch = capture_size(detector.info)
    rng = np.random.default_rng(3)
    frame = rng.integers(0, 255, (ch, cw, 3), dtype=np.uint8)
    for _ in range(3):                                   # warm-up: first runs allocate and pack
        detector.detect(frame, conf_thresh=0.99)
    times = []
    for _ in range(args.runs):
        t0 = time.perf_counter()
        detector.detect(frame, conf_thresh=0.99)
        times.append((time.perf_counter() - t0) * 1000.0)
    median = statistics.median(times)
    p90 = sorted(times)[int(0.9 * (len(times) - 1))]
    print(json.dumps(dict(detect_ms=round(median, 1), detect_p90_ms=round(p90, 1),
                          detect_hz=round(1000.0 / median, 1), load_s=round(load_s, 2), runs=args.runs,
                          format=detector.info['format'], input=list(detector.info['input']))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
