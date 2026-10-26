# Plan: NanoTrack on the Hailo-8L AI HAT

Written 2026-10-26. Status: **planned, waiting for the Hailo Dataflow Compiler 3.34.0** (the same
download needed to convert the YOLO26n person model).

## Goal and why

While a person is locked, the A8 tracker runs at ~13 fps instead of the camera's 25. NanoTrack
(the visual tracker that follows the person between detections) takes 36-65 ms per frame on the
CPU, and the Pi 5 is 94-98 % busy. Faster CPU engines did not fix it (see
`IMPLEMENTATION_STATUS.md`, "NanoTrack speed-up attempt"): the ncnn port gained only ~0.5 fps and
blocked the gimbal thread. Moving NanoTrack's networks to the HAT removes most of that CPU work.

**Targets** (measured with the same 2-minute walking test as before):

| | Now (OpenCV, CPU) | Target |
|---|---|---|
| Tracker fps while locked | 12.7 | 25 (camera rate) |
| NanoTrack per frame | 51 ms | under 10 ms including transfer |
| Gimbal command loop | 45 Hz | 45 Hz or more (must not drop) |
| Box and score vs OpenCV | — | centre median within 2 px, scores within 0.03 near 0.45 / 0.6 |

Expected NanoTrack time on the HAT is a few milliseconds (about 0.16 GFLOP per frame, one
context). This is an estimate until compiled and measured.

## What NanoTrack is made of (checked)

- **Backbone** (`nanotrack_backbone_sim.onnx`, 98 MFLOP at 255x255): Conv, HardSwish, ReLU,
  squeeze-excite blocks (GlobalAveragePool, Gemm, HardSigmoid, Mul). Runs once per frame on the
  255x255 search crop and once per re-seed on the 127x127 template crop.
- **Head** (`nanotrack_head_sim.onnx`, 66 MFLOP): two inputs, template features 48x8x8 and search
  features 48x16x16. Pointwise correlation = `MatMul` of two live feature maps (64x48 by 48x256),
  channel attention, Conv + ReLU6 towers, outputs class scores 2x16x16 and box 4x16x16 through a
  final `Exp`.
- Hailo-friendly except: the activation-by-activation `MatMul` (support on Hailo-8L to be
  confirmed by the parser) and the final `Exp` (done on the CPU: 1,024 values).

## Constraints found on the Pi

1. **One process per HAT.** The image's HailoRT 4.24 has no multi-process service
   (`hailort_service` is not built). The detector worker process already owns `/dev/hailo0`, so
   NanoTrack's HAT inference must run **inside that process** (second thread, both models on one
   VDevice with the HailoRT scheduler). Building the service instead needs gRPC in the image and a
   daemon; not worth it for this.
2. **HAT time budget.** yolov8s (3 contexts) uses ~33 ms of HAT time per frame at ~24 frames/s,
   about 80 % of the chip. A NanoTrack request could wait up to ~33 ms behind a YOLO frame, which
   would cancel the gain. **Do this after switching the detector to a single-context model**
   (the user's YOLO26n: Hailo lists yolo26n at 111 FPS on Hailo-8L, about 9 ms per frame). The
   NanoTrack model gets the higher scheduler priority.
3. **GIL.** The tracker's main process must not hold the GIL while waiting (the ncnn lesson). It
   only sends the crop through shared memory and waits on a pipe, which releases the GIL. Verify
   with the 1 ms thread test (worst gap must stay ~3 ms).

## Steps

### 0. Prerequisites
- [ ] User downloads `hailo_dataflow_compiler-3.34.0-py3-none-linux_x86_64.whl`.
- [ ] Laptop: separate venv `~/hailo_dfc` (Python 3.10), install the wheel (+ Hailo Model Zoo
      v2.20.0 for the YOLO26n compile). Keeps Ultralytics/ROS untouched.
- [ ] Detector moved to a single-context HEF (YOLO26n person, the other plan) and measured.

### 1. Build the NanoTrack network for the HAT (laptop)
- [ ] Merge backbone + head into one ONNX `nanotrack_search`: inputs `search` 1x3x255x255
      (RGB 0-255, as OpenCV's blobFromImage with scale 1) and `zf` 1x48x8x8; outputs `cls`
      1x2x16x16 and `bbox_raw` 1x4x16x16 (cut before `Exp`). Check against the two original
      models with onnxruntime (max difference ~1e-5).
- [ ] Template backbone (127x127, once per re-seed) stays on the CPU with OpenCV (fp32).
- [ ] Parse with the DFC for `hailo8l`. **If the two-input MatMul is refused**, plan B: HEF 1 =
      search backbone + both `conv_search` layers; the CPU does the two correlations (64x48 by
      48x256, ~0.2 ms with NumPy); HEF 2 = everything after the correlation. Two chip calls per
      frame instead of one.
- [ ] Calibration data from real tracking, made with the fp32 NanoTrack (`nano_ncnn.py`,
      identical to OpenCV): ~1,000 (search crop, zf) pairs from the user's person dataset, the
      walking clip and 2-3 minutes of A8 footage recorded on the Pi (record first). Crops exactly
      as the tracker makes them.
- [ ] Optimize with 16-bit precision on the correlation and output layers (as Hailo's own
      yolo26n script does for its outputs); compile; profiler must show **1 context**.

### 2. Accuracy gate (before touching the Pi)
- [ ] Hailo emulator (quantized) vs fp32 NanoTrack on 4+ clips: centre difference median within
      2 px, 99th percentile within 10 px, score difference within 0.03 around 0.45 and 0.6, no
      drift onto another person (the fp16 failure). Fail: more 16-bit layers, more calibration,
      or plan B.

### 3. Runtime on the Pi
- [ ] `tracker/src/nano_hailo.py`: refactor `nano_ncnn.py` so the crop, penalties, Hanning window
      and box update are shared; only "features + head" changes per engine (OpenCV / ncnn / Hailo).
- [ ] Detector worker process: load `nanotrack_search.hef` beside the YOLO HEF on the same
      VDevice (HailoRT scheduler, NanoTrack priority higher); a second thread serves NanoTrack
      requests.
- [ ] Main process to worker: 255x255x3 crop through shared memory, request and reply
      (16x16 outputs) through a pipe; timeout 30 ms.
- [ ] Failure handling: at startup, missing HEF or HAT means OpenCV NanoTrack (as today). At
      runtime, a timeout or error marks NanoTrack uncertain (gimbal stops, as for a low score) and
      the tracker re-seeds on the OpenCV engine; logged once.
- [ ] Setting `A8_NANO_BACKEND=hailo`; becomes the default only after step 4 passes.
- [ ] Tests: shared NanoTrack maths (existing parity test), request/reply over shared memory,
      timeout fallback, the 1 ms thread GIL test.

### 4. Live test on the bench (props off)
- [ ] Same 2-minute walking test as the gimbal tuning: tracker fps while locked, NanoTrack ms,
      gimbal loop Hz, detector rate and latency, lock losses, distance from centre, Pi CPU idle.
      Compare with the OpenCV baseline in the table above.
- [ ] Record results in `IMPLEMENTATION_STATUS.md`; image rebuild so it survives a container
      recreate.

## Risks

| Risk | Effect | Mitigation |
|---|---|---|
| Two-input MatMul unsupported or slow on Hailo-8L | parser error / extra contexts | plan B split (CPU correlation) |
| Quantization changes NanoTrack scores | lock thresholds 0.45 / 0.6 behave differently | 16-bit layers, real-crop calibration, accuracy gate; thresholds changed only with measured evidence |
| HAT busy with the detector | NanoTrack waits behind YOLO | single-context detector first, scheduler priority |
| Worker process crash | no NanoTrack and no detection | existing worker restart; OpenCV NanoTrack fallback |
| Shared-memory handoff adds delay | eats the gain | measured in step 4; 195 KB per frame is small |

## Not in scope
Flying with it: bench validation only, props off. YOLO26n conversion is its own plan (same
compiler, done first).
