# dualtrack2: findings and measured limits

This revision is based on the local A8_DUAL_TRACKER dualtrack1 package. The latest Windows run was supplied as a summary, not the raw log or the contents of A8_DUAL_TRACKER_STAR. Its reported 112–135 ms detector time and intermittent lock loss therefore cannot be attributed to one proven runtime cause.

## What the reported numbers mean

- At 112–135 ms per asynchronous detector job, the theoretical throughput is about 7.4–8.9 jobs per second before scheduling overhead. NanoTrack can still update around the reported 25 video frames per second.
- `YOLO-Worker` measures decoded source-frame timestamp to worker completion, including dispatch, preparation, inference and postprocessing. It is not exclusively neural-network time.
- 49 Hz is the application's command transmission cadence. UDP transmission does not prove receipt, mechanical response, or accurate centering.
- A two-thread NCNN setting does not restrict the whole program to two threads or two CPU cores. OpenCV, capture, control, preview and the child detector process also consume CPU. The previous health `threads` value counted Python threads in the parent; `rss` measured the parent alone. New logs distinguish Python threads, summed native threads and summed process RSS. Summed RSS can double-count shared pages.
- Main-loop timing and decoded-image age exclude exposure, encoding, transport and decode delay. They cannot establish camera-to-motion latency.

## Verified defects and changes

1. **Selection release on a failed Nano update.** The previous code checked whether an estimated box was outside the frame before checking success/confidence, then cleared the selected ID. An invalid box could therefore release the person and allow a new selection. The revised code keeps the selection and sends no target to the controller on invalid observations. Deliberate reset is required after an unrecoverable loss; an estimated off-screen box is not proof of exit.
2. **Ambiguity did not actually block motion.** Previously the code set a text reason saying movement stopped but left a valid Nano target eligible for commands. An explicit motion block now survives subsequent Nano updates until a detector match resolves it. An OC-SORT ID change requires three distinct matching detections before resuming.
3. **A single score dip disabled visual tracking.** Brief uncertainty now stops motion while allowing a bounded 250 ms recovery attempt with the existing visual template. Two fresh, valid visual updates are required to resume; duplicate display frames do not count. A fresh, unambiguous detection can also reseed. The two-second detector confirmation limit and 400 ms observation age limit remain.
4. **Every reseed loaded the ONNX networks again.** Reinitialization now uses `init()` on the loaded NanoTrack instance. Historical replay has a 60 ms budget checked between updates and an eight-frame cap. A slow single OpenCV call cannot be interrupted by that budget; stale observations remain ineligible for commands.
5. **Extra detector input conversion.** The same OpenCV resize and padding are retained, followed by native NCNN RGB packing/normalization. The model, 512 × 512 input, confidence/aspect/height filters, NCNN numeric options and two inference threads are unchanged.

No changes were made to gimbal gain, command cap, startup centering, SIYI packets or physical limit handling. This revision improves failure handling; it does not guarantee identity through crossings or occlusion. OC-SORT and NanoTrack do not provide person re-identification.

## Measurements on this development host

Linux x86_64; OpenCV 5.0.0; NCNN 1.0.20260526. These are not Windows or Raspberry Pi measurements.

The detector comparison used the user's supplied person screenshot, resized to 640 × 360, with the same 512 × 512 model, two NCNN threads and one OpenCV preprocessing thread. Four warm-up pairs were followed by 40 measured pairs, alternating order.

| Measurement | Legacy packing | Native packing |
|---|---:|---:|
| Median complete detector call | 91.36 ms | 91.23 ms |
| 95th percentile complete detector call | 94.26 ms | 96.50 ms |
| Median preprocessing | 0.89 ms | 0.74 ms |
| Median neural-network extraction | 90.29 ms | 90.30 ms |

Inputs and returned detections were identical on this image, with no count mismatches. The total median difference is tiny and the tail was worse in this sample: **there is no demonstrated material end-to-end detector speedup**. An isolated packing microbenchmark showed a larger reduction, but the complete detector comparison is the relevant result. Neither benchmark measures live FPS or dataset accuracy.

For Nano reinitialization, 12 warm paired trials with two OpenCV threads measured a median of 5.73 ms with model reload versus 1.67 ms with model reuse. The following update returned the same box and score. This removes approximately 4 ms from each reseed on this host, not from every video frame. Raw results are in `measurements/`.

## Measure the remaining problem on the laptop

1. Extract into a new directory and verify `python .\src\hybrid_tracker.py --version` prints `A8-20261020-dualtrack2`. Keep dualtrack1 available for comparison.
2. Run `python benchmark_detector.py --image person.jpg --output benchmark_detector.json` using a representative saved camera image. This is offline and sends no camera commands. Use several near, far, edge-of-frame and partly obscured examples; a single image cannot establish accuracy.
3. Compare the same local video in **preview mode** with both builds. In each package, run `.\run_preview.bat --video test.mp4 --headless --stream 8080`. This checks behavior on the same content; file playback is not a real-time camera latency measurement. Count wrong-person switches, tracking interruptions, reacquisition time, detection misses and false detections. Preserving the weights alone does not prove equal tracking accuracy.
4. For live comparison, keep the camera settings, lighting, CPU power mode, launch flags and preview method identical. Test one person moving left/right, then partial occlusion and a second person crossing. Start with `.\run_preview.bat --headless --stream 8080`. If the installed OpenCV is headless, do not enable the desktop GUI.
5. With the gimbal secured and propellers removed, use `.\run_tracking.bat --headless --stream 8080`. Confirm startup center ACK and settled attitude, correct yaw/pitch direction, stop on loss and absence of uncontrolled motion. This command intentionally centers first.
6. Record physical action and the displayed image together with an external high-frame-rate camera or visible LED/timer. Measure exposure-to-display delay and action-to-first-gimbal-motion separately. Report median and 95th percentile over repeated movements; a 50 Hz send loop is not a latency measurement.
7. Preserve the raw `logs/hybrid_tracker.log`, the detector benchmark JSON, and a short representative recording. Match loss moments to `Lock state=COAST` and its reason. Check `discarded_stale`, `result_age`, `last_seed`, `last_geometry_rejected`, scores and temperatures. Do not tune gains upward to compensate for old video frames.

The existing filters reject people shorter than 50 pixels or with height/width outside 0.85–5.0 after clipping to the image. Small distant people, crouching and edge clipping may therefore remove valid detections. The new rejection counters help investigate this; lowering those gates without representative data could increase false locks. The counters describe candidate boxes before suppression, not distinct people.

If the network itself still takes 112–135 ms on that laptop at two threads, a large reduction needs a separately benchmarked model/backend or different hardware. Reducing resolution, using quantization or skipping more detection work can change accuracy or reacquisition behavior; none is silently enabled here. The Raspberry Pi additionally needs a sustained temperature/throttling check because overheating was previously reported.

## Implementation references

- [OpenCV NanoTrack implementation](https://github.com/opencv/opencv/blob/4.13.0/modules/video/src/tracking/tracker_nano.cpp): networks load in construction; `init()` replaces the target template.
- [NCNN Python bindings](https://github.com/Tencent/ncnn/blob/master/python/src/main.cpp): native pixel conversion.
- [NCNN normalization documentation](https://ncnn.readthedocs.io/en/latest/home.html): pixel packing followed by normalization.

No SIYI hardware, packet delivery, actual camera response, user Windows environment, Pi thermal behavior or flight performance was tested here.
