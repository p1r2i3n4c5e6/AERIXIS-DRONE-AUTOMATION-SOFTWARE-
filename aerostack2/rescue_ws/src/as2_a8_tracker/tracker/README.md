# A8 Mini dual person tracking

Build: A8-20261020-dualtrack2

This build combines YOLO person detections, the official upstream OC-SORT association and Kalman tracker, and OpenCV NanoTrack. YOLO provides class-specific person detections. OC-SORT associates those detections and IDs. NanoTrack follows the selected box between detector results. Each detector result is matched to the Nano box at that result's own source-frame timestamp; delayed detections are not compared with a box from a later frame.

## Startup order

The movement-enabled launcher loads the NanoTrack models, sends a zero-rate packet, requests the SIYI one-key center command, and waits for a center ACK plus three fresh attitude samples near zero. Only then does it start the 50 Hz control loop, NCNN detector process, camera capture and tracking. If the center response or attitude data is missing, it stops startup and sends zero-rate packets. Preview mode sends no gimbal commands and performs no center action.

The center check assumes the drone/camera base is stationary during startup. SIYI's command ACK means the command was accepted; the following attitude samples confirm reported yaw and pitch reached the center threshold (±3°) and settled. The SDK attitude format and center command must match your firmware. Firmware behavior still needs bench confirmation.

## Install and run on Windows

Extract this folder and open PowerShell inside A8_DUAL_TRACKER. Use an environment where NCNN and OpenCV contrib are installed. requirements-dual.txt lists the package requirements; use a virtual environment or Conda environment if changes to Python packages are needed.

Verify the build:

    python .\src\hybrid_tracker.py --version

Preview only (no centering or gimbal control):

    .\run_preview.bat --headless --stream 8080

Movement-enabled bench run:

    .\run_tracking.bat --headless --stream 8080

Open http://localhost:8080. The tracking command centers and verifies before camera detection starts. It uses camera RTSP rtsp://192.168.144.25:8554/main.264 and SIYI control UDP 192.168.144.25:37260. Do not launch a second controller at the same time.

## Raspberry Pi 5

Preview without gimbal commands:

    bash run.sh --stream 8080

Center then tracking:

    bash run_tracking.sh --stream 8080

The Pi OpenCV build must expose cv2.TrackerNano and accept both included ONNX models. The actual Pi installation must confirm OpenCV API compatibility.

## Lock and movement behavior

The first person is selected after OC-SORT confirms detections. The displayed locked box is red. NanoTrack supplies interim positions; each new YOLO/OC-SORT result checks the selected trajectory and can repair an ID change within five seconds after three distinct observations when one spatially plausible candidate exists. The selection stays held through uncertainty and does not jump to a different person. If NanoTrack confidence fails, video goes stale, or detector confirmation is absent for two seconds, gimbal speed requests go to zero while the selected identity remains held. An off-screen Nano estimate no longer releases the selection: a tracker failure cannot prove the person left. Use R in the desktop preview to deliberately select again, or restart if using the browser preview only. Brief Nano uncertainty permits up to 250 ms of visual recovery, with movement stopped until two fresh valid updates pass; a fresh unambiguous detector result can also reinitialize the tracker. Conflicting candidates block movement until an unambiguous match returns.

Position and overlap do not prove a person's identity. Another person following the same path can look similar to the tracker. Long occlusion, blur, abrupt camera motion and a detection failure can still lose the target. No person re-identification embedding is included. Zero latency or guaranteed permanent identity is not possible.

The existing command cap stays at 35 SIYI units, with the existing gains and separate 50 Hz command loop. SDK units are not degrees per second; physical speed depends on the gimbal configuration. Verify movement direction and speed on a secured bench before flight. The tracker stops at its configured command cap; it does not read physical mechanical yaw limits for trajectory planning.

## Sources

The OC-SORT Python sources and MIT license are included under src/vendor/ocsort; a pure NumPy assignment and small FilterPy-compatible helper functions avoid a SciPy runtime dependency. NanoTrack's included model files and license are under model/nano. THIRD_PARTY_MANIFEST.json records upstream source revisions, download URLs and hashes. OpenCV TrackerNano requires the nanotrack_backbone_sim.onnx and nanotrack_head_sim.onnx pair.

This package has not been run on your SIYI camera, Raspberry Pi or Windows machine in this turn. Validate startup center ACK and measured attitude, RTSP image, Nano score and zero-stop behavior with propellers removed before any flight.

## dualtrack2 changes and measurements

Read OPTIMIZATION_REPORT.md for the measured results, remaining limits, and comparison procedure. Both OC-SORT and NanoTrack remain enabled. No smaller model, lower resolution, changed detection threshold, new frame skipping or higher thread count is used. A native NCNN input conversion avoids an intermediate blob; loaded NanoTrack networks are reused during reinitialization. Detector-stage timing, source-frame age, rejected candidate counts, lock transition reasons, and child-process resource usage are now logged.

Offline detector comparison (no camera commands):

    python benchmark_detector.py --image person.jpg --output benchmark_detector.json

Offline regressions (no camera commands):

    python -m unittest discover -s tests -v
