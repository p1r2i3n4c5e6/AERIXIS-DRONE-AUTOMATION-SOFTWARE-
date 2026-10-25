# Implementation evidence and remaining gates

Date: 2026-10-15. Tests used this development host, local test ROS domain 91, synthetic data and harmless BTs. No aircraft was armed and no payload was actuated. These are software checks, not Raspberry Pi performance measurements or flight qualification.

## Implemented

- Main `as2_behavior_tree` runner: explicit SUCCESS (0), FAILURE (1), CANCELLED (2), ERROR (3); one tick per scheduling loop; exception reporting; halt on controlled shutdown; namespaced `mission/cancel` Trigger service; structured node and terminal events on `mission/events` and stdout.
- Event identity includes schema version, mission ID, execution run ID, sequence, drone namespace and node UID/path. The initial instance list includes unticked branches. UIDs are scoped to an execution, not permanent editor IDs.
- Local GCS path: verified structured results, direct execution of the installed runner, unique mission files, invalid namespace rejection, single local BT ownership, and graceful cancellation of its actual process. A zero exit without a matching explicit result is ERROR; unticked actions remain pending.
- GCS changed at `/home/prince/custom_gcs/server.py`; helper at `/home/prince/custom_gcs/bt_execution.py`. Backup: `/home/prince/custom_gcs/server.py.before_phase1_20261015_173847`. The running server was not restarted; source changes take effect on its next launch. The packaged `as2_ground_control.server` entry point was not changed; this integration targets the root `server.py` used by `1_launch_gcs.sh`.
- `rescue_ws` overlay with `as2_rescue_perception`: A8 Ethernet video node; bounded latest-frame cache; reconnect and stale health; CameraInfo; existing NCNN model adapter; person detections, tracks and freshness; explicit BGR8 conversion; cores 2/3 launch profile.
- Registered observation-only `IsPersonDetected` and `person_detection_check.xml`. No flight, gimbal or payload actuation is added by that tree.

## Passing checks

| Check | Evidence |
|---|---|
| Main AERIXIS package build | `colcon build --packages-select as2_behavior_tree ...` passed |
| C++ execution tests | 6 cases: success/failure distinction, cancel between ticks, cancellation before action, exception, skipped root, repeated-node identity/unticked branch |
| GCS event tests | 7 cases: missing terminal result, failed/unticked nodes, repeated types/IDLE reset, malformed/stale/wrong mission, contradictory exit, cancellation, identifier injection rejection |
| Actual runner process | SUCCESS/0, FAILURE/1, invalid XML node ERROR/3, running Sleep cancellation CANCELLED/2 |
| Installed GCS path | Failure preserved; unticked action pending; overlapping BT rejected; stop reached actual running tree |
| Perception utility tests | 3 cases: cache freshness/replay/reconnect, padded BGR/corrupt buffers, invalid calibration rejection |
| NCNN weights | Supplied 512x512 single-person model loaded and ran on a blank image; zero detections. This does not establish detection accuracy |
| ROS detector | Synthetic image passed through real NCNN and ROS messages; stale tracks explicitly cleared; stale input rejected |
| Camera node with synthetic decoder | Images and uncalibrated CameraInfo published; no duplicate stamps; reconnect attempted; stale health reported |
| BT condition over ROS | Fresh person -> SUCCESS; stale or low confidence -> FAILURE; cancellation service -> CANCELLED |
| Overlay build | Explicit `--base-paths src` discovery and build passed using build-only `PYTHONNOUSERSITE=1` |

Test sources: `as2_behavior_tree/tests/execution_gtest.cpp`, `integrations/gcs/test_bt_execution.py`, `integrations/gcs/smoke_runner.py`, `integrations/gcs/smoke_gcs.py`, and `rescue_ws/tests/` plus package `test/`.

The development host has incompatible user-site setuptools/packaging versions for the default overlay build; disabling user-site packages only during the build resolved this. Its cv_bridge binary also conflicts with NumPy 2; explicit BGR8 conversion avoids that dependency. System Python packages were not changed. ROS socket tests needed sandbox escalation; they used synthetic data on a dedicated test domain.

## Open gates

1. A BT cancellation result means tree halt, not confirmed vehicle hold. Existing AERIXIS action cancellation acknowledgement, pending-goal races, motion termination and recovery must be validated before autonomous handover.
2. Superseded by the Phase 2 update below: the Fleet UI now addresses independent onboard mission managers. Protected rescue coverage checkpoints remain unimplemented.
3. The 3D editor has not yet been converted to a general typed graph consuming all actual node statuses. Full structured instance data is exposed as `mission_bt_nodes`; the current survey UI still has its existing layout. Adding the new condition to XML is supported; a new editor palette/control was not added.
4. No real A8 connection was tested. Confirm stream configuration, firmware, actual camera latency, calibration, gimbal SDK feedback and transforms on the Pi.
5. Model throughput/thermals on the Pi and detection quality on representative flood footage remain unmeasured. Model metadata lists AGPL-3.0; wrapper integration preserves that metadata.
6. No geolocation, autonomous approach, payload driver, swarm mesh setup or fleet assignment was activated. The old human-height `dist_m` estimate is excluded from the new ROS interfaces.
7. Actual Pi OS/version, aircraft count, radios, payload sensors/actuator, camera calibration and altitude/range sensing remain to be confirmed.

Next implementation priority: verified motion-stop/ownership, hardware camera/gimbal validation, and rescue interruption/resume in simulation. Camera bench validation can proceed separately using [the overlay instructions](../rescue_ws/README.md).

## Phase 2 / fleet UI update — 2026-10-15

- Rebuilt the C++ runner with machine-readable registry export and parent/child UIDs in actual execution events.
- Built `as2_mission_manager` in the rescue overlay.
- Ten integration tests passed: actual executor success/failure/cancellation, independent namespaces, exact revision/checksum checks, duplicate-start handling, observation-only flight inhibition, restart/directory lockout, schema rejection, HTTP authentication, real ROS namespace isolation and stale platform telemetry.
- End-to-end laptop HTTP test passed against two real onboard executor instances: pairing, upload without execution, independent start/live running counts, cancellation, wrong-aircraft rejection, protected configuration permissions, no token in status responses and static UI assets served.
- All five GCS JavaScript files passed parser checks. Python modules compiled and startup shell syntax passed.
- HTTP preview returned 200. Browser visual QA was unavailable: the computer-use provider reported no available in-app browser. No screenshot or interactive-layout verification is claimed.
- Packaged startup script passed a real launch/API/ROS-observer check and exited cleanly; flight remained disabled. HTML IDs, referenced assets and JS element references checked.
- No vehicle was armed, no flight was commanded, and no payload was actuated. Test managers and missions were observation-only. Phase 2 used unique bench namespaces on the host ROS domain; earlier Phase 1/perception checks used domain 91.
- See [onboard setup and explicit remaining gates](ONBOARD_MISSIONS.md).
