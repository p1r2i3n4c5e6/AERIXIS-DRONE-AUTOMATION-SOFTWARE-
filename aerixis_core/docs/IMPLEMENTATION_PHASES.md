# Flood-rescue system: phased implementation

Status date: 2026-10-15. The operator authorized phased planning followed by implementation. Mission: find flood-stranded people and deliver food/medical supplies. Existing architecture and detailed contracts: [extension design](SWARM_RESCUE_EXTENSION_PLAN.md). Existing node counts and graphs: [BT inventory](BT_INVENTORY_AND_GRAPHS.md).

Each phase has an acceptance gate. Passing software tests is not flight qualification. Hardware-dependent gates remain open until measured on the intended aircraft. Existing unrelated workspace changes are preserved.

| Phase | Deliverables | Acceptance gate | Dependencies/status |
|---|---|---|---|
| 0 — Baseline and interfaces | Source audit, current BT graphs, supported hardware questionnaire, execution/status contracts | Existing versus proposed behavior explicitly recorded | Audit complete; Pi 5 16 GB, A8 Ethernet and model supplied; remaining hardware details pending |
| 1 — Truthful execution | Explicit BT terminal result, identified node transitions, responsive tick loop, cancel request, GCS result consumer, isolated mission artifacts, duplicate-run prevention | Failure never reported as success; unknown/crashed process never inferred successful; cancel stops ticking and invokes halt; repeated nodes distinguishable | Runner/local GCS foundation implemented and synthetic tests passed; physical hold verification remains open |
| 2 — Onboard mission management | Per-drone executor; versioned bundles; capabilities/port validation; stage/validate/start/cancel APIs; structured telemetry; restart reconciliation | Two independent drone namespaces cannot overwrite missions; checksum acknowledged before start; stale commands/revisions rejected; competing motion ownership rejected | Onboard service and two-executor bench tests implemented; physical motion ownership/stop verification open |
| 3 — Camera and gimbal workspace | ROS 2 overlay; A8 RTSP driver; calibrated CameraInfo; measured gimbal state/control; stream health and reconnect | Bench video and timestamps verified; stale frames rejected; actual gimbal attitude returned; disconnection/reconnection measured | Camera/perception overlay implemented and synthetic tests passed; real A8/gimbal and OS validation open |
| 4 — Human perception | Model adapter; persistent tracks; confidence/freshness; recording/replay; observation/geolocation quality | Representative aerial footage benchmark; identity/occlusion tests; stale detections rejected; location uncertainty exposed | Supplied NCNN model integrated; blank-frame ROS test passed; representative data and calibration/range validation open |
| 5 — Single-drone rescue BT | Optional detection subtree; assignment gate; coverage checkpoint; cancel/hold handover; bounded approach; target-loss recovery; safe rejoin | SITL proves interruption without competing commands; lost track cannot continue approach; resumed coverage does not repeat takeoff or erase progress | Phases 2 and 4; validated motion stop acknowledgement |
| 6 — Payload delivery | Physical presence/latch sensing; delivery-zone checks; durable release transaction; release feedback; waypoint delivery subtree | Bench inert payload: missing load inhibited; duplicate/replayed request cannot actuate twice; uncertain release requires reconciliation | Payload mechanism/sensor details; Phase 5 for detected-person delivery |
| 7 — KML fleet planning | Validated polygons/holes/exclusions; altitude semantics; capability/endurance-aware coverage partition; per-drone trees; separation/return corridors | Deterministic assignments; invalid geometry rejected; coverage/reserve constraints checked; per-drone preview matches uploaded plan | Phase 2 |
| 8 — Network and swarm coordination | Radio profiles; peer states/common reference; freshness; responder ownership; observation deduplication; partition policy; optional formation mode | Three-drone SITL with delay/loss/partition; one responder per assignment; no automatic reassignment based only on missing heartbeat; separation recovery verified | Phases 5 and 7; actual mesh radios for network qualification |
| 9 — GCS mission product | Capability-driven templates; typed graph editor; node registry; per-drone 2D/3D views; real states/reasons; simulation preview; upload/readiness workflow | Optional features compile correctly; mandatory safety cannot be removed; stale telemetry clearly shown; beginner completes scripted mission workflow | Incremental UI integration from Phase 1; final gate after Phase 8 |
| 10 — Deployment and field qualification | Supported Pi 5/OBC images; pinned dependencies; calibration/identity wizard; watchdogs; authenticated commands; logs; update/rollback; documented operating envelope | Repeatable setup; CPU/thermal budgets measured; recovery after power/process/link faults; staged field trials with inert then approved aid payloads | Hardware profiles and all prior acceptance evidence |

## Phase 1 implementation slices

1. Runner: single-tick scheduling; explicit SUCCESS/FAILURE/CANCELLED/ERROR outcomes; machine-readable events with mission/run/drone identity and node UID/path; handle exceptions and halt on interruption.
2. Cancellation: expose a namespaced request endpoint; service acceptance means request received, not aircraft held. Record terminal BT cancellation separately from physical motion/hold verification. Keep the ROS context alive during controlled shutdown so action cancel calls can run.
3. GCS: consume structured events; reject success without a matching explicit terminal result; keep untouched/skipped branches truthful; use unique mission files; reject overlapping local runs; remove shell interpolation of user identifiers.
4. Verification: harmless synthetic trees for success, failure, skipped results, exceptions, repeated nodes and interruption. Test the GCS parser against stale/malformed events and crashes. Do not arm, launch flight simulation, or activate payload hardware for these tests.
5. Remaining safety gate: real behavior cancellation acknowledgement and confirmed hold, including pending-goal races. Do not claim aircraft motion stopped from a BT halt or process exit alone.

## Stable contracts

- One mission ID identifies the operator's plan; a distinct run ID identifies an execution attempt. Node identity is scoped to a run and revision. UIDs alone are not portable across recompiled trees.
- Explicit terminal outcomes are authoritative. Exit codes are supporting diagnostics; a missing terminal result is unknown/error, never success.
- Each aircraft has one flight-motion owner. Coverage, tracking approach, formation and recovery cannot issue concurrent motion commands.
- Optional capabilities may be removed from templates; mandatory readiness, supervision and recovery policy cannot.
- Detection/geolocation does not authorize payload release. Release requires delivery checks, current physical payload state and a durable unique transaction.
- Camera calibration, vehicle/gimbal frames, timestamps and coordinate origins are part of configuration, not assumptions embedded in UI code.
- Network loss, stale positioning, lost target and uncertain payload state have explicit bounded recovery policies.

## Hardware inputs still needed

Confirmed: Raspberry Pi 5 16 GB, vision assigned to cores 2 and 3, A8 mini over Ethernet at the default address, and RASPI5_DEPLOYMENT_PACKAGE with the 512x512 single-class YOLO11n NCNN model. Still needed: Pi OS/version, camera firmware, aircraft count, radio hardware, payload actuator and presence/release sensors, calibration and altitude/range sensing. Work on software contracts and synthetic tests proceeds while these are collected.

## Progress log

- 2026-10-15: Created the complete phased plan; Phase 1 implementation started. No flight or payload actions authorized by a software test.

- 2026-10-15: Phase 1 runner/local GCS foundation built and tested; explicit results, node identities, graceful cancel requests and local-run exclusivity installed. GCS backup created; server not restarted. Full physical stop verification and action-goal race handling remain open.
- 2026-10-15: Initial Phase 3/4 software slice built in `rescue_ws`: A8 image node, existing NCNN adapter, detection/tracks/health, two-core launch profile. Added observation-only `IsPersonDetected` and a harmless bench tree. This parallel software preparation does not close the Phase 2 or real-hardware gates.
- Test evidence and remaining limitations: [implementation validation](IMPLEMENTATION_VALIDATION.md).

- 2026-10-15: Phase 2 onboard mission service and Phase 9 fleet UI slice implemented. Real runner registry, checksum staging, separate authenticated start/cancel, durable state/restart lockout, namespaced graph observation, per-drone live trees/counts and glass-style interface. See [setup and boundaries](ONBOARD_MISSIONS.md). Full rescue/swarm deployment gates remain open.
