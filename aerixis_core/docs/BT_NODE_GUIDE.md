# Complete executable BT node guide

Installed AERIXIS runner snapshot; final validation occurs onboard

| Category | Registered types |
|---|---:|
| Condition | 26 |
| Action | 47 |
| Control | 18 |
| Decorator | 21 |
| SubTree | 1 |

Actions perform work. Conditions inspect evidence. Decorators change one child’s execution. Control nodes select or sequence children. SubTree reuses a named tree. These are BT control nodes, not the aircraft’s PID controller.

## Action

| Node | Purpose | Failure / safety use |
|---|---|---|
| `AcquireAssignment` | Obtain responder ownership from the swarm authority. | All aircraft must share mission/target IDs; no owner guessed from silence. |
| `AlwaysFailure` | Return failure immediately. | Useful to continue a fallback or test recovery. |
| `AlwaysSuccess` | Return success immediately. | Bench/control-flow use; never substitute for a safety check. |
| `ApproachDeliveryZone` | Move toward the independently approved aid-delivery position. | Uses world coordinates, not raw gimbal angles; stops on lost guards. |
| `ApproachOverhead` | Fly towards the locked person until the gimbal looks straight down with them centred. | Range comes from gimbal pitch and height above take-off on flat ground; fails if the person is lost. |
| `Arm` | Request motor arming. | Only after aircraft readiness; service acceptance is not a flight plan. |
| `CheckpointCoverage` | Persist current segment, pose and completed segments. | Protects progress across interruption; never replays takeoff on resume. |
| `CompleteRescue` | Mark a handled track after the safe rejoin completes. | Prevents repeatedly interrupting coverage for the same handled track. |
| `ConfirmHold` | Wait for confirmed idle motion and low measured speed. | Requires fresh vehicle telemetry; no new owner before this passes. |
| `Disarm` | Request motor disarming. | Use only after verified landing; never as an airborne fallback. |
| `DropPayload` | Release the payload once over the geotagged person (PX4 AUX servo). | Needs operator/sensor load confirmation, verified overhead envelope and height limits; never retried. |
| `Echo` | Write a diagnostic message. | No aircraft movement. |
| `FollowPath` | Execute a typed AERIXIS path. | Not supported by the current mission-upload validator; use explicit validated segments. |
| `GeotagOverhead` | Record the person's latitude/longitude while the gimbal is at nadir over them. | Needs a fresh GNSS fix; the geotag also excludes that spot from new detections. |
| `GeotagTarget` | Record a calibrated target fix with uncertainty. | Rejects unknown surface height, frame mismatch and poor accuracy. |
| `GetOrigin` | Read the configured geographic reference. | Check availability before geographic conversion. |
| `GoTo` | Fly to one point in a specified coordinate frame. | A point does not establish an obstacle-free path. |
| `GoToGps` | Fly to a geodetic waypoint using AERIXIS conversion. | Requires a verified origin and altitude interpretation. |
| `GpsToCartesian` | Convert geographic coordinates to a local frame. | Needs origin; conversion alone does not validate a route. |
| `Land` | Run the standard AERIXIS landing action. | Use ManagedLand for monitored rescue lifecycle completion. |
| `ManagedLand` | Run AERIXIS landing through the rescue motion owner. | Requires a landed-state report before allowing disarm. |
| `ManagedTakeoff` | Run AERIXIS takeoff through the rescue motion owner. | Late goal acceptance and cancellation remain owned until terminal result. |
| `Offboard` | Request AERIXIS offboard control authority. | PX4 offboard readiness and loss-of-control policy must be configured. |
| `OrbitTarget` | Circle the locked person at radius_m for the given laps, facing them. | Keep the radius clear of obstacles; fails if the person is lost. |
| `PauseCoverage` | Cancel owned motion and request a holding mode. | Succeeds only after action termination and measured hold evidence. |
| `ReleaseAssignment` | Report a completed assignment after verified hold. | Authority must confirm completion before future reassignment. |
| `ReleasePayload` | Reserve a durable release transaction and request AUX actuation once. | Never repeats an uncertain command; requires physical load/latch evidence. |
| `ReleaseTargetLock` | Drop the tracker selection and suppress new locks for suppress_s. | Use after geotag/drop so the search resumes instead of re-locking the same person. |
| `ReportObservation` | Record target evidence and the current recovery reason. | Can record an unresolved observation without authorizing delivery. |
| `ReportPayloadOutcome` | Record the reported payload command outcome. | Does not claim physical delivery; use for explicit operator-only payload mode. |
| `RequestRecovery` | Latch abort, stop tracking, cancel owned motion and verify hold. | Does not blindly request RTL after position loss; PX4 failsafe remains independent. |
| `ResumeCoverage` | Rejoin the persistent coverage checkpoint on an approved route. | Resumes remaining segments, not the mission-start sequence. |
| `ReturnToLaunch` | Fly back over the take-off point at the current or higher altitude. | Direct path: check the route is clear; allowed during abort recovery. |
| `Script` | Run a blackboard script. | Disabled in the deployed mission-upload profile. |
| `SendEvent` | Publish an AERIXIS mission event. | An event is not physical completion evidence. |
| `SetBlackboard` | Store a value on the BT blackboard. | Dynamic remapping is not supported by this upload profile. |
| `SetGimbalSearch` | Point the gimbal at the search pitch while no person is locked. | Refused by the tracker while a person is locked or in preview mode. |
| `SetOrigin` | Set the common geographic reference. | Do not change origin during flight or mix different fleet origins. |
| `Sleep` | Wait for the given milliseconds without blocking the executor. | Waiting alone does not command hover. |
| `SurveyWaypoint` | Execute one approved route segment and persist completion. | Blocks if route, vehicle or required peers are stale. |
| `TakeOff` | Run the standard AERIXIS takeoff action. | Use ManagedTakeoff in rescue templates for runtime motion ownership. |
| `TrackPerson` | Lock a confirmed track and command bounded A8 image centering. | Stops on stale data, identity change or gimbal oscillation. |
| `TrackTarget` | Hold position, face the locked person and keep the A8 gimbal on them. | Fails with TARGET_LOST after lost_timeout_s; mission 1 then returns to launch. |
| `UnsetBlackboard` | Remove a blackboard entry. | Do not erase persistent flight or payload records. |
| `VerifyDeliveryZone` | Check the fresh approved clear delivery zone. | Must include person clearance and match the current target. |
| `VerifyPayloadRelease` | Check physical release confirmation for the transaction. | A servo command ACK does not mean the package fell. |
| `WasEntryUpdated` | Check whether a blackboard value changed. | A change does not prove sensor freshness. |

## Condition

| Node | Purpose | Failure / safety use |
|---|---|---|
| `AbortRequested` | Check the latched critical recovery state. | A failed branch cannot clear it and resume flight by itself. |
| `AssignmentOwned` | Check a fresh lease for this responder and mission/target. | An expired lease is not silently transferred to another aircraft. |
| `CameraFresh` | Check recent locally generated camera health. | A connected Ethernet cable alone is not fresh video. |
| `CenteredOnTarget` | Check verified nadir and centering within the configured overhead delivery envelope. | Requires current matching track and calibrated geometry, not a UI checkbox. |
| `DeliveryZoneClear` | Check an approved clear zone with recipient separation. | No release directly over an occupied impact zone. |
| `FlightAvailable` | Position and FC connection are valid for a recovery flight. | Used by return-to-launch and landing after an abort. |
| `GeolocationValid` | Check current target identity, frame and location uncertainty. | Yaw/pitch alone cannot establish a geographic location. |
| `GimbalStable` | Check verified measured gimbal feedback and bounded rates. | Stale telemetry or oscillation fails this condition. |
| `HoldConfirmed` | Check accepted hold, completed motion cancellation and low speed. | Used for motion handover and delivery. |
| `HumanDetected` | A new person is locked: not handled before and not next to an earlier geotag. | Detection only; approach, geotag and drop have their own gates. |
| `IsFlying` | Check reported flying state. | Do not treat as complete position or battery readiness. |
| `IsPersonDetected` | Check a fresh person detection above confidence threshold. | Detection does not establish stable identity, location or delivery permission. |
| `OverheadTarget` | Gimbal at nadir with the locked person centred. | Use with StableFor before releasing a payload. |
| `PayloadAvailable` | A payload is confirmed loaded and not yet used. | Operator confirmation is not a physical sensor. |
| `PayloadLoaded` | Check fresh physical presence and latch sensors. | Unknown sensor state inhibits delivery. |
| `PayloadOperatorConfirmed` | Check a mission-bound preflight operator load assertion. | Separate from physical PayloadLoaded sensing; release remains unverified without sensors. |
| `PayloadReleased` | Check the durable confirmed payload outcome. | A missing acknowledgement remains uncertain. |
| `PeersFresh` | Check all configured peers have fresh common-frame reports. | Different from GCS link health or mere radio power. |
| `RescuePending` | Check for a confirmed track not yet handled in this run. | Track IDs must not be reused across detector sessions. |
| `SafeToRelease` | Check continuous stable hold, vehicle health, zone and payload gates. | The payload adapter independently checks authority again. |
| `ScriptCondition` | Evaluate a scripted condition. | Disabled in the deployed mission-upload profile. |
| `SeparationSafe` | Check conservative distance margins using uncertainty and age. | A separation monitor is not a collision-free trajectory planner. |
| `TargetLocked` | The tracker currently has a fresh locked person. | Lock is a session label, not a recognised identity. |
| `TrackConfirmed` | Check an observed, confirmed person track. | Predicted-only boxes cannot authorize approach. |
| `VehicleHealthy` | Check fresh position, attitude/rates, vibration and battery evidence. | A latched critical fault blocks further mission movement. |
| `VisionHealthy` | Tracker telemetry is fresh. | A healthy tracker can still miss people; this only checks liveness. |

## Decorator

| Node | Purpose | Failure / safety use |
|---|---|---|
| `Delay` | Wait before ticking a child. | Does not establish physical stability; use StableFor with measured conditions. |
| `FailureLatch` | Latch a child failure for the lifetime of this tree. | Survives halt; only a new tree clears this local latch. |
| `ForceFailure` | Convert child completion into failure. | Used to return priority selection to coverage after a handled rescue. |
| `ForceSuccess` | Convert child completion into success. | Never hide failed motion cancellation or uncertain release. |
| `Inverter` | Swap success and failure of a child. | Do not invert a safety gate to authorize unsafe operation. |
| `KeepRunningUntilFailure` | Keep a child active until it fails. | Pair with a mission timeout or explicit exit condition. |
| `LoopBool` | Iterate over a typed blackboard collection. | Typed dynamic blackboard inputs are not enabled by the current upload profile. |
| `LoopDouble` | Iterate over a typed blackboard collection. | Typed dynamic blackboard inputs are not enabled by the current upload profile. |
| `LoopInt` | Iterate over a typed blackboard collection. | Typed dynamic blackboard inputs are not enabled by the current upload profile. |
| `LoopString` | Iterate over a typed blackboard collection. | Typed dynamic blackboard inputs are not enabled by the current upload profile. |
| `Precondition` | Guard a child with a blackboard expression. | Scripted expressions are restricted in deployment; prefer typed conditions. |
| `Repeat` | Repeat a child a specified number of times. | Never wrap physical payload release as a retry loop. |
| `RetryUntilSuccessful` | Retry a failed child up to a limit. | Use only for repeat-safe operations; never blindly retry release. |
| `RunOnce` | Remember that a child has already completed in this tree instance. | Not durable across restart; use the payload transaction journal. |
| `SafetyGate` | Recheck a fresh supervisor flag before every child tick. | Lost/stale evidence halts the child and returns failure. |
| `SkipUnlessUpdated` | Skip child execution until a blackboard entry changes. | Do not skip continuous safety supervision. |
| `StableFor` | Require uninterrupted child success for a duration. | Useful for measured stability; failure resets the timer. |
| `Timeout` | Halt a child after a deadline. | Timeout does not prove that the aircraft stopped. |
| `WaitForAlert` | Gate behavior using an AERIXIS alert. | An independent flight supervisor is still needed. |
| `WaitForEvent` | Gate a child on the expected event. | Bound the wait with Timeout and provide a recovery branch. |
| `WaitValueUpdate` | Wait for a blackboard value update before proceeding. | Use a bounded timeout and check actual sample timestamps. |

## Control

| Node | Purpose | Failure / safety use |
|---|---|---|
| `AsyncFallback` | Yield between fallback alternatives. | Recovery must not hide unresolved motion or payload uncertainty. |
| `AsyncSequence` | Yield between sequence children so other reactive work can run. | Does not create a separate safe flight owner. |
| `Fallback` | Try alternatives in order until one succeeds. | Use for bounded reacquisition followed by report/resume. |
| `IfThenElse` | Select a then/else branch from a condition result. | Requires two or three children. |
| `Parallel` | Tick multiple children with configured success/failure thresholds. | Never run competing flight actions in parallel. |
| `ParallelAll` | Tick all children and aggregate their results. | Use for independent monitors, not multiple motion commands. |
| `ReactiveFallback` | Recheck higher-priority branches while a lower branch runs. | Suitable for rescue interrupting coverage; never bypass motion handover. |
| `ReactiveSequence` | Recheck earlier conditions while a later child runs. | A lost guard can halt an action; cancellation still needs confirmation. |
| `RecoverThenFail` | Run recovery after task failure, then preserve FAILURE. | A successful recovery must not label the failed mission successful. |
| `Sequence` | Run children in order; stop at the first failure. | Useful for arm, takeoff, mission, land. It is not a durable checkpoint. |
| `SequenceWithMemory` | Keep progress through a sequence according to BT memory semantics. | Memory is in-process; persistent coverage is stored by CheckpointCoverage. |
| `Switch2` | Select one of 2 cases from a blackboard value. | Requires compatible blackboard wiring; do not use stale mode values. |
| `Switch3` | Select one of 3 cases from a blackboard value. | Requires compatible blackboard wiring; do not use stale mode values. |
| `Switch4` | Select one of 4 cases from a blackboard value. | Requires compatible blackboard wiring; do not use stale mode values. |
| `Switch5` | Select one of 5 cases from a blackboard value. | Requires compatible blackboard wiring; do not use stale mode values. |
| `Switch6` | Select one of 6 cases from a blackboard value. | Requires compatible blackboard wiring; do not use stale mode values. |
| `TryCatch` | Run ordered try-children; run the final cleanup child on failure, and preserve failure. | Not a substitute for normal failure paths or flight-controller failsafes. |
| `WhileDoElse` | Reevaluate a condition to select an ongoing branch. | Branch changes can halt running children; bound transitions. |

## SubTree

| Node | Purpose | Failure / safety use |
|---|---|---|
| `SubTree` | Call another named tree definition. | Reusable mission modules; this validator currently forbids port remapping. |
