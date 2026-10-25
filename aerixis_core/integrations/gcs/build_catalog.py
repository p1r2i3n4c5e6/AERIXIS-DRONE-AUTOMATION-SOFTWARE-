"""Export the installed runner's manifest, node help, and editable mission templates."""
import collections
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'rescue_ws/src/as2_rescue_runtime'))
sys.path.insert(0,str(ROOT/'rescue_ws/src/as2_rescue_planner'))
from as2_rescue_runtime.templates import mission, xml
from as2_rescue_planner.trees import vision_mission, VISION_MODES

HELP={
'ReportPayloadOutcome':('Record the reported payload command outcome.','Does not claim physical delivery; use for explicit operator-only payload mode.'),
'CenteredOnTarget':('Check verified nadir and centering within the configured overhead delivery envelope.','Requires current matching track and calibrated geometry, not a UI checkbox.'),
'PayloadOperatorConfirmed':('Check a mission-bound preflight operator load assertion.','Separate from physical PayloadLoaded sensing; release remains unverified without sensors.'),
'Arm':('Request motor arming.','Only after aircraft readiness; service acceptance is not a flight plan.'),
'Disarm':('Request motor disarming.','Use only after verified landing; never as an airborne fallback.'),
'Offboard':('Request AS2 offboard control authority.','PX4 offboard readiness and loss-of-control policy must be configured.'),
'TakeOff':('Run the standard AS2 takeoff action.','Use ManagedTakeoff in rescue templates for runtime motion ownership.'),
'Land':('Run the standard AS2 landing action.','Use ManagedLand for monitored rescue lifecycle completion.'),
'GoTo':('Fly to one point in a specified coordinate frame.','A point does not establish an obstacle-free path.'),
'GoToGps':('Fly to a geodetic waypoint using AS2 conversion.','Requires a verified origin and altitude interpretation.'),
'FollowPath':('Execute a typed AS2 path.','Not supported by the current mission-upload validator; use explicit validated segments.'),
'SetOrigin':('Set the common geographic reference.','Do not change origin during flight or mix different fleet origins.'),
'GetOrigin':('Read the configured geographic reference.','Check availability before geographic conversion.'),
'GpsToCartesian':('Convert geographic coordinates to a local frame.','Needs origin; conversion alone does not validate a route.'),
'Echo':('Write a diagnostic message.','No aircraft movement.'),
'SendEvent':('Publish an AS2 mission event.','An event is not physical completion evidence.'),
'IsFlying':('Check reported flying state.','Do not treat as complete position or battery readiness.'),
'IsPersonDetected':('Check a fresh person detection above confidence threshold.','Detection does not establish stable identity, location or delivery permission.'),
'WaitForEvent':('Gate a child on the expected event.','Bound the wait with Timeout and provide a recovery branch.'),
'WaitForAlert':('Gate behavior using an AS2 alert.','An independent flight supervisor is still needed.'),
'Sequence':('Run children in order; stop at the first failure.','Useful for arm, takeoff, mission, land. It is not a durable checkpoint.'),
'SequenceWithMemory':('Keep progress through a sequence according to BT memory semantics.','Memory is in-process; persistent coverage is stored by CheckpointCoverage.'),
'ReactiveSequence':('Recheck earlier conditions while a later child runs.','A lost guard can halt an action; cancellation still needs confirmation.'),
'AsyncSequence':('Yield between sequence children so other reactive work can run.','Does not create a separate safe flight owner.'),
'Fallback':('Try alternatives in order until one succeeds.','Use for bounded reacquisition followed by report/resume.'),
'ReactiveFallback':('Recheck higher-priority branches while a lower branch runs.','Suitable for rescue interrupting coverage; never bypass motion handover.'),
'AsyncFallback':('Yield between fallback alternatives.','Recovery must not hide unresolved motion or payload uncertainty.'),
'Parallel':('Tick multiple children with configured success/failure thresholds.','Never run competing flight actions in parallel.'),
'ParallelAll':('Tick all children and aggregate their results.','Use for independent monitors, not multiple motion commands.'),
'IfThenElse':('Select a then/else branch from a condition result.','Requires two or three children.'),
'WhileDoElse':('Reevaluate a condition to select an ongoing branch.','Branch changes can halt running children; bound transitions.'),
'TryCatch':('Run ordered try-children; run the final cleanup child on failure, and preserve failure.','Not a substitute for normal failure paths or flight-controller failsafes.'),
'SubTree':('Call another named tree definition.','Reusable mission modules; this validator currently forbids port remapping.'),
'AlwaysSuccess':('Return success immediately.','Bench/control-flow use; never substitute for a safety check.'),
'AlwaysFailure':('Return failure immediately.','Useful to continue a fallback or test recovery.'),
'Sleep':('Wait for the given milliseconds without blocking the executor.','Waiting alone does not command hover.'),
'Script':('Run a blackboard script.','Disabled in the deployed mission-upload profile.'),
'ScriptCondition':('Evaluate a scripted condition.','Disabled in the deployed mission-upload profile.'),
'SetBlackboard':('Store a value on the BT blackboard.','Dynamic remapping is not supported by this upload profile.'),
'UnsetBlackboard':('Remove a blackboard entry.','Do not erase persistent flight or payload records.'),
'WasEntryUpdated':('Check whether a blackboard value changed.','A change does not prove sensor freshness.'),
'Inverter':('Swap success and failure of a child.','Do not invert a safety gate to authorize unsafe operation.'),
'ForceSuccess':('Convert child completion into success.','Never hide failed motion cancellation or uncertain release.'),
'ForceFailure':('Convert child completion into failure.','Used to return priority selection to coverage after a handled rescue.'),
'Repeat':('Repeat a child a specified number of times.','Never wrap physical payload release as a retry loop.'),
'RetryUntilSuccessful':('Retry a failed child up to a limit.','Use only for repeat-safe operations; never blindly retry release.'),
'Timeout':('Halt a child after a deadline.','Timeout does not prove that the aircraft stopped.'),
'Delay':('Wait before ticking a child.','Does not establish physical stability; use StableFor with measured conditions.'),
'RunOnce':('Remember that a child has already completed in this tree instance.','Not durable across restart; use the payload transaction journal.'),
'KeepRunningUntilFailure':('Keep a child active until it fails.','Pair with a mission timeout or explicit exit condition.'),
'Precondition':('Guard a child with a blackboard expression.','Scripted expressions are restricted in deployment; prefer typed conditions.'),
'SkipUnlessUpdated':('Skip child execution until a blackboard entry changes.','Do not skip continuous safety supervision.'),
'WaitValueUpdate':('Wait for a blackboard value update before proceeding.','Use a bounded timeout and check actual sample timestamps.'),
'CheckpointCoverage':('Persist current segment, pose and completed segments.','Protects progress across interruption; never replays takeoff on resume.'),
'PauseCoverage':('Cancel owned motion and request a holding mode.','Succeeds only after action termination and measured hold evidence.'),
'ConfirmHold':('Wait for confirmed idle motion and low measured speed.','Requires fresh vehicle telemetry; no new owner before this passes.'),
'TrackPerson':('Lock a confirmed track and command bounded A8 image centering.','Stops on stale data, identity change or gimbal oscillation.'),
'GeotagTarget':('Record a calibrated target fix with uncertainty.','Rejects unknown surface height, frame mismatch and poor accuracy.'),
'ApproachDeliveryZone':('Move toward the independently approved aid-delivery position.','Uses world coordinates, not raw gimbal angles; stops on lost guards.'),
'VerifyDeliveryZone':('Check the fresh approved clear delivery zone.','Must include person clearance and match the current target.'),
'ReleasePayload':('Reserve a durable release transaction and request AUX actuation once.','Never repeats an uncertain command; requires physical load/latch evidence.'),
'VerifyPayloadRelease':('Check physical release confirmation for the transaction.','A servo command ACK does not mean the package fell.'),
'ResumeCoverage':('Rejoin the persistent coverage checkpoint on an approved route.','Resumes remaining segments, not the mission-start sequence.'),
'ReportObservation':('Record target evidence and the current recovery reason.','Can record an unresolved observation without authorizing delivery.'),
'RequestRecovery':('Latch abort, stop tracking, cancel owned motion and verify hold.','Does not blindly request RTL after position loss; PX4 failsafe remains independent.'),
'AcquireAssignment':('Obtain responder ownership from the swarm authority.','All aircraft must share mission/target IDs; no owner guessed from silence.'),
'ReleaseAssignment':('Report a completed assignment after verified hold.','Authority must confirm completion before future reassignment.'),
'SurveyWaypoint':('Execute one approved route segment and persist completion.','Blocks if route, vehicle or required peers are stale.'),
'CompleteRescue':('Mark a handled track after the safe rejoin completes.','Prevents repeatedly interrupting coverage for the same handled track.'),
'ManagedTakeoff':('Run AS2 takeoff through the rescue motion owner.','Late goal acceptance and cancellation remain owned until terminal result.'),
'ManagedLand':('Run AS2 landing through the rescue motion owner.','Requires a landed-state report before allowing disarm.'),
'VehicleHealthy':('Check fresh position, attitude/rates, vibration and battery evidence.','A latched critical fault blocks further mission movement.'),
'CameraFresh':('Check recent locally generated camera health.','A connected Ethernet cable alone is not fresh video.'),
'GimbalStable':('Check verified measured gimbal feedback and bounded rates.','Stale telemetry or oscillation fails this condition.'),
'TrackConfirmed':('Check an observed, confirmed person track.','Predicted-only boxes cannot authorize approach.'),
'GeolocationValid':('Check current target identity, frame and location uncertainty.','Yaw/pitch alone cannot establish a geographic location.'),
'HoldConfirmed':('Check accepted hold, completed motion cancellation and low speed.','Used for motion handover and delivery.'),
'DeliveryZoneClear':('Check an approved clear zone with recipient separation.','No release directly over an occupied impact zone.'),
'PayloadLoaded':('Check fresh physical presence and latch sensors.','Unknown sensor state inhibits delivery.'),
'SafeToRelease':('Check continuous stable hold, vehicle health, zone and payload gates.','The payload adapter independently checks authority again.'),
'AbortRequested':('Check the latched critical recovery state.','A failed branch cannot clear it and resume flight by itself.'),
'RescuePending':('Check for a confirmed track not yet handled in this run.','Track IDs must not be reused across detector sessions.'),
'AssignmentOwned':('Check a fresh lease for this responder and mission/target.','An expired lease is not silently transferred to another aircraft.'),
'PeersFresh':('Check all configured peers have fresh common-frame reports.','Different from GCS link health or mere radio power.'),
'SeparationSafe':('Check conservative distance margins using uncertainty and age.','A separation monitor is not a collision-free trajectory planner.'),
'PayloadReleased':('Check the durable confirmed payload outcome.','A missing acknowledgement remains uncertain.'),
'StableFor':('Require uninterrupted child success for a duration.','Useful for measured stability; failure resets the timer.'),
'SafetyGate':('Recheck a fresh supervisor flag before every child tick.','Lost/stale evidence halts the child and returns failure.'),
'FailureLatch':('Latch a child failure for the lifetime of this tree.','Survives halt; only a new tree clears this local latch.'),
'RecoverThenFail':('Run recovery after task failure, then preserve FAILURE.','A successful recovery must not label the failed mission successful.')}
HELP.update({
'TrackTarget':('Hold position, face the locked person and keep the A8 gimbal on them.','Fails with TARGET_LOST after lost_timeout_s; mission 1 then returns to launch.'),
'ApproachOverhead':('Fly towards the locked person until the gimbal looks straight down with them centred.','Range comes from gimbal pitch and height above take-off on flat ground; fails if the person is lost.'),
'OrbitTarget':('Circle the locked person at radius_m for the given laps, facing them.','Keep the radius clear of obstacles; fails if the person is lost.'),
'GeotagOverhead':('Record the person\'s latitude/longitude while the gimbal is at nadir over them.','Needs a fresh GNSS fix; the geotag also excludes that spot from new detections.'),
'DropPayload':('Release the payload once over the geotagged person (PX4 AUX servo).','Needs operator/sensor load confirmation, verified overhead envelope and height limits; never retried.'),
'ReleaseTargetLock':('Drop the tracker selection and suppress new locks for suppress_s.','Use after geotag/drop so the search resumes instead of re-locking the same person.'),
'SetGimbalSearch':('Point the gimbal at the search pitch while no person is locked.','Refused by the tracker while a person is locked or in preview mode.'),
'ReturnToLaunch':('Fly back over the take-off point at the current or higher altitude.','Direct path: check the route is clear; allowed during abort recovery.'),
'HumanDetected':('A new person is locked: not handled before and not next to an earlier geotag.','Detection only; approach, geotag and drop have their own gates.'),
'TargetLocked':('The tracker currently has a fresh locked person.','Lock is a session label, not a recognised identity.'),
'OverheadTarget':('Gimbal at nadir with the locked person centred.','Use with StableFor before releasing a payload.'),
'PayloadAvailable':('A payload is confirmed loaded and not yet used.','Operator confirmation is not a physical sensor.'),
'VisionHealthy':('Tracker telemetry is fresh.','A healthy tracker can still miss people; this only checks liveness.'),
'FlightAvailable':('Position and FC connection are valid for a recovery flight.','Used by return-to-launch and landing after an abort.')})
for i in range(2,7):HELP['Switch'+str(i)]=(f'Select one of {i} cases from a blackboard value.','Requires compatible blackboard wiring; do not use stale mode values.')
for name in ('LoopBool','LoopDouble','LoopInt','LoopString'):HELP[name]=('Iterate over a typed blackboard collection.','Typed dynamic blackboard inputs are not enabled by the current upload profile.')

def main():
    runner=ROOT/'install/as2_behavior_tree/lib/as2_behavior_tree/as2_behavior_tree_node'
    out=subprocess.check_output([str(runner),'--ros-args','-p','describe_nodes:=true'],text=True)
    registry=json.loads(next(line.split(' ',1)[1] for line in out.splitlines() if line.startswith('AS2_BT_REGISTRY ')))
    missing=set(registry)-set(HELP)
    if missing:raise ValueError('Missing node explanations: '+str(missing))
    nodes={name:dict(spec,description=HELP[name][0],safety=HELP[name][1],upload_supported=name not in ('Script','ScriptCondition','Precondition','FollowPath')) for name,spec in registry.items()}
    modes=['waypoints','surveillance','search_geotag','rescue_delivery','waypoint_delivery','swarm_search','swarm_delivery']
    data=dict(schema_version=1,source='Installed AS2 runner snapshot; final validation occurs onboard',
        registry_hash=hashlib.sha256(json.dumps(registry,sort_keys=True).encode()).hexdigest(),nodes=nodes,
        templates={mode:mission(mode,[[0,0,5],[10,0,5],[10,10,5]],'shared-mission-group') for mode in modes})
    for mode in VISION_MODES:
        data['templates'][mode]=vision_mission(mode,[[0,0,15],[20,0,15],[20,20,15]],'shared-mission-group')
    modes=list(data['templates'])
    template_dir=ROOT/'integrations/gcs/templates';template_dir.mkdir(exist_ok=True)
    for mode,tree in data['templates'].items():(template_dir/(mode+'.xml')).write_text(xml(tree)+'\n')
    # The GCS checkout is not always at a fixed absolute path; resolve it from
    # the environment, then from the usual sibling location, and finally skip
    # that copy with a warning rather than failing the whole export.
    web_root=Path(os.environ.get('AS2_GCS_WEB','') or (Path.home()/'custom_gcs'/'web'))
    if web_root.is_dir():
        (web_root/'data').mkdir(exist_ok=True)
        (web_root/'data'/'bt_catalog.json').write_text(json.dumps(data,indent=2))
    else:
        print('Warning: GCS web directory not found at',web_root,'; skipped that copy. Set AS2_GCS_WEB if elsewhere.')
    (ROOT/'integrations/gcs/bt_catalog.json').write_text(json.dumps(data,indent=2))
    lines=['# Complete executable BT node guide','',data['source'],'', '| Category | Registered types |','|---|---:|']
    for kind,count in collections.Counter(n['category'] for n in registry.values()).items():lines.append(f'| {kind} | {count} |')
    lines+=['','Actions perform work. Conditions inspect evidence. Decorators change one child’s execution. Control nodes select or sequence children. SubTree reuses a named tree. These are BT control nodes, not the aircraft’s PID controller.','']
    for kind in ('Action','Condition','Decorator','Control','SubTree'):
        lines += ['## '+kind,'','| Node | Purpose | Failure / safety use |','|---|---|---|']
        for name,spec in sorted(nodes.items()):
            if spec['category']==kind:lines.append(f"| `{name}` | {spec['description']} | {spec['safety']} |")
        lines.append('')
    (ROOT/'docs/BT_NODE_GUIDE.md').write_text('\n'.join(lines))
    print('Catalog exported:',len(nodes),'node types;',len(modes),'templates')
if __name__=='__main__':main()
