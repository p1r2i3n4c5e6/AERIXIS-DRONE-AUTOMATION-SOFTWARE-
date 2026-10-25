# Behavior tree inventory and graphs

Inspected 2026-10-15. Source registrations plus the installed BehaviorTree.CPP 4.10.0 factory manifest. No flight processes were started. Phase 2 update: the rebuilt installed runner now exports the registry, confirming the category totals below.

## Registered node types

| Category | AERIXIS custom | Library built-ins | Combined |
|---|---:|---:|---:|
| Action | 13 | 7 | 20 |
| Condition | 2 | 1 | 3 |
| Decorator | 2 | 16 | 18 |
| Control | 0 | 17 | 17 |
| SubTree | 0 | 1 | 1 |

SubTree is listed separately. Node *types* differ from node *instances* in a mission. Model declarations inside TreeNodesModel do not register executable implementations.

### AERIXIS registrations

- **Action:** `Arm`, `Disarm`, `Offboard`, `TakeOff`, `GoTo`, `Land`, `SendEvent`, `Echo`, `SetOrigin`, `GetOrigin`, `GpsToCartesian`, `GoToGps`, `FollowPath`
- **Condition:** `IsFlying`, `IsPersonDetected`
- **Decorator:** `WaitForEvent`, `WaitForAlert`

### Library registrations

- **Action:** `AlwaysFailure`, `AlwaysSuccess`, `Script`, `SetBlackboard`, `Sleep`, `UnsetBlackboard`, `WasEntryUpdated`
- **Condition:** `ScriptCondition`
- **Decorator:** `Delay`, `ForceFailure`, `ForceSuccess`, `Inverter`, `KeepRunningUntilFailure`, `LoopBool`, `LoopDouble`, `LoopInt`, `LoopString`, `Precondition`, `Repeat`, `RetryUntilSuccessful`, `RunOnce`, `SkipUnlessUpdated`, `Timeout`, `WaitValueUpdate`
- **Control:** `AsyncFallback`, `AsyncSequence`, `Fallback`, `IfThenElse`, `Parallel`, `ParallelAll`, `ReactiveFallback`, `ReactiveSequence`, `Sequence`, `SequenceWithMemory`, `Switch2`, `Switch3`, `Switch4`, `Switch5`, `Switch6`, `TryCatch`, `WhileDoElse`
- **SubTree:** `SubTree`

## Checked-in XML trees

These are structural graphs, not flight validation. Graphs include every BehaviorTree definition in as2_behavior_tree/resource. SubTree calls are shown as call nodes; definitions are graphed separately. Legacy XML and missing ports can still prevent execution. UNKNOWN marks IDs absent from the inspected factory plus AERIXIS registrations.

### RTL_test_2.xml

Tree `BehaviorTree`

Action: 11, Control: 3, Decorator: 1.

```mermaid
flowchart TD
  n0["Sequence (Control)"]
  n1["SetOrigin (Action) / altitude=0.0 / latitude=52.171776 / longitude=4.416351 / service_name=set_origin"]
  n0 --> n1
  n2["Arm (Action) / service_name=set_arming_state"]
  n0 --> n2
  n3["Offboard (Action) / service_name=set_offboard_mode"]
  n0 --> n3
  n4["TakeOff (Action) / height=2 / speed=0.5"]
  n0 --> n4
  n5["Parallel (Control) / failure_threshold=1 / success_threshold=1"]
  n0 --> n5
  n6["WaitForAlert (Decorator) / alert={} / topic_name=/drone_sim_javilino0/alert"]
  n5 --> n6
  n7["AlwaysSuccess (Action)"]
  n6 --> n7
  n8["Sequence (Control)"]
  n5 --> n8
  n9["GpsToCartesian (Action) / latitude=52.172046 / longitude=4.416790 / out_pose={var} / service_name=geopath_to_path / z=2.0"]
  n8 --> n9
  n10["GoTo (Action) / max_speed=1.0 / pose={var} / yaw_angle=0.0 / yaw_mode=0"]
  n8 --> n10
  n11["GetOrigin (Action) / altitude={alt} / latitude={lat} / longitude={lon} / service_name=get_origin"]
  n0 --> n11
  n12["GpsToCartesian (Action) / latitude={lat} / longitude={lon} / out_pose={pose} / service_name=geopath_to_path / z=2.0"]
  n0 --> n12
  n13["GoTo (Action) / max_speed=1.0 / pose={pose} / yaw_angle=0.0 / yaw_mode=0"]
  n0 --> n13
  n14["Land (Action) / speed=0.5"]
  n0 --> n14
```

### actions.xml

Node-model metadata only; no executable BehaviorTree definition.

### arm_test.xml

Tree `BehaviorTree`

Action: 1.

```mermaid
flowchart TD
  n0["Arm (Action) / service_name=set_arming_state"]
```

### as2_mission_tree.xml

Tree `MainTree`

Action: 9, Control: 1.

```mermaid
flowchart TD
  n0["Sequence (Control) / name=FlightMissionSequence"]
  n1["Offboard (Action) / service_name=set_offboard_mode"]
  n0 --> n1
  n2["Arm (Action) / service_name=set_arming_state"]
  n0 --> n2
  n3["TakeOff (Action) / height=2.0 / speed=0.5"]
  n0 --> n3
  n4["GoTo (Action) / max_speed=1.5 / pose=5;0;2 / yaw_angle=0.0 / yaw_mode=1 / frame_id=earth"]
  n0 --> n4
  n5["GoTo (Action) / max_speed=1.5 / pose=5;5;2 / yaw_angle=0.0 / yaw_mode=1 / frame_id=earth"]
  n0 --> n5
  n6["GoTo (Action) / max_speed=1.5 / pose=0;5;2 / yaw_angle=0.0 / yaw_mode=1 / frame_id=earth"]
  n0 --> n6
  n7["GoTo (Action) / max_speed=1.5 / pose=0;0;2 / yaw_angle=0.0 / yaw_mode=1 / frame_id=earth"]
  n0 --> n7
  n8["Land (Action) / speed=0.5"]
  n0 --> n8
  n9["Disarm (Action) / service_name=set_arming_state"]
  n0 --> n9
```

### basic_mission.xml

Tree `BehaviorTree`

Action: 6, Control: 1.

```mermaid
flowchart TD
  n0["Sequence (Control)"]
  n1["Arm (Action) / service_name=set_arming_state"]
  n0 --> n1
  n2["Offboard (Action) / service_name=set_offboard_mode"]
  n0 --> n2
  n3["TakeOff (Action) / height=2 / speed=0.5"]
  n0 --> n3
  n4["GoTo (Action) / max_speed=2 / pose=5;5;2 / yaw_angle=0.0 / yaw_mode=0"]
  n0 --> n4
  n5["Land (Action) / speed=0.5"]
  n0 --> n5
  n6["Disarm (Action) / service_name=set_arming_state"]
  n0 --> n6
```

### event_test.xml

Tree `BehaviorTree`

Action: 2, Control: 1, Decorator: 3.

```mermaid
flowchart TD
  n0["Repeat (Decorator) / num_cycles=10"]
  n1["Delay (Decorator) / delay_msec=1000"]
  n0 --> n1
  n2["Parallel (Control) / failure_threshold=1 / success_threshold=2"]
  n1 --> n2
  n3["SendEvent (Action) / data=Hello / topic_name=/test"]
  n2 --> n3
  n4["WaitForEvent (Decorator) / result={result} / topic_name=/test"]
  n2 --> n4
  n5["Echo (Action) / data={result}"]
  n4 --> n5
```

### event_test_2.xml

Tree `BehaviorTree`

Action: 8, Control: 4, Decorator: 8, UNKNOWN: 1.

```mermaid
flowchart TD
  n0["SequenceStar (UNKNOWN)"]
  n1["Delay (Decorator) / delay_msec=1000"]
  n0 --> n1
  n2["Parallel (Control) / failure_threshold=1 / success_threshold=2"]
  n1 --> n2
  n3["SendEvent (Action) / data=1 / topic_name=/test"]
  n2 --> n3
  n4["WaitForEvent (Decorator) / result={result} / topic_name=/test"]
  n2 --> n4
  n5["Echo (Action) / data={result}"]
  n4 --> n5
  n6["Delay (Decorator) / delay_msec=1000"]
  n0 --> n6
  n7["Parallel (Control) / failure_threshold=1 / success_threshold=2"]
  n6 --> n7
  n8["SendEvent (Action) / data=2 / topic_name=/test"]
  n7 --> n8
  n9["WaitForEvent (Decorator) / result={result} / topic_name=/test"]
  n7 --> n9
  n10["Echo (Action) / data={result}"]
  n9 --> n10
  n11["Delay (Decorator) / delay_msec=1000"]
  n0 --> n11
  n12["Parallel (Control) / failure_threshold=1 / success_threshold=2"]
  n11 --> n12
  n13["SendEvent (Action) / data=3 / topic_name=/test"]
  n12 --> n13
  n14["WaitForEvent (Decorator) / result={result} / topic_name=/test"]
  n12 --> n14
  n15["Echo (Action) / data={result}"]
  n14 --> n15
  n16["Delay (Decorator) / delay_msec=1000"]
  n0 --> n16
  n17["Parallel (Control) / failure_threshold=1 / success_threshold=2"]
  n16 --> n17
  n18["SendEvent (Action) / data=4 / topic_name=/test"]
  n17 --> n18
  n19["WaitForEvent (Decorator) / result={result} / topic_name=/test"]
  n17 --> n19
  n20["Echo (Action) / data={result}"]
  n19 --> n20
```

### follow_path_test.xml

Tree `ArmTakeoff`

Action: 3, UNKNOWN: 1.

```mermaid
flowchart TD
  n0["SequenceStar (UNKNOWN)"]
  n1["Arm (Action) / service_name=set_arming_state"]
  n0 --> n1
  n2["Offboard (Action) / service_name=set_offboard_mode"]
  n0 --> n2
  n3["TakeOff (Action) / height={tk_height} / speed={tk_speed}"]
  n0 --> n3
```

Tree `BehaviorTree`

Action: 2, Control: 1, Decorator: 1.

```mermaid
flowchart TD
  n0["Delay (Decorator) / delay_msec=3000"]
  n1["Sequence (Control)"]
  n0 --> n1
  n2["Echo (Action) / data=IN"]
  n1 --> n2
  n3["FollowPath (Action) / path_topic_name=test / server_timeout=1000 / service_name=traj_gen/send_traj_wayp / speed=2 / yaw_mode=0"]
  n1 --> n3
```

Tree `RTL`

Action: 3, UNKNOWN: 1.

```mermaid
flowchart TD
  n0["SequenceStar (UNKNOWN)"]
  n1["GoTo (Action) / max_speed=2 / pose={land_pose} / yaw_angle=0.0 / yaw_mode=0"]
  n0 --> n1
  n2["Land (Action) / speed=0.5"]
  n0 --> n2
  n3["Disarm (Action) / service_name=set_arming_state"]
  n0 --> n3
```

### takeoff.xml

Tree `ArmTakeoff`

Action: 3, UNKNOWN: 1.

```mermaid
flowchart TD
  n0["SequenceStar (UNKNOWN)"]
  n1["Arm (Action) / service_name=set_arming_state"]
  n0 --> n1
  n2["Offboard (Action) / service_name=set_offboard_mode"]
  n0 --> n2
  n3["TakeOff (Action) / height={tk_height} / speed={tk_speed}"]
  n0 --> n3
```

Tree `BehaviorTree`

Condition: 1, Control: 1, Decorator: 1, SubTree: 1.

```mermaid
flowchart TD
  n0["WaitForEvent (Decorator) / result= / topic_name=mission/start"]
  n1["Fallback (Control)"]
  n0 --> n1
  n2["IsFlying (Condition)"]
  n1 --> n2
  n3["ArmTakeoff (SubTree) / __shared_blackboard=true / tk_height=2 / tk_speed=0.5"]
  n1 --> n3
```

### test.xml

Tree `BehaviorTree`

Action: 3, Control: 1.

```mermaid
flowchart TD
  n0["Sequence (Control)"]
  n1["Arm (Action) / service_name=set_arming_state"]
  n0 --> n1
  n2["Offboard (Action) / service_name=set_offboard_mode"]
  n0 --> n2
  n3["TakeOff (Action) / height=2 / speed=0.5"]
  n0 --> n3
```

### person_detection_check.xml (new observation-only bench tree)

Tree `PersonObservation`: 3 decorators, 1 condition. No flight/payload actions.

```mermaid
flowchart TD
  A[Timeout: 10 seconds] --> B[RetryUntilSuccessful]
  B --> C[Delay: 100 ms]
  C --> D[IsPersonDetected: fresh person observation]
```

## Current GCS-generated survey tree

Source: /home/prince/custom_gcs/web/js/bt_studio.js generateBtXml(). For N GoTo survey waypoints: N + 5 action instances, 2 control instances, 0 condition instances, 0 decorator instances. The root and BehaviorTree XML wrappers are not executable nodes.

```mermaid
flowchart TD
  M[FlightMissionSequence: Sequence] --> O[Offboard]
  M --> A[Arm]
  M --> T[TakeOff]
  M --> S[AreaCoverageScanTrajectory: Sequence]
  S --> W1[GoTo: waypoint 1]
  S --> WN[GoTo: remaining waypoints in order]
  M --> L[Land]
  M --> D[Disarm]
```

This does not count a live uploaded /tmp mission: its contents and execution state were not inspected. The checked-in as2_mission_tree.xml has 9 action instances and 1 Sequence, without the GCS coverage grouping.

## Integration gaps found

- Fixed in the Phase 2 UI update: the GCS dwell item now exports `Sleep msec="..."`, converting seconds to milliseconds. `Wait` is not a registered tag.
- GCS emits FollowPath without the required mission data. Its wrapper expects path, frame_id, speed and yaw_mode.
- actions.xml advertises IsTargetDetected, IsArucoDetected and UnPick, which are not registered by this engine.
- FollowReference implementation is compiled, but not registered in the engine.
- SwarmFlocking, PointGimbal and gripper behaviors exist elsewhere; their presence does not make them available as BT XML nodes.
- IsPersonDetected is now registered as an observation-only condition. Mesh-health, battery, payload-loaded and release nodes remain unimplemented. The default survey mission does not automatically include detection.
