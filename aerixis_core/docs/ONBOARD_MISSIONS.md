# Onboard missions and fleet dashboard

Implemented software slice: Phase 2 mission service and the first Phase 9 fleet interface. This extends the existing AERIXIS runner; it is custom project code, not an assertion that upstream AERIXIS supplies this HTTP fleet service. Each aircraft runs its own AERIXIS action servers, runner, and mission manager. The laptop creates XML and monitors execution. Closing the browser does not cancel an onboard mission.

## Build

On a supported ROS 2 Humble installation, from the AERIXIS workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select as2_behavior_tree --allow-overriding as2_behavior_tree
source install/setup.bash
cd rescue_ws
PYTHONNOUSERSITE=1 colcon build --base-paths src --packages-select as2_mission_manager
source install/setup.bash
```

The `PYTHONNOUSERSITE` setting is needed for this development machine's build-tool incompatibility; it is not applied to vision inference. Build the camera package using its separate README.

## First harmless mission

On the companion computer:

```bash
cd ~/aerostack2
./rescue_ws/scripts/start_mission_manager.sh drone0
```

This creates a private state directory and pairing token at `~/.local/share/as2-missions/drone0/pairing.token`. It listens on loopback port 8790 and defaults to **observation-only** execution. It does not launch the AERIXIS flight stack, configure a mesh radio, arm motors, or release a payload. Give every aircraft a different namespace.

For the first laptop bench connection, an SSH tunnel provides encrypted transport without deploying certificates:

```bash
ssh -N -L 8790:127.0.0.1:8790 USER@DRONE_ADDRESS
```

Use the actual onboard username/address. For another aircraft, forward a different laptop port, for example `8791:127.0.0.1:8790`. SSH authentication and host verification remain enabled. For direct network access, the manager requires `--host ADDRESS --tls-cert CERT --tls-key KEY`; certificates must match the endpoint hostname/IP and be trusted by the laptop. A private CA can be configured with `AS2_FLEET_CA_FILE`. Certificate verification is never disabled. Tokens are not certificates.

On the laptop, launch the existing GCS with `./1_launch_gcs.sh`, then open `http://127.0.0.1:8765`. The launcher serves `/home/prince/custom_gcs/server.py` and its `web/` directory.

1. Open **Fleet → Add aircraft**. Enter `drone0`, a display name, `http://127.0.0.1:8790` for the tunnel, and that aircraft's token. The token is stored in the laptop's private `custom_gcs/config/fleet.json`, not returned in status responses.
2. Save the following as `bench.xml`, choose it, and select **Upload & validate**.
3. Check the displayed revision and select **Start onboard**.
4. In **Live tree**, inspect the actual runner's nodes and transitions. In **ROS nodes**, inspect only the selected namespace. Discovery does not prove component health.
5. Upload a longer `Sleep` tree to test cancellation. Cancellation acknowledges a BT request; it does not assert the aircraft has physically stopped.

```xml
<root BTCPP_format="4" main_tree_to_execute="Bench">
  <BehaviorTree ID="Bench">
    <Sequence name="ObservationBench">
      <Sleep msec="3000"/>
      <AlwaysSuccess/>
    </Sequence>
  </BehaviorTree>
</root>
```

Optional user-service template: `rescue_ws/deploy/as2-mission-manager@.service`. Adapt installation paths before using it. It is supplied but not installed or enabled automatically. Its defaults remain loopback and observation-only.

## What the service checks

- Installed runner exports its real node registry, categories, port types and defaults. Uploads are checked against that registry, not a hardcoded GCS palette.
- Uploaded UTF-8 XML is SHA-256 checked, stored as a unique immutable artifact, and associated with a generated mission ID and registry hash. Start must match that mission ID and revision. Stored XML is hashed again before execution.
- Upload and Start are separate. A repeated Start command ID returns the existing execution; a new command cannot restart the same uploaded mission. Upload again to intentionally repeat it.
- One execution per manager/state directory; staging cannot replace a running tree. File locking prevents competing owners of that directory. This is not yet a distributed motion lease across every possible external AERIXIS client.
- Start, status and cancellation are authenticated. The laptop checks endpoint drone identity before forwarding a command. HTTPS is mandatory for direct non-loopback exposure.
- State uses atomic writes with fsync. Events are journaled per mission. A restart with an uncertain active execution enters `INTERRUPTED` and requires reconciliation.
- Flight actions default to disabled. The optional `--allow-flight` switch also requires fresh, connected, disarmed ground telemetry. This gate is not flight qualification, collision avoidance, battery validation or complete rescue supervision.
- After a flight-capable run, a ground-state lockout remains until explicit reconciliation verifies fresh disarmed ground telemetry. A potentially surviving old process blocks reconciliation.

The current conservative validator rejects scripts, external XML includes, dynamic blackboard remapping, typed `FollowPath` inputs, unknown nodes, missing required ports, and invalid primitive numeric inputs. It imposes basic speed/height caps, a 500-node expansion cap and depth limit. It does not establish geofence validity, terrain clearance or all node-specific runtime semantics. Runtime construction failures are explicitly reported as errors.

## Interface

All onboard requests use `Authorization: Bearer TOKEN`.

| Route | Purpose |
|---|---|
| `GET /api/status` | Mission, execution, actual node states, registered category counts, namespaced ROS graph, logs, freshness inputs |
| `GET /api/registry` | Installed node manifest and registry hash |
| `POST /api/mission/stage` | `drone_id`, `xml`, `sha256`; returns mission ID/revision/schema summary |
| `POST /api/mission/start` | `mission_id`, `revision`, unique `command_id` |
| `POST /api/mission/cancel` | Exact executing `mission_id` and `revision` |
| `POST /api/mission/reconcile` | Verify ground state and clear recovery lockout |

Laptop routes `/api/fleet`, `/api/fleet/profile`, `/api/fleet/command` aggregate status and forward commands. Mutating fleet routes require the browser's custom request header. The GCS binds loopback by default; it is not an authenticated multiuser web service.

## UI delivered

Glass-style responsive fleet workspace, connection indicators/spinners, per-aircraft cards, fresh ROS and running-BT counts, registry category totals, XML selection and onboard revision, separate start/cancel/reconcile controls, pan/zoom live tree, namespaced ROS list, filterable activity, error dialogs and toasts, and reduced-motion support. Unknown/stale data never appears as a fresh zero. A `+` after an aggregate count indicates aircraft with unavailable reports are excluded.

The existing 3D **planning** studio remains available. The new live fleet graph is a **2D actual-execution** graph. Per-aircraft live 3D synchronization and a complete capability-driven editor remain later Phase 9 work. Legacy single-link flight controls are hidden while Fleet is selected so they cannot appear to target the selected fleet aircraft.

## Still required before autonomous rescue deployment

- Verified cancellation/hold and exclusive motion handover, monitored flight readiness and onboard link-loss policy.
- Real A8 stream/gimbal calibration and telemetry, Pi model benchmarks, target localization with uncertainty.
- Rescue interruption/resume and target-loss behavior tested in SITL, including protected coverage checkpoints.
- Physical payload presence/latch/release feedback and durable, duplicate-safe release transactions.
- Multi-aircraft KML partitioning, common-frame peer positioning, separation and assignment coordination; mesh radios configured and measured separately. AERIXIS does not create a radio mesh just because multiple namespaces exist.
- Visual browser QA, hardware trials, operational logs/retention, image packaging, updates and rollback. This slice has not made the system production- or flight-qualified.
