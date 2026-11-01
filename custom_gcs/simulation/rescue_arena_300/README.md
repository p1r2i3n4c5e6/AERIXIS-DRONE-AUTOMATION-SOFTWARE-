# AERIXIS / Aerostack2 rescue arena

A self-contained **300 × 300 metre** Gazebo arena for PX4 SITL search and
geotagging missions. All dimensions are metres.

- **50 static human figures**, one per zone across a 10 × 5 layout. Closest pair:
  **20.55 m** with the supplied seed; minimum permitted spacing: 20 m.
- **180 trees**, 6.5–10.5 m tall, with varied foliage and trunk/canopy collisions.
- A central **18 m radius clearing**, marked landing pad, and visible arena edges.
- Trees stay at least **4 m from each human's feet to the canopy envelope**.
- No network assets, Fuel models, external meshes, or custom plugins are required.
- World coordinates: **X east, Y north, Z up**; arena bounds −150…+150 m on X/Y.
- Static targets have stable IDs `human_001`…`human_050`.

## Which file to use

| File | Purpose |
| --- | --- |
| `worlds/rescue_arena_300.sdf` | **Use this for Gazebo + PX4 SITL.** Includes geometry, physics, lighting, GPS origin and sensor systems. |
| `urdf/rescue_arena_300.urdf.xacro` | Editable Xacro version, with reusable shape macros and fixed object placements. |
| `urdf/rescue_arena_300.urdf` | Ready-made URDF for RViz or a static environment model. |
| `ground_truth/humans.csv` | Target IDs, positions, orientations, heights, latitude, longitude and altitude. |
| `ground_truth/layout.json` | Complete tree/target layout and generation settings. |
| `arena_overview.png` | Labelled overhead map of the actual generated placements. |
| `preview/overview.png`, `preview/human_detail.png` | Actual Gazebo camera renders of the arena and one human figure. |

URDF describes a model; SDF also describes the simulator world. Loading only the
URDF does not configure PX4 sensors, lighting, or georeferencing. Do not insert
the URDF into this supplied SDF world: that would duplicate the arena.

## Launch PX4 with this arena

On this machine, the included launcher defaults to the existing built PX4 1.16
checkout and Gazebo Harmonic:

Stop an existing PX4 SITL instance using the same instance number before starting.

```bash
cd /home/prince/custom_gcs/simulation/rescue_arena_300
bash launch_px4.sh
```

The launcher starts Gazebo, waits for its scene service, then starts PX4's X500
at `(0, 0, 0.5)`. Ctrl+C stops the processes it started. Each run keeps its logs
and parameters in a new `/tmp/rescue_arena_px4.*` directory. Existing PX4 world
files and GCS settings are not modified.

To use your older PX4 / Garden checkout:

```bash
PX4_AUTOPILOT_DIR=/home/prince/px4_sitl/PX4-Autopilot bash launch_px4.sh
```

For a server without a GUI:

```bash
HEADLESS=1 bash launch_px4.sh
```

`PX4_AUTOPILOT_DIR`, `PX4_INSTANCE`, `PX4_UXRCE_DDS_NS`, `PX4_SYS_AUTOSTART`,
`PX4_GZ_MODEL_POSE`, and `GZ_PARTITION` can be set before launch. Namespace defaults
to `drone0`. For PX4 1.16, select another installed vehicle with `PX4_SIM_MODEL`
and its matching `PX4_SYS_AUTOSTART`; for the older checkout use `PX4_GZ_MODEL`.
The default X500 does **not** provide a downward mission camera: retain your
existing camera-equipped model or add a suitable camera payload for detection.

Start your existing Micro-XRCE-DDS Agent, Aerostack2 **PX4 platform**, mission
nodes and GCS as usual. This package supplies the environment and SITL startup;
it does not start your mission stack or implement the detection/geotagging node.
For ROS nodes using simulation time, bridge `/clock` and set `use_sim_time: true`.

The launcher uses `GZ_PARTITION=rescue_arena_$USER` by default to keep world
discovery separate from other simulations. Any Gazebo-to-ROS camera or clock
bridge must use the **same partition and Gazebo generation**. For example, in
the bridge terminal:

```bash
export GZ_PARTITION="rescue_arena_$USER"
```

ROS 2 Humble's existing Garden bridges may not match Harmonic. Use bridges built
for your chosen Gazebo generation. PX4's DDS connection is separate from Gazebo
transport and does not depend on this partition.

## Preview just the world

```bash
bash launch_world.sh
# Or Garden:
ARENA_GZ_VERSION=7 bash launch_world.sh
```

The default preview uses Harmonic. The initial camera gives an arena-wide view;
zoom in to see the human figures. The world has no aircraft until PX4 spawns one.
Avoid leaving a separate world preview running in the same partition when using
the combined launcher.

## Use an Aerostack2 world launcher

The included `config/as2_world.yaml` contains the world name and empty drone/object
lists, because the environment is already in SDF and PX4 owns the drone:

```bash
export GZ_SIM_RESOURCE_PATH="/home/prince/custom_gcs/simulation/rescue_arena_300/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
ros2 launch as2_gazebo_assets launch_simulation.py \
  simulation_config_file:=/home/prince/custom_gcs/simulation/rescue_arena_300/config/as2_world.yaml
```

This is an alternative **world-only** launch path. Attach PX4 separately using
your version's standalone workflow. Do not also run `launch_px4.sh` in that
partition: it starts and owns its own world. Do not use the native AS2 Gazebo
flight platform for a PX4-controlled aircraft.

## Ground truth and GPS origin

The default origin is PX4's usual Zurich simulation location:
`47.3979710577° N, 8.5461637398° E`, elevation `488 m`. This is a simulation
reference, not a claim about your current physical location.

Every row in `humans.csv` identifies the person's **feet** at local `z=0`.
Latitude/longitude/altitude are computed using a WGS84 ENU → ECEF → geodetic
conversion, with the same origin and zero heading as the SDF. Flat-world points
away from the origin can have a slightly different geodetic altitude because of
Earth curvature. Use a consistent altitude datum when comparing with PX4 GPS;
do not mix ellipsoid heights with mean-sea-level heights.

PX4 local NED differs from world ENU: `north = world_y`, `east = world_x`,
`down = -world_z` **only when the local origins coincide**. Estimator home/origin
offsets must be included when evaluating your mission's geotags. Check the
aircraft's reported global origin before evaluating a mission.

Change `config/arena.json` to set your mission origin, tree count, target count,
spacing or seed, then regenerate **all** files together:

```bash
python3 generate_arena.py
python3 validate_arena.py
python3 render_overview.py  # optional; requires matplotlib
```

The generator uses only the Python standard library. The preview uses matplotlib.
For new counts the grid adapts; impossible clearance constraints produce an
error rather than overlapping objects. A fixed seed reproduces the same layout.
Direct edits to generated SDF/Xacro/URDF are overwritten by regeneration.

Expand the Xacro yourself if desired:

```bash
xacro urdf/rescue_arena_300.urdf.xacro -o /tmp/rescue_arena_300.urdf
check_urdf /tmp/rescue_arena_300.urdf
gz sdf -k worlds/rescue_arena_300.sdf
```

## Scope and practical limits

People are coloured, static humanoid primitives with limbs, heads and collisions.
They do not walk or provide photorealistic training data. A detector trained on
real people may not recognize these figures; replace their visuals with human
meshes for that use case, retaining model IDs, poses and ground-truth alignment.
Tree canopies are solid collision shapes. Arena lines are visual boundaries,
not a PX4 geofence. Configure your mission's altitude and limits separately.

For overhead coverage, 25–40 m AGL clears these trees; choose the actual survey
altitude and track spacing from your camera's field of view, image resolution,
and required target pixel size. Clearance around targets avoids direct canopy
coverage, but oblique camera views can still encounter occlusion.

## References

- [Aerostack2: adding Gazebo worlds and assets](https://aerostack2.github.io/_09_development/_tutorials/_tutorials/adding_gazebo_assets.html)
- [PX4: Gazebo simulation and custom worlds](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [Gazebo: resource lookup](https://gazebosim.org/api/sim/8/resources.html)

See [VALIDATION.md](VALIDATION.md) for the checks performed on this machine.
