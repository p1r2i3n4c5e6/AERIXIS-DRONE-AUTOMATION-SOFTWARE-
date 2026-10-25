# trajectory_generator_jerk_limited

ROS-free C++17 jerk-limited 3D trajectory generator (PX4-style S-curve
profile, axis-synchronised, optional L1 corner-cut), with optional
pybind11 bindings and a self-contained CLI example. Released under
BSD-3-Clause.

## Overview

`trajectory_generator_jerk_limited` plans smooth jerk-limited
trajectories through 3D waypoint sequences. Each axis runs a one-axis
S-curve profile (T1/T2/T3 jerk phases); the three axes are then
time-synchronised so they reach the target simultaneously. Velocity and
acceleration bounds are enforced as 3D norms (`‖v‖ ≤ v_max`,
`‖a‖ ≤ a_max`). Multi-waypoint paths add an L1 corner-cut and a
backward-sweep corner-speed cap so the trajectory keeps momentum across
turns. The public API is built around a single one-shot entry point
`TrajectoryGenerator(GeneratorConfig)`. Frame is ENU, units are SI,
trajectory time always starts at `t = 0`. Sampled CSV output is the canonical 10-column schema
`t,px,py,pz,vx,vy,vz,ax,ay,az`; the underlying spline is serialisable to
`<base>.spline.csv`.

## Repository layout

```
.
├── CMakeLists.txt
├── include/trajectory_generator_jerk_limited/  # Public headers (stable API)
│   ├── parameters.hpp              # TrajectoryParameters: kinematic bounds
│   ├── velocity_profile_1d.hpp     # JerkLimitedVelocityProfile1D (one-axis S-curve)
│   ├── trajectory_generator_3d.hpp # JerkLimitedTrajectoryGenerator3D
│   ├── trajectory_math.hpp         # Free helpers (corner-cut / sweep / braking)
│   ├── waypoint_controller.hpp     # WaypointTrajectoryController (streaming API)
│   └── trajectory_generator.hpp    # TrajectoryGenerator (one-shot API)
├── src/                            # Implementations
├── example/                        # End-to-end CLI demo (utils + run_example.{cpp,py})
├── tests/                          # GoogleTest, auto-discovered *_gtest.cpp
└── pybind/                         # pybind11 module + Python package mirror
                                    # (`trajectory_generator_jerk_limited`)
```

## Dependencies

- C++17 compiler (GCC ≥ 9 / Clang ≥ 10).
- CMake ≥ 3.16.
- [Eigen 3](https://eigen.tuxfamily.org/).
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) (only when `BUILD_EXAMPLES=ON`).
- [GoogleTest](https://github.com/google/googletest) (C++ test suite).
- [pybind11](https://github.com/pybind/pybind11) ≥ 2.9 + Python ≥ 3.8 + numpy
  (Python bindings).
- `PyYAML`, `matplotlib` (used by `example/run_example.py` and
  `example/plot_results.py`).

Ubuntu / Debian:

```bash
sudo apt install build-essential cmake libeigen3-dev libyaml-cpp-dev \
                 libgtest-dev pybind11-dev \
                 python3-numpy python3-yaml python3-matplotlib python3-pytest
```

## Clone

Clone recursively to fetch the vendored upstream submodule:

```bash
git submodule update --init --recursive
```

## Build

```bash
cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=ON \
      -DBUILD_EXAMPLES=ON \
      -DBUILD_PYBIND=ON
cmake --build build -j
```

CMake options:

| Option                  | Default   | Description                                |
| ----------------------- | --------- | ------------------------------------------ |
| `BUILD_TESTING`         | `ON`      | Build and register the GoogleTest suite    |
| `BUILD_EXAMPLES`        | `OFF`     | Build the C++ example binary               |
| `BUILD_PYBIND`          | `OFF`     | Build the pybind11 module from the root    |
| `BUILD_DEVELOPER_TESTS` | `OFF`     | Enable lint/format/coverage checks         |
| `CMAKE_BUILD_TYPE`      | `Release` | Use `Release` for realistic solver timings |

## Install the Python package

**Development** (no `pip install` required): the build step with `BUILD_PYBIND=ON`
stages the package under `build/pybind/python/`. Add it to your path:

```bash
export PYTHONPATH=$PWD/build/pybind/python:$PYTHONPATH
```

The example launchers export this path automatically.

**Pip install** (for use as a library in another project):

```bash
pip install ./pybind          # standard
pip install -e ./pybind       # editable
```

The wheel is built with `scikit-build-core` and is self-contained
(`RPATH=$ORIGIN`).

## Tests

### C++

```bash
ctest --test-dir build --output-on-failure
```

### Python

Ensure the package is in `sys.path` (see
[Install the Python package](#install-the-python-package)), then:

```bash
python3 -m pytest pybind/tests/ -q
```

## Run the C++ example

```bash
./example/run_example_cpp.sh
```

Outputs `example/trajectory_logs/trajectory.csv` (10 cols
`t,px,py,pz,vx,vy,vz,ax,ay,az`) plus the spline CSV
`example/trajectory_logs/trajectory.spline.csv` and renders `plot_3d.png`
+ `plot_time.png`.

## Run the Python example

```bash
./example/run_example_py.sh
```

Same CSV / plots written under `example/trajectory_logs/` with the `_py`
suffix on the file names. The launcher exports `build/pybind/python/` on
`PYTHONPATH` automatically.

## C++ usage

```cpp
#include <vector>
#include <Eigen/Core>
#include <trajectory_generator_jerk_limited/trajectory_generator.hpp>

using namespace trajectory_generator_jerk_limited;

GeneratorConfig cfg;
cfg.params.max_speed        = 2.0;     // [m/s] 3D-norm bound
cfg.params.max_acceleration = 1.0;     // [m/s^2] 3D-norm bound
cfg.params.max_jerk         = 5.0;     // [m/s^3] per-axis bound

TrajectoryGenerator generator(cfg);
std::vector<Waypoint> waypoints = {
    Waypoint(Eigen::Vector3d(0.0, 0.0, 1.0)),
    Waypoint(Eigen::Vector3d(5.0, 2.0, 1.5)),
    Waypoint(Eigen::Vector3d(8.0, 0.0, 2.0)),
};
if (!generator.generate(waypoints, /*max_speed=*/3.0)) {
  throw std::runtime_error("Trajectory generation failed");
}
const auto sample = generator.evaluate(0.5 * generator.duration());
// sample.position, sample.velocity, sample.acceleration, sample.jerk
```

Link against the `trajectory_generator_jerk_limited` CMake target (e.g.
`target_link_libraries(my_app PRIVATE trajectory_generator_jerk_limited)`).

## Python usage

```python
import numpy as np
from trajectory_generator_jerk_limited import (
    GeneratorConfig, TrajectoryGenerator, Waypoint,
)

cfg = GeneratorConfig()
cfg.params.max_speed        = 2.0      # [m/s] 3D-norm bound
cfg.params.max_acceleration = 1.0      # [m/s^2] 3D-norm bound
cfg.params.max_jerk         = 5.0      # [m/s^3] per-axis bound

generator = TrajectoryGenerator(cfg)
waypoints = [
    Waypoint(np.array([0.0, 0.0, 1.0])),
    Waypoint(np.array([5.0, 2.0, 1.5])),
    Waypoint(np.array([8.0, 0.0, 2.0])),
]
assert generator.generate(waypoints, 3.0)
sample = generator.evaluate(0.5 * generator.duration())
# sample.position, sample.velocity, sample.acceleration, sample.jerk
```

## Conventions

- Frame: ENU (X east, Y north, Z up).
- Units: SI (m, m/s, m/s², m/s³, s, rad, kg, N).
- Trajectory time always starts at `t = 0`.
- After failed `generate()`, every accessor throws `std::runtime_error`
  (C++) / `RuntimeError` (Python).
- CSV schema: 10 columns `t,px,py,pz,vx,vy,vz,ax,ay,az`. Spline
  serialisation in `<base>.spline.csv` with metadata header.

## Repo-specific extras

In addition to the one-shot `TrajectoryGenerator(GeneratorConfig)`, this
library exposes a streaming API designed for real-time control loops:

- **`WaypointTrajectoryController(TrajectoryParameters)`** — feed one
  control step at a time via `update(dt, current_position, target)`
  (single-waypoint) or `update(dt, current_position, {prev, target, next})`
  (multi-waypoint with L1 corner-cut and corner-speed cap). The caller
  is responsible for rotating the `{prev, target, next}` triplet when
  the vehicle enters the acceptance radius of the active target.
- Free helpers in `trajectory_math.hpp`
  (`maxSpeedFromBrakingDistance`, `computeMaxSpeedInWaypoint`,
  `computeStartSpeedFromWaypoints`, `computeSpeedFromWaypoints`).
- Python wrapper `trajectory_generator_jerk_limited.Trajectory`
  (`pybind/python/trajectory_generator_jerk_limited/trajectory.py`):
  high-level helper that builds the native `GeneratorConfig` from a
  `GeneratorConfig` dataclass parsed from YAML
  (`load_generator_config`).

Streaming-API C++ snippet:

```cpp
TrajectoryParameters params;
params.max_speed          = 2.0;
params.max_acceleration   = 1.0;
params.max_jerk           = 5.0;
params.max_tracking_error = 0.5;
params.acceptance_radius  = 1.0;   // > 0 enables L1 corner-cut.
WaypointTrajectoryController controller(params);
controller.reset(Eigen::Vector3d::Zero());

const std::array<Eigen::Vector3d, 3> triplet = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(10.0, 0.0, 0.0),
    Eigen::Vector3d(10.0, 10.0, 0.0)};
const auto setpoints = controller.update(0.02, current_position, triplet);
```

## License

BSD 3-Clause — see [`LICENSE`](LICENSE). Copyright © 2026 Universidad
Politécnica de Madrid.

This project is an independent C++17 reimplementation of the
trajectory-shaping concepts used in
[PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)'s motion-planning
modules. The jerk-limited S-curve, axis-time synchronisation,
braking-distance speed bound, L1 corner-cut, corner-speed cap and
backward-sweep follow that design at a conceptual level. No PX4 source
code is linked or redistributed.
