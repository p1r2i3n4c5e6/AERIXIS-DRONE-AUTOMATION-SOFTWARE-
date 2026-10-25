# GCOPTER Trajectory Generation Library

ROS-free C++17 wrapper around the
[GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) MINCO + L-BFGS quintic
polynomial trajectory optimiser, with optional pybind11 bindings and a
self-contained CLI example. Released under BSD-3-Clause.

## Overview

`gcopter_lib` plans smooth quintic-piece polynomial trajectories through
3D waypoint sequences inside an axis-aligned safe-flight corridor (SFC),
honouring drone-level magnitude bounds (velocity, body rate, tilt,
collective thrust) via the L-BFGS optimiser. The public API is built
around a single one-shot entry point `TrajectoryGenerator(GeneratorConfig)`
common to the trajectory-generation library family. Frame is ENU, units
are SI, trajectory time always starts at `t = 0`. Sampled CSV output is
the canonical 10-column schema `t,px,py,pz,vx,vy,vz,ax,ay,az`; the
underlying spline is serialisable to `<base>.spline.csv`.

## Repository layout

```
.
├── CMakeLists.txt
├── include/gcopter_lib/        # Public headers (stable API)
│   ├── types.hpp               # POD configs (GeneratorConfig, DroneParameters, ...)
│   ├── corridor_builder.hpp    # AABB H-rep builder (gcopter-specific extra)
│   ├── trajectory_generator.hpp
│   └── trajectory_logger.hpp
├── src/                        # Implementations (Pimpl)
├── example/                    # End-to-end CLI demo (utils + run_example.{cpp,py})
├── tests/                      # GoogleTest, auto-discovered *_gtest.cpp
├── pybind/                     # pybind11 module + Python package mirror (`gcopterpy`)
└── GCOPTER/                    # Vendored upstream submodule (ROS-free)
```

## Dependencies

- C++17 compiler (GCC ≥ 9 / Clang ≥ 10).
- CMake ≥ 3.5.
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
#include <gcopter_lib/trajectory_generator.hpp>

using namespace gcopter_lib;

GeneratorConfig cfg;
cfg.limits.max_velocity = 4.0;          // [m/s] hard magnitude bound

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

Link against the `gcopter_lib` CMake target (e.g.
`target_link_libraries(my_app PRIVATE gcopter_lib)`).

## Python usage

```python
import numpy as np
from gcopterpy import GeneratorConfig, TrajectoryGenerator, Waypoint

cfg = GeneratorConfig()
cfg.limits.max_velocity = 4.0           # [m/s] hard magnitude bound

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

- `buildAabbCorridor(positions, margin)` — explicit AABB H-representation
  builder. Useful to inspect or pre-validate the SFC the optimiser sees.
- `generate(waypoints, max_speed, corridor_margins)` — overload accepting
  per-segment AABB margins (`corridor_margins.size() == waypoints.size() - 1`,
  every entry > 0).
- Python wrapper `gcopterpy.Trajectory` (`pybind/python/gcopterpy/trajectory.py`):
  high-level helper that builds the native `GeneratorConfig` from a
  `TrajectoryConfig` dataclass parsed from YAML (`load_trajectory_config`
  + `load_sim_config`).

## License

BSD 3-Clause — see [`LICENSE`](LICENSE). Copyright © 2026 Universidad
Politécnica de Madrid.

The vendored upstream optimiser under `GCOPTER/` is authored by the
ZJU-FAST-Lab team (Zhepei Wang et al.) and retains its own license; see
`GCOPTER/LICENSE`. Distributions of this repository must preserve both
notices.
