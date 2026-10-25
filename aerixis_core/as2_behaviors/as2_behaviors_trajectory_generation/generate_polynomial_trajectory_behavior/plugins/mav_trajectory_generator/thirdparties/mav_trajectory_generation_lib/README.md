# mav_trajectory_generation_lib

ROS-free C++17 facade around ETH-ASL
[mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation)
(degree-10 polynomial trajectories, Linear closed-form or NLopt
nonlinear solver), with optional pybind11 bindings and a self-contained
CLI example. Released under BSD-3-Clause.

## Overview

`mav_trajectory_generation_lib` plans smooth polynomial trajectories
(degree 10) through 3D waypoint sequences. The closed-form linear solver
uses the segment-time allocation only; the NLopt-backed nonlinear solver
additionally enforces velocity and acceleration magnitude bounds. The
public API is built around a single one-shot entry point
`TrajectoryGenerator(GeneratorConfig)` common to the trajectory-generation
library family. Frame is ENU, units are SI, trajectory time always
starts at `t = 0`. Sampled CSV output is the canonical 10-column schema
`t,px,py,pz,vx,vy,vz,ax,ay,az`; the underlying spline is serialisable to
`<base>.spline.csv`.

## Repository layout

```
.
├── CMakeLists.txt
├── cmake/                                          # FetchContent shims for glog and NLopt
├── include/mav_trajectory_generation_cpp/          # Public headers (stable API)
│   ├── types.hpp                                   # POD configs (GeneratorConfig, OptimizationConfig, ...)
│   └── trajectory_generator.hpp
├── src/                                            # Implementation (Pimpl)
├── example/                                        # End-to-end CLI demo (utils + run_example.{cpp,py})
├── tests/                                          # GoogleTest, auto-discovered *_gtest.cpp
├── pybind/                                         # pybind11 module + Python package mirror (`mav_trajectory_generation_py`)
└── mav_trajectory_generation/                      # Vendored upstream submodule (immutable)
```

## Dependencies

- C++17 compiler (GCC ≥ 9 / Clang ≥ 10).
- CMake ≥ 3.16.
- [Eigen 3](https://eigen.tuxfamily.org/).
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) (only when `BUILD_EXAMPLES=ON`).
- [glog](https://github.com/google/glog) and [NLopt](https://github.com/stevengj/nlopt).
  If not installed system-wide, both are auto-fetched via CMake
  `FetchContent` (see [`cmake/`](./cmake/)).
- [GoogleTest](https://github.com/google/googletest) (C++ test suite).
- [pybind11](https://github.com/pybind/pybind11) ≥ 2.9 + Python ≥ 3.8 + numpy
  (Python bindings).
- `PyYAML`, `matplotlib` (used by `example/run_example.py` and
  `example/plot_results.py`).

Ubuntu / Debian:

```bash
sudo apt install build-essential cmake libeigen3-dev libyaml-cpp-dev \
                 libgoogle-glog-dev libnlopt-dev \
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
#include <mav_trajectory_generation_cpp/trajectory_generator.hpp>

using namespace mav_trajectory_generation_cpp;

GeneratorConfig cfg;
cfg.optimization.solver = Solver::Nonlinear;   // NLopt; enforces v/a bounds
cfg.optimization.a_max  = 4.0;                  // [m/s^2]

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

Link against the `mav_trajectory_generation_cpp` CMake target (e.g.
`target_link_libraries(my_app PRIVATE mav_trajectory_generation_cpp)`).

## Python usage

```python
import numpy as np
from mav_trajectory_generation_py import (
    GeneratorConfig, Solver, TrajectoryGenerator, Waypoint,
)

cfg = GeneratorConfig()
cfg.optimization.solver = Solver.nonlinear      # lower-case in Python
cfg.optimization.a_max  = 4.0                   # [m/s^2]

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

- Solver selector `Solver::Linear` / `Solver::Nonlinear`
  (`Solver.linear` / `Solver.nonlinear` in Python). The linear path is
  closed-form least-squares; only the nonlinear path enforces velocity
  and acceleration magnitude bounds via NLopt.
- `setSpline(const Spline&)` — replay a previously serialised spline
  without re-running the solver. Useful for offline-planned trajectories
  fed to the same evaluator.
- Python wrapper `mav_trajectory_generation_py.Trajectory`
  (`pybind/python/mav_trajectory_generation_py/trajectory.py`):
  high-level helper that builds the native `GeneratorConfig` from a
  `PyGeneratorConfig` dataclass parsed from YAML
  (`load_generator_config`).

## License

BSD 3-Clause — see [`LICENSE`](LICENSE). Copyright © 2026 Universidad
Politécnica de Madrid.

The vendored upstream algorithmic core under `mav_trajectory_generation/`
is authored by the ETH-ASL team (Achtelik, Burri, Oleynikova, Bähnemann,
Popović) and licensed under Apache-2.0. BSD-3-Clause and Apache-2.0 are
compatible: distributions of this repository must preserve both notices.

If the upstream submodule ever adds or renames `.cpp` files, the list
`MTG_CORE_SRC` in [`CMakeLists.txt`](./CMakeLists.txt) must be updated to
keep the facade in sync.
