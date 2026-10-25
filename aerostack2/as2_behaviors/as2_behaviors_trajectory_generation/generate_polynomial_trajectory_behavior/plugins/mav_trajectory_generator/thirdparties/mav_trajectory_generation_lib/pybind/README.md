# mav_trajectory_generation_py

Python bindings for
[`mav_trajectory_generation_lib`](../README.md) — a pure C++ (ROS-free)
wrapper around ETH-ASL `mav_trajectory_generation`.

## Install

```bash
pip install ./pybind          # from the repository root
# or, from this directory:
pip install .
```

This uses [scikit-build-core](https://scikit-build-core.readthedocs.io/) to
build the native extension (`_mav_trajectory_generation_bindings`) alongside
the pure-Python package.

## Quick start

```python
import numpy as np
from mav_trajectory_generation_py import (
    EndWaypoint, GeneratorConfig, Trajectory, Waypoint,
)

cfg = GeneratorConfig(solver="nonlinear", a_max=4.0)
traj = Trajectory(cfg)

waypoints = [
    EndWaypoint(np.array([0.0, 0.0, 1.0])),   # start at rest
    Waypoint(np.array([5.0, 2.0, 1.5])),      # intermediate (free vel/acc)
    EndWaypoint(np.array([8.0, 0.0, 2.0])),   # end at rest
]
assert traj.generate(waypoints, max_speed=3.0)

t = 0.5 * traj.duration
point = traj.evaluate(t)
print(point.position, point.velocity, point.acceleration)
```

Plain numpy arrays are also accepted (auto-wrapped as unconstrained
``Waypoint``; endpoints still default to rest thanks to the facade logic).

## Public API

Re-exported from `mav_trajectory_generation_py`:

| Symbol | Origin | Purpose |
| --- | --- | --- |
| `Solver` | native | Enum: `linear` / `nonlinear`. |
| `OptimizationConfig` | native | Native tunables struct. |
| `Waypoint` | native | Waypoint with optional velocity/acceleration constraints. |
| `EndWaypoint` | native | `Waypoint` subtype with velocity / acceleration pinned to zero. |
| `TrajectorySample` | native | Evaluation result (`position`/`velocity`/`acceleration`). |
| `TrajectoryGenerator` | native | Low-level `generate()` / `evaluate()` engine. |
| `GeneratorConfig` | pure-Python | Dataclass mirror of `OptimizationConfig`. |
| `TrajectoryPoint` | pure-Python | Dataclass returned by `Trajectory.evaluate`. |
| `Trajectory` | pure-Python | High-level wrapper with input validation. |
| `load_generator_config` | pure-Python | YAML loader (`optimization:` mapping). |

## Tests

```bash
pytest pybind/tests
```

The suite requires the native bindings to be importable; either install the
package with `pip install .` or add the local build mirror to `PYTHONPATH`:

```bash
export PYTHONPATH="$(pwd)/build/pybind/python:${PYTHONPATH}"
pytest pybind/tests
```
