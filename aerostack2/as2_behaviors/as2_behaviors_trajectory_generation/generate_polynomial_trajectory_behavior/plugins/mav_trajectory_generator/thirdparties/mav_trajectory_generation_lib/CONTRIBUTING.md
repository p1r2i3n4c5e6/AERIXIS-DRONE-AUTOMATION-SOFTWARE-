# Contributing

Thanks for your interest. Please keep the following in mind.

## Scope

This repository is a **thin, ROS-free wrapper** around the ETH-ASL
[`mav_trajectory_generation`](https://github.com/ethz-asl/mav_trajectory_generation)
core. Keep contributions focused on:

* The C++ facade in [`include/mav_trajectory_generation_cpp/`](./include/mav_trajectory_generation_cpp/)
  and [`src/`](./src/).
* The pybind11 bindings and the `mav_trajectory_generation_py` Python
  package under [`pybind/`](./pybind/).
* Tests ([`tests/`](./tests/), [`pybind/tests/`](./pybind/tests/)) and
  example usage ([`example/`](./example/)).
* Build system glue at the top level.

## Do not modify the submodule

[`mav_trajectory_generation/`](./mav_trajectory_generation/) is a pinned git
submodule of the upstream repository. **It is not modified here.** If a
patch is unavoidable, upstream it first and bump the submodule pointer
afterwards.

If the upstream adds / renames / removes compilable files, update the
`MTG_CORE_SRC` list in [`CMakeLists.txt`](./CMakeLists.txt); that is the
single source of truth for which upstream translation units the facade
compiles against.

## Coding style

* C++17, modern RAII.
* Formatter: `clang-format` against the repository
  [`.clang-format`](./.clang-format).
* Comments in English.
* Keep the facade's public API (`include/mav_trajectory_generation_cpp/`)
  free of upstream headers — consumers should not transitively pick up glog
  or NLopt through our public includes.

## Running the tests

```bash
cmake -S . -B build -DBUILD_TESTING=ON -DBUILD_PYBIND=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
export PYTHONPATH="$(pwd)/build/pybind/python:${PYTHONPATH}"
pytest pybind/tests
```

## Dependencies

Runtime / build: Eigen3, glog, NLopt, yaml-cpp, pybind11 (optional),
GoogleTest (optional, for the test suite). `glog` and `NLopt` are fetched
via `FetchContent` when not found on the system.
