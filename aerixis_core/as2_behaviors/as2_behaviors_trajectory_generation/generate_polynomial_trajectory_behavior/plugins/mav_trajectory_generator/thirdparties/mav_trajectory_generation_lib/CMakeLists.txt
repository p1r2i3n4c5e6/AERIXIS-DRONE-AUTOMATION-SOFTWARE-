cmake_minimum_required(VERSION 3.16)
set(PROJECT_NAME mav_trajectory_generation_cpp)
project(${PROJECT_NAME})

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to Release.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Position-independent code so this library can be linked into shared objects
# (namely the pybind11 Python module).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Dependencies.
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/glog_dependencies.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/nlopt_dependencies.cmake)
find_package(yaml-cpp REQUIRED)

# ---------------------------------------------------------------------------
# Core sublibrary: compile upstream .cpp sources from *outside* the submodule.
#
# The submodule ships with its own catkin-based CMakeLists.txt which we do not
# use. We list the compilable sources here and expose the upstream include
# directory as SYSTEM to silence third-party warnings.
#
# trajectory_sampling.cpp is deliberately excluded because it is the only
# upstream translation unit that requires mav_msgs (a ROS-only dependency).
# Equivalent sampling is available natively through
# mav_trajectory_generation::Trajectory::evaluate(t, derivative_order).
#
# io.cpp is also excluded: it transitively includes trajectory_sampling.h,
# which would drag in mav_msgs. The facade does not expose YAML I/O of
# trajectories — callers that need it can re-enable io.cpp by vendoring stubs
# for mav_msgs::EigenTrajectoryPoint.
# ---------------------------------------------------------------------------
set(MTG_CORE_ROOT
  ${CMAKE_CURRENT_SOURCE_DIR}/mav_trajectory_generation/mav_trajectory_generation)

if(NOT EXISTS ${MTG_CORE_ROOT}/include/mav_trajectory_generation/trajectory.h)
  message(FATAL_ERROR
    "ETH-ASL mav_trajectory_generation submodule not found at ${MTG_CORE_ROOT}. "
    "Did you forget to run 'git submodule update --init --recursive'?")
endif()

set(MTG_CORE_SRC
  ${MTG_CORE_ROOT}/src/motion_defines.cpp
  ${MTG_CORE_ROOT}/src/polynomial.cpp
  ${MTG_CORE_ROOT}/src/segment.cpp
  ${MTG_CORE_ROOT}/src/timing.cpp
  ${MTG_CORE_ROOT}/src/trajectory.cpp
  ${MTG_CORE_ROOT}/src/vertex.cpp
  ${MTG_CORE_ROOT}/src/rpoly/rpoly_ak1.cpp
)

# Named distinctly from a plain `mav_trajectory_generation` target so this
# library can coexist with other packages that vendor the same upstream
# sources in one build tree, without CMake target collisions.
add_library(mav_trajectory_generation_core STATIC ${MTG_CORE_SRC})
target_include_directories(mav_trajectory_generation_core SYSTEM PUBLIC
  $<BUILD_INTERFACE:${MTG_CORE_ROOT}/include>)
target_link_libraries(mav_trajectory_generation_core PUBLIC
  Eigen3::Eigen
  glog::glog
  nlopt
  yaml-cpp)
set_target_properties(mav_trajectory_generation_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Public facade (Pimpl).
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} src/trajectory_generator.cpp src/segment_timing.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
  PRIVATE
    ${MTG_CORE_ROOT}/include)
target_link_libraries(${PROJECT_NAME}
  PUBLIC  Eigen3::Eigen
  PRIVATE mav_trajectory_generation_core)
target_compile_options(${PROJECT_NAME} PRIVATE -O3 -Wall -Wextra)

# Hide the symbols inherited from the statically-linked upstream
# `mav_trajectory_generation_core` so they are not exported by this .so. This
# avoids ODR / symbol-collision when the same process also loads another
# library that vendors the upstream ETH-ASL `mav_trajectory_generation`.
# Without this, the dynamic linker collapses both copies into one global
# definition, and the facade ends up calling the wrong implementation of
# `estimateSegmentTimes`, `solveLinear`, etc., yielding trajectories that
# differ from the standalone library and from the Python bindings.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(${PROJECT_NAME} PRIVATE
    "LINKER:--exclude-libs,ALL")
endif()

# ---------------------------------------------------------------------------
# Tests / examples / pybind11 (all opt-in).
# ---------------------------------------------------------------------------
include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

option(BUILD_EXAMPLES "Build C++ examples" OFF)
if(BUILD_EXAMPLES)
  add_subdirectory(example)
endif()

option(BUILD_PYBIND "Build Python bindings (pybind11 module)" OFF)
if(BUILD_PYBIND)
  add_subdirectory(pybind)
endif()

# Developer-only tooling (cpplint, clang-format, coverage). OFF by default to
# match the sibling libraries gcopter_lib and trajectory_generator_jerk_limited.
option(BUILD_DEVELOPER_TESTS "Run cpplint / clang-format / lcov on the source tree" OFF)
