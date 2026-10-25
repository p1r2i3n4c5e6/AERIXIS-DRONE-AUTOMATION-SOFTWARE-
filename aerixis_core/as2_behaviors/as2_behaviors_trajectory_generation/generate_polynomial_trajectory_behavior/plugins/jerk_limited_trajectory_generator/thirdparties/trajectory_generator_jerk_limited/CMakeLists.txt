cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME trajectory_generator_jerk_limited)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Position-independent code so this static library can be linked into
# shared objects (namely the pybind11 Python module).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
set(PROJECT_DEPENDENCIES
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Library
set(SOURCE_LIB_FILES
  src/trajectory_math.cpp
  src/velocity_profile_1d.cpp
  src/trajectory_generator_3d.cpp
  src/waypoint_controller.cpp
  src/types.cpp
  src/trajectory_generator.cpp
)

add_library(${PROJECT_NAME} ${SOURCE_LIB_FILES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

# Tests
include(CTest)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

# Examples
option(BUILD_EXAMPLES "Build C++ examples" OFF)
if(BUILD_EXAMPLES)
  add_subdirectory(example)
endif()

# Python bindings
option(BUILD_PYBIND "Build Python bindings (pybind11 module)" OFF)
if(BUILD_PYBIND)
  add_subdirectory(pybind)
endif()
