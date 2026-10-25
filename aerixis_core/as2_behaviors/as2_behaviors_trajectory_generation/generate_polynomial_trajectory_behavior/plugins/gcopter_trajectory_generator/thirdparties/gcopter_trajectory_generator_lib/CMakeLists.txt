cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME gcopter_lib)
project(${PROJECT_NAME})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Position-independent code so this library can be linked into shared objects
# (namely the pybind11 Python module).
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
set(PROJECT_DEPENDENCIES
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# GCOPTER upstream headers (ROS-free algorithmic core).
# GCOPTER/ is vendored as an in-tree "submodule" so the library is
# self-contained. Only the algorithmic headers under gcopter/include are used;
# the ROS-dependent src/ node and the visualizer are ignored.
set(GCOPTER_UPSTREAM_INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/GCOPTER/gcopter/include
)

if(NOT EXISTS ${GCOPTER_UPSTREAM_INCLUDE}/gcopter/gcopter.hpp)
  message(FATAL_ERROR
    "GCOPTER upstream headers not found at ${GCOPTER_UPSTREAM_INCLUDE}. "
    "Expected vendored directory GCOPTER/gcopter/include inside this package.")
endif()

# Library
set(SOURCE_LIB_FILES
  src/corridor_builder.cpp
  src/trajectory_generator.cpp
  src/trajectory_logger.cpp
)

add_library(${PROJECT_NAME} ${SOURCE_LIB_FILES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    $<BUILD_INTERFACE:${GCOPTER_UPSTREAM_INCLUDE}>
)

# Treat GCOPTER headers as system headers to suppress third-party warnings
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-isystem;${GCOPTER_UPSTREAM_INCLUDE}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PRIVATE -O3 -Wall -Wextra)

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
