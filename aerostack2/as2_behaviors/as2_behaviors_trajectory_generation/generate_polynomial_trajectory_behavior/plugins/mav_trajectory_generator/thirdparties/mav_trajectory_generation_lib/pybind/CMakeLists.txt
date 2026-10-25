cmake_minimum_required(VERSION 3.16)
set(PYBIND_TARGET _mav_trajectory_generation_bindings)
set(PY_PACKAGE    mav_trajectory_generation_py)
project(${PYBIND_TARGET})

# Default to C++17.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Default to Release.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Pull mav_trajectory_generation_cpp from parent directory when not already a target.
if(NOT TARGET mav_trajectory_generation_cpp)
  set(BUILD_TESTING  OFF CACHE BOOL "" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_BINARY_DIR}/mav_trajectory_generation_cpp_build)
endif()

# pybind11 module.
pybind11_add_module(${PYBIND_TARGET} src/module.cpp)

target_include_directories(${PYBIND_TARGET} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(${PYBIND_TARGET} PRIVATE
  mav_trajectory_generation_cpp
  mav_trajectory_generation_core)

set_target_properties(${PYBIND_TARGET} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  INSTALL_RPATH "\$ORIGIN")

# Local mirror of the Python package. When PYBIND_PY_MIRROR_ROOT is set by a
# parent umbrella build, all sub-pybinds share the same python/ tree;
# otherwise fall back to this subdir's binary tree, matching gcopter's
# development layout at build/pybind/python/<pkg> when built from the
# repository root.
set(PYTHON_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/python/${PY_PACKAGE}")
if(DEFINED PYBIND_PY_MIRROR_ROOT)
  set(LOCAL_PY_MIRROR_ROOT "${PYBIND_PY_MIRROR_ROOT}")
else()
  set(LOCAL_PY_MIRROR_ROOT "${CMAKE_CURRENT_BINARY_DIR}/python")
endif()
set(PYTHON_PACKAGE_BUILD_DIR "${LOCAL_PY_MIRROR_ROOT}/${PY_PACKAGE}")

file(MAKE_DIRECTORY "${PYTHON_PACKAGE_BUILD_DIR}")
set_target_properties(${PYBIND_TARGET} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${PYTHON_PACKAGE_BUILD_DIR}")

add_custom_command(TARGET ${PYBIND_TARGET} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${PYTHON_PACKAGE_SOURCE_DIR}"
    "${PYTHON_PACKAGE_BUILD_DIR}"
  COMMENT "Mirroring ${PY_PACKAGE} sources to build tree")

# pip install via scikit-build-core.
if(DEFINED SKBUILD_PLATLIB_DIR)
  install(DIRECTORY "${PYTHON_PACKAGE_SOURCE_DIR}/"
          DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}")
  install(TARGETS ${PYBIND_TARGET}
    LIBRARY DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}"
    RUNTIME DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}"
    ARCHIVE DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}")
  # Install transitive shared libraries so the wheel is self-contained.
  foreach(_dep_target IN ITEMS mav_trajectory_generation_cpp mav_trajectory_generation_core nlopt)
    if(TARGET ${_dep_target})
      get_target_property(_dep_type ${_dep_target} TYPE)
      if(_dep_type STREQUAL "SHARED_LIBRARY")
        set_target_properties(${_dep_target} PROPERTIES
          INSTALL_RPATH "\$ORIGIN"
          BUILD_WITH_INSTALL_RPATH TRUE)
        install(TARGETS ${_dep_target}
          LIBRARY DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}"
          RUNTIME DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}")
      endif()
    endif()
  endforeach()
endif()
