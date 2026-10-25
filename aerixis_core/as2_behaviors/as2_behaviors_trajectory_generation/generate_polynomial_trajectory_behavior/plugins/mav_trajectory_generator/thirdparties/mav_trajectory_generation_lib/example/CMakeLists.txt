find_package(yaml-cpp REQUIRED)

set(_example_target ${PROJECT_NAME}_run_example)

add_executable(${_example_target} run_example.cpp)
target_link_libraries(${_example_target} PRIVATE
  mav_trajectory_generation_cpp
  yaml-cpp)
target_compile_options(${_example_target} PRIVATE -O3 -Wall -Wextra)
