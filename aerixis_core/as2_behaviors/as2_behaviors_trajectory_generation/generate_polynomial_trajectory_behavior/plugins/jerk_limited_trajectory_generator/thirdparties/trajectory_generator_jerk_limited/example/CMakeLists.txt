find_package(yaml-cpp REQUIRED)

add_executable(${PROJECT_NAME}_run_example run_example.cpp)

target_include_directories(${PROJECT_NAME}_run_example PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}_run_example
  ${PROJECT_NAME}
  yaml-cpp
)

target_compile_features(${PROJECT_NAME}_run_example PRIVATE cxx_std_17)

# Stable location for the example binary; mirrors the layout of the other
# trajectory_generator libraries in the workspace (gcopter_lib, etc.).
set_target_properties(${PROJECT_NAME}_run_example PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/example"
)
