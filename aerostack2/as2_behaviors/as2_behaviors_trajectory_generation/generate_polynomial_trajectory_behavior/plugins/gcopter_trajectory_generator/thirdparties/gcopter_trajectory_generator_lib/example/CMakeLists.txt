find_package(yaml-cpp REQUIRED)

add_executable(${PROJECT_NAME}_run_example run_example.cpp)

target_include_directories(${PROJECT_NAME}_run_example PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}_run_example
  ${PROJECT_NAME}
  yaml-cpp
)

# Stable location for the example binary; mirrors mpc_examples layout.
set_target_properties(${PROJECT_NAME}_run_example PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/example"
)
