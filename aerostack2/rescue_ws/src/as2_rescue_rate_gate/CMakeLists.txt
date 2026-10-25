cmake_minimum_required(VERSION 3.10)
project(as2_rescue_rate_gate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(rate_gate src/rate_gate.cpp)
ament_target_dependencies(rate_gate rclcpp tf2_msgs)
install(TARGETS rate_gate DESTINATION lib/${PROJECT_NAME})

ament_package()
