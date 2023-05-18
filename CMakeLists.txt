cmake_minimum_required(VERSION 3.5)
project(turn_offset_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)

add_executable(turn_offset_controller src/turn_offset_controller.cpp)
ament_target_dependencies(turn_offset_controller rclcpp autoware_auto_vehicle_msgs tier4_planning_msgs)

install(TARGETS
  turn_offset_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()

