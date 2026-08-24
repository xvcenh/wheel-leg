cmake_minimum_required(VERSION 3.8)
project(wheel_leg_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(wheel_leg_msgs REQUIRED)

# State machine node
add_executable(state_machine_node src/state_machine_node.cpp)
ament_target_dependencies(state_machine_node
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs wheel_leg_msgs)

install(TARGETS state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
