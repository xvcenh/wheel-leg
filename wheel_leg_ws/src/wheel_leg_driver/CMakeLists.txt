cmake_minimum_required(VERSION 3.8)
project(wheel_leg_driver)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(wheel_leg_msgs REQUIRED)

include_directories(include)

# Main driver node
add_executable(wheel_leg_driver_node
  src/wheel_leg_driver.cpp
  src/uart_protocol.cpp
  src/wheel_leg_driver_node.cpp
)

target_link_libraries(wheel_leg_driver_node)

ament_target_dependencies(wheel_leg_driver_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  wheel_leg_msgs
)

install(TARGETS wheel_leg_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
