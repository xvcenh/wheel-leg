cmake_minimum_required(VERSION 3.8)
project(rl_policy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ONNX Runtime (for RL inference on BPU or CPU)
# rl_policy_node needs onnxruntime; mcu_telemetry_node does not.
# Build rl_policy_node only when onnxruntime is available: -DBUILD_RL_POLICY=ON
option(BUILD_RL_POLICY "Build RL policy node (requires onnxruntime)" OFF)
find_library(ONNXRUNTIME onnxruntime HINTS /usr/lib /usr/local/lib)
if(BUILD_RL_POLICY AND ONNXRUNTIME)

# Main RL policy node
add_executable(rl_policy_node
  src/rl_policy_node.cpp
  src/uart_bridge.cpp
  src/can_bridge.cpp
)

target_include_directories(rl_policy_node PUBLIC
  include
)

target_link_libraries(rl_policy_node
  ${ONNXRUNTIME}
  pthread
)

ament_target_dependencies(rl_policy_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS rl_policy_node
  DESTINATION lib/${PROJECT_NAME}
)

endif()  # BUILD_RL_POLICY

# MCU telemetry node: parse STM32 sensor data, publish ROS topics
add_executable(mcu_telemetry_node
  src/mcu_telemetry_node.cpp
  src/uart_bridge.cpp
  src/can_bridge.cpp
)

target_include_directories(mcu_telemetry_node PUBLIC
  include
)

ament_target_dependencies(mcu_telemetry_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
)

install(TARGETS mcu_telemetry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models scripts
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 可执行脚本
install(FILES scripts/telemetry_dashboard.html DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/telemetry_web_node.py
  scripts/pos_web_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
