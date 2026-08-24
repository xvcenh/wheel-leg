cmake_minimum_required(VERSION 3.8)
project(wheel_leg_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(wheel_leg_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# Line follower node (HSV color thresholding)
add_executable(line_detector_node src/line_detector_node.cpp)
target_link_libraries(line_detector_node ${OpenCV_LIBS})
ament_target_dependencies(line_detector_node rclcpp sensor_msgs cv_bridge wheel_leg_msgs)

# Box detector node (HSV + rough depth from stereo)
add_executable(box_detector_node src/box_detector_node.cpp)
target_link_libraries(box_detector_node ${OpenCV_LIBS})
ament_target_dependencies(box_detector_node rclcpp sensor_msgs cv_bridge wheel_leg_msgs)

install(TARGETS line_detector_node box_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
