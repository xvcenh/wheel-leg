/**
 * @file wheel_leg_driver_node.cpp
 * @brief Main entry point for the wheel_leg_driver node.
 */

#include "wheel_leg_driver/wheel_leg_driver.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<wheel_leg_driver::WheelLegDriver>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
