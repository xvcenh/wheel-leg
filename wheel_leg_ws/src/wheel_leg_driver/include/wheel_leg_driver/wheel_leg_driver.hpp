/**
 * @file wheel_leg_driver.hpp
 * @brief ROS2 node: UART bridge between RDK X5 and STM32 wheel-legged robot.
 *
 * Subscribes:
 *   /cmd_vel (geometry_msgs/Twist) -- standard velocity commands
 *   /wheel_leg/cmd_start (std_msgs/Bool) -- start/stop the robot
 *
 * Publishes:
 *   /odom       (nav_msgs/Odometry) -- wheel odometry
 *   /wheel_leg/status (wheel_leg_msgs/LegStatus) -- robot status
 *   /tf         -- odom -> base_link transform
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <wheel_leg_msgs/msg/leg_status.hpp>

#include "wheel_leg_driver/uart_protocol.hpp"
#include <string>
#include <fstream>
#include <termios.h>

namespace wheel_leg_driver
{

class WheelLegDriver : public rclcpp::Node
{
public:
  explicit WheelLegDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~WheelLegDriver();

private:
  /* Timer: 10ms cycle (100Hz) for UART send/receive */
  rclcpp::TimerBase::SharedPtr timer_;

  /* Subscribers */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr     cmd_start_sub_;

  /* Publishers */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              odom_pub_;
  rclcpp::Publisher<wheel_leg_msgs::msg::LegStatus>::SharedPtr        status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                 imu_pub_;

  /* TF broadcaster */
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /* Callbacks */
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmd_start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void timer_callback();

  /* UART I/O */
  bool uart_open();
  void uart_close();
  bool uart_send_frame(const uint8_t* data, uint8_t len);
  int  uart_read_frame();

  static constexpr int SERIAL_READ_TIMEOUT_MS = 5;

  /* State */
  int      serial_fd_ = -1;
  bool     robot_started_ = false;
  float    cmd_vx_ = 0.0f;
  float    cmd_omega_ = 0.0f;
  float    cmd_leg_ = 0.08f;
  uint8_t  rx_buffer_[256];

  /* Odometry */
  float    odom_x_ = 0.0f;
  float    odom_y_ = 0.0f;
  float    odom_yaw_ = 0.0f;
  float    last_odom_x_ = 0.0f;
  float    last_odom_yaw_ = 0.0f;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_telemetry_time_;

  /* Parameters */
  std::string serial_port_;
  int         baud_rate_;
  float       wheel_radius_;
  float       track_width_;
};

}  // namespace wheel_leg_driver
