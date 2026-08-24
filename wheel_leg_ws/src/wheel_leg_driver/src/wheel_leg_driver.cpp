/**
 * @file wheel_leg_driver.cpp
 * @brief Implementation of the wheel-legged robot UART bridge driver.
 */

#include "wheel_leg_driver/wheel_leg_driver.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

namespace wheel_leg_driver
{

WheelLegDriver::WheelLegDriver(const rclcpp::NodeOptions& options)
  : Node("wheel_leg_driver", options)
{
  /* Declare parameters */
  this->declare_parameter("serial_port", "/dev/ttyS1");
  this->declare_parameter("baud_rate", 115200);
  this->declare_parameter("wheel_radius", 0.0603);   /* from VMC code */
  this->declare_parameter("track_width", 0.30);       /* adjustable */

  serial_port_  = this->get_parameter("serial_port").as_string();
  baud_rate_    = this->get_parameter("baud_rate").as_int();
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  track_width_  = this->get_parameter("track_width").as_double();

  /* Publishers */
  odom_pub_   = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  status_pub_ = this->create_publisher<wheel_leg_msgs::msg::LegStatus>("/wheel_leg/status", 10);
  imu_pub_    = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

  /* Subscribers */
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&WheelLegDriver::cmd_vel_callback, this, std::placeholders::_1));

  cmd_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/wheel_leg/cmd_start", 10,
    std::bind(&WheelLegDriver::cmd_start_callback, this, std::placeholders::_1));

  /* TF broadcaster */
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  /* Timer: 10ms cycle for UART communication */
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&WheelLegDriver::timer_callback, this));

  /* Open serial port */
  if (!uart_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
  }

  last_odom_time_ = this->now();
  last_telemetry_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "WheelLegDriver started on %s @ %d baud",
    serial_port_.c_str(), baud_rate_);
}

WheelLegDriver::~WheelLegDriver()
{
  /* Send emergency stop before closing */
  uint8_t stop_buf[RX_FRAME_SIZE];
  pack_emergency_cmd(stop_buf);
  uart_send_frame(stop_buf, RX_FRAME_SIZE);
  uart_close();
}

/* ====== Subscriber Callbacks ====== */

void WheelLegDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vx_   = static_cast<float>(msg->linear.x);
  cmd_omega_ = static_cast<float>(msg->angular.z);
}

void WheelLegDriver::cmd_start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  uint8_t buf[RX_FRAME_SIZE];
  pack_start_cmd(buf, msg->data);
  if (uart_send_frame(buf, RX_FRAME_SIZE)) {
    robot_started_ = msg->data;
    if (robot_started_) {
      /* Reset odometry on start */
      odom_x_ = 0.0f;
      odom_y_ = 0.0f;
      odom_yaw_ = 0.0f;
      last_odom_x_ = 0.0f;
      last_odom_yaw_ = 0.0f;
    }
    RCLCPP_INFO(this->get_logger(), "Robot %s", robot_started_ ? "STARTED" : "STOPPED");
  }
}

/* ====== Timer Callback (10ms) ====== */

void WheelLegDriver::timer_callback()
{
  /* 1. Send velocity command over UART */
  if (robot_started_) {
    uint8_t tx_buf[RX_FRAME_SIZE];
    pack_velocity_cmd(tx_buf, cmd_vx_, cmd_omega_, cmd_leg_, robot_started_);
    uart_send_frame(tx_buf, RX_FRAME_SIZE);
  } else {
    /* Keep sending zero velocity to let STM32 know we're alive */
    uint8_t tx_buf[RX_FRAME_SIZE];
    pack_velocity_cmd(tx_buf, 0.0f, 0.0f, cmd_leg_, false);
    uart_send_frame(tx_buf, RX_FRAME_SIZE);
  }

  /* 2. Read telemetry from UART */
  int n = uart_read_frame();
  Telemetry telem;
  if (n >= TX_FRAME_SIZE && parse_telemetry(rx_buffer_, TX_FRAME_SIZE, telem)) {
    last_telemetry_time_ = this->now();

    /* Publish odometry */
    auto now = this->now();
    double dt = (now - last_odom_time_).seconds();
    if (dt <= 0.0) dt = 0.01;

    /* Integrate from STM32 odometry */
    odom_x_   = telem.x_pos;
    odom_yaw_ = telem.yaw;

    /* Publish standard odometry message */
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0.0;

    double half_yaw = odom_yaw_ * 0.5;
    odom_msg.pose.pose.orientation.z = std::sin(half_yaw);
    odom_msg.pose.pose.orientation.w = std::cos(half_yaw);

    /* Twist from telemetry */
    odom_msg.twist.twist.linear.x = telem.vx;
    odom_msg.twist.twist.angular.z = telem.omega;

    odom_pub_->publish(odom_msg);

    /* Publish TF */
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = odom_x_;
    tf_msg.transform.translation.y = odom_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.z = std::sin(half_yaw);
    tf_msg.transform.rotation.w = std::cos(half_yaw);
    tf_broadcaster_->sendTransform(tf_msg);

    /* Publish IMU */
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation_covariance[0] = -1.0;  /* orientation unknown */
    imu_msg.angular_velocity.z = telem.omega;
    imu_msg.linear_acceleration.x = 0.0;  /* not available from BMI088 over UART yet */
    imu_pub_->publish(imu_msg);

    /* Publish leg status */
    wheel_leg_msgs::msg::LegStatus status_msg;
    status_msg.left_leg_length  = telem.left_leg;
    status_msg.right_leg_length = telem.right_leg;
    status_msg.pitch  = telem.pitch;
    status_msg.roll   = telem.roll;
    status_msg.vx     = telem.vx;
    status_msg.omega  = telem.omega;
    status_msg.motor_state = robot_started_ ? 1 : 0;
    status_pub_->publish(status_msg);

    last_odom_time_ = now;
    last_odom_x_ = odom_x_;
    last_odom_yaw_ = odom_yaw_;
  }

  /* Check telemetry timeout */
  double t_since = (this->now() - last_telemetry_time_).seconds();
  if (t_since > 0.5) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "No telemetry from STM32 for %.1f sec", t_since);
  }
}

/* ====== UART Operations ====== */

bool WheelLegDriver::uart_open()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open %s: %s", serial_port_.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  /* 8N1, raw mode */
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
  tty.c_oflag &= ~(OPOST | ONLCR);

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 1;  /* 100ms timeout */

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void WheelLegDriver::uart_close()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool WheelLegDriver::uart_send_frame(const uint8_t* data, uint8_t len)
{
  if (serial_fd_ < 0) return false;
  ssize_t written = write(serial_fd_, data, len);
  return (written == len);
}

int WheelLegDriver::uart_read_frame()
{
  if (serial_fd_ < 0) return 0;
  int total = 0;
  int retries = 10;

  /* Attempt to read a full TX frame (35 bytes) */
  while (total < TX_FRAME_SIZE && retries > 0) {
    int n = read(serial_fd_, rx_buffer_ + total, TX_FRAME_SIZE - total);
    if (n > 0) {
      total += n;
    } else if (n < 0 && errno != EAGAIN) {
      break;
    }
    retries--;
  }

  /* Frame sync: discard bytes until header found */
  int header_pos = -1;
  for (int i = 0; i < total - 1; ++i) {
    if (rx_buffer_[i] == FRAME_HEADER0 && rx_buffer_[i+1] == FRAME_HEADER1) {
      header_pos = i;
      break;
    }
  }

  if (header_pos > 0) {
    /* Shift buffer to align header at position 0 */
    std::memmove(rx_buffer_, rx_buffer_ + header_pos, total - header_pos);
    total -= header_pos;
  }

  return total;
}

}  // namespace wheel_leg_driver
