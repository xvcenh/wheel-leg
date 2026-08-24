#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rl_policy/bridge_base.hpp"
#include "rl_policy/uart_bridge.hpp"
#include "rl_policy/can_bridge.hpp"

#include <memory>
#include <string>

namespace rl_policy
{

// 独立节点：解析 STM32 传感器数据并发布为 ROS topic（CAN3 或 UART 通道），
// 同时接收控制信号（/cmd_vel, /mcu/torque_cmd, /mcu/enable）下发到 STM32。
// transport 参数: "can" (默认, CAN3 FD 主通道) | "uart" (USART1)
class McuTelemetryNode : public rclcpp::Node
{
public:
    explicit McuTelemetryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~McuTelemetryNode() override;

private:
    void setup_ros();
    void publish_loop();

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void torque_cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void pos_cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // 通信桥接（CAN3 或 UART）
    std::unique_ptr<BridgeBase> bridge_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr power_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr torque_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Params
    double publish_freq_ = 100.0;
};

}  // namespace rl_policy
