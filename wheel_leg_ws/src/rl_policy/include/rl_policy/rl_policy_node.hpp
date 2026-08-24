#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rl_policy/bridge_base.hpp"
#include "rl_policy/can_bridge.hpp"
#include "rl_policy/uart_bridge.hpp"
#include <string>
#include <vector>
#include <memory>
#include <chrono>

namespace rl_policy
{

// State vector for RL policy (18-dim default)
struct RLState {
    float roll, pitch, yaw;
    float gyro_x, gyro_y, gyro_z;
    float joint_pos[6];
    float joint_vel[6];

    float* data() { return &roll; }
    static constexpr int SIZE = 18;
};

// Action vector from RL policy (6-dim = 6 motor torques)
struct RLAction {
    float torques[6];  // hip_R, knee_R, hip_L, knee_L, wheel_R, wheel_L

    float* data() { return &torques[0]; }
    static constexpr int SIZE = 6;
};

class RLPolicyNode : public rclcpp::Node
{
public:
    explicit RLPolicyNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~RLPolicyNode() override;

private:
    // ROS2 pub/sub
    void setup_ros();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Inference loop (runs at control_freq_)
    void control_loop();

    // RL inference (placeholder - replace with ONNX Runtime)
    RLAction inference(const RLState& state);

    // 通信桥接（CAN3 或 UART）
    std::unique_ptr<BridgeBase> bridge_;

    // ROS2 publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr torque_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rl_mode_pub_;

    // ROS2 subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

    // TF
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State
    RLState current_state_;
    RLAction last_action_;
    bool rl_enabled_ = false;
    float max_torque_ = 10.0f;
};

}  // namespace rl_policy
