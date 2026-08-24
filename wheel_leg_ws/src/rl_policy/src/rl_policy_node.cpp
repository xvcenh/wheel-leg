#include "rl_policy/rl_policy_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

namespace rl_policy
{

RLPolicyNode::RLPolicyNode(const rclcpp::NodeOptions& options)
    : Node("rl_policy_node", options)
{
    // Parameters
    std::string transport = this->declare_parameter<std::string>("transport", "can");
    std::string port      = this->declare_parameter<std::string>("uart_port", "/dev/ttyS1");
    int baudrate          = this->declare_parameter<int>("uart_baudrate", 115200);
    std::string can_if    = this->declare_parameter<std::string>("can_interface", "can0");
    double freq           = this->declare_parameter<double>("control_freq", 100.0);
    max_torque_           = this->declare_parameter<double>("max_torque", 10.0);

    // 通信桥接：CAN3 (默认, CAN FD 主通道) 或 UART
    if (transport == "can") {
        bridge_ = std::make_unique<CanBridge>(can_if);
    } else {
        bridge_ = std::make_unique<UartBridge>(port, baudrate);
    }
    if (!bridge_->open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open %s bridge", transport.c_str());
        return;
    }
    // Enable RL mode on STM32
    bridge_->send_velocity(0.0f, 0.0f, 0.08f, 0.0f, true);

    // ROS2
    setup_ros();

    // Control timer
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / freq));
    control_timer_ = this->create_wall_timer(period,
        std::bind(&RLPolicyNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "RL Policy Node started. transport: %s, freq: %.1f Hz", transport.c_str(), freq);
}

RLPolicyNode::~RLPolicyNode()
{
    // Send zero torques before shutdown
    RLAction zero = {};
    if (bridge_ && bridge_->is_open()) {
        bridge_->send_velocity(0.0f, 0.0f, 0.08f, 0.0f, true);
    }
}

void RLPolicyNode::setup_ros()
{
    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_rl", 10);
    odom_pub_    = this->create_publisher<nav_msgs::msg::Odometry>("/rl_odom", 10);
    imu_pub_     = this->create_publisher<sensor_msgs::msg::Imu>("/rl_imu", 10);
    torque_pub_  = this->create_publisher<std_msgs::msg::Float32MultiArray>("/rl_torques", 10);
    rl_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/rl_mode_active", 10);

    // Subscribers
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/rl_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            rl_enabled_ = msg->data;
            RCLCPP_INFO(this->get_logger(), "RL %s", rl_enabled_ ? "ENABLED" : "DISABLED");
        });

    // TF
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void RLPolicyNode::control_loop()
{
    if (!bridge_ || !bridge_->is_open()) return;

    // 1. Read state from STM32
    OdomData odom;
    ImuData imu;
    bridge_->read_odom(odom);
    bridge_->read_imu(imu);

    // Publish telemetry for debugging
    if (imu.valid) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";
        // Convert Euler to quaternion
        double cr = std::cos(imu.roll * 0.5), sr = std::sin(imu.roll * 0.5);
        double cp = std::cos(imu.pitch * 0.5), sp = std::sin(imu.pitch * 0.5);
        double cy = std::cos(imu.yaw * 0.5), sy = std::sin(imu.yaw * 0.5);
        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
        imu_msg.angular_velocity.x = imu.gyro_x;
        imu_msg.angular_velocity.y = imu.gyro_y;
        imu_msg.angular_velocity.z = imu.gyro_z;
        imu_msg.linear_acceleration.x = imu.accel_x;
        imu_msg.linear_acceleration.y = imu.accel_y;
        imu_msg.linear_acceleration.z = imu.accel_z;
        imu_pub_->publish(imu_msg);
    }

    if (odom.valid) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = odom.x;
        odom_msg.pose.pose.position.y = odom.y;
        odom_msg.twist.twist.linear.x = odom.vx;
        odom_msg.twist.twist.angular.z = odom.wz;
        odom_pub_->publish(odom_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = odom.x;
        tf_msg.transform.translation.y = odom.y;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    // 2. Build state vector
    RLState state = {};
    if (imu.valid) {
        state.roll  = imu.roll;
        state.pitch = imu.pitch;
        state.yaw   = imu.yaw;
        state.gyro_x = imu.gyro_x;
        state.gyro_y = imu.gyro_y;
        state.gyro_z = imu.gyro_z;
    }
    // Joint positions/velocities would come from /mcu_motor_feedback
    // For now, use odom velocity as a simple state proxy
    if (odom.valid) {
        state.joint_pos[0] = 0.0f;  // placeholder
        state.joint_vel[0] = odom.vx;
    }

    // 3. RL inference
    if (rl_enabled_) {
        RLAction action = inference(state);
        last_action_ = action;

        // 4. Send torques to STM32
        TorqueCommand cmd;
        cmd.joint_torques[0] = action.torques[0];  // hip_R
        cmd.joint_torques[1] = action.torques[1];  // knee_R
        cmd.joint_torques[2] = action.torques[2];  // hip_L
        cmd.joint_torques[3] = action.torques[3];  // knee_L
        cmd.wheel_torques[0] = action.torques[4];  // wheel_R
        cmd.wheel_torques[1] = action.torques[5];  // wheel_L
        cmd.rl_mode = 1;
        bridge_->send_torques(cmd);

        // Publish torques
        std_msgs::msg::Float32MultiArray torque_msg;
        torque_msg.data.assign(action.torques, action.torques + 6);
        torque_pub_->publish(torque_msg);
    }

    // 5. Publish RL mode status
    std_msgs::msg::Bool mode_msg;
    mode_msg.data = rl_enabled_;
    rl_mode_pub_->publish(mode_msg);
}

RLAction RLPolicyNode::inference(const RLState& state)
{
    // ==========================================
    // PLACEHOLDER: Replace with real ONNX inference
    // ==========================================
    // This is a simple PID stand controller as a baseline.
    // Replace this with:
    //   1. ONNX Runtime inference on BPU
    //   2. PyTorch export to ONNX
    //   3. Deploy model to /home/sunrise/models/rl_policy.onnx

    RLAction action = {};

    // Simple gravity compensation + roll stabilization
    // hip torques: gravity compensation
    float g = 9.81f;
    float m_leg = 1.3f;  // approximate leg mass
    float leg_len = 0.10f;

    // Pitch stabilization (proportional to pitch error from 0)
    float kp_pitch = 3.0f;
    float kd_pitch = 0.5f;

    // Simple stand: hip torque to counteract gravity + pitch correction
    float hip_torque = m_leg * g * leg_len * std::sin(state.pitch)
                     - kd_pitch * state.gyro_x;

    // Clamp
    hip_torque = std::clamp(hip_torque, -max_torque_, max_torque_);

    action.torques[0] = hip_torque;   // hip_R
    action.torques[1] = 0.0f;         // knee_R (hold by motor stiffness)
    action.torques[2] = hip_torque;   // hip_L
    action.torques[3] = 0.0f;         // knee_L
    action.torques[4] = 0.0f;         // wheel_R
    action.torques[5] = 0.0f;         // wheel_L

    return action;
}

}  // namespace rl_policy

// ============ Main ============

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rl_policy::RLPolicyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

