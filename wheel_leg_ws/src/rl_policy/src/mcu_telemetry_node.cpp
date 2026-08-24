#include "rl_policy/mcu_telemetry_node.hpp"

#include <chrono>
#include <cmath>

namespace rl_policy
{

McuTelemetryNode::McuTelemetryNode(const rclcpp::NodeOptions& options)
    : Node("mcu_telemetry_node", options)
{
    // Parameters
    std::string transport = this->declare_parameter<std::string>("transport", "can");
    std::string port      = this->declare_parameter<std::string>("uart_port", "/dev/ttyS1");
    int baudrate          = this->declare_parameter<int>("uart_baudrate", 921600);
    std::string can_if    = this->declare_parameter<std::string>("can_interface", "can0");
    publish_freq_         = this->declare_parameter<double>("publish_freq", 100.0);

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

    setup_ros();

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_freq_));
    publish_timer_ = this->create_wall_timer(period,
        std::bind(&McuTelemetryNode::publish_loop, this));

    RCLCPP_INFO(this->get_logger(), "MCU Telemetry Node started. transport: %s, freq: %.1f Hz",
                transport.c_str(), publish_freq_);
}

McuTelemetryNode::~McuTelemetryNode()
{
    if (bridge_ && bridge_->is_open()) {
        bridge_->send_enable(false);  // 关节点失能，安全
    }
}

void McuTelemetryNode::setup_ros()
{
    imu_pub_         = this->create_publisher<sensor_msgs::msg::Imu>("/mcu/imu", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/mcu/joint_states", 10);
    motor_temp_pub_  = this->create_publisher<std_msgs::msg::Float32MultiArray>("/mcu/motor_temp", 10);
    power_pub_       = this->create_publisher<std_msgs::msg::Float32MultiArray>("/mcu/power", 10);
    wheel_pub_       = this->create_publisher<std_msgs::msg::Float32MultiArray>("/mcu/wheel_states", 10);
    odom_pub_        = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&McuTelemetryNode::cmd_vel_callback, this, std::placeholders::_1));

    torque_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/mcu/torque_cmd", 10,
        std::bind(&McuTelemetryNode::torque_cmd_callback, this, std::placeholders::_1));

    pos_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/mcu/pos_cmd", 10,
        std::bind(&McuTelemetryNode::pos_cmd_callback, this, std::placeholders::_1));

    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/mcu/enable", 10,
        std::bind(&McuTelemetryNode::enable_callback, this, std::placeholders::_1));
}

void McuTelemetryNode::publish_loop()
{
    if (!bridge_ || !bridge_->is_open()) return;

    // --- IMU ---
    ImuData imu;
    if (bridge_->read_imu(imu)) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";
        // Euler (roll,pitch,yaw) -> quaternion
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

    // --- Joint states (4 motors: hip_R, knee_R, hip_L, knee_L) ---
    MotorData motor;
    if (bridge_->read_motor(motor)) {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = {"hip_R", "knee_R", "hip_L", "knee_L"};
        js.position.assign(motor.pos, motor.pos + 4);
        js.velocity.assign(motor.vel, motor.vel + 4);
        js.effort.assign(motor.tor, motor.tor + 4);
        joint_state_pub_->publish(js);
    }

    // --- Motor temperature [Tmos0..3, Tcoil0..3] ---
    MotorTempData temp;
    if (bridge_->read_temp(temp)) {
        std_msgs::msg::Float32MultiArray mt;
        mt.data.assign(temp.tmos, temp.tmos + 4);
        mt.data.insert(mt.data.end(), temp.tcoil, temp.tcoil + 4);
        motor_temp_pub_->publish(mt);
    }

    // --- Power [bat_voltage, imu_temp, protect_state] ---
    PowerData power;
    if (bridge_->read_power(power)) {
        std_msgs::msg::Float32MultiArray pw;
        pw.data = {power.bat_voltage, power.imu_temp, power.reserved};
        power_pub_->publish(pw);
    }

    // --- Wheel states [pos0,pos1, vel0,vel1, tor0,tor1] (按字段分组，与固件一致) ---
    WheelData wheel;
    if (bridge_->read_wheel(wheel)) {
        std_msgs::msg::Float32MultiArray wd;
        wd.data = {wheel.pos[0], wheel.pos[1],
                   wheel.vel[0], wheel.vel[1],
                   wheel.tor[0], wheel.tor[1]};
        wheel_pub_->publish(wd);
    }

    // --- Odom（占位）---
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom_pub_->publish(odom);
}

void McuTelemetryNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!bridge_ || !bridge_->is_open()) return;
    bridge_->send_velocity(msg->linear.x, msg->angular.z, 0.08f, 0.0f, true);
}

void McuTelemetryNode::torque_cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!bridge_ || !bridge_->is_open()) return;
    if (msg->data.size() < 6) {
        RCLCPP_WARN(this->get_logger(), "torque_cmd needs 6 elements (4 joint + 2 wheel)");
        return;
    }
    TorqueCommand cmd;
    cmd.joint_torques[0] = msg->data[0];  // hip_R
    cmd.joint_torques[1] = msg->data[1];  // knee_R
    cmd.joint_torques[2] = msg->data[2];  // hip_L
    cmd.joint_torques[3] = msg->data[3];  // knee_L
    cmd.wheel_torques[0] = msg->data[4];  // wheel_R
    cmd.wheel_torques[1] = msg->data[5];  // wheel_L
    cmd.rl_mode = 1;
    bridge_->send_torques(cmd);
}

void McuTelemetryNode::pos_cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!bridge_ || !bridge_->is_open()) return;
    if (msg->data.size() < 6) {
        RCLCPP_WARN(this->get_logger(), "pos_cmd needs 6 elements (4 pos + kp + kd)");
        return;
    }
    bridge_->send_pos_cmd(&msg->data[0], msg->data[4], msg->data[5]);
}

void McuTelemetryNode::enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!bridge_ || !bridge_->is_open()) return;
    bridge_->send_enable(msg->data);
    RCLCPP_INFO(this->get_logger(), "Motor %s", msg->data ? "ENABLED" : "DISABLED");
}

}  // namespace rl_policy

// ============ Main ============

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rl_policy::McuTelemetryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

