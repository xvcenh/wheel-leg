#pragma once

#include <cstdint>

namespace rl_policy
{

// 协议常量（与 STM32 uart_task.h / can_bridge 一致）
static constexpr uint8_t HEAD1 = 0xA5;
static constexpr uint8_t HEAD2 = 0x5A;

static constexpr uint8_t TYPE_CMD       = 0x01;
static constexpr uint8_t TYPE_MODE      = 0x02;
static constexpr uint8_t TYPE_TORQUE_CMD = 0x03;
static constexpr uint8_t TYPE_ENABLE    = 0x04;
static constexpr uint8_t TYPE_POS_CMD   = 0x05;
static constexpr uint8_t TYPE_ODOM      = 0x81;
static constexpr uint8_t TYPE_IMU       = 0x82;
static constexpr uint8_t TYPE_MOTOR     = 0x83;
static constexpr uint8_t TYPE_MOTOR_TEMP = 0x84;
static constexpr uint8_t TYPE_POWER     = 0x85;
static constexpr uint8_t TYPE_WHEEL     = 0x86;
static constexpr uint8_t TYPE_WHEEL_TEMP = 0x87;

static constexpr uint8_t ODOM_LEN = 24;
static constexpr uint8_t IMU_LEN  = 40;
static constexpr uint8_t MOTOR_LEN = 48;
static constexpr uint8_t MOTOR_TEMP_LEN = 32;
static constexpr uint8_t POWER_LEN = 12;
static constexpr uint8_t WHEEL_LEN = 24;
static constexpr uint8_t WHEEL_TEMP_LEN = 16;

// 下行命令（RDK → 固件）
struct TorqueCommand {
    float joint_torques[4]; // hip_R, knee_R, hip_L, knee_L
    float wheel_torques[2]; // wheel_R, wheel_L
    uint8_t rl_mode;        // 1=RL active
};

// 上行遥测（固件 → RDK）
struct OdomData {
    float vx, vy, wz;
    float x, y, yaw;
    bool valid = false;
};

struct ImuData {
    float roll, pitch, yaw;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    bool valid = false;
};

struct MotorData {
    float pos[4];   // joint positions (rad)
    float vel[4];   // joint velocities (rad/s)
    float tor[4];   // joint torques (Nm)
    bool valid = false;
};

struct MotorTempData {
    float tmos[4];  // MOS temperatures (degC)
    float tcoil[4]; // coil temperatures (degC)
    bool valid = false;
};

struct PowerData {
    float bat_voltage;  // battery voltage (V)
    float imu_temp;     // IMU temperature (degC)
    float reserved;
    bool valid = false;
};

struct WheelData {
    float pos[2];   // wheel positions (rad)
    float vel[2];   // wheel velocities (rad/s)
    float tor[2];   // wheel torques (Nm)
    bool valid = false;
};

struct WheelTempData {
    float tmos[2];   // wheel MOS temperatures (degC)
    float tcoil[2];  // wheel coil temperatures (degC)
    bool valid = false;
};

// 通信桥接抽象基类：UART 与 CAN3 共用接口
class BridgeBase {
public:
    virtual ~BridgeBase() = default;

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;

    // 下行命令（RDK → 固件）
    virtual void send_torques(const TorqueCommand& cmd) = 0;
    virtual void send_velocity(float vx, float wz, float leg_height, float roll, bool enable) = 0;
    virtual void send_enable(bool enable) = 0;
    virtual void send_pos_cmd(const float* pos, float kp, float kd) = 0;

    // 上行遥测（固件 → RDK）
    virtual bool read_odom(OdomData& out) = 0;
    virtual bool read_imu(ImuData& out) = 0;
    virtual bool read_motor(MotorData& out) = 0;
    virtual bool read_temp(MotorTempData& out) = 0;
    virtual bool read_power(PowerData& out) = 0;
    virtual bool read_wheel(WheelData& out) = 0;
    virtual bool read_wheel_temp(WheelTempData& out) = 0;
};

}  // namespace rl_policy
