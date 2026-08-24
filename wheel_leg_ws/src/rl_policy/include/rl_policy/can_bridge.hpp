#pragma once

#include "rl_policy/bridge_base.hpp"   // 复用协议常量/数据结构与抽象接口

#include <cstdint>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>

namespace rl_policy
{

// CAN3 主通道桥接（SocketCAN + CAN FD）
// 帧协议与 UART 完全一致: [0xA5][0x5A][Len][Type][Payload...][CRC8]
//   TX (RDK→固件): ID 0x701  命令帧
//   RX (固件→RDK): ID 0x700  状态帧
class CanBridge : public BridgeBase {
public:
    explicit CanBridge(const std::string& interface);
    ~CanBridge() override;

    bool open() override;
    void close() override;

    // 与 UartBridge 相同的发送接口
    void send_torques(const TorqueCommand& cmd) override;
    void send_velocity(float vx, float wz, float leg_height, float roll, bool enable) override;
    void send_enable(bool enable) override;
    void send_pos_cmd(const float* pos, float kp, float kd) override;

    // 与 UartBridge 相同的读取接口
    bool read_odom(OdomData& out) override;
    bool read_imu(ImuData& out) override;
    bool read_motor(MotorData& out) override;
    bool read_temp(MotorTempData& out) override;
    bool read_power(PowerData& out) override;
    bool read_wheel(WheelData& out) override;
    bool read_wheel_temp(WheelTempData& out) override;

    bool is_open() const override { return fd_ >= 0; }

private:
    std::string interface_;
    int fd_ = -1;

    static constexpr uint16_t CAN_TX_ID = 0x701;   // RDK → 固件 命令
    static constexpr uint16_t CAN_RX_ID = 0x700;   // 固件 → RDK 状态

    // 遥测数据存储
    std::mutex state_mutex_;
    OdomData latest_odom_;
    ImuData latest_imu_;
    MotorData latest_motor_;
    MotorTempData latest_temp_;
    PowerData latest_power_;
    WheelData latest_wheel_;
    WheelTempData latest_wheel_temp_;

    // 多帧重组缓冲区
    static constexpr int MAX_MSG_LEN = 64;
    uint8_t reassembly_buf_[MAX_MSG_LEN];
    int reassembly_offset_ = 0;
    uint8_t reassembly_type_ = 0;

    void rx_thread_func();
    std::thread rx_thread_;
    std::atomic<bool> running_{false};

    static uint8_t crc8(const uint8_t* data, uint8_t len);
    void process_frame(uint8_t type, const uint8_t* payload, uint8_t len);
    void send_frame(uint8_t type, const uint8_t* payload, uint8_t len);
};

}  // namespace rl_policy
