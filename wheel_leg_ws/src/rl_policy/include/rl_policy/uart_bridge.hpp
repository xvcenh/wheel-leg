#pragma once

#include <string>
#include <atomic>
#include <mutex>
#include <thread>

#include "rl_policy/bridge_base.hpp"

namespace rl_policy
{

class UartBridge : public BridgeBase {
public:
    UartBridge(const std::string& port, int baudrate);
    ~UartBridge() override;

    bool open() override;
    void close() override;

    // Send RL torque commands to STM32
    void send_torques(const TorqueCommand& cmd) override;

    // Send velocity command (backward compatible with legwheel_uart_node)
    void send_velocity(float vx, float wz, float leg_height, float roll, bool enable) override;

    // Send motor enable/disable command (TYPE_ENABLE)
    void send_enable(bool enable) override;

    // Send position command (TYPE_POS_CMD): 4x pos + kp + kd
    void send_pos_cmd(const float* pos, float kp, float kd) override;

    // Read telemetry (non-blocking, returns true if new data available)
    bool read_odom(OdomData& out) override;
    bool read_imu(ImuData& out) override;
    bool read_motor(MotorData& out) override;
    bool read_temp(MotorTempData& out) override;
    bool read_power(PowerData& out) override;
    bool read_wheel(WheelData& out) override { return false; }
    bool read_wheel_temp(WheelTempData& out) override { return false; }

    bool is_open() const override { return fd_ >= 0; }

private:
    std::string port_;
    int baudrate_;
    int fd_ = -1;

    // RX ring buffer
    static constexpr int RX_BUF_SIZE = 512;
    uint8_t rx_buf_[RX_BUF_SIZE];
    int rx_head_ = 0;
    int rx_tail_ = 0;

    // Telemetry storage
    std::mutex state_mutex_;
    OdomData latest_odom_;
    ImuData latest_imu_;
    MotorData latest_motor_;
    MotorTempData latest_temp_;
    PowerData latest_power_;

    // Parser state
    enum ParseState { SYNC0, SYNC1, PAYLOAD };
    ParseState parse_state_ = SYNC0;
    uint8_t parse_len_ = 0;
    uint8_t parse_type_ = 0;
    uint8_t parse_idx_ = 0;
    uint8_t parse_payload_[64];

    void rx_thread_func();
    std::thread rx_thread_;
    std::atomic<bool> running_{false};

    static uint8_t crc8(const uint8_t* data, uint8_t len);
    void process_frame(uint8_t type, const uint8_t* payload, uint8_t len);

    void send_frame(uint8_t type, const uint8_t* payload, uint8_t len);
};

}  // namespace rl_policy
