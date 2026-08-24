#include "rl_policy/can_bridge.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/if.h>
#include <cstring>
#include <iostream>
#include <cerrno>

namespace rl_policy
{

CanBridge::CanBridge(const std::string& interface)
    : interface_(interface) {}

CanBridge::~CanBridge() { close(); }

bool CanBridge::open()
{
    fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd_ < 0) {
        perror("socket can");
        return false;
    }

    // 启用 CAN FD 帧
    int enable_fd = 1;
    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0) {
        perror("setsockopt CAN_RAW_FD_FRAMES");
        close();
        return false;
    }

    // 只接收 ID 0x700（固件状态帧），其余过滤
    struct can_filter filter;
    filter.can_id = CAN_RX_ID;
    filter.can_mask = CAN_EFF_FLAG | CAN_RTR_FLAG | 0x7FF;   // 标准帧，精确匹配 0x700
    if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
        perror("setsockopt CAN_RAW_FILTER");
        close();
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        close();
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind can");
        close();
        return false;
    }

    running_ = true;
    rx_thread_ = std::thread(&CanBridge::rx_thread_func, this);

    std::cout << "[CAN] Opened " << interface_ << " (FD, RX id=0x" << std::hex
              << CAN_RX_ID << " TX id=0x" << CAN_TX_ID << std::dec << ")" << std::endl;
    return true;
}

void CanBridge::close()
{
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// CRC8 matching STM32 firmware
uint8_t CanBridge::crc8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

void CanBridge::send_frame(uint8_t type, const uint8_t* payload, uint8_t len)
{
    struct canfd_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = CAN_TX_ID;                 // 标准帧 ID
    frame.flags = 0;                          // 经典CAN，无BRS
    frame.len = 5 + len;                      // [A5][5A][Len][Type][Payload][CRC]

    frame.data[0] = HEAD1;
    frame.data[1] = HEAD2;
    frame.data[2] = len;
    frame.data[3] = type;
    if (len > 0 && payload) std::memcpy(frame.data + 4, payload, len);
    frame.data[4 + len] = crc8(&frame.data[2], 2 + len);

    ::write(fd_, &frame, sizeof(frame));
}

void CanBridge::send_torques(const TorqueCommand& cmd)
{
    struct __attribute__((packed)) {
        float joint[4];
        float wheel[2];
        uint8_t mode;
    } payload;

    std::memcpy(payload.joint, cmd.joint_torques, sizeof(float) * 4);
    std::memcpy(payload.wheel, cmd.wheel_torques, sizeof(float) * 2);
    payload.mode = cmd.rl_mode;

    send_frame(TYPE_TORQUE_CMD, reinterpret_cast<uint8_t*>(&payload), 25);
}

void CanBridge::send_velocity(float vx, float wz, float leg, float roll, bool enable)
{
    struct __attribute__((packed)) {
        float vx, vy, wz, leg, roll;
        uint8_t enable;
    } payload;

    payload.vx = vx;
    payload.vy = 0.0f;
    payload.wz = wz;
    payload.leg = leg;
    payload.roll = roll;
    payload.enable = enable ? 1 : 0;

    send_frame(TYPE_CMD, reinterpret_cast<uint8_t*>(&payload), 21);
}

void CanBridge::send_enable(bool enable)
{
    uint8_t payload = enable ? 1 : 0;
    send_frame(TYPE_ENABLE, &payload, 1);
}

void CanBridge::send_pos_cmd(const float* pos, float kp, float kd)
{
    struct __attribute__((packed)) {
        float pos[4];
        float kp;
        float kd;
    } payload;

    std::memcpy(payload.pos, pos, sizeof(float) * 4);
    payload.kp = kp;
    payload.kd = kd;

    send_frame(TYPE_POS_CMD, reinterpret_cast<uint8_t*>(&payload), 24);
}

void CanBridge::rx_thread_func()
{
    struct canfd_frame frame;
    while (running_) {
        int n = ::read(fd_, &frame, sizeof(frame));
        if (n <= 0) continue;

        if ((frame.can_id & 0x7FF) != CAN_RX_ID) continue;

        const uint8_t* d = frame.data;

        // 经典CAN多帧格式: [Type][Flags][Data0..5]
        uint8_t type  = d[0];
        uint8_t flags = d[1];
        bool last     = (flags & 0x01) != 0;   // bit0 = last frame flag
        uint8_t seq   = (flags >> 1) & 0x7F;   // bits 1-7 = sequence
        uint8_t data_len = 6;                  // 6 bytes of data per frame

        // 新消息开始（seq == 0）：重置缓冲区
        if (seq == 0) {
            reassembly_offset_ = 0;
            reassembly_type_ = type;
        }

        // 将数据复制到缓冲区
        if (reassembly_offset_ + data_len <= MAX_MSG_LEN) {
            std::memcpy(reassembly_buf_ + reassembly_offset_, d + 2, data_len);
            reassembly_offset_ += data_len;
        }

        // 最后一帧：提交完整消息到 process_frame
        if (last) {
            process_frame(reassembly_type_, reassembly_buf_, reassembly_offset_);
        }
    }
}

void CanBridge::process_frame(uint8_t type, const uint8_t* payload, uint8_t len)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (type == TYPE_ODOM && len >= ODOM_LEN) {
        std::memcpy(&latest_odom_, payload, ODOM_LEN);
        latest_odom_.valid = true;
    }
    else if (type == TYPE_IMU && len >= IMU_LEN) {
        std::memcpy(&latest_imu_, payload, IMU_LEN);
        latest_imu_.valid = true;
    }
    else if (type == TYPE_MOTOR && len >= MOTOR_LEN) {
        std::memcpy(&latest_motor_, payload, MOTOR_LEN);
        latest_motor_.valid = true;
    }
    else if (type == TYPE_MOTOR_TEMP && len >= MOTOR_TEMP_LEN) {
        std::memcpy(&latest_temp_, payload, MOTOR_TEMP_LEN);
        latest_temp_.valid = true;
    }
    else if (type == TYPE_POWER && len >= POWER_LEN) {
        std::memcpy(&latest_power_, payload, POWER_LEN);
        latest_power_.valid = true;
    }
    else if (type == TYPE_WHEEL && len >= WHEEL_LEN) {
        std::memcpy(&latest_wheel_, payload, WHEEL_LEN);
        latest_wheel_.valid = true;
    }
    else if (type == TYPE_WHEEL_TEMP && len >= WHEEL_TEMP_LEN) {
        std::memcpy(&latest_wheel_temp_, payload, WHEEL_TEMP_LEN);
        latest_wheel_temp_.valid = true;
    }
}

bool CanBridge::read_odom(OdomData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_odom_.valid) return false;
    out = latest_odom_;
    return true;
}

bool CanBridge::read_imu(ImuData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_imu_.valid) return false;
    out = latest_imu_;
    return true;
}

bool CanBridge::read_motor(MotorData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_motor_.valid) return false;
    out = latest_motor_;
    return true;
}

bool CanBridge::read_temp(MotorTempData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_temp_.valid) return false;
    out = latest_temp_;
    return true;
}

bool CanBridge::read_power(PowerData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_power_.valid) return false;
    out = latest_power_;
    return true;
}

bool CanBridge::read_wheel(WheelData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_wheel_.valid) return false;
    out = latest_wheel_;
    return true;
}

bool CanBridge::read_wheel_temp(WheelTempData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_wheel_temp_.valid) return false;
    out = latest_wheel_temp_;
    return true;
}

}  // namespace rl_policy
