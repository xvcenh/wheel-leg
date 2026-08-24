#include "rl_policy/uart_bridge.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstring>
#include <iostream>
#include <cerrno>

namespace rl_policy
{

UartBridge::UartBridge(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate) {}

UartBridge::~UartBridge() { close(); }

bool UartBridge::open()
{
    // 阻塞模式串口（O_NONBLOCK 高速率下可能持续 EAGAIN 读不到数据）
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        perror("open serial");
        return false;
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) { close(); return false; }

    cfmakeraw(&tty);

    speed_t speed = B115200;
    switch (baudrate_) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:     speed = B115200; break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 1;   // 至少读1字节才返回
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; }
    tcflush(fd_, TCIOFLUSH);

    running_ = true;
    rx_thread_ = std::thread(&UartBridge::rx_thread_func, this);

    std::cout << "[UART] Opened " << port_ << " @ " << baudrate_ << std::endl;
    return true;
}

void UartBridge::close()
{
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// CRC8 matching STM32 firmware
uint8_t UartBridge::crc8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

void UartBridge::send_frame(uint8_t type, const uint8_t* payload, uint8_t len)
{
    uint8_t frame[64];
    frame[0] = HEAD1;
    frame[1] = HEAD2;
    frame[2] = len;
    frame[3] = type;
    if (len > 0 && payload) std::memcpy(frame + 4, payload, len);
    frame[4 + len] = crc8(&frame[2], 2 + len);
    ::write(fd_, frame, 5 + len);
}

void UartBridge::send_torques(const TorqueCommand& cmd)
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

void UartBridge::send_velocity(float vx, float wz, float leg, float roll, bool enable)
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

void UartBridge::send_enable(bool enable)
{
    uint8_t payload = enable ? 1 : 0;
    send_frame(TYPE_ENABLE, &payload, 1);
}

void UartBridge::send_pos_cmd(const float* pos, float kp, float kd)
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

void UartBridge::rx_thread_func()
{
    uint8_t byte;
    while (running_) {
        int n = ::read(fd_, &byte, 1);   // 阻塞读取（VMIN=1）
        if (n <= 0) continue;

        switch (parse_state_) {
            case SYNC0:
                if (byte == HEAD1) parse_state_ = SYNC1;
                break;
            case SYNC1:
                if (byte == HEAD2) { parse_state_ = PAYLOAD; parse_len_ = 0; }
                else parse_state_ = SYNC0;
                break;
            case PAYLOAD:
                if (parse_idx_ == 0) { parse_len_ = byte; parse_idx_ = 1; }
                else if (parse_idx_ == 1) { parse_type_ = byte; parse_idx_ = 2; }
                else {
                    if (parse_idx_ - 2 < parse_len_ && parse_idx_ - 2 < sizeof(parse_payload_))
                        parse_payload_[parse_idx_ - 2] = byte;
                    parse_idx_++;
                    if (parse_idx_ >= parse_len_ + 2) {
                        // Read CRC (guard against oversized/corrupt frames)
                        if (parse_len_ <= sizeof(parse_payload_)) {
                            uint8_t crc;
                            if (::read(fd_, &crc, 1) == 1) {
                                uint8_t buf[70];
                                buf[0] = parse_len_;
                                buf[1] = parse_type_;
                                std::memcpy(buf + 2, parse_payload_, parse_len_);
                                if (crc == crc8(buf, 2 + parse_len_))
                                    process_frame(parse_type_, parse_payload_, parse_len_);
                            }
                        }
                        parse_state_ = SYNC0;
                    }
                }
                break;
        }
    }
}

void UartBridge::process_frame(uint8_t type, const uint8_t* payload, uint8_t len)
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
}

bool UartBridge::read_odom(OdomData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_odom_.valid) return false;
    out = latest_odom_;
    return true;
}

bool UartBridge::read_imu(ImuData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_imu_.valid) return false;
    out = latest_imu_;
    return true;
}

bool UartBridge::read_motor(MotorData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_motor_.valid) return false;
    out = latest_motor_;
    return true;
}

bool UartBridge::read_temp(MotorTempData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_temp_.valid) return false;
    out = latest_temp_;
    return true;
}

bool UartBridge::read_power(PowerData& out)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_power_.valid) return false;
    out = latest_power_;
    return true;
}

}  // namespace rl_policy
