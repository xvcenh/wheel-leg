/**
 * @file uart_protocol.hpp
 * @brief Protocol definitions for RDK X5 <-> STM32 UART communication.
 *
 * Frame formats (little-endian):
 *
 *   RX (RDK -> STM32) 18 bytes:
 *     [0]   0xAA header0
 *     [1]   0x55 header1
 *     [2]   cmd_type: 0x01=velocity, 0x02=leg, 0x03=start/stop, 0x04=home, 0x05=emergency
 *     [3]   reserved
 *     [4-7] v_set (float, m/s)
 *     [8-11] turn_set (float, rad)
 *     [12-15] leg_set (float, m)
 *     [16]  start_flag (uint8, 0/1)
 *     [17]  checksum (XOR of bytes 0-16)
 *
 *   TX (STM32 -> RDK) 35 bytes:
 *     [0]   0xAA header0
 *     [1]   0x55 header1
 *     [2-5] vx (float, m/s)
 *     [6-9] omega (float, rad/s) [NOTE: compute derivative on RDK side]
 *     [10-13] pitch (float, rad)
 *     [14-17] roll (float, rad)
 *     [18-21] left_leg (float, m)
 *     [22-25] right_leg (float, m)
 *     [26-29] x_pos (float, m)
 *     [30-33] yaw (float, rad)
 *     [34]  checksum (XOR of bytes 0-33)
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <ostream>

namespace wheel_leg_driver
{

/* Frame sizes */
constexpr uint8_t RX_FRAME_SIZE = 18;
constexpr uint8_t TX_FRAME_SIZE = 35;
constexpr uint8_t FRAME_HEADER0  = 0xAA;
constexpr uint8_t FRAME_HEADER1  = 0x55;

/* Command types */
enum class CmdType : uint8_t {
  VELOCITY   = 0x01,
  LEG_HEIGHT = 0x02,
  START_STOP = 0x03,
  HOME_MARK  = 0x04,
  EMERGENCY  = 0x05,
  REQ_STATUS = 0x10
};

/* RX Frame: RDK -> STM32 */
#pragma pack(push, 1)
struct RxFrame {
  uint8_t header0;
  uint8_t header1;
  uint8_t cmd_type;
  uint8_t reserved;
  float   v_set;
  float   turn_set;
  float   leg_set;
  uint8_t start_flag;
  uint8_t checksum;
};
#pragma pack(pop)

/* TX Frame: STM32 -> RDK */
#pragma pack(push, 1)
struct TxFrame {
  uint8_t header0;
  uint8_t header1;
  float   vx;
  float   omega;
  float   pitch;
  float   roll;
  float   left_leg;
  float   right_leg;
  float   x_pos;
  float   yaw;
  uint8_t checksum;
};
#pragma pack(pop)

/* Telemetry data struct (parsed from TxFrame) */
struct Telemetry {
  float vx = 0.0f;
  float omega = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
  float left_leg = 0.0f;
  float right_leg = 0.0f;
  float x_pos = 0.0f;
  float yaw = 0.0f;
  bool  valid = false;
};

/* ====== Functions ====== */

/** Compute XOR checksum over [start, start+len) */
inline uint8_t checksum(const uint8_t* data, uint8_t len)
{
  uint8_t ck = 0;
  for (uint8_t i = 0; i < len; ++i) { ck ^= *(data + i); }
  return ck;
}

/** Pack a velocity command frame into buf (18 bytes). Returns true on success. */
inline bool pack_velocity_cmd(uint8_t* buf, float v, float turn, float leg, bool start)
{
  if (!buf) return false;
  RxFrame f;
  f.header0   = FRAME_HEADER0;
  f.header1   = FRAME_HEADER1;
  f.cmd_type  = static_cast<uint8_t>(CmdType::VELOCITY);
  f.reserved  = 0;
  f.v_set     = v;
  f.turn_set  = turn;
  f.leg_set   = leg;
  f.start_flag = start ? 1 : 0;
  f.checksum  = checksum(reinterpret_cast<const uint8_t*>(&f), RX_FRAME_SIZE - 1);
  std::memcpy(buf, &f, RX_FRAME_SIZE);
  return true;
}

/** Pack a start/stop command frame into buf (18 bytes). Returns true on success. */
inline bool pack_start_cmd(uint8_t* buf, bool start)
{
  if (!buf) return false;
  RxFrame f;
  f.header0   = FRAME_HEADER0;
  f.header1   = FRAME_HEADER1;
  f.cmd_type  = static_cast<uint8_t>(CmdType::START_STOP);
  f.reserved  = 0;
  f.v_set     = 0.0f;
  f.turn_set  = 0.0f;
  f.leg_set   = 0.08f;
  f.start_flag = start ? 1 : 0;
  f.checksum  = checksum(reinterpret_cast<const uint8_t*>(&f), RX_FRAME_SIZE - 1);
  std::memcpy(buf, &f, RX_FRAME_SIZE);
  return true;
}

/** Pack an emergency stop command frame into buf (18 bytes). Returns true on success. */
inline bool pack_emergency_cmd(uint8_t* buf)
{
  if (!buf) return false;
  RxFrame f;
  f.header0   = FRAME_HEADER0;
  f.header1   = FRAME_HEADER1;
  f.cmd_type  = static_cast<uint8_t>(CmdType::EMERGENCY);
  f.reserved  = 0;
  f.v_set     = 0.0f;
  f.turn_set  = 0.0f;
  f.leg_set   = 0.08f;
  f.start_flag = 0;
  f.checksum  = checksum(reinterpret_cast<const uint8_t*>(&f), RX_FRAME_SIZE - 1);
  std::memcpy(buf, &f, RX_FRAME_SIZE);
  return true;
}

/** Parse received TX telemetry frame. Returns true if valid frame parsed. */
inline bool parse_telemetry(const uint8_t* buf, uint8_t len, Telemetry& out)
{
  if (!buf || len < TX_FRAME_SIZE) return false;
  const TxFrame* f = reinterpret_cast<const TxFrame*>(buf);
  if (f->header0 != FRAME_HEADER0 || f->header1 != FRAME_HEADER1) return false;
  uint8_t ck = checksum(buf, TX_FRAME_SIZE - 1);
  if (ck != f->checksum) return false;

  out.vx        = f->vx;
  out.omega     = f->omega;
  out.pitch     = f->pitch;
  out.roll      = f->roll;
  out.left_leg  = f->left_leg;
  out.right_leg = f->right_leg;
  out.x_pos     = f->x_pos;
  out.yaw       = f->yaw;
  out.valid     = true;
  return true;
}

}  // namespace wheel_leg_driver
