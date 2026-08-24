/**
  ******************************************************************************
  * @file    can3_task.c
  * @brief   CAN3 多帧上报：电机状态 + 温度 + 电压
  *
  * 帧格式: [Type][Flags][Data0..5]  8字节/帧, CAN ID=0x700
  * Flags: bit0=last, bits1-7=seq
  ******************************************************************************
  */
#include "can3_task.h"
#include "can_bsp.h"
#include "uart_task.h"
#include "dm4310_drv.h"
#include "imu_task.h"
#include "adc.h"
#include <string.h>

#define CAN3_TX_ID      0x700
#define TYPE_WHEEL_TEMP 0x87

/* 发送一帧 */
static void can3_send_frame(uint8_t type, uint8_t flags, const uint8_t *data, uint8_t len)
{
    uint8_t frame[8] = {0};
    frame[0] = type;
    frame[1] = flags;
    uint8_t copy_len = (len > 6) ? 6 : len;
    if (copy_len > 0) memcpy(&frame[2], data, copy_len);
    can3_send_raw(CAN3_TX_ID, frame, 8);
}

/* 多帧分包发送 */
static void can3_send_multiframe(uint8_t type, const uint8_t *data, uint16_t total)
{
    uint16_t offset = 0;
    uint8_t seq = 0;
    while (offset < total)
    {
        uint8_t remaining = (uint8_t)(total - offset);
        uint8_t chunk = (remaining > 6) ? 6 : remaining;
        uint8_t flags = (seq << 1) | ((offset + chunk >= total) ? 1 : 0);
        can3_send_frame(type, flags, &data[offset], chunk);
        offset += chunk;
        seq++;
    }
}

/* 4 关节电机状态 (TYPE_MOTOR 0x83, 48B) */
static void can3_send_motor_status(void)
{
    uart_motor_tx_t motor;
    for (int i = 0; i < 4; i++)
    {
        motor.pos[i] = joint_motors[i].para.pos;
        motor.vel[i] = joint_motors[i].para.vel;
        motor.tor[i] = joint_motors[i].para.tor;
    }
    can3_send_multiframe(TYPE_MOTOR, (const uint8_t *)&motor, MOTOR_LEN);
}

/* 2 轮毂电机状态 (TYPE_WHEEL 0x86, 24B) */
#define WHEEL_LEN 24
#define TYPE_WHEEL 0x86
static void can3_send_wheel_status(void)
{
    struct __attribute__((packed)) {
        float pos[2], vel[2], tor[2];
    } wheel;
    for (int i = 0; i < 2; i++)
    {
        wheel.pos[i] = wheel_motors[i].para.pos;
        wheel.vel[i] = wheel_motors[i].para.vel;
        wheel.tor[i] = wheel_motors[i].para.tor;
    }
    can3_send_multiframe(TYPE_WHEEL, (const uint8_t *)&wheel, WHEEL_LEN);
}

/* 关节温度 (TYPE_MOTOR_TEMP 0x84, 32B) */
static void can3_send_motor_temp(void)
{
    uart_motor_temp_tx_t mt;
    for (int i = 0; i < 4; i++)
    {
        mt.tmos[i]  = joint_motors[i].para.Tmos;
        mt.tcoil[i] = joint_motors[i].para.Tcoil;
    }
    can3_send_multiframe(TYPE_MOTOR_TEMP, (const uint8_t *)&mt, MOTOR_TEMP_LEN);
}

/* IMU 姿态 (TYPE_IMU 0x82, 40B = 9 floats) */
static void can3_send_imu(void)
{
    uart_imu_tx_t imu;
    imu.roll   = g_imu.roll;
    imu.pitch  = g_imu.pitch;
    imu.yaw    = g_imu.yaw;
    imu.gyro_x = g_imu.gyro[0];
    imu.gyro_y = g_imu.gyro[1];
    imu.gyro_z = g_imu.gyro[2];
    imu.accel_x = g_imu.accel[0];
    imu.accel_y = g_imu.accel[1];
    imu.accel_z = g_imu.accel[2];
    can3_send_multiframe(TYPE_IMU, (const uint8_t *)&imu, IMU_LEN);
}

/* 电源 (TYPE_POWER 0x85, 12B) — 单独发送避免 FIFO 拥塞 */
static float g_vbat = 0.0f;
static uint8_t g_protect_state = 0;
void can3_set_power(float vbat, uint8_t prot)
{
    g_vbat = vbat;
    g_protect_state = prot;
}

static void can3_send_power(void)
{
    uart_power_tx_t pw;
    pw.bat_voltage = g_vbat;
    pw.imu_temp    = 0.0f;
    pw.reserved    = (float)g_protect_state;
    can3_send_multiframe(TYPE_POWER, (const uint8_t *)&pw, POWER_LEN);
}

/* 主循环调用 */
void can3_task_process(void)
{
    uint32_t tick = HAL_GetTick();
    static uint32_t last_motor = 0;
    static uint32_t last_wheel = 0;
    static uint32_t last_temp = 0;
    static uint32_t last_power = 0;

    // 关节电机状态 50Hz (20ms)
    if (tick - last_motor >= 20)
    {
        last_motor = tick;
        can3_send_motor_status();
    }
    // 轮毂电机状态 20Hz (50ms) — 非 20 整数倍错开 motor
    if (tick - last_wheel >= 50)
    {
        last_wheel = tick;
        can3_send_wheel_status();
    }
    // IMU 姿态 25Hz (40ms) — 错开避免与 motor(20ms) FIFO 拥塞
    static uint32_t last_imu = 0;
    if (g_imu.imu_ready && (tick - last_imu >= 40))
    {
        last_imu = tick;
        can3_send_imu();
    }
    // 关节温度 1Hz (1000ms)
    if (tick - last_temp >= 1000)
    {
        last_temp = tick;
        can3_send_motor_temp();
    }
    // 电源电压 2Hz (500ms) — 相位偏移 250ms 避开 motor 发送瞬间
    if (tick >= 250 && tick - last_power >= 500)
    {
        last_power = tick;
        can3_send_power();
    }
}
