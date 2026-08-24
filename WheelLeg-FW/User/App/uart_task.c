/**
  *********************************************************************
  * @file      uart_task.c
  * @brief     UART communication between STM32 and RDK X5
  *
  * Protocol: 0xA5 0x5A [Len][Type][Payload...][CRC8]
  * CRC8 polynomial: 0x07
  *
  * RX types:
  *   TYPE_CMD (0x01): velocity command (backward compatible with PS2 mode)
  *   TYPE_TORQUE_CMD (0x03): direct torque command from RL policy
  *   TYPE_MODE (0x02): mode switch command
  *
  * TX types:
  *   TYPE_ODOM (0x81): odometry feedback
  *   TYPE_IMU (0x82): IMU data
  *   TYPE_MOTOR (0x83): motor feedback
  *********************************************************************
  */

#include "uart_task.h"
#include "bsp_dwt.h"
#include "can_bsp.h"
#include "dm4310_drv.h"
#include "imu_task.h"
#include "main_task.h"
#include <string.h>
#include <math.h>

/* ===== Global State ===== */
volatile ctrl_mode_t       g_ctrl_mode  = CTRL_LQR;
volatile uart_torque_rx_t  g_rl_torques = {0};
volatile uart_cmd_rx_t     g_uart_cmd   = {0};
volatile uart_pos_cmd_t    g_pos_cmd    = {0};
volatile uint8_t           g_uart_cmd_valid = 0;

/* ===== UART Hardware ===== */
/* Use USART1 on DM-MC02 (PA9=TX, PA10=RX) */
extern UART_HandleTypeDef huart1;

#define huart_cmd  huart1

/* ===== RX Ring Buffer ===== */
static uint8_t  rx_buf[UART_RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

/* ===== CRC8 Calculation (matches legwheel_uart_node.cpp) ===== */
uint8_t crc8_calc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ===== Ring Buffer Operations ===== */
static void rx_put(uint8_t byte)
{
    uint8_t next = (rx_head + 1) % UART_RX_BUF_SIZE;
    if (next != rx_tail)
    {
        rx_buf[rx_head] = byte;
        rx_head = next;
    }
}

static uint8_t rx_get(uint8_t *out)
{
    if (rx_tail == rx_head) return 0;
    *out = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;
    return 1;
}

static uint8_t rx_available(void)
{
    return (rx_head - rx_tail) % UART_RX_BUF_SIZE;
}

/* ===== UART RX ISR ===== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart_cmd)
    {
        rx_put(rx_buf[0]);
        HAL_UART_Receive_IT(&huart_cmd, rx_buf, 1);
    }
}

/* ===== UART TX Helpers ===== */
static void uart_tx_bytes(const uint8_t *data, uint8_t len)
{
    HAL_UART_Transmit(&huart_cmd, (uint8_t *)data, len, 20);
}

/* ===== Frame Parser ===== */
typedef enum {
    PARSE_SYNC0,
    PARSE_SYNC1,
    PARSE_LEN,
    PARSE_TYPE,
    PARSE_PAYLOAD,
    PARSE_CRC
} parse_state_t;

static parse_state_t parse_state = PARSE_SYNC0;
static uint8_t parse_len   = 0;
static uint8_t parse_type  = 0;
static uint8_t parse_idx   = 0;
static uint8_t parse_payload[64];

/* 处理完整帧（UART 与 CAN3 共用：CAN3 收到 0xA5 0x5A 帧后调用） */
void process_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    switch (type)
    {
        case TYPE_CMD:
        {
            if (len < CMD_LEN) return;
            const uart_cmd_rx_t *cmd = (const uart_cmd_rx_t *)payload;

            /* Enable/disable motor control via command */
            if (cmd->enable)
                g_ctrl_mode = CTRL_LQR;
            else
                g_ctrl_mode = CTRL_PS2;

            g_uart_cmd_valid = 1;
            break;
        }

        case TYPE_TORQUE_CMD:
        {
            if (len < TORQUE_CMD_LEN) return;
            const uart_torque_rx_t *trq = (const uart_torque_rx_t *)payload;

            /* Copy torques to global (volatile) for consumption by chassis tasks */
            for (int i = 0; i < 4; i++)
                g_rl_torques.joint_torque[i] = trq->joint_torque[i];
            g_rl_torques.wheel_torque[0] = trq->wheel_torque[0];
            g_rl_torques.wheel_torque[1] = trq->wheel_torque[1];
            g_rl_torques.rl_mode         = trq->rl_mode;

            /* Switch to RL mode when first torque command received */
            if (trq->rl_mode == 1)
                g_ctrl_mode = CTRL_RL;

            g_uart_cmd_valid = 1;
            break;
        }

        case TYPE_MODE:
        {
            if (len >= 1)
            {
                uint8_t mode = payload[0];
                if (mode <= 2)
                    g_ctrl_mode = (ctrl_mode_t)mode;
            }
            break;
        }

        case TYPE_ENABLE:
        {
            /* 1=使能4电机(先清错再使能), 0=失能4电机 */
            if (len >= 1)
            {
                uint8_t en = payload[0];
                if (en == 1)
                {
                    for (int j = 0; j < 10; j++)
                    {
                        /* 先清错(过载保护后必须), 再使能 */
                        clear_motor_error(&hfdcan1, joint_motors[0].para.id, joint_motors[0].mode);
                        clear_motor_error(&hfdcan1, joint_motors[1].para.id, joint_motors[1].mode);
                        clear_motor_error(&hfdcan2, joint_motors[2].para.id, joint_motors[2].mode);
                        clear_motor_error(&hfdcan2, joint_motors[3].para.id, joint_motors[3].mode);
                        enable_motor_mode(&hfdcan1, joint_motors[0].para.id, joint_motors[0].mode);
                        enable_motor_mode(&hfdcan1, joint_motors[1].para.id, joint_motors[1].mode);
                        enable_motor_mode(&hfdcan2, joint_motors[2].para.id, joint_motors[2].mode);
                        enable_motor_mode(&hfdcan2, joint_motors[3].para.id, joint_motors[3].mode);
                        HAL_Delay(1);
                    }
                }
                else
                {
                    for (int j = 0; j < 10; j++)
                    {
                        disable_motor_mode(&hfdcan1, joint_motors[0].para.id, joint_motors[0].mode);
                        disable_motor_mode(&hfdcan1, joint_motors[1].para.id, joint_motors[1].mode);
                        disable_motor_mode(&hfdcan2, joint_motors[2].para.id, joint_motors[2].mode);
                        disable_motor_mode(&hfdcan2, joint_motors[3].para.id, joint_motors[3].mode);
                        HAL_Delay(1);
                    }
                }
            }
            break;
        }

        case TYPE_POS_CMD:
        {
            /* 上位机位置命令: 4x pos + kp + kd -> 位置刚度控制 */
            if (len < POS_CMD_LEN) return;
            const uart_pos_cmd_t *pc = (const uart_pos_cmd_t *)payload;
            for (int i = 0; i < 4; i++)
                g_pos_cmd.pos[i] = pc->pos[i];
            g_pos_cmd.kp = pc->kp;
            g_pos_cmd.kd = pc->kd;
            g_ctrl_mode = CTRL_POS;
            g_uart_cmd_valid = 1;
            break;
        }

        default:
            break;
    }
}

static void parse_byte(uint8_t byte)
{
    switch (parse_state)
    {
        case PARSE_SYNC0:
            if (byte == UART_HEAD1)
                parse_state = PARSE_SYNC1;
            break;

        case PARSE_SYNC1:
            if (byte == UART_HEAD2)
                parse_state = PARSE_LEN;
            else
                parse_state = PARSE_SYNC0;
            break;

        case PARSE_LEN:
            parse_len = byte;
            parse_idx = 0;
            parse_state = PARSE_TYPE;
            break;

        case PARSE_TYPE:
            parse_type = byte;
            if (parse_len == 0)
                parse_state = PARSE_CRC;
            else
                parse_state = PARSE_PAYLOAD;
            break;

        case PARSE_PAYLOAD:
            if (parse_idx < sizeof(parse_payload))
                parse_payload[parse_idx] = byte;
            parse_idx++;
            if (parse_idx >= parse_len)
                parse_state = PARSE_CRC;
            break;

        case PARSE_CRC:
        {
            /* Verify CRC over [len][type][payload] */
            uint8_t crc_data[66];
            crc_data[0] = parse_len;
            crc_data[1] = parse_type;
            for (uint8_t i = 0; i < parse_len; i++)
                crc_data[2 + i] = parse_payload[i];

            uint8_t calc_crc = crc8_calc(crc_data, 2 + parse_len);
            if (calc_crc == byte)
            {
                process_frame(parse_type, parse_payload, parse_len);
            }
            parse_state = PARSE_SYNC0;
            break;
        }
    }
}

/* ===== Motor Status Feedback ===== */
static void send_motor_status(void)
{
    uart_motor_tx_t motor;
    for (int i = 0; i < 4; i++)
    {
        motor.pos[i] = joint_motors[i].para.pos;
        motor.vel[i] = joint_motors[i].para.vel;
        motor.tor[i] = joint_motors[i].para.tor;
    }

    uint8_t frame[2 + 1 + 1 + MOTOR_LEN + 1];
    frame[0] = UART_HEAD1;
    frame[1] = UART_HEAD2;
    frame[2] = MOTOR_LEN;
    frame[3] = TYPE_MOTOR;
    memcpy(&frame[4], &motor, MOTOR_LEN);
    frame[4 + MOTOR_LEN] = crc8_calc(&frame[2], 2 + MOTOR_LEN);

    uart_tx_bytes(frame, sizeof(frame));
}

/* ===== IMU Feedback (TYPE_IMU 0x82) ===== */
static void send_imu(void)
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

    uint8_t frame[2 + 1 + 1 + IMU_LEN + 1];
    frame[0] = UART_HEAD1;
    frame[1] = UART_HEAD2;
    frame[2] = IMU_LEN;
    frame[3] = TYPE_IMU;
    memcpy(&frame[4], &imu, IMU_LEN);
    frame[4 + IMU_LEN] = crc8_calc(&frame[2], 2 + IMU_LEN);

    uart_tx_bytes(frame, sizeof(frame));
}

/* ===== Motor Temperature (TYPE_MOTOR_TEMP 0x84) ===== */
static void send_motor_temp(void)
{
    uart_motor_temp_tx_t mt;
    for (int i = 0; i < 4; i++)
    {
        mt.tmos[i]  = joint_motors[i].para.Tmos;
        mt.tcoil[i] = joint_motors[i].para.Tcoil;
    }

    uint8_t frame[2 + 1 + 1 + MOTOR_TEMP_LEN + 1];
    frame[0] = UART_HEAD1;
    frame[1] = UART_HEAD2;
    frame[2] = MOTOR_TEMP_LEN;
    frame[3] = TYPE_MOTOR_TEMP;
    memcpy(&frame[4], &mt, MOTOR_TEMP_LEN);
    frame[4 + MOTOR_TEMP_LEN] = crc8_calc(&frame[2], 2 + MOTOR_TEMP_LEN);

    uart_tx_bytes(frame, sizeof(frame));
}

/* ===== Power/Voltage (TYPE_POWER 0x85) ===== */
static void send_power(void)
{
    uart_power_tx_t pw;
    pw.bat_voltage = g_imu.power_voltage;
    pw.imu_temp    = g_imu.temperature;
    /* reserved: 电池过放保护状态 0=正常 1=低压警告 2=低压锁定 */
    pw.reserved    = (float)vbat_protect_check();

    uint8_t frame[2 + 1 + 1 + POWER_LEN + 1];
    frame[0] = UART_HEAD1;
    frame[1] = UART_HEAD2;
    frame[2] = POWER_LEN;
    frame[3] = TYPE_POWER;
    memcpy(&frame[4], &pw, POWER_LEN);
    frame[4 + POWER_LEN] = crc8_calc(&frame[2], 2 + POWER_LEN);

    uart_tx_bytes(frame, sizeof(frame));
}

/* ===== Process (裸机轮询，由主循环调用) ===== */
void uart_task_process(void)
{
    /* --- RX: Parse incoming frames --- */
    uint8_t byte;
    while (rx_get(&byte))
    {
        parse_byte(byte);
    }

    /* --- TX: 分频发送各数据帧 --- */
    uint32_t tick = HAL_GetTick();
    static uint32_t last_motor = 0;
    static uint32_t last_imu = 0;
    static uint32_t last_slow = 0;

    /* 电机状态 100Hz (10ms) */
    if (tick - last_motor >= 10)
    {
        last_motor = tick;
        send_motor_status();
    }
    /* IMU 100Hz (10ms) */
    if (g_imu.imu_ready && (tick - last_imu >= 10))
    {
        last_imu = tick;
        send_imu();
    }
    /* 温度/电压 5Hz (200ms) */
    if (tick - last_slow >= 200)
    {
        last_slow = tick;
        send_motor_temp();
        send_power();
    }

    /* --- Safety: limit RL torques --- */
    if (g_ctrl_mode == CTRL_RL)
    {
        for (int i = 0; i < 4; i++)
        {
            /* Joint torque limit: ±10 Nm for DM4310 */
            if (g_rl_torques.joint_torque[i] > 10.0f)
                ((volatile uart_torque_rx_t *)&g_rl_torques)->joint_torque[i] = 10.0f;
            if (g_rl_torques.joint_torque[i] < -10.0f)
                ((volatile uart_torque_rx_t *)&g_rl_torques)->joint_torque[i] = -10.0f;
        }
        /* Wheel torque limit: ±18 Nm for DM6215 */
        for (int i = 0; i < 2; i++)
        {
            if (g_rl_torques.wheel_torque[i] > 18.0f)
                ((volatile uart_torque_rx_t *)&g_rl_torques)->wheel_torque[i] = 18.0f;
            if (g_rl_torques.wheel_torque[i] < -18.0f)
                ((volatile uart_torque_rx_t *)&g_rl_torques)->wheel_torque[i] = -18.0f;
        }
    }
}

void uart_task_init(void)
{
    /* Enable UART RX interrupt */
    HAL_UART_Receive_IT(&huart_cmd, (uint8_t *)rx_buf, 1);
}
