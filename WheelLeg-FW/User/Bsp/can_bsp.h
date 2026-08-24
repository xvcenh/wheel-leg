#ifndef _CAN_BSP_H
#define _CAN_BSP_H

#include "main.h"
#include "dm4310_drv.h"

// 4 关节电机 + 2 轮毂电机
extern Joint_Motor_t joint_motors[4];
extern Wheel_Motor_t wheel_motors[2];

// 收发诊断
extern volatile uint32_t g_tx_ok, g_tx_fail;
extern volatile uint32_t g_rx1_cnt, g_rx2_cnt;
extern volatile uint16_t g_fb_can_id1, g_fb_can_id2;
extern volatile uint8_t g_rx1_frame[8], g_rx2_frame[8];

// CAN3 诊断
extern volatile uint32_t g_can3_tx_cnt, g_can3_rx_cnt;

// 配置函数
void FDCAN1_Config(void);
void FDCAN2_Config(void);
void FDCAN3_Config(void);

// 发送函数
uint8_t can3_send_raw(uint16_t id, uint8_t *data, uint32_t len);
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

// 电机初始化
void all_motors_init(void);

#endif
