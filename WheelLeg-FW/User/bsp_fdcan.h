#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"

#define hcan_t FDCAN_HandleTypeDef

extern volatile uint32_t g_tx_cnt;   /* 发送计数 */
extern volatile uint32_t g_rx_cnt;   /* 接收计数 */
extern volatile uint8_t  g_last_rx_id;   /* 最近收到的帧ID */

uint8_t bsp_can3_init(void);   /* 返回 0=成功 1=失败 */
uint8_t can3_test_send(void);  /* 返回 0=发送成功 1=发送失败 */
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);

#endif /* __BSP_FDCAN_H_ */
