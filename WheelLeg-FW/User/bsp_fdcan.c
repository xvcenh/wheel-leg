/**
  ******************************************************************************
  * @file    bsp_fdcan.c
  * @brief   CAN3 独立测试程序：FDCAN3 (PD12/PD13) 经典CAN 1Mbps
  *
  * 功能：
  *   1. 周期发送 ID 0x700：data[0]=0xA5, data[1]=0x5A, data[2..3]=发送计数(高/低), data[4..7]=0x00
  *   2. 收到任意帧后回发 ID 0x701 应答：data[0..1]=源帧ID, data[2]=接收计数, 其余0
  *   3. 收发计数可通过调试器查看全局变量 g_tx_cnt / g_rx_cnt
  ******************************************************************************
  */
#include "bsp_fdcan.h"

#define CAN3_TEST_TX_ID   0x700   /* 固件周期上报 */
#define CAN3_TEST_RX_ID   0x701   /* 固件回环应答 */

volatile uint32_t g_tx_cnt = 0;   /* 发送计数 */
volatile uint32_t g_rx_cnt = 0;   /* 接收计数 */
volatile uint8_t  g_last_rx_id = 0;   /* 最近收到的帧ID */

/* 经典CAN 8字节发送 */
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    if (len == 1)       TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    else if (len == 2)  TxHeader.DataLength = FDCAN_DLC_BYTES_2;
    else if (len == 3)  TxHeader.DataLength = FDCAN_DLC_BYTES_3;
    else if (len == 4)  TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    else if (len == 5)  TxHeader.DataLength = FDCAN_DLC_BYTES_5;
    else if (len == 6)  TxHeader.DataLength = FDCAN_DLC_BYTES_6;
    else if (len == 7)  TxHeader.DataLength = FDCAN_DLC_BYTES_7;
    else if (len == 8)  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    else return 1;

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data) != HAL_OK)
        return 1;
    return 0;
}

/* FDCAN3 初始化：启动 + 接收中断，返回 0=成功 1=失败 */
uint8_t bsp_can3_init(void)
{
    FDCAN_FilterTypeDef filter;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;   /* mask 全0 = 收所有标准帧 */
    filter.FilterID2 = 0x000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK)
        return 1;

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
        return 1;

    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
        return 1;

    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        return 1;
    return 0;
}

/* 周期上报帧：ID 0x700，返回 0=发送成功 1=发送失败 */
uint8_t can3_test_send(void)
{
    uint8_t data[8] = {0xA5, 0x5A, 0, 0, 0, 0, 0, 0};
    data[2] = (uint8_t)(g_tx_cnt >> 8);
    data[3] = (uint8_t)(g_tx_cnt & 0xFF);
    if (fdcanx_send_data(&hfdcan3, CAN3_TEST_TX_ID, data, 8) == 0)
    {
        g_tx_cnt++;
        return 0;
    }
    return 1;
}

/* FDCAN3 接收回调已移至 main.c（统一处理 FDCAN1+FDCAN3） */
