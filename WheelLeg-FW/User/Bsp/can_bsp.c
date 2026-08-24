#include "can_bsp.h"
#include "fdcan.h"
#include "string.h"

// 4 关节 + 2 轮毂
Joint_Motor_t joint_motors[4];
Wheel_Motor_t wheel_motors[2];

// 诊断
volatile uint32_t g_tx_ok = 0, g_tx_fail = 0;
volatile uint32_t g_rx1_cnt = 0, g_rx2_cnt = 0;
volatile uint16_t g_fb_can_id1 = 0xFFFF, g_fb_can_id2 = 0xFFFF;
volatile uint8_t g_rx1_frame[8] = {0}, g_rx2_frame[8] = {0};
volatile uint32_t g_can3_tx_cnt = 0, g_can3_rx_cnt = 0;

FDCAN_RxHeaderTypeDef RxHeader1, RxHeader2, RxHeader3;
uint8_t g_Can1RxData[8], g_Can2RxData[8], g_Can3RxData[8];

// ===== FDCAN1 配置（左关节 + 左轮毂） =====
void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef f;
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = 0;
    f.FilterType = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = 0x000;
    f.FilterID2 = 0x000;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
}

// ===== FDCAN2 配置（右关节 + 右轮毂） =====
void FDCAN2_Config(void)
{
    FDCAN_FilterTypeDef f;
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = 1;
    f.FilterType = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    f.FilterID1 = 0x000;
    f.FilterID2 = 0x000;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &f);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);
}

// ===== FDCAN3 配置（CAN 主通道） =====
void FDCAN3_Config(void)
{
    FDCAN_FilterTypeDef f;
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = 2;
    f.FilterType = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = 0x000;
    f.FilterID2 = 0x000;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &f);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan3);
}

// ===== CAN3 发送（经典 CAN 8 字节） =====
uint8_t can3_send_raw(uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef tx;
    tx.Identifier = id;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    if (len == 8) tx.DataLength = FDCAN_DLC_BYTES_8;
    else if (len == 1) tx.DataLength = FDCAN_DLC_BYTES_1;
    else if (len == 2) tx.DataLength = FDCAN_DLC_BYTES_2;
    else if (len == 3) tx.DataLength = FDCAN_DLC_BYTES_3;
    else if (len == 4) tx.DataLength = FDCAN_DLC_BYTES_4;
    else if (len == 5) tx.DataLength = FDCAN_DLC_BYTES_5;
    else if (len == 6) tx.DataLength = FDCAN_DLC_BYTES_6;
    else if (len == 7) tx.DataLength = FDCAN_DLC_BYTES_7;
    else return 1;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx, data) != HAL_OK) return 1;
    g_can3_tx_cnt++;
    return 0;
}

// ===== 通用 FDCAN 发送 =====
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef tx;
    tx.Identifier = id;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    if (len == 8) tx.DataLength = FDCAN_DLC_BYTES_8; else return 1;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx, data) != HAL_OK) { g_tx_fail++; return 1; }
    g_tx_ok++;
    return 0;
}

// ===== 电机初始化 =====
// FDCAN1: 关节 ID 6/8(反馈3/4) + 轮毂 ID 1(反馈0)
// FDCAN2: 关节 ID 6/8(反馈3/4) + 轮毂 ID 1(反馈0)
void all_motors_init(void)
{
    joint_motor_init(&joint_motors[0], 6, MIT_MODE);  // FDCAN1 左下
    joint_motor_init(&joint_motors[1], 8, MIT_MODE);  // FDCAN1 左上
    joint_motor_init(&joint_motors[2], 6, MIT_MODE);  // FDCAN2 右下
    joint_motor_init(&joint_motors[3], 8, MIT_MODE);  // FDCAN2 右上
    wheel_motor_init(&wheel_motors[0], 1, MIT_MODE);   // FDCAN1 左轮毂
    wheel_motor_init(&wheel_motors[1], 1, MIT_MODE);   // FDCAN2 右轮毂
}

// ===== FDCAN1 + FDCAN3 RX 回调（共用 FIFO0） =====
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t its)
{
    if ((its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) return;

    if (hfdcan->Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
        g_fb_can_id1 = RxHeader1.Identifier;
        g_rx1_cnt++;
        for (int i = 0; i < 8; i++) g_rx1_frame[i] = g_Can1RxData[i];

        switch (RxHeader1.Identifier)
        {
            case 3: dm4310_fbdata(&joint_motors[0], g_Can1RxData, RxHeader1.DataLength); break;
            case 4: dm4310_fbdata(&joint_motors[1], g_Can1RxData, RxHeader1.DataLength); break;
            case 0: dm6215_fbdata(&wheel_motors[0], g_Can1RxData, RxHeader1.DataLength); break;
            default: break;
        }
    }
    else if (hfdcan->Instance == FDCAN3)
    {
        // CAN3 接收 — 多帧协议命令（后续实现）
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader3, g_Can3RxData);
        g_can3_rx_cnt++;
    }
}

// ===== FDCAN2 RX 回调（FIFO1） =====
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t its)
{
    if ((its & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == RESET) return;
    if (hfdcan->Instance == FDCAN2)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
        g_fb_can_id2 = RxHeader2.Identifier;
        g_rx2_cnt++;
        for (int i = 0; i < 8; i++) g_rx2_frame[i] = g_Can2RxData[i];

        switch (RxHeader2.Identifier)
        {
            case 3: dm4310_fbdata(&joint_motors[2], g_Can2RxData, RxHeader2.DataLength); break;
            case 4: dm4310_fbdata(&joint_motors[3], g_Can2RxData, RxHeader2.DataLength); break;
            case 0: dm6215_fbdata(&wheel_motors[1], g_Can2RxData, RxHeader2.DataLength); break;
            default: break;
        }
    }
}
