/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : WheelLeg-FW: 6 电机控制 + CAN3 多帧上报 + LED + 过放保护
  ******************************************************************************
  */
#include "main.h"
#include "fdcan.h"
#include "gpio.h"
#include "spi.h"
#include "ws2812.h"
#include "adc.h"
#include "tim.h"
#include "can_bsp.h"
#include "can3_task.h"
#include "dm4310_drv.h"
#include "imu_task.h"

#define LED_LOW  8
#define VBAT_WARN_V     21.6f
#define VBAT_LOCK_V     20.4f
#define VBAT_DIVIDER    11.0f
#define ADC_FULL_SCALE  4095.0f
#define ECR_ERROR_THRESH 10

/* ===== 过放保护 ===== */
static uint8_t prot_state = 0;

void check_overdischarge(float vbat)
{
    if (vbat < VBAT_LOCK_V) prot_state = 2;
    else if (vbat < VBAT_WARN_V) { if (prot_state != 2) prot_state = 1; }
    else if (prot_state != 2) prot_state = 0;
}

/* ===== LED ===== */
static uint32_t bo_recover_tick = 0;

void recover_fdcan3(void)
{
    HAL_FDCAN_Stop(&hfdcan3);
    HAL_Delay(5);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void update_led(uint32_t tick)
{
    FDCAN_ProtocolStatusTypeDef psr;
    FDCAN_ErrorCountersTypeDef cnt;
    HAL_FDCAN_GetProtocolStatus(&hfdcan3, &psr);
    HAL_FDCAN_GetErrorCounters(&hfdcan3, &cnt);

    if (psr.BusOff)
    {
        if (tick - bo_recover_tick >= 1000) {
            bo_recover_tick = tick;
            recover_fdcan3();
        }
        WS2812_Ctrl(LED_LOW, 0, 0);
        return;
    }
    uint8_t can3_err = (cnt.TxErrorCnt > ECR_ERROR_THRESH);
    if (prot_state == 2)
        WS2812_Ctrl(LED_LOW, 0, 0);
    else if (prot_state == 1)
        WS2812_Ctrl(LED_LOW, LED_LOW, 0);
    else if (can3_err)
        WS2812_Ctrl(LED_LOW, 0, 0);
    else
        WS2812_Ctrl(0, LED_LOW, 0);
}

/* ===== 蜂鸣器 ===== */
void buzzer_on(void) { TIM12->CCR2 = 500; }
void buzzer_off(void) { TIM12->CCR2 = 0; }

static uint32_t buzz_tick = 0, buzz_phase = 0;

void handle_buzzer(void)
{
    if (prot_state == 2)
    {
        if (buzz_phase == 0 && buzz_tick < 33) { buzzer_on(); buzz_tick++; }
        else if (buzz_phase == 0) { buzz_phase = 1; buzz_tick = 0; }
        if (buzz_phase == 1 && buzz_tick < 667) { buzzer_off(); buzz_tick++; }
        else if (buzz_phase == 1) { buzz_phase = 0; buzz_tick = 0; }
    }
    else if (prot_state == 1)
    {
        if (buzz_phase == 0 && buzz_tick < 67) { buzzer_on(); buzz_tick++; }
        else if (buzz_phase == 0) { buzz_phase = 1; buzz_tick = 0; }
        if (buzz_phase == 1 && buzz_tick < 1333) { buzzer_off(); buzz_tick++; }
        else if (buzz_phase == 1) { buzz_phase = 0; buzz_tick = 0; }
    }
    else { buzzer_off(); buzz_tick = 0; buzz_phase = 0; }
}

/* ===== 主函数 ===== */
void SystemClock_Config(void);
void Error_Handler(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_FDCAN2_Init();
    MX_FDCAN3_Init();
    MX_SPI6_Init();
    MX_SPI2_Init();         // BMI088 IMU
    MX_ADC1_Init();
    MX_TIM12_Init();

    // 电机供电
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_SET);

    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    buzzer_off();

    // 配置 FDCAN1/2/3（RX 过滤器 + 中断 + Start）
    FDCAN1_Config();
    FDCAN2_Config();
    FDCAN3_Config();

    // 初始化电机结构体
    all_motors_init();

    // 初始化 BMI088 IMU（SPI2）
    imu_task_init();

    // 使能 4 关节电机
    for (int j = 0; j < 10; j++)
    {
        enable_motor_mode(&hfdcan1, 6, MIT_MODE);
        enable_motor_mode(&hfdcan1, 8, MIT_MODE);
        enable_motor_mode(&hfdcan2, 6, MIT_MODE);
        enable_motor_mode(&hfdcan2, 8, MIT_MODE);
        HAL_Delay(1);
    }
    // 使能 2 轮毂电机
    for (int j = 0; j < 10; j++)
    {
        enable_motor_mode(&hfdcan1, 1, MIT_MODE);
        enable_motor_mode(&hfdcan2, 1, MIT_MODE);
        HAL_Delay(1);
    }

    // 主循环
    uint32_t tick_3ms = 0;
    uint32_t tick_led = 0, tick_vbat = 0, tick_motor = 0;
    float vbat = 0.0f;

    while (1)
    {
        tick_3ms++;

        // IMU 姿态解算（BMI088 + Mahony，~1ms 周期）
        imu_task_process();

        // CAN3 多帧上报（电机状态 + 轮毂 + IMU + 温度 + 电压）
        can3_task_process();

        // LED 每 300ms
        if (tick_3ms - tick_led >= 100) {
            tick_led = tick_3ms;
            update_led(tick_3ms);
        }

        // 电池电压每 500ms
        if (tick_3ms - tick_vbat >= 167) {
            tick_vbat = tick_3ms;
            vbat = (float)adc_read_power_raw() / ADC_FULL_SCALE * 3.3f * VBAT_DIVIDER;
            check_overdischarge(vbat);
            can3_set_power(vbat, prot_state);
        }

        // 过放保护
        if (prot_state == 2) {
            disable_motor_mode(&hfdcan1, 6, MIT_MODE);
            disable_motor_mode(&hfdcan1, 8, MIT_MODE);
            disable_motor_mode(&hfdcan2, 6, MIT_MODE);
            disable_motor_mode(&hfdcan2, 8, MIT_MODE);
            disable_motor_mode(&hfdcan1, 1, MIT_MODE);
            disable_motor_mode(&hfdcan2, 1, MIT_MODE);
            handle_buzzer();
            HAL_Delay(3);
            continue;
        }
        handle_buzzer();

        // 重发电机使能帧每 50ms
        if (tick_3ms - tick_motor >= 16) {
            tick_motor = tick_3ms;
            enable_motor_mode(&hfdcan1, 6, MIT_MODE);
            enable_motor_mode(&hfdcan1, 8, MIT_MODE);
            enable_motor_mode(&hfdcan2, 6, MIT_MODE);
            enable_motor_mode(&hfdcan2, 8, MIT_MODE);
            enable_motor_mode(&hfdcan1, 1, MIT_MODE);
            enable_motor_mode(&hfdcan2, 1, MIT_MODE);
        }

        HAL_Delay(3);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 2;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = 1;
    RCC_OscInitStruct.PLL.PLLQ = 6;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
