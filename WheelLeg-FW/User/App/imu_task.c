#include "imu_task.h"
#include "spi.h"
#include "adc.h"
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "mahony_filter.h"
#include "bsp_dwt.h"
#include <string.h>

ImuData_t g_imu;
struct MAHONY_FILTER_t mahony;
static Axis3f gyro_axis, accel_axis;
static uint32_t imu_dwt_cnt = 0;

/* 电池电压分压倍率：R67/R68分压（100k/10k≈11倍，待实测标定） */
#define VBAT_DIVIDER   11.0f
#define ADC_REF_VOLT   3.3f
#define ADC_FULL_SCALE 4095.0f

/* BMI088 Middleware 实现：GPIO/SPI 初始化 */
void BMI088_GPIO_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* ACC_CS = PC0, GYRO_CS = PC3 输出高 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_3, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void BMI088_com_init(void)
{
    BMI088_SPI = &hspi2;
}

void BMI088_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (uint32_t)((float)us * 480.0f);  /* 480MHz */
    while ((DWT->CYCCNT - start) < ticks);
}

/* 读取电源电压（分压倍率待标定） */
static void imu_read_power(void)
{
    uint32_t raw = adc_read_power_raw();
    g_imu.power_voltage = (float)raw / ADC_FULL_SCALE * ADC_REF_VOLT * VBAT_DIVIDER;
}

void imu_task_init(void)
{
    memset(&g_imu, 0, sizeof(g_imu));

    BMI088_GPIO_init();
    BMI088_com_init();
    DWT_Init(480);

    /* BMI088 初始化（calibrate=0 不标定，用出厂校准） */
    uint8_t err = BMI088_init(&hspi2, 0);
    if (err == BMI088_NO_ERROR)
    {
        mahony_init(&mahony, 1.0f, 0.0f, 0.001f);
        g_imu.imu_ready = 1;
    }
}

void imu_task_process(void)
{
    if (!g_imu.imu_ready)
        return;

    float dt = DWT_GetDeltaT(&imu_dwt_cnt);
    if (dt <= 0.0f || dt > 0.1f)
        dt = 0.001f;
    mahony.dt = dt;

    BMI088_Read(&BMI088);

    g_imu.accel[0] = BMI088.Accel[0];
    g_imu.accel[1] = BMI088.Accel[1];
    g_imu.accel[2] = BMI088.Accel[2];
    g_imu.gyro[0] = BMI088.Gyro[0];
    g_imu.gyro[1] = BMI088.Gyro[1];
    g_imu.gyro[2] = BMI088.Gyro[2];
    g_imu.temperature = BMI088.Temperature;

    accel_axis.x = BMI088.Accel[0];
    accel_axis.y = BMI088.Accel[1];
    accel_axis.z = BMI088.Accel[2];
    gyro_axis.x = BMI088.Gyro[0];
    gyro_axis.y = BMI088.Gyro[1];
    gyro_axis.z = BMI088.Gyro[2];

    mahony_input(&mahony, gyro_axis, accel_axis);
    mahony_update(&mahony);
    mahony_output(&mahony);

    /* 旧版映射：INS.Pitch=mahony.roll, INS.Roll=mahony.pitch */
    g_imu.roll = mahony.pitch;
    g_imu.pitch = mahony.roll;
    g_imu.yaw = mahony.yaw;

    /* 电压低频采样（每100次约100-200ms） */
    static uint8_t vbat_div = 0;
    if (++vbat_div >= 100)
    {
        vbat_div = 0;
        imu_read_power();
    }
}
