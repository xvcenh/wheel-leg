#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "main.h"

/* IMU/传感器数据（供uart_task读取上报） */
typedef struct {
    float roll;          /* 横滚角 (rad) */
    float pitch;         /* 俯仰角 (rad) */
    float yaw;           /* 偏航角 (rad) */
    float gyro[3];       /* 角速度 (rad/s) */
    float accel[3];      /* 加速度 (g) */
    float temperature;   /* IMU芯片温度 (℃) */
    float power_voltage; /* 电池电压 (V) */
    uint8_t imu_ready;   /* IMU初始化完成标志 */
} ImuData_t;

extern ImuData_t g_imu;

void imu_task_init(void);
void imu_task_process(void);   /* 周期调用（约1-2ms） */

#endif /* __IMU_TASK_H__ */
