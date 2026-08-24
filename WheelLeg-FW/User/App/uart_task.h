#ifndef __UART_TASK_H
#define __UART_TASK_H

#include "main.h"
#include "fdcan.h"

/* ===== Protocol Constants (matches legwheel_uart_node.cpp) ===== */

#define UART_HEAD1          0xA5
#define UART_HEAD2          0x5A

/* TX frame types (STM32 -> RDK) */
#define TYPE_ODOM           0x81    /* 24 bytes: vx,vy,wz,x,y,yaw */
#define TYPE_IMU            0x82    /* 40 bytes: roll,pitch,yaw,gyro_x/y/z,accel_x/y/z */
#define TYPE_MOTOR          0x83    /* 48 bytes: 4x(pos,vel,tor) floats */
#define TYPE_MOTOR_TEMP     0x84    /* 32 bytes: 4x(Tmos,Tcoil) floats */
#define TYPE_POWER          0x85    /* 12 bytes: bat_voltage, imu_temp, reserved */

/* RX frame types (RDK -> STM32) */
#define TYPE_CMD            0x01    /* 21 bytes: vx,vy,wz,leg,roll,enable */
#define TYPE_TORQUE_CMD     0x03    /* 25 bytes: 6x torque + mode */
#define TYPE_MODE           0x02    /* mode switch */
#define TYPE_ENABLE         0x04    /* 1 byte: 1=enable motors, 0=disable */
#define TYPE_POS_CMD        0x05    /* 24 bytes: 4x pos + kp + kd */

/* Frame sizes */
#define ODOM_LEN            24
#define IMU_LEN             40
#define CMD_LEN             21
#define TORQUE_CMD_LEN      25
#define MOTOR_LEN           48      /* 4x (pos, vel, tor) floats = 48 bytes */
#define MOTOR_TEMP_LEN      32      /* 4x (Tmos, Tcoil) floats = 32 bytes */
#define POWER_LEN           12      /* bat_voltage, imu_temp, reserved */
#define POS_CMD_LEN         24      /* 4x pos + kp + kd = 24 bytes */

/* Maximum UART buffer size */
#define UART_RX_BUF_SIZE    512
#define UART_TX_BUF_SIZE    128

/* ===== Control Mode ===== */
typedef enum {
    CTRL_LQR  = 0,  /* Original LQR balance control */
    CTRL_RL   = 1,  /* RL torque command mode */
    CTRL_PS2  = 2,  /* PS2 hand control */
    CTRL_POS  = 3,  /* 上位机位置刚度控制 (pos + kp/kd) */
} ctrl_mode_t;

/* ===== RX frame: TYPE_POS_CMD (上位机位置命令) ===== */
typedef struct __attribute__((packed)) {
    float pos[4];   /* 4x joint target position (rad) */
    float kp;       /* position stiffness */
    float kd;       /* damping */
} uart_pos_cmd_t;

/* ===== RX frame: TYPE_TORQUE_CMD (RDK -> STM32) ===== */
typedef struct __attribute__((packed)) {
    float joint_torque[4];   /* [0]=hip_R, [1]=knee_R, [2]=hip_L, [3]=knee_L */
    float wheel_torque[2];   /* [0]=wheel_R, [1]=wheel_L */
    uint8_t rl_mode;         /* 1=RL active, 0=hold position */
} uart_torque_rx_t;

/* ===== RX frame: TYPE_CMD (backward compatible with legwheel_uart_node) ===== */
typedef struct __attribute__((packed)) {
    float vx;
    float vy;
    float wz;
    float leg_height;
    float roll;
    uint8_t enable;
} uart_cmd_rx_t;

/* ===== TX frame: Odom ===== */
typedef struct __attribute__((packed)) {
    float vx, vy, wz;
    float x, y, yaw;
} uart_odom_tx_t;

/* ===== TX frame: IMU ===== */
typedef struct __attribute__((packed)) {
    float roll, pitch, yaw;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
} uart_imu_tx_t;

/* ===== TX frame: Motor (TYPE_MOTOR) ===== */
typedef struct __attribute__((packed)) {
    float pos[4];   /* 4x joint position (rad) */
    float vel[4];   /* 4x joint velocity (rad/s) */
    float tor[4];   /* 4x joint torque (Nm) */
} uart_motor_tx_t;

/* ===== TX frame: Motor Temperature (TYPE_MOTOR_TEMP) ===== */
typedef struct __attribute__((packed)) {
    float tmos[4];  /* 4x MOS temperature (degC) */
    float tcoil[4]; /* 4x coil temperature (degC) */
} uart_motor_temp_tx_t;

/* ===== TX frame: Power/Voltage (TYPE_POWER) ===== */
typedef struct __attribute__((packed)) {
    float bat_voltage;  /* battery voltage (V) */
    float imu_temp;     /* IMU temperature (degC) */
    float reserved;     /* reserved */
} uart_power_tx_t;

/* ===== Public Variables ===== */
extern volatile ctrl_mode_t g_ctrl_mode;
extern volatile uart_torque_rx_t g_rl_torques;
extern volatile uart_cmd_rx_t g_uart_cmd;
extern volatile uart_pos_cmd_t g_pos_cmd;
extern volatile uint8_t g_uart_cmd_valid;

/* ===== Public Functions ===== */
extern void uart_task_init(void);       /* Initialize UART hardware */
extern void uart_task_process(void);    /* 主循环轮询：解析命令+发送状态+限幅 */
extern uint8_t crc8_calc(const uint8_t *data, uint8_t len);
extern void process_frame(uint8_t type, const uint8_t *payload, uint8_t len);   /* 完整帧处理（UART/CAN3 共用） */

#endif /* __UART_TASK_H */
