#include "main_task.h"
#include "can_bsp.h"
#include "uart_task.h"
#include "dm4310_drv.h"
#include "imu_task.h"
#include "tim.h"
#include "ws2812.h"

// 外部变量
extern UART_HandleTypeDef huart1;

// 控制周期（ms）
#define CONTROL_PERIOD_MS   3

// ===== 电池过放保护（6S LiPo: 额定22.2V / 限制25.2V）=====
#define VBAT_WARN_V     21.6f   // 3.6V/节: 报警+蜂鸣器间隔4s响0.2s
#define VBAT_LOCK_V     20.4f   // 3.4V/节: 禁电机+蜂鸣器间隔2s响0.1s
#define BUZZER_ON       (TIM12->CCR2 = 500)   // 4KHz 50%占空比
#define BUZZER_OFF      (TIM12->CCR2 = 0)

// 蜂鸣器定时（3ms周期）
#define WARN_ON_TICKS   67    // 0.2s / 3ms
#define WARN_OFF_TICKS  1333  // 4s / 3ms
#define LOCK_ON_TICKS   33    // 0.1s / 3ms
#define LOCK_OFF_TICKS  667   // 2s / 3ms

// WS2812 亮度（低亮度）
#define LED_LOW         15

// 过放保护状态
static uint8_t vbat_prot_state = 0;   // 0=正常 1=警告 2=锁定
static uint8_t vbat_buzz_toggle = 0;

// 电池保护检查：返回0正常 / 1警告 / 2锁定
uint8_t vbat_protect_check(void)
{
    if (!g_imu.imu_ready)
        return vbat_prot_state;

    float vbat = g_imu.power_voltage;

    if (vbat < VBAT_LOCK_V)
    {
        vbat_prot_state = 2;
    }
    else if (vbat < VBAT_WARN_V)
    {
        // 只有从正常状态进入才升级为警告（锁定后保持锁定，需重新上电）
        if (vbat_prot_state != 2)
            vbat_prot_state = 1;
    }
    else if (vbat_prot_state != 2)
    {
        vbat_prot_state = 0;
    }
    return vbat_prot_state;
}

// 主任务初始化
void main_task_init(void)
{
    // 初始化FDCAN
    FDCAN1_Config();
    FDCAN2_Config();
    
    // 初始化4个关节电机
    joint_motors_init();
    
    // 初始化蜂鸣器PWM（TIM12 CH2, 4KHz）
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    BUZZER_OFF;

    // 初始化WS2812 LED（关灯）
    WS2812_Ctrl(0, 0, 0);

    // 使能所有电机（发送MIT模式使能指令）
    for(int j = 0; j < 10; j++)
    {
        enable_motor_mode(&hfdcan1, joint_motors[0].para.id, joint_motors[0].mode);
        enable_motor_mode(&hfdcan1, joint_motors[1].para.id, joint_motors[1].mode);
        enable_motor_mode(&hfdcan2, joint_motors[2].para.id, joint_motors[2].mode);
        enable_motor_mode(&hfdcan2, joint_motors[3].para.id, joint_motors[3].mode);
        HAL_Delay(1);
    }
    
    // 初始化UART任务
    uart_task_init();
}

// 主任务循环
void main_task_loop(void)
{
    // 电池过放保护检查
    uint8_t prot = vbat_protect_check();
    if (prot == 2)
    {
        // 锁定: 禁电机 + 蜂鸣器间隔2s响0.1s
        disable_motor_mode(&hfdcan1, joint_motors[0].para.id, joint_motors[0].mode);
        disable_motor_mode(&hfdcan1, joint_motors[1].para.id, joint_motors[1].mode);
        disable_motor_mode(&hfdcan2, joint_motors[2].para.id, joint_motors[2].mode);
        disable_motor_mode(&hfdcan2, joint_motors[3].para.id, joint_motors[3].mode);
        if (vbat_buzz_toggle < LOCK_ON_TICKS)
        {
            BUZZER_ON;
            vbat_buzz_toggle++;
        }
        else if (vbat_buzz_toggle < LOCK_ON_TICKS + LOCK_OFF_TICKS)
        {
            BUZZER_OFF;
            vbat_buzz_toggle++;
        }
        else
        {
            vbat_buzz_toggle = 0;
        }
        HAL_Delay(CONTROL_PERIOD_MS);
        return;     // 不再发送任何扭矩/位置命令
    }
    if (prot == 1)
    {
        // 警告: 蜂鸣器间隔4s响0.2s
        if (vbat_buzz_toggle < WARN_ON_TICKS)
        {
            BUZZER_ON;
            vbat_buzz_toggle++;
        }
        else if (vbat_buzz_toggle < WARN_ON_TICKS + WARN_OFF_TICKS)
        {
            BUZZER_OFF;
            vbat_buzz_toggle++;
        }
        else
        {
            vbat_buzz_toggle = 0;
        }
    }

    // CAN3 连接状态 LED（低亮度彩色指示）
    // 每100次更新一次（~300ms），避免 SPI 过于频繁
    static uint16_t led_tick = 0;
    if (++led_tick >= 100)
    {
        led_tick = 0;
        if (g_can3_tx_cnt > 0)
            WS2812_Ctrl(0, LED_LOW, 0);      // 绿 = CAN3 正常通信
        else
            WS2812_Ctrl(LED_LOW, 0, 0);      // 红 = CAN3 未连接/无数据
    }

    // 周期性重发使能帧（每50ms），确保电机任何时刻就绪后都能被使能
    static uint32_t enable_tick = 0;
    if (++enable_tick >= 16)   // 16 × 3ms ≈ 50ms
    {
        enable_tick = 0;
        enable_motor_mode(&hfdcan1, joint_motors[0].para.id, joint_motors[0].mode);
        enable_motor_mode(&hfdcan1, joint_motors[1].para.id, joint_motors[1].mode);
        enable_motor_mode(&hfdcan2, joint_motors[2].para.id, joint_motors[2].mode);
        enable_motor_mode(&hfdcan2, joint_motors[3].para.id, joint_motors[3].mode);
    }

    // 1. 获取控制命令（扭矩 or 位置刚度）
    float torque[4] = {0};
    if (g_ctrl_mode == CTRL_RL)
    {
        // 从RL模式获取扭矩
        torque[0] = g_rl_torques.joint_torque[0];  // 左下关节
        torque[1] = g_rl_torques.joint_torque[1];  // 左上关节
        torque[2] = g_rl_torques.joint_torque[2];  // 右下关节
        torque[3] = g_rl_torques.joint_torque[3];  // 右上关节
    }
    
    // 2. 发送命令给4个关节电机
    if (g_ctrl_mode == CTRL_POS)
    {
        // 位置刚度控制: mit_ctrl(pos, 0, kp, kd, 0)
        // kp限幅<=20 (防过载: 扭矩=kp*误差, DM4310峰值7Nm)
        float kp = g_pos_cmd.kp, kd = g_pos_cmd.kd;
        if (kp > 20.0f) kp = 20.0f;
        if (kd > 2.0f) kd = 2.0f;
        mit_ctrl(&hfdcan1, joint_motors[0].para.id, g_pos_cmd.pos[0], 0, kp, kd, 0);  // 右前
        mit_ctrl(&hfdcan1, joint_motors[1].para.id, g_pos_cmd.pos[1], 0, kp, kd, 0);  // 右后
        mit_ctrl(&hfdcan2, joint_motors[2].para.id, g_pos_cmd.pos[2], 0, kp, kd, 0);  // 左前
        mit_ctrl(&hfdcan2, joint_motors[3].para.id, g_pos_cmd.pos[3], 0, kp, kd, 0);  // 左后
    }
    else
    {
        // 扭矩模式
        mit_ctrl(&hfdcan1, joint_motors[0].para.id, 0, 0, 0, 0, torque[0]);  // 左下关节
        mit_ctrl(&hfdcan1, joint_motors[1].para.id, 0, 0, 0, 0, torque[1]);  // 左上关节
        mit_ctrl(&hfdcan2, joint_motors[2].para.id, 0, 0, 0, 0, torque[2]);  // 右下关节
        mit_ctrl(&hfdcan2, joint_motors[3].para.id, 0, 0, 0, 0, torque[3]);  // 右上关节
    }
    
    // 3. 延时控制周期
    HAL_Delay(CONTROL_PERIOD_MS);
}
