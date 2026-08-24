#ifndef __MAIN_TASK_H__
#define __MAIN_TASK_H__

#include "main.h"

// 函数声明
void main_task_init(void);
void main_task_loop(void);
uint8_t vbat_protect_check(void);   // 电池过放保护状态: 0=正常 1=低压警告 2=低压锁定

#endif /* __MAIN_TASK_H__ */
