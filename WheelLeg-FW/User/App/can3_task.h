#ifndef _CAN3_TASK_H
#define _CAN3_TASK_H

#include "main.h"

void can3_task_process(void);
void can3_set_power(float vbat, uint8_t prot);

#endif
