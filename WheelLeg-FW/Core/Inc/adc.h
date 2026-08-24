#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern ADC_HandleTypeDef hadc1;

void MX_ADC1_Init(void);
uint32_t adc_read_power_raw(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
