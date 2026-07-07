#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BSP_ADC_INJECTED_CHANNELS 4u

HAL_StatusTypeDef BspAdc_StartInjected(void);
uint8_t BspAdc_UpdateInjected(ADC_HandleTypeDef *hadc);
uint16_t BspAdc_GetInjectedRaw(uint8_t index);

float BspAdc_GetIa(void);
float BspAdc_GetIb(void);
float BspAdc_GetIc(void);
float BspAdc_GetIbus(void);

#ifdef __cplusplus
}
#endif

#endif
