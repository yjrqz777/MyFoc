#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

HAL_StatusTypeDef BspPwm_Start(void);
void BspPwm_Stop(void);
uint16_t BspPwm_GetPeriod(void);
void BspPwm_SetCompare(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);
void BspPwm_SetVoltageABC(float ua, float ub, float uc);

#ifdef __cplusplus
}
#endif

#endif
