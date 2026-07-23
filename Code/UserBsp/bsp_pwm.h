/**
 * @file    bsp_pwm.h
 * @brief   PWM 底层驱动头文件 — 三相六路 PWM 输出
 *******************************************************************************
 * @note    TIM1 高级定时器，6 路互补 PWM（CH1~CH3 + CH1N~CH3N）+ CH4 辅助输出
 *          支持电压标幺值到占空比的转换
 *******************************************************************************
 */

#ifndef __BSP_PWM_H__
#define __BSP_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

HAL_StatusTypeDef BspPwm_Start(void);
HAL_StatusTypeDef BspPwm_StartAdcTrigger(void);
HAL_StatusTypeDef BspPwm_StartPowerOutputs(void);
void BspPwm_Stop(void);
uint16_t BspPwm_GetPeriod(void);
uint16_t BspPwm_GetMinCompare(void);
void BspPwm_SetCompare(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);
void BspPwm_SetVoltageABC(float ua, float ub, float uc);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H__ */
