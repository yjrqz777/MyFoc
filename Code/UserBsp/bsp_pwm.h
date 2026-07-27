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

HAL_StatusTypeDef BspPwmStart(void);
HAL_StatusTypeDef BspPwmStartAdcTrigger(void);
HAL_StatusTypeDef BspPwmStartPowerOutputs(void);
void BspPwmStop(void);
uint16_t BspPwmGetPeriod(void);
uint16_t BspPwmGetMinCompare(void);
void BspPwmSetCompare(uint16_t u16Ccr1, uint16_t u16Ccr2, uint16_t u16Ccr3);
void BspPwmSetVoltageAbc(float f32Ua, float f32Ub, float f32Uc);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_PWM_H__ */
