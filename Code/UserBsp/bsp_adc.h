/**
 * @file    bsp_adc.h
 * @brief   ADC 底层驱动头文件 — 注入模式三相电流 + 母线电流采样
 *******************************************************************************
 * @note    ADC1 工作在注入扫描模式，4 通道分别采样 Ia、Ib、Ic、Ibus
 *          采样结果通过中断方式更新，提供电流转换接口
 *******************************************************************************
 */

#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

/** @brief ADC 注入通道总数（Ia, Ib, Ic, Ibus） */
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

#endif /* __BSP_ADC_H__ */
