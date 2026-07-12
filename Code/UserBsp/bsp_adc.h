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

/** @brief ADC2 常规通道总数（SHA, SHB, SHC, POT, VBUS） */
#define BSP_ADC2_REGULAR_CHANNELS 5u

/** @brief ADC2 常规通道枚举 */
typedef enum
{
    BSP_ADC2_SHA = 0,   /**< PC0: SHA 通道 */
    BSP_ADC2_SHB,       /**< PC1: SHB 通道 */
    BSP_ADC2_SHC,       /**< PC2: SHC 通道 */
    BSP_ADC2_POT,       /**< PC4: 电位器 */
    BSP_ADC2_VBUS,      /**< PC5: 母线电压 */
} BspAdc2Channel_t;

HAL_StatusTypeDef BspAdc_StartInjected(void);
void BspAdc_RestartCurrentOffsetCalibration(void);
uint8_t BspAdc_UpdateInjected(ADC_HandleTypeDef *hadc);
uint16_t BspAdc_GetInjectedRaw(uint8_t index);
float BspAdc_GetCurrentOffsetRaw(uint8_t index);
float BspAdc_GetCurrentOffsetVoltage(uint8_t index);
uint8_t BspAdc_IsCurrentOffsetReady(void);

HAL_StatusTypeDef BspAdc2_UpdateAll(void);
uint16_t BspAdc2_GetRaw(BspAdc2Channel_t channel);
float BspAdc2_GetVoltage(BspAdc2Channel_t channel);

float BspAdc_GetIa(void);
float BspAdc_GetIb(void);
float BspAdc_GetIc(void);
float BspAdc_GetIbus(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H__ */
