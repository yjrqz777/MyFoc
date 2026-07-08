/**
 * @file    bsp_adc.c
 * @brief   ADC 底层驱动实现 — 注入模式三相电流 + 母线电流采样
 *******************************************************************************
 * @note    电流采样电路：3.3V 参考，12-bit ADC，
 *          采样电阻 20Ω，放大器增益 0.1，
 *          转换公式：I = (raw - 2048) * (3.3 / 4096 / 20 / 0.1)
 *******************************************************************************
 */

#include "bsp_adc.h"
#include "adc.h"

/**
 * @brief 注入通道采样原始值缓冲（ISR 中更新）
 * @note  索引 0~3：Ia, Ib, Ic, Ibus
 */
static volatile uint16_t s_injected_raw[BSP_ADC_INJECTED_CHANNELS] = {0};

/**
 * @brief  ADC 原始值转电流值
 * @param[in] raw  ADC 原始采样值（12-bit，0~4095）
 * @return 实际电流值，单位 A
 * @note   中点偏移 2048（对应 0A），
 *         比例系数 3.3V / 4096 / 20Ω / 0.1 ≈ 0.0004028
 */
static float BspAdc_RawToCurrent(uint16_t raw)
{
    const float adc_scale = 3.3f / 4096.0f / 20.0f / 0.1f;
    return ((float)raw - 2048.0f) * adc_scale;
}

/**
 * @brief  启动 ADC1 注入组采样（含自校准 + 中断模式）
 * @retval HAL_OK       启动成功
 * @retval HAL_ERROR    校准失败
 * @retval HAL_BUSY     外设忙
 * @note   必须先调用此函数使能采样，之后 ADC 转换完成会触发中断
 */
HAL_StatusTypeDef BspAdc_StartInjected(void)
{
    HAL_StatusTypeDef status;

    status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/**
 * @brief  更新注入组采样缓冲（在 HAL_ADCEx_InjectedConvCpltCallback 中调用）
 * @param[in] hadc  ADC 句柄指针
 * @retval 1  更新成功
 * @retval 0  参数无效（NULL 或不是 ADC1）
 * @note   读取顺序：Rank1=Ia, Rank2=Ib, Rank3=Ic, Rank4=Ibus
 */
uint8_t BspAdc_UpdateInjected(ADC_HandleTypeDef *hadc)
{
    if ((hadc == NULL) || (hadc->Instance != ADC1))
    {
        return 0u;
    }

    s_injected_raw[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    s_injected_raw[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    s_injected_raw[2] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    s_injected_raw[3] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);

    return 1u;
}

/**
 * @brief  获取指定注入通道的原始 ADC 值
 * @param[in] index  通道索引（0~3）
 * @return 原始 ADC 值（0~4095），越界返回 0
 */
uint16_t BspAdc_GetInjectedRaw(uint8_t index)
{
    if (index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0u;
    }

    return s_injected_raw[index];
}

/**
 * @brief  获取 A 相电流
 * @return A 相电流值（A）
 */
float BspAdc_GetIa(void)
{
    return BspAdc_RawToCurrent(s_injected_raw[0]);
}

/**
 * @brief  获取 B 相电流
 * @return B 相电流值（A）
 */
float BspAdc_GetIb(void)
{
    return BspAdc_RawToCurrent(s_injected_raw[1]);
}

/**
 * @brief  获取 C 相电流
 * @return C 相电流值（A）
 */
float BspAdc_GetIc(void)
{
    return BspAdc_RawToCurrent(s_injected_raw[2]);
}

/**
 * @brief  获取母线电流
 * @return 母线电流值（A）
 */
float BspAdc_GetIbus(void)
{
    return BspAdc_RawToCurrent(s_injected_raw[3]);
}
