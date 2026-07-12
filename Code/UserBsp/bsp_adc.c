/**
 * @file    bsp_adc.c
 * @brief   ADC 底层驱动实现 — 注入模式三相电流 + 母线电流采样
 *******************************************************************************
 * @note    电流采样电路：3.3V 参考，12-bit ADC，
 *          采样电阻 10mΩ，放大器增益 30，
 *          转换公式：I = (raw - offset) * (3.3 / 4096 / 0.010 / 30)
 *******************************************************************************
 */

#include "bsp_adc.h"
#include "adc.h"
#include "user_motor.h"

#define BSP_ADC_OFFSET_SAMPLE_COUNT 1024u
#define BSP_ADC2_POLL_TIMEOUT_MS    2u
#define BSP_ADC_REF_VOLTAGE         3.3f
#define BSP_ADC_CONVERSION_STEPS    4096.0f
#define BSP_ADC_FULL_SCALE          4095.0f
#define BSP_ADC_CURRENT_SHUNT_OHM   0.010f
#define BSP_ADC_CURRENT_GAIN        30.0f
#define BSP_ADC_PHASE_CURRENT_SIGN  (-1.0f)

/**
 * @brief 注入通道采样原始值缓冲（ISR 中更新）
 * @note  索引 0~3：Ia, Ib, Ic, Ibus
 */
static volatile uint16_t s_injected_raw[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile uint16_t s_adc2_regular_raw[BSP_ADC2_REGULAR_CHANNELS] = {0};
static volatile uint32_t s_offset_sum[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile float s_current_offset[BSP_ADC_INJECTED_CHANNELS] = {2048.0f, 2048.0f, 2048.0f, 2048.0f};
static volatile uint16_t s_offset_samples = 0u;
static volatile uint8_t s_offset_ready = 0u;
static uint8_t s_adc2_calibrated = 0u;

static const uint32_t s_adc2_channel_map[BSP_ADC2_REGULAR_CHANNELS] = {
    ADC_CHANNEL_6,     /* PC0: SHA */
    ADC_CHANNEL_7,     /* PC1: SHB */
    ADC_CHANNEL_8,     /* PC2: SHC */
    ADC_CHANNEL_5,     /* PC4: POT */
    ADC_CHANNEL_11,    /* PC5: VBUS */
};

/**
 * @brief  复位 ADC 零电流偏移累积
 * @note   清零各通道偏移和值和采样计数，
 *         偏移准备标志置 0，等待重新累积。
 *         在 ADC 启动注入采样时调用。
 */
static void BspAdc_ResetCurrentOffset(void)
{
    uint8_t index;

    for (index = 0u; index < BSP_ADC_INJECTED_CHANNELS; index++)
    {
        s_offset_sum[index] = 0u;
        s_current_offset[index] = 2048.0f;
    }

    s_offset_samples = 0u;
    s_offset_ready = 0u;
}

/**
 * @brief  ADC 原始值转电流值
 * @param[in] raw  ADC 原始采样值（12-bit，0~4095）
 * @return 实际电流值，单位 A
 * @note   offset 为零电流 ADC 偏置，
 *         比例系数 3.3V / 4096 / 0.010Ω / 30 ≈ 0.0026855 A/LSB
 */
static float BspAdc_RawToCurrent(uint16_t raw, float offset)
{
    const float adc_scale = BSP_ADC_REF_VOLTAGE / BSP_ADC_CONVERSION_STEPS /
                            BSP_ADC_CURRENT_SHUNT_OHM / BSP_ADC_CURRENT_GAIN;
    return ((float)raw - offset) * adc_scale;
}

/**
 * @brief  读取 ADC2 单个通道值（阻塞轮询模式）
 * @param[in]  adc_channel   ADC 通道号（如 ADC_CHANNEL_6）
 * @param[out] raw           原始 ADC 值（12-bit，0~4095）
 * @retval HAL_OK   读取成功
 * @retval other    HAL 错误码
 * @note   每次调用均重新配置通道和启动转换，适用于低速轮询。
 *         超时时间由 BSP_ADC2_POLL_TIMEOUT_MS 定义。
 */
static HAL_StatusTypeDef BspAdc2_ReadChannel(uint32_t adc_channel, uint16_t *raw)
{
    ADC_ChannelConfTypeDef config = {0};
    HAL_StatusTypeDef status;

    config.Channel = adc_channel;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0;

    status = HAL_ADC_ConfigChannel(&hadc2, &config);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_ADC_Start(&hadc2);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_ADC_PollForConversion(&hadc2, BSP_ADC2_POLL_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        *raw = (uint16_t)HAL_ADC_GetValue(&hadc2);
    }

    (void)HAL_ADC_Stop(&hadc2);
    return status;
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

    BspAdc_ResetCurrentOffset();

    status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);/* 校准adc */
    if (status != HAL_OK)
    {
        return status;
    }
    status = HAL_ADCEx_InjectedStart_IT(&hadc1);
    return status;
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
    uint8_t index;

    if ((hadc == NULL) || (hadc->Instance != ADC1))
    {
        return 0u;
    }

    s_injected_raw[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    s_injected_raw[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    s_injected_raw[2] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    s_injected_raw[3] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);

    if (s_offset_ready == 0u)
    {
        for (index = 0u; index < BSP_ADC_INJECTED_CHANNELS; index++)
        {
            s_offset_sum[index] += s_injected_raw[index];
        }

        s_offset_samples++;
        if (s_offset_samples >= BSP_ADC_OFFSET_SAMPLE_COUNT)
        {
            for (index = 0u; index < BSP_ADC_INJECTED_CHANNELS; index++)
            {
                s_current_offset[index] = (float)s_offset_sum[index] / (float)BSP_ADC_OFFSET_SAMPLE_COUNT;
            }
            s_offset_ready = 1u;
        }
    }

    return 1u;
}

/**
 * @brief  ADC1 注入转换完成中断回调（HAL 库重写）
 * @param[in] hadc  ADC 句柄指针
 * @note   由 TIM1 CH4 触发 ADC1 注入转换，转换完成后硬件触发此回调。
 *         在回调中更新采样缓冲并执行电机快速控制环。
 *         此函数运行在中断上下文中，必须保持高效。
 *         控制频率由 TIM1 配置决定，标称 20kHz。
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (BspAdc_UpdateInjected(hadc) != 0u)
    {
        UserMotor_FastLoop();
    }
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
 * @brief  查询 ADC 零电流偏移校准是否完成
 * @retval 1  偏移校准已完成，电流采样值可用
 * @retval 0  偏移收集中，电流值尚不可靠
 * @note   采集 1024 个样本后自动置位
 */

/**
 * @brief  Get calibrated zero-current offset in ADC counts.
 * @param[in] index  Injected channel index: 0=Ia, 1=Ib, 2=Ic, 3=Ibus
 * @return Offset average in raw ADC counts; returns 0 for invalid index.
 */
float BspAdc_GetCurrentOffsetRaw(uint8_t index)
{
    if (index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0.0f;
    }

    return s_current_offset[index];
}

/**
 * @brief  Get calibrated zero-current offset voltage.
 * @param[in] index  Injected channel index: 0=Ia, 1=Ib, 2=Ic, 3=Ibus
 * @return Offset voltage in V; returns 0 for invalid index.
 */
float BspAdc_GetCurrentOffsetVoltage(uint8_t index)
{
    return (BspAdc_GetCurrentOffsetRaw(index) * BSP_ADC_REF_VOLTAGE) / BSP_ADC_FULL_SCALE;
}

uint8_t BspAdc_IsCurrentOffsetReady(void)
{
    return s_offset_ready;
}

/**
 * @brief  轮询更新所有 ADC2 通道值
 * @retval HAL_OK      所有通道更新成功
 * @retval HAL_ERROR   校准失败
 * @retval other       通道读取失败时的 HAL 错误码
 * @note   包含 ADC2 自校准（仅首次执行），然后依次读取
 *         各通道：SHA、SHB、SHC、POT、VBUS
 *         在 foreground 任务中调用，不在中断中使用。
 */
HAL_StatusTypeDef BspAdc2_UpdateAll(void)
{
    HAL_StatusTypeDef status;
    uint8_t index;
    uint16_t raw;

    if (s_adc2_calibrated == 0u)
    {
        status = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
        if (status != HAL_OK)
        {
            return status;
        }
        s_adc2_calibrated = 1u;
    }

    for (index = 0u; index < BSP_ADC2_REGULAR_CHANNELS; index++)
    {
        status = BspAdc2_ReadChannel(s_adc2_channel_map[index], &raw);
        if (status != HAL_OK)
        {
            return status;
        }
        s_adc2_regular_raw[index] = raw;
    }

    return HAL_OK;
}

/**
 * @brief  获取指定 ADC2 通道的原始采样值
 * @param[in] channel  通道枚举（SHA/SHB/SHC/POT/VBUS）
 * @return 原始 ADC 值（0~4095），无效通道返回 0
 * @note   返回最近一次 BspAdc2_UpdateAll() 采集的值
 */
uint16_t BspAdc2_GetRaw(BspAdc2Channel_t channel)
{
    if ((uint8_t)channel >= BSP_ADC2_REGULAR_CHANNELS)
    {
        return 0u;
    }

    return s_adc2_regular_raw[(uint8_t)channel];
}

/**
 * @brief  获取指定 ADC2 通道的电压值
 * @param[in] channel  通道枚举（SHA/SHB/SHC/POT/VBUS）
 * @return 电压值（V），计算公式：raw * 3.3V / 4095
 */
float BspAdc2_GetVoltage(BspAdc2Channel_t channel)
{
    return ((float)BspAdc2_GetRaw(channel) * BSP_ADC_REF_VOLTAGE) / BSP_ADC_FULL_SCALE;
}

/**
 * @brief  获取 A 相电流
 * @return A 相电流值（A）
 */
float BspAdc_GetIa(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdc_RawToCurrent(s_injected_raw[0], s_current_offset[0]);
}

/**
 * @brief  获取 B 相电流
 * @return B 相电流值（A）
 */
float BspAdc_GetIb(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdc_RawToCurrent(s_injected_raw[1], s_current_offset[1]);
}

/**
 * @brief  获取 C 相电流
 * @return C 相电流值（A）
 */
float BspAdc_GetIc(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdc_RawToCurrent(s_injected_raw[2], s_current_offset[2]);
}

/**
 * @brief  获取母线电流
 * @return 母线电流值（A）
 */
float BspAdc_GetIbus(void)
{
    return BspAdc_RawToCurrent(s_injected_raw[3], s_current_offset[3]);
}
