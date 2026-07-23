/**
 * @file    bsp_adc.c
 * @brief   ADC 底层驱动实现 — 注入模式三相电流采样 + 偏置校准状态机
 *******************************************************************************
 * @note    电流采样电路：3.3V 参考，12-bit ADC，
 *          采样电阻 10mΩ，放大器增益 30，
 *          转换公式：I = (raw - offset) * (3.3 / 4096 / 0.010 / 30)
 *
 *          偏置校准采用非阻塞状态机 (doc §9)：
 *          SETTLE → DISCARD → ACCUMULATE → CALCULATE → READY
 *          中断中仅做累计和 min/max 更新，除法和检查在 BspAdc_Process() 中执行。
 *          校准完成后在 READY 状态每周期计算有符号电流码并检查采样窗口有效性。
 *******************************************************************************
 */

#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "adc.h"
#include "user_motor.h"

#define BSP_ADC2_POLL_TIMEOUT_MS    2u
#define BSP_ADC_REF_VOLTAGE         3.3f
#define BSP_ADC_CONVERSION_STEPS    4096.0f
#define BSP_ADC_FULL_SCALE          4095.0f
#define BSP_ADC_CURRENT_SHUNT_OHM   0.010f
#define BSP_ADC_CURRENT_GAIN        30.0f
#define BSP_ADC_PHASE_CURRENT_SIGN  (-1.0f)

/* ---- ADC2 通道映射 ---- */
static const uint32_t s_adc2_channel_map[BSP_ADC2_REGULAR_CHANNELS] = {
    ADC_CHANNEL_6,     /* PC0: SHA */
    ADC_CHANNEL_7,     /* PC1: SHB */
    ADC_CHANNEL_8,     /* PC2: SHC */
    ADC_CHANNEL_5,     /* PC4: POT */
    ADC_CHANNEL_11,    /* PC5: VBUS */
};

/* ===================================================================== *
 *  运行时变量
 * ===================================================================== */

/** @brief 注入通道采样原始值缓冲（ISR 中更新） */
static volatile uint16_t s_injected_raw[BSP_ADC_INJECTED_CHANNELS] = {0};

/** @brief ADC2 常规通道原始值 */
static volatile uint16_t s_adc2_regular_raw[BSP_ADC2_REGULAR_CHANNELS] = {0};

/** @brief 零电流偏置（ADC counts），校准前默认 2048 */
static volatile uint16_t s_current_offset[BSP_ADC_INJECTED_CHANNELS] = {2048u, 2048u, 2048u};

/** @brief 有符号电流码 (raw - offset)，ISR 中在 READY 状态更新 */
static volatile int32_t s_current_code[BSP_ADC_INJECTED_CHANNELS] = {0};

/** @brief 本周期采样窗口有效标志 */
static volatile uint8_t s_sample_valid = 0u;

/* ---- 校准累计数据（ISR 写，Process 读）---- */
static volatile uint64_t s_cal_sum_first_half[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile uint64_t s_cal_sum_second_half[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile uint16_t s_cal_min[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile uint16_t s_cal_max[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile uint16_t s_cal_sample_count = 0u;

/* ---- 校准调试信息 ---- */
static volatile uint16_t s_cal_span[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile int16_t s_cal_drift[BSP_ADC_INJECTED_CHANNELS] = {0};

/* ---- 状态机 ---- */
static volatile BspAdc_CalState_t s_cal_state = CS_CAL_IDLE;
static volatile uint8_t s_cal_retry_count = 0u;
static volatile uint32_t s_cal_settle_start_tick = 0u;

/* ---- ADC2 校准标志 ---- */
static uint8_t s_adc2_calibrated = 0u;

/* ===================================================================== *
 *  内部函数
 * ===================================================================== */

/**
 * @brief  复位校准累计数据
 * @note   清零各通道累计值、min/max，为新一轮校准做准备。
 */
static void BspAdc_ResetCalibrationData(void)
{
    uint8_t i;

    for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
    {
        s_cal_sum_first_half[i] = 0u;
        s_cal_sum_second_half[i] = 0u;
        s_cal_min[i] = 0xFFFFu;
        s_cal_max[i] = 0u;
        s_current_offset[i] = 2048u;
    }

    s_cal_sample_count = 0u;
}

/**
 * @brief  ADC 原始码转电流值
 * @param[in] code  有符号电流码 (raw - offset)
 * @return 实际电流值（A）
 */
static float BspAdc_CodeToCurrent(int32_t code)
{
    static const float adc_scale = BSP_ADC_REF_VOLTAGE / BSP_ADC_CONVERSION_STEPS /
                                   BSP_ADC_CURRENT_SHUNT_OHM / BSP_ADC_CURRENT_GAIN;
    return (float)code * adc_scale;
}

/**
 * @brief  读取 ADC2 单个通道值（阻塞轮询模式）
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
 * @brief  检查本周期采样窗口是否有效
 * @retval 1  有效：min(CCR1,CCR2,CCR3) >= CCR_MIN
 * @retval 0  无效
 * @note   检查条件 (doc §5, 用户要求第8点)：
 *         CCR4 + ADC_SAMPLE_TICKS + BLANK_TICKS <= min(CCR1, CCR2, CCR3)
 */
static uint8_t BspAdc_CheckSampleValid(void)
{
    uint16_t min_ccr = BspPwm_GetMinCompare();
    return (min_ccr >= CS_CCR_MIN) ? 1u : 0u;
}

/* ===================================================================== *
 *  公开接口
 * ===================================================================== */

/**
 * @brief  启动 ADC1 注入组采样（含自校准 + 中断模式）
 * @retval HAL_OK       启动成功
 * @retval HAL_ERROR    校准失败
 * @retval HAL_BUSY     外设忙
 * @note   初始化状态机为 IDLE，偏置设为默认 2048。
 *         之后需调用 BspAdc_CalibrationStart() 开始偏置校准。
 */
HAL_StatusTypeDef BspAdc_StartInjected(void)
{
    HAL_StatusTypeDef status;
    uint8_t i;

    s_cal_state = CS_CAL_IDLE;
    s_cal_retry_count = 0u;
    s_sample_valid = 0u;

    for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
    {
        s_current_offset[i] = 2048u;
        s_current_code[i] = 0;
        s_injected_raw[i] = 0u;
    }

    status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_ADCEx_InjectedStart_IT(&hadc1);
    return status;
}

/**
 * @brief  启动偏置校准状态机
 * @note   先将状态设为 IDLE（阻止 ISR 累计），再复位数据，最后进入 SETTLE。
 *         调用前必须确保：驱动器 EN 关闭、功率 PWM 未启动、TIM1_CH4 触发已运行。
 */
void BspAdc_CalibrationStart(void)
{
    s_cal_state = CS_CAL_IDLE;  /* 先停止 ISR 累计 */
    BspAdc_ResetCalibrationData();
    s_cal_settle_start_tick = HAL_GetTick();
    s_cal_state = CS_CAL_SETTLE;
}

/**
 * @brief  偏置校准状态机处理（由主循环调用）
 * @note   SETTLE：计时到达后切换到 DISCARD。
 *         CALCULATE：计算偏置平均值，检查范围/跨度/漂移，通过则 READY，
 *         否则重试（最多 CS_CAL_MAX_RETRY 次），超限则 ERROR。
 *         中断中不做除法或浮点运算 (doc §13)。
 */
void BspAdc_Process(void)
{
    switch (s_cal_state)
    {
        case CS_CAL_SETTLE:
            if ((HAL_GetTick() - s_cal_settle_start_tick) >= CS_CAL_SETTLE_TIME_MS)
            {
                s_cal_sample_count = 0u;
                s_cal_state = CS_CAL_DISCARD;
            }
            break;

        case CS_CAL_CALCULATE:
        {
            uint8_t i;
            uint8_t check_failed = 0u;

            for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
            {
                uint32_t total = (uint32_t)s_cal_sum_first_half[i] +
                                 (uint32_t)s_cal_sum_second_half[i];
                uint16_t offset = (uint16_t)((total + (CS_CAL_SAMPLE_COUNT / 2u)) /
                                              CS_CAL_SAMPLE_COUNT);
                uint16_t span = (uint16_t)(s_cal_max[i] - s_cal_min[i]);
                uint16_t first_avg = (uint16_t)(s_cal_sum_first_half[i] /
                                                 (CS_CAL_SAMPLE_COUNT / 2u));
                uint16_t second_avg = (uint16_t)(s_cal_sum_second_half[i] /
                                                  (CS_CAL_SAMPLE_COUNT / 2u));
                int16_t drift = (int16_t)first_avg - (int16_t)second_avg;
                if (drift < 0)
                {
                    drift = (int16_t)(-drift);
                }

                /* 记录调试信息 */
                s_current_offset[i] = offset;
                s_cal_span[i] = span;
                s_cal_drift[i] = drift;

                /* 范围检查 (doc §15.1) */
                if ((offset < CS_CAL_OFFSET_MIN) || (offset > CS_CAL_OFFSET_MAX))
                {
                    check_failed = 1u;
                }

                /* 噪声跨度检查 (doc §15.2) */
                if (span > CS_CAL_MAX_SPAN)
                {
                    check_failed = 1u;
                }

                /* 均值漂移检查 (doc §15.3) */
                if ((uint16_t)drift > CS_CAL_DRIFT_LIMIT)
                {
                    check_failed = 1u;
                }
            }

            if (check_failed == 0u)
            {
                s_cal_state = CS_CAL_READY;
            }
            else
            {
                s_cal_retry_count++;
                if (s_cal_retry_count >= CS_CAL_MAX_RETRY)
                {
                    s_cal_state = CS_CAL_ERROR;
                }
                else
                {
                    /* 重试：重新等待稳定 */
                    BspAdc_ResetCalibrationData();
                    s_cal_settle_start_tick = HAL_GetTick();
                    s_cal_state = CS_CAL_SETTLE;
                }
            }
            break;
        }

        default:
            /* IDLE / DISCARD / ACCUMULATE / READY / ERROR：主循环无需处理 */
            break;
    }
}

/**
 * @brief  更新注入组采样缓冲（在 HAL_ADCEx_InjectedConvCpltCallback 中调用）
 * @param[in] hadc  ADC 句柄指针
 * @retval 1  可执行 FOC（仅 READY 状态）
 * @retval 0  不执行 FOC
 * @note   ISR 中仅做：读取原始值、累计、更新 min/max、计算有符号码、检查采样窗口。
 *         禁止在中断中执行除法、浮点运算、复杂判断、日志打印 (doc §13)。
 */
uint8_t BspAdc_UpdateInjected(ADC_HandleTypeDef *hadc)
{
    uint8_t i;

    if ((hadc == NULL) || (hadc->Instance != ADC1))
    {
        return 0u;
    }

    s_injected_raw[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    s_injected_raw[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    s_injected_raw[2] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    switch (s_cal_state)
    {
        case CS_CAL_DISCARD:
            s_cal_sample_count++;
            if (s_cal_sample_count >= CS_CAL_DISCARD_COUNT)
            {
                /* 进入累计阶段前复位累计数据 */
                for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
                {
                    s_cal_sum_first_half[i] = 0u;
                    s_cal_sum_second_half[i] = 0u;
                    s_cal_min[i] = 0xFFFFu;
                    s_cal_max[i] = 0u;
                }
                s_cal_sample_count = 0u;
                s_cal_state = CS_CAL_ACCUMULATE;
            }
            break;

        case CS_CAL_ACCUMULATE:
        {
            uint8_t is_first_half = (s_cal_sample_count < (CS_CAL_SAMPLE_COUNT / 2u)) ? 1u : 0u;

            for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
            {
                uint16_t raw = s_injected_raw[i];

                if (is_first_half != 0u)
                {
                    s_cal_sum_first_half[i] += raw;
                }
                else
                {
                    s_cal_sum_second_half[i] += raw;
                }

                if (raw < s_cal_min[i])
                {
                    s_cal_min[i] = raw;
                }
                if (raw > s_cal_max[i])
                {
                    s_cal_max[i] = raw;
                }
            }

            s_cal_sample_count++;
            if (s_cal_sample_count >= CS_CAL_SAMPLE_COUNT)
            {
                s_cal_state = CS_CAL_CALCULATE;
            }
            break;
        }

        case CS_CAL_READY:
            /* 正常 FOC 阶段：计算有符号电流码 (doc §16.2) */
            for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
            {
                s_current_code[i] = (int32_t)s_injected_raw[i] -
                                     (int32_t)s_current_offset[i];
            }
            /* 检查采样窗口有效性 (doc §5, §6) */
            s_sample_valid = BspAdc_CheckSampleValid();
            return 1u;

        default:
            /* IDLE / SETTLE / CALCULATE / ERROR：不调用 FOC */
            break;
    }

    return 0u;
}

/**
 * @brief  ADC1 注入转换完成中断回调（HAL 库重写）
 * @param[in] hadc  ADC 句柄指针
 * @note   由 TIM1_CH4 → TRGO2 触发 ADC1 注入转换，转换完成后硬件触发此回调。
 *         在回调中更新采样缓冲，READY 状态下执行电机快速控制环。
 *         控制频率由 TIM1 配置决定，标称 20 kHz。
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (BspAdc_UpdateInjected(hadc) != 0u)
    {
        UserMotor_FastLoop();
    }
}

/* ---- 原始值 / 偏置 / 电流码查询 ---- */

uint16_t BspAdc_GetInjectedRaw(uint8_t index)
{
    if (index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0u;
    }
    return s_injected_raw[index];
}

uint16_t BspAdc_GetCurrentOffsetRaw(uint8_t index)
{
    if (index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0u;
    }
    return s_current_offset[index];
}

float BspAdc_GetCurrentOffsetVoltage(uint8_t index)
{
    if (index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0.0f;
    }
    return ((float)s_current_offset[index] * BSP_ADC_REF_VOLTAGE) / BSP_ADC_FULL_SCALE;
}

int32_t BspAdc_GetCurrentCode(uint8_t index)
{
    if (index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0;
    }
    return s_current_code[index];
}

uint8_t BspAdc_IsCurrentOffsetReady(void)
{
    return (s_cal_state == CS_CAL_READY) ? 1u : 0u;
}

BspAdc_CalState_t BspAdc_GetCalState(void)
{
    return s_cal_state;
}

uint8_t BspAdc_IsSampleValid(void)
{
    return s_sample_valid;
}

/* ---- 三相电流 ---- */

float BspAdc_GetIa(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdc_CodeToCurrent(s_current_code[0]);
}

float BspAdc_GetIb(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdc_CodeToCurrent(s_current_code[1]);
}

float BspAdc_GetIc(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdc_CodeToCurrent(s_current_code[2]);
}

/* ---- 校准调试信息 ---- */

void BspAdc_GetCalDebug(BspAdc_CalDebug_t *info)
{
    uint8_t i;

    if (info == NULL)
    {
        return;
    }

    for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
    {
        info->min_raw[i] = s_cal_min[i];
        info->max_raw[i] = s_cal_max[i];
        info->span[i] = s_cal_span[i];
        info->offset[i] = s_current_offset[i];
        info->drift[i] = s_cal_drift[i];
    }

    info->retry_count = s_cal_retry_count;
    info->state = s_cal_state;
}

/* ===================================================================== *
 *  ADC2 接口（保持不变）
 * ===================================================================== */

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

uint16_t BspAdc2_GetRaw(BspAdc2Channel_t channel)
{
    if ((uint8_t)channel >= BSP_ADC2_REGULAR_CHANNELS)
    {
        return 0u;
    }
    return s_adc2_regular_raw[(uint8_t)channel];
}

float BspAdc2_GetVoltage(BspAdc2Channel_t channel)
{
    return ((float)BspAdc2_GetRaw(channel) * BSP_ADC_REF_VOLTAGE) / BSP_ADC_FULL_SCALE;
}
