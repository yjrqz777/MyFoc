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
 *          中断中仅做累计和 min/max 更新，除法和检查在 BspAdcProcess() 中执行。
 *          校准完成后在 READY 状态每周期计算有符号电流码并检查采样窗口有效性。
 *******************************************************************************
 */

#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "adc.h"
#include "user_motor.h"

#define BSP_ADC2_POLL_TIMEOUT_MS    (2u)
#define BSP_ADC_REF_VOLTAGE         (3.3f)
#define BSP_ADC_CONVERSION_STEPS    (4096.0f)
#define BSP_ADC_FULL_SCALE          (4095.0f)
#define BSP_ADC_CURRENT_SHUNT_OHM   (0.010f)
#define BSP_ADC_CURRENT_GAIN        (30.0f)
#define BSP_ADC_PHASE_CURRENT_SIGN  (-1.0f)

/* ---- ADC2 通道映射 ---- */
static const uint32_t u32Adc2ChannelMap[BSP_ADC2_REGULAR_CHANNELS] = {
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
static volatile uint16_t u16InjectedRaw[BSP_ADC_INJECTED_CHANNELS] = {0};

/** @brief ADC2 常规通道原始值 */
static volatile uint16_t u16Adc2RegularRaw[BSP_ADC2_REGULAR_CHANNELS] = {0};

/** @brief 零电流偏置（ADC counts），校准前默认 2048 */
static volatile uint16_t u16CurrentOffset[BSP_ADC_INJECTED_CHANNELS] = {2048u, 2048u, 2048u};

/** @brief 有符号电流码 (raw - offset)，ISR 中在 READY 状态更新 */
static volatile int32_t s32CurrentCode[BSP_ADC_INJECTED_CHANNELS] = {0};

/** @brief 本周期采样窗口有效标志 */
static volatile uint8_t u8SampleValid = 0u;

/* ---- 校准累计数据（ISR 写，Process 读）---- */
static volatile uint64_t u64CalSumFirstHalf[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile uint64_t u64CalSumSecondHalf[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile uint16_t u16CalMin[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile uint16_t u16CalMax[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile uint16_t u16CalSampleCount = 0u;

/* ---- 校准调试信息 ---- */
static volatile uint16_t u16CalSpan[BSP_ADC_INJECTED_CHANNELS] = {0};
static volatile int16_t s16CalDrift[BSP_ADC_INJECTED_CHANNELS] = {0};

/* ---- 状态机 ---- */
static volatile eBspAdcCalStateDef eCalState = E_BSP_ADC_CAL_IDLE;
static volatile uint8_t u8CalRetryCount = 0u;
static volatile uint32_t u32CalSettleStartTick = 0u;

/* ---- ADC2 校准标志 ---- */
static uint8_t u8Adc2Calibrated = 0u;

/* ===================================================================== *
 *  内部函数
 * ===================================================================== */

/**
 * @brief  复位校准累计数据
 * @note   清零各通道累计值、min/max，为新一轮校准做准备。
 */
static void BspAdcResetCalibrationData(void)
{
    uint8_t i;

    for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
    {
        u64CalSumFirstHalf[i] = 0u;
        u64CalSumSecondHalf[i] = 0u;
        u16CalMin[i] = 0xFFFFu;
        u16CalMax[i] = 0u;
        u16CurrentOffset[i] = 2048u;
    }

    u16CalSampleCount = 0u;
}

/**
 * @brief  ADC 原始码转电流值
 * @param[in] code  有符号电流码 (raw - offset)
 * @return 实际电流值（A）
 */
static float BspAdcCodeToCurrent(int32_t s32Code)
{
    static const float f32AdcScale = BSP_ADC_REF_VOLTAGE / BSP_ADC_CONVERSION_STEPS /
                                   BSP_ADC_CURRENT_SHUNT_OHM / BSP_ADC_CURRENT_GAIN;
    return (float)s32Code * f32AdcScale;
}

/**
 * @brief  读取 ADC2 单个通道值（阻塞轮询模式）
 */
static HAL_StatusTypeDef BspAdc2ReadChannel(uint32_t u32AdcChannel, uint16_t *pu16Raw)
{
    ADC_ChannelConfTypeDef Config = {0};
    HAL_StatusTypeDef Status;

    Config.Channel = u32AdcChannel;
    Config.Rank = ADC_REGULAR_RANK_1;
    Config.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    Config.SingleDiff = ADC_SINGLE_ENDED;
    Config.OffsetNumber = ADC_OFFSET_NONE;
    Config.Offset = 0;

    Status = HAL_ADC_ConfigChannel(&hadc2, &Config);
    if (Status != HAL_OK)
    {
        return Status;
    }

    Status = HAL_ADC_Start(&hadc2);
    if (Status != HAL_OK)
    {
        return Status;
    }

    Status = HAL_ADC_PollForConversion(&hadc2, BSP_ADC2_POLL_TIMEOUT_MS);
    if (Status == HAL_OK)
    {
        *pu16Raw = (uint16_t)HAL_ADC_GetValue(&hadc2);
    }

    (void)HAL_ADC_Stop(&hadc2);
    return Status;
}

/**
 * @brief  检查本周期采样窗口是否有效
 * @retval 1  有效：min(CCR1,CCR2,CCR3) >= CCR_MIN
 * @retval 0  无效
 * @note   检查条件 (doc §5, 用户要求第8点)：
 *         CCR4 + ADC_SAMPLE_TICKS + BLANK_TICKS <= min(CCR1, CCR2, CCR3)
 */
static uint8_t BspAdcCheckSampleValid(void)
{
    uint16_t MinCcr = BspPwmGetMinCompare();
    return (MinCcr >= CS_CCR_MIN) ? 1u : 0u;
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
 *         之后需调用 BspAdcCalibrationStart() 开始偏置校准。
 */
HAL_StatusTypeDef BspAdcStartInjected(void)
{
    HAL_StatusTypeDef Status;
    uint8_t i;

    eCalState = E_BSP_ADC_CAL_IDLE;
    u8CalRetryCount = 0u;
    u8SampleValid = 0u;

    for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
    {
        u16CurrentOffset[i] = 2048u;
        s32CurrentCode[i] = 0;
        u16InjectedRaw[i] = 0u;
    }

    Status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    if (Status != HAL_OK)
    {
        return Status;
    }

    Status = HAL_ADCEx_InjectedStart_IT(&hadc1);
    return Status;
}

/**
 * @brief  启动偏置校准状态机
 * @note   先将状态设为 IDLE（阻止 ISR 累计），再复位数据，最后进入 SETTLE。
 *         调用前必须确保：TIM1_CH4 触发已运行、三相命令为零电压且电机无实际相电流；允许功率 PWM 以三相相同占空比运行。
 */
void BspAdcCalibrationStart(void)
{
    eCalState = E_BSP_ADC_CAL_IDLE;  /* 先停止 ISR 累计 */
    BspAdcResetCalibrationData();
    u32CalSettleStartTick = HAL_GetTick();
    eCalState = E_BSP_ADC_CAL_SETTLE;
}

/**
 * @brief  偏置校准状态机处理（由主循环调用）
 * @note   SETTLE：计时到达后切换到 DISCARD。
 *         CALCULATE：计算偏置平均值，检查范围/跨度/漂移，通过则 READY，
 *         否则重试（最多 CS_CAL_MAX_RETRY 次），超限则 ERROR。
 *         中断中不做除法或浮点运算 (doc §13)。
 */
void BspAdcProcess(void)
{
    switch (eCalState)
    {
        case E_BSP_ADC_CAL_SETTLE:
            if ((HAL_GetTick() - u32CalSettleStartTick) >= CS_CAL_SETTLE_TIME_MS)
            {
                u16CalSampleCount = 0u;
                eCalState = E_BSP_ADC_CAL_DISCARD;
            }
            break;

        case E_BSP_ADC_CAL_CALCULATE:
        {
            uint8_t i;
            uint8_t CheckFailed = 0u;

            for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
            {
                uint32_t Total = (uint32_t)u64CalSumFirstHalf[i] +
                                 (uint32_t)u64CalSumSecondHalf[i];
                uint16_t Offset = (uint16_t)((Total + (CS_CAL_SAMPLE_COUNT / 2u)) /
                                              CS_CAL_SAMPLE_COUNT);
                uint16_t Span = (uint16_t)(u16CalMax[i] - u16CalMin[i]);
                uint16_t FirstAverage = (uint16_t)(u64CalSumFirstHalf[i] /
                                                 (CS_CAL_SAMPLE_COUNT / 2u));
                uint16_t SecondAverage = (uint16_t)(u64CalSumSecondHalf[i] /
                                                  (CS_CAL_SAMPLE_COUNT / 2u));
                int16_t Drift = (int16_t)FirstAverage - (int16_t)SecondAverage;
                if (Drift < 0)
                {
                    Drift = (int16_t)(-Drift);
                }

                /* 记录调试信息 */
                u16CurrentOffset[i] = Offset;
                u16CalSpan[i] = Span;
                s16CalDrift[i] = Drift;

                /* 范围检查 (doc §15.1) */
                if ((Offset < CS_CAL_OFFSET_MIN) || (Offset > CS_CAL_OFFSET_MAX))
                {
                    SEGGER_RTT_WriteString(0, "Current offset out of range\r\n");
                    CheckFailed = 1u;
                }

                /* 噪声跨度检查 (doc §15.2) */
                if (Span > CS_CAL_MAX_SPAN)
                {
                    SEGGER_RTT_WriteString(0, "Current noise span too large\r\n");
                    CheckFailed = 1u;
                }

                /* 均值漂移检查 (doc §15.3) */
                if ((uint16_t)Drift > CS_CAL_DRIFT_LIMIT)
                {
                    SEGGER_RTT_WriteString(0, "Current average drift too large\r\n");
                    CheckFailed = 1u;
                }
            }

            if (CheckFailed == 0u)
            {
                eCalState = E_BSP_ADC_CAL_READY;
            }
            else
            {
                u8CalRetryCount++;
                if (u8CalRetryCount >= CS_CAL_MAX_RETRY)
                {
                    eCalState = E_BSP_ADC_CAL_ERROR;
                }
                else
                {
                    /* 重试：重新等待稳定 */
                    BspAdcResetCalibrationData();
                    u32CalSettleStartTick = HAL_GetTick();
                    eCalState = E_BSP_ADC_CAL_SETTLE;
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
 * @param[in] ptAdc  ADC 句柄指针
 * @retval 1  可执行 FOC（仅 READY 状态）
 * @retval 0  不执行 FOC
 * @note   ISR 中仅做：读取原始值、累计、更新 min/max、计算有符号码、检查采样窗口。
 *         禁止在中断中执行除法、浮点运算、复杂判断、日志打印 (doc §13)。
 */
uint8_t BspAdcUpdateInjected(ADC_HandleTypeDef *ptAdc)
{
    uint8_t i;

    if ((ptAdc == NULL) || (ptAdc->Instance != ADC1))
    {
        return 0u;
    }

    u16InjectedRaw[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(ptAdc, ADC_INJECTED_RANK_1);
    u16InjectedRaw[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(ptAdc, ADC_INJECTED_RANK_2);
    u16InjectedRaw[2] = (uint16_t)HAL_ADCEx_InjectedGetValue(ptAdc, ADC_INJECTED_RANK_3);

    switch (eCalState)
    {
        case E_BSP_ADC_CAL_DISCARD:
            u16CalSampleCount++;
            if (u16CalSampleCount >= CS_CAL_DISCARD_COUNT)
            {
                /* 进入累计阶段前复位累计数据 */
                for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
                {
                    u64CalSumFirstHalf[i] = 0u;
                    u64CalSumSecondHalf[i] = 0u;
                    u16CalMin[i] = 0xFFFFu;
                    u16CalMax[i] = 0u;
                }
                u16CalSampleCount = 0u;
                eCalState = E_BSP_ADC_CAL_ACCUMULATE;
            }
            break;

        case E_BSP_ADC_CAL_ACCUMULATE:
        {
            uint8_t IsFirstHalf = (u16CalSampleCount < (CS_CAL_SAMPLE_COUNT / 2u)) ? 1u : 0u;

            for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
            {
                uint16_t Raw = u16InjectedRaw[i];

                if (IsFirstHalf != 0u)
                {
                    u64CalSumFirstHalf[i] += Raw;
                }
                else
                {
                    u64CalSumSecondHalf[i] += Raw;
                }

                if (Raw < u16CalMin[i])
                {
                    u16CalMin[i] = Raw;
                }
                if (Raw > u16CalMax[i])
                {
                    u16CalMax[i] = Raw;
                }
            }

            u16CalSampleCount++;
            if (u16CalSampleCount >= CS_CAL_SAMPLE_COUNT)
            {
                eCalState = E_BSP_ADC_CAL_CALCULATE;
            }
            break;
        }

        case E_BSP_ADC_CAL_READY:
            /* 正常 FOC 阶段：计算有符号电流码 (doc §16.2) */
            for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
            {
                s32CurrentCode[i] = (int32_t)u16InjectedRaw[i] -
                                     (int32_t)u16CurrentOffset[i];
            }
            /* 检查采样窗口有效性 (doc §5, §6) */
            u8SampleValid = BspAdcCheckSampleValid();
            return 1u;

        default:
            /* IDLE / SETTLE / CALCULATE / ERROR：不调用 FOC */
            break;
    }

    return 0u;
}

/**
 * @brief  ADC1 注入转换完成中断回调（HAL 库重写）
 * @param[in] ptAdc  ADC 句柄指针
 * @note   由 TIM1_CH4 → TRGO2 触发 ADC1 注入转换，转换完成后硬件触发此回调。
 *         在回调中更新采样缓冲，READY 状态下执行电机快速控制环。
 *         控制频率由 TIM1 配置决定，标称 20 kHz。
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *ptAdc)
{
    if (BspAdcUpdateInjected(ptAdc) != 0u)
    {
        UsrMotorFastLoop();
    }
}

/* ---- 原始值 / 偏置 / 电流码查询 ---- */

uint16_t BspAdcGetInjectedRaw(uint8_t u8Index)
{
    if (u8Index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0u;
    }
    return u16InjectedRaw[u8Index];
}

uint16_t BspAdcGetCurrentOffsetRaw(uint8_t u8Index)
{
    if (u8Index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0u;
    }
    return u16CurrentOffset[u8Index];
}

float BspAdcGetCurrentOffsetVoltage(uint8_t u8Index)
{
    if (u8Index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0.0f;
    }
    return ((float)u16CurrentOffset[u8Index] * BSP_ADC_REF_VOLTAGE) / BSP_ADC_FULL_SCALE;
}

int32_t BspAdcGetCurrentCode(uint8_t u8Index)
{
    if (u8Index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0;
    }
    return s32CurrentCode[u8Index];
}

uint8_t BspAdcIsCurrentOffsetReady(void)
{
    return (eCalState == E_BSP_ADC_CAL_READY) ? 1u : 0u;
}

eBspAdcCalStateDef BspAdcGetCalState(void)
{
    return eCalState;
}

uint8_t BspAdcIsSampleValid(void)
{
    return u8SampleValid;
}

/* ---- 三相电流 ---- */

float BspAdcGetIa(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdcCodeToCurrent(s32CurrentCode[0]);
}

float BspAdcGetIb(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdcCodeToCurrent(s32CurrentCode[1]);
}

float BspAdcGetIc(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdcCodeToCurrent(s32CurrentCode[2]);
}

/* ---- 校准调试信息 ---- */

void BspAdcGetCalDebug(tBspAdcCalDebugDef *ptInfo)
{
    uint8_t i;

    if (ptInfo == NULL)
    {
        return;
    }

    for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
    {
        ptInfo->u16MinRaw[i] = u16CalMin[i];
        ptInfo->u16MaxRaw[i] = u16CalMax[i];
        ptInfo->u16Span[i] = u16CalSpan[i];
        ptInfo->u16Offset[i] = u16CurrentOffset[i];
        ptInfo->s16Drift[i] = s16CalDrift[i];
    }

    ptInfo->u8RetryCount = u8CalRetryCount;
    ptInfo->eState = eCalState;
}

/* ===================================================================== *
 *  ADC2 接口（保持不变）
 * ===================================================================== */

HAL_StatusTypeDef BspAdc2UpdateAll(void)
{
    HAL_StatusTypeDef Status;
    uint8_t Index;
    uint16_t Raw;

    if (u8Adc2Calibrated == 0u)
    {
        Status = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
        if (Status != HAL_OK)
        {
            return Status;
        }
        u8Adc2Calibrated = 1u;
    }

    for (Index = 0u; Index < BSP_ADC2_REGULAR_CHANNELS; Index++)
    {
        Status = BspAdc2ReadChannel(u32Adc2ChannelMap[Index], &Raw);
        if (Status != HAL_OK)
        {
            return Status;
        }
        u16Adc2RegularRaw[Index] = Raw;
    }

    return HAL_OK;
}

uint16_t BspAdc2GetRaw(eBspAdc2ChannelDef eChannel)
{
    if ((uint8_t)eChannel >= BSP_ADC2_REGULAR_CHANNELS)
    {
        return 0u;
    }
    return u16Adc2RegularRaw[(uint8_t)eChannel];
}

float BspAdc2GetVoltage(eBspAdc2ChannelDef eChannel)
{
    return ((float)BspAdc2GetRaw(eChannel) * BSP_ADC_REF_VOLTAGE) / BSP_ADC_FULL_SCALE;
}
