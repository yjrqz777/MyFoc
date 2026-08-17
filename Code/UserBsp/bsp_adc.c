/***************************************************************************************************
* @版    权：深圳拓邦股份有限公司-微电研发中心-家电部
* @文 件 名：bsp_adc.c
* @内容摘要：ADC底层驱动实现 — 注入模式三相电流采样与偏置校准状态机
* @详细说明：本文件实现ADC1注入组三相电流采样及零电流偏置校准功能：
*           1. 电流采样电路：3.3V参考、12-bit ADC、采样电阻10mΩ、运放增益30，
*              转换公式：I = (raw - offset) * (3.3 / 4096 / 0.010 / 30)；
*           2. 偏置校准采用非阻塞状态机：SETTLE → DISCARD → ACCUMULATE → CALCULATE → READY，
*              中断中仅做累计和min/max更新，除法和检查在BspAdcProcess()中执行；
*           3. 校准完成后在READY状态每周期计算有符号电流码并检查采样窗口有效性；
*           4. 提供启动前普通轮询预采样零偏功能(BspAdcPreOffset)及ADC2常规通道采样接口。
* @当前版本：V1.0
* @作    者：家电部-软件组
* @完成日期：
* @记    录：
* @修改记录：
* @修改日期：
* @版 本 号：
* @修 改 人：
***************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "adc.h"
#include "user_motor.h"

#define BSP_ADC2_POLL_TIMEOUT_MS    (2u)          /* ADC2常规通道单次转换轮询超时(ms) */
#define BSP_ADC_REF_VOLTAGE         (3.3f)        /* ADC参考电压(V) */
#define BSP_ADC_CONVERSION_STEPS    (4096.0f)     /* ADC量化步进数(12bit) */
#define BSP_ADC_FULL_SCALE          (4095.0f)     /* ADC满量程值 */
#define BSP_ADC_CURRENT_SHUNT_OHM   (0.010f)      /* 相电流采样电阻阻值(Ω) */
#define BSP_ADC_CURRENT_GAIN        (30.0f)       /* 相电流运放放大倍数 */
#define BSP_ADC_PHASE_CURRENT_SIGN  (-1.0f)       /* ADC 正向为电机流出，转换为 FOC 流入电机为正 */

/* ---- ADC1 相电流通道映射 ---- */
static const uint32_t u32Adc1PhaseChannelMap[BSP_ADC_INJECTED_CHANNELS] = {
    ADC_CHANNEL_1,     /* PA0: Ia */
    ADC_CHANNEL_2,     /* PA1: Ib */
    ADC_CHANNEL_3,     /* PA2: Ic */
};

/* ---- ADC2 通道映射 ---- */
static const uint32_t u32Adc2ChannelMap[BSP_ADC2_REGULAR_CHANNELS] = {
    ADC_CHANNEL_6,     /* PC0: SHA */
    ADC_CHANNEL_7,     /* PC1: SHB */
    ADC_CHANNEL_8,     /* PC2: SHC */
    ADC_CHANNEL_5,     /* PC4: POT */
    ADC_CHANNEL_11,    /* PC5: VBUS */
};

#if (BSP_ADC_INJECTED_CHANNELS != 3u)
#error "BspAdcPreOffset currently supports exactly three ADC1 phase-current channels"
#endif

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


/** @brief 预采样偏置有效性标志：1=有效，0=超出范围 */
static volatile uint8_t u8PreOffsetValid = 0u;

/* ---- ADC2 校准标志 ---- */
static uint8_t u8Adc2Calibrated = 0u;

/* ===================================================================== *
 *  内部函数
 * ===================================================================== */

/***************************************************************************************************
* @功    能: ADC原始码转电流值
* @详细描述: 将有符号电流码(raw - offset)乘以固定的ADC比例系数换算为实际电流(A)。
*           比例系数 = 参考电压 / 转换步进数 / 采样电阻 / 运放增益，
*           使用static局部常量避免每次调用重复计算。
* @输入参数: s32Code - 有符号电流码(raw - offset)
* @输出参数: 无
* @返 回 值: 实际电流值(A)
***************************************************************************************************/
static float BspAdcCodeToCurrent(int32_t s32Code)
{
    static const float f32AdcScale = BSP_ADC_REF_VOLTAGE / BSP_ADC_CONVERSION_STEPS /
                                   BSP_ADC_CURRENT_SHUNT_OHM / BSP_ADC_CURRENT_GAIN;
    return (float)s32Code * f32AdcScale;   /* 码值乘以比例系数得到电流 */
}

/***************************************************************************************************
* @功    能: 读取ADC1单个相电流通道值(阻塞轮询模式)
* @详细描述: 将指定通道配置为常规转换Rank1(采样时间12.5周期)，启动单次转换并轮询等待结果，
*           转换完成后停止ADC并返回状态。供启动前预采样零电流偏置使用。
* @输入参数: u32AdcChannel - ADC1通道号；pu16Raw - 转换结果存储指针
* @输出参数: pu16Raw - 指向转换得到的原始ADC值(0~4095)
* @返 回 值: HAL_OK - 转换成功；其他 - HAL库错误状态
***************************************************************************************************/
static HAL_StatusTypeDef BspAdc1ReadPhaseChannel(uint32_t u32AdcChannel, uint16_t * pu16Raw)
{
    ADC_ChannelConfTypeDef Config = {0};
    HAL_StatusTypeDef Status;

    Config.Channel = u32AdcChannel;                  /* 待转换通道 */
    Config.Rank = ADC_REGULAR_RANK_1;                /* 常规序列第1位 */
    Config.SamplingTime = ADC_SAMPLETIME_12CYCLES_5; /* 采样时间12.5个ADC时钟周期 */
    Config.SingleDiff = ADC_SINGLE_ENDED;            /* 单端输入 */
    Config.OffsetNumber = ADC_OFFSET_NONE;           /* 不使用硬件偏移 */
    Config.Offset = 0;

    Status = HAL_ADC_ConfigChannel(&hadc1, &Config);
    if (Status != HAL_OK)
    {
        return Status;   /* 通道配置失败直接返回 */
    }

    Status = HAL_ADC_Start(&hadc1);
    if (Status != HAL_OK)
    {
        return Status;   /* 启动转换失败直接返回 */
    }

    Status = HAL_ADC_PollForConversion(&hadc1, BSP_ADC_PRE_OFFSET_POLL_TIMEOUT_MS);
    if (Status == HAL_OK)
    {
        *pu16Raw = (uint16_t)HAL_ADC_GetValue(&hadc1);   /* 读取转换结果 */
    }

    (void)HAL_ADC_Stop(&hadc1);   /* 停止ADC，退出轮询模式 */
    return Status;
}

/***************************************************************************************************
* @功    能: 读取ADC2单个通道值(阻塞轮询模式)
* @详细描述: 将指定通道配置为常规转换Rank1(采样时间47.5周期)，启动单次转换并轮询等待结果，
*           转换完成后停止ADC并返回状态。供ADC2常规通道(SHA/SHB/SHC/POT/VBUS)采样使用。
* @输入参数: u32AdcChannel - ADC2通道号；pu16Raw - 转换结果存储指针
* @输出参数: pu16Raw - 指向转换得到的原始ADC值(0~4095)
* @返 回 值: HAL_OK - 转换成功；其他 - HAL库错误状态
***************************************************************************************************/
static HAL_StatusTypeDef BspAdc2ReadChannel(uint32_t u32AdcChannel, uint16_t *pu16Raw)
{
    ADC_ChannelConfTypeDef Config = {0};
    HAL_StatusTypeDef Status;

    Config.Channel = u32AdcChannel;                  /* 待转换通道 */
    Config.Rank = ADC_REGULAR_RANK_1;                /* 常规序列第1位 */
    Config.SamplingTime = ADC_SAMPLETIME_47CYCLES_5; /* 采样时间47.5个ADC时钟周期 */
    Config.SingleDiff = ADC_SINGLE_ENDED;            /* 单端输入 */
    Config.OffsetNumber = ADC_OFFSET_NONE;           /* 不使用硬件偏移 */
    Config.Offset = 0;

    Status = HAL_ADC_ConfigChannel(&hadc2, &Config);
    if (Status != HAL_OK)
    {
        return Status;   /* 通道配置失败直接返回 */
    }

    Status = HAL_ADC_Start(&hadc2);
    if (Status != HAL_OK)
    {
        return Status;   /* 启动转换失败直接返回 */
    }

    Status = HAL_ADC_PollForConversion(&hadc2, BSP_ADC2_POLL_TIMEOUT_MS);
    if (Status == HAL_OK)
    {
        *pu16Raw = (uint16_t)HAL_ADC_GetValue(&hadc2);   /* 读取转换结果 */
    }

    (void)HAL_ADC_Stop(&hadc2);   /* 停止ADC，退出轮询模式 */
    return Status;
}

/***************************************************************************************************
* @功    能: 检查本周期采样窗口是否有效
* @详细描述: 比较PWM三个比较通道的最小值(MinCcr)与采样窗口下限CS_CCR_MIN，
*           判断采样窗口是否足够宽，保证电流采样发生在消隐期之后的稳定区间。
*           检查条件：CCR4 + ADC_SAMPLE_TICKS + BLANK_TICKS <= min(CCR1, CCR2, CCR3)
* @输入参数: 无
* @输出参数: 无
* @返 回 值: 1 - 采样窗口有效；0 - 采样窗口无效
***************************************************************************************************/
static uint8_t BspAdcCheckSampleValid(void)
{
    uint16_t MinCcr = BspPwmGetMinCompare();   /* 获取三相PWM比较值最小值 */
    return (MinCcr >= CS_CCR_MIN) ? 1u : 0u;   /* 最小比较值不低于下限则有效 */
}

/* ===================================================================== *
 *  公开接口
 * ===================================================================== */

/***************************************************************************************************
* @功    能: 启动ADC1注入组采样(含自校准+中断模式)
* @详细描述: 初始化校准状态机为IDLE、复位重试计数与采样有效标志，将三相零电流偏置设为
*           默认值2048，清零原始值与电流码，最后以中断方式启动ADC1注入组转换。
*           之后需调用BspAdcCalibrationStart()开始正式偏置校准。
* @输入参数: 无
* @输出参数: 无
* @返 回 值: HAL_OK - 启动成功；HAL_ERROR - 启动失败；HAL_BUSY - 外设忙
***************************************************************************************************/
HAL_StatusTypeDef BspAdcStartInjected(void)
{
    /* 偏置已由 BspAdcPreOffset 完成，此处仅启动注入中断 */
    u8SampleValid = 0u;               /* 采样有效标志清零 */

    return HAL_ADCEx_InjectedStart_IT(&hadc1);   /* 以中断方式启动注入组转换 */
}

/***************************************************************************************************
* @功    能: 更新注入组采样缓冲(在注入转换完成回调中调用)
* @详细描述: 从ADC句柄读取3个注入通道原始值，计算有符号电流码并检查采样窗口有效性。
*           偏置校准由 BspAdcPreOffset 在启动前完成，ISR 中不再做校准状态机处理。
* @输入参数: ptAdc - ADC句柄指针，须指向ADC1
* @输出参数: 无
* @返 回 值: 1 - 可执行FOC；0 - 采样无效
* @其    他: 在中断中调用，禁止执行除法、浮点运算、复杂判断及日志打印。
***************************************************************************************************/
uint8_t BspAdcUpdateInjected(ADC_HandleTypeDef *ptAdc)
{
    uint8_t i;

    if ((ptAdc == NULL) || (ptAdc->Instance != ADC1))
    {
        return 0u;   /* 空指针或非ADC1句柄直接返回 */
    }

    /* 读取3个注入通道原始值 */
    u16InjectedRaw[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(ptAdc, ADC_INJECTED_RANK_1);
    u16InjectedRaw[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(ptAdc, ADC_INJECTED_RANK_2);
    u16InjectedRaw[2] = (uint16_t)HAL_ADCEx_InjectedGetValue(ptAdc, ADC_INJECTED_RANK_3);

    /* 计算有符号电流码 = 原始值 - 偏置 */
    for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
    {
        s32CurrentCode[i] = (int32_t)u16InjectedRaw[i] - (int32_t)u16CurrentOffset[i];
    }

    /* 检查采样窗口有效性 */
    u8SampleValid = BspAdcCheckSampleValid();

    return 1u;
}

/***************************************************************************************************
* @功    能: ADC1注入转换完成中断回调(HAL库重写)
* @详细描述: 由TIM1_CH4 → TRGO2触发ADC1注入组转换，转换完成后硬件调用此回调；
*           更新采样缓冲，若返回1(READY状态)则执行电机快速控制环。
* @输入参数: ptAdc - ADC句柄指针
* @输出参数: 无
* @返 回 值: 无
* @其    他: 控制频率由TIM1配置决定，标称20kHz。
***************************************************************************************************/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *ptAdc)
{
    if (BspAdcUpdateInjected(ptAdc) != 0u)
    {
        UsrMotorFastLoop();   /* READY状态下执行电机快速控制环 */
    }
}

/* ---- 原始值 / 偏置 / 电流码查询 ---- */

/***************************************************************************************************
* @功    能: 获取指定注入通道的原始ADC值
* @详细描述: 按通道索引返回注入通道原始值，索引越界时返回0。
* @输入参数: u8Index - 通道索引(0=Ia, 1=Ib, 2=Ic)
* @输出参数: 无
* @返 回 值: 原始ADC值(0~4095)；越界返回0
***************************************************************************************************/
uint16_t BspAdcGetInjectedRaw(uint8_t u8Index)
{
    if (u8Index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0u;   /* 索引越界返回0 */
    }
    return u16InjectedRaw[u8Index];
}

/***************************************************************************************************
* @功    能: 获取零电流偏置原始值
* @详细描述: 按通道索引返回校准得到的零电流偏置(ADC counts)，索引越界时返回0。
* @输入参数: u8Index - 通道索引(0=Ia, 1=Ib, 2=Ic)
* @输出参数: 无
* @返 回 值: 偏置平均值(ADC counts)；越界返回0
***************************************************************************************************/
uint16_t BspAdcGetCurrentOffsetRaw(uint8_t u8Index)
{
    if (u8Index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0u;   /* 索引越界返回0 */
    }
    return u16CurrentOffset[u8Index];
}

/***************************************************************************************************
* @功    能: 获取零电流偏置电压
* @详细描述: 将偏置原始值按满量程换算为电压值(V)，索引越界时返回0。
* @输入参数: u8Index - 通道索引(0=Ia, 1=Ib, 2=Ic)
* @输出参数: 无
* @返 回 值: 偏置电压(V)；越界返回0
***************************************************************************************************/
float BspAdcGetCurrentOffsetVoltage(uint8_t u8Index)
{
    if (u8Index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0.0f;   /* 索引越界返回0 */
    }
    return ((float)u16CurrentOffset[u8Index] * BSP_ADC_REF_VOLTAGE) / BSP_ADC_FULL_SCALE;   /* 原始值换算为电压 */
}

/***************************************************************************************************
* @功    能: 获取有符号电流码(raw - offset)
* @详细描述: 按通道索引返回有符号电流码，使用int32_t避免无符号减法下溢，索引越界时返回0。
* @输入参数: u8Index - 通道索引(0=Ia, 1=Ib, 2=Ic)
* @输出参数: 无
* @返 回 值: 有符号电流码，正电流为正；越界返回0
***************************************************************************************************/
int32_t BspAdcGetCurrentCode(uint8_t u8Index)
{
    if (u8Index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0;   /* 索引越界返回0 */
    }
    return s32CurrentCode[u8Index];
}



/**
 * @brief  查询预采样偏置是否有效
 * @retval 1  所有通道偏置在有效范围内
 * @retval 0  任一通道偏置超出范围
 */
uint8_t BspAdcIsPreOffsetValid(void)
{
    return u8PreOffsetValid;
}


/***************************************************************************************************
* @功    能: 查询本周期采样窗口是否有效
* @详细描述: 返回ISR中计算出的采样窗口有效标志，无效时应跳过积分更新。
* @输入参数: 无
* @输出参数: 无
* @返 回 值: 1 - 采样有效；0 - 采样窗口无效
***************************************************************************************************/
uint8_t BspAdcIsSampleValid(void)
{
    return u8SampleValid;
}

/* ---- 三相电流 ---- */

/***************************************************************************************************
* @功    能: 获取A相电流
* @详细描述: 将A相有符号电流码换算为实际电流(A)，并按硬件方向修正符号。
* @输入参数: 无
* @输出参数: 无
* @返 回 值: A相电流值(A)
***************************************************************************************************/
float BspAdcGetIa(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdcCodeToCurrent(s32CurrentCode[0]);   /* A相电流 */
}

/***************************************************************************************************
* @功    能: 获取B相电流
* @详细描述: 将B相有符号电流码换算为实际电流(A)，并按硬件方向修正符号。
* @输入参数: 无
* @输出参数: 无
* @返 回 值: B相电流值(A)
***************************************************************************************************/
float BspAdcGetIb(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdcCodeToCurrent(s32CurrentCode[1]);   /* B相电流 */
}

/***************************************************************************************************
* @功    能: 获取C相电流
* @详细描述: 将C相有符号电流码换算为实际电流(A)，并按硬件方向修正符号。
* @输入参数: 无
* @输出参数: 无
* @返 回 值: C相电流值(A)
***************************************************************************************************/
float BspAdcGetIc(void)
{
    return BSP_ADC_PHASE_CURRENT_SIGN * BspAdcCodeToCurrent(s32CurrentCode[2]);   /* C相电流 */
}

/* ===================================================================== *
 *  ADC2 接口（保持不变）
 * ===================================================================== */

/***************************************************************************************************
* @功    能: 更新ADC2全部常规通道采样值
* @详细描述: 首次调用时先执行ADC2单端自校准，然后按通道映射表依次轮询读取
*           SHA/SHB/SHC/POT/VBUS共5个通道的原始值。
* @输入参数: 无
* @输出参数: 无
* @返 回 值: HAL_OK - 全部通道读取成功；其他 - HAL库错误状态
***************************************************************************************************/
HAL_StatusTypeDef BspAdc2UpdateAll(void)
{
    HAL_StatusTypeDef Status;
    uint8_t Index;
    uint16_t Raw;

    // if (u8Adc2Calibrated == 0u)
    // {
    //     Status = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);   /* 首次使用前执行ADC2自校准 */
    //     if (Status != HAL_OK)
    //     {
    //         return Status;
    //     }
        u8Adc2Calibrated = 1u;   /* 标记校准完成 */
    // }

    for (Index = 0u; Index < BSP_ADC2_REGULAR_CHANNELS; Index++)
    {
        Status = BspAdc2ReadChannel(u32Adc2ChannelMap[Index], &Raw);   /* 轮询读取单通道 */
        if (Status != HAL_OK)
        {
            return Status;   /* 任一通道失败立即返回 */
        }
        u16Adc2RegularRaw[Index] = Raw;   /* 保存通道原始值 */
    }

    return HAL_OK;
}

/***************************************************************************************************
* @功    能: 获取ADC2指定通道原始值
* @详细描述: 按通道枚举返回对应常规通道的原始值，通道号越界时返回0。
* @输入参数: eChannel - ADC2通道枚举(SHA/SHB/SHC/POT/VBUS)
* @输出参数: 无
* @返 回 值: 通道原始ADC值(0~4095)；越界返回0
***************************************************************************************************/
uint16_t BspAdc2GetRaw(eBspAdc2ChannelDef eChannel)
{
    if ((uint8_t)eChannel >= BSP_ADC2_REGULAR_CHANNELS)
    {
        return 0u;   /* 通道号越界返回0 */
    }
    return u16Adc2RegularRaw[(uint8_t)eChannel];
}

/***************************************************************************************************
* @功    能: 获取ADC2指定通道电压值
* @详细描述: 将指定通道原始值按满量程换算为电压值(V)。
* @输入参数: eChannel - ADC2通道枚举(SHA/SHB/SHC/POT/VBUS)
* @输出参数: 无
* @返 回 值: 通道电压值(V)
***************************************************************************************************/
float BspAdc2GetVoltage(eBspAdc2ChannelDef eChannel)
{
    return ((float)BspAdc2GetRaw(eChannel) * BSP_ADC_REF_VOLTAGE) / BSP_ADC_FULL_SCALE;   /* 原始值换算为电压 */
}

/***************************************************************************************************
* @功    能: 启动前使用ADC1普通轮询方式预采样三相零电流偏置
* @详细描述: 临时停止ADC1注入中断采样，将触发源改为软件启动，按
*           BSP_ADC_PRE_OFFSET_SAMPLE_COUNT次对Ia/Ib/Ic三个通道求平均作为零电流偏置，
*           随后恢复TIM1硬件触发配置并输出结果。后续BspAdcCalibrationStart()仍会执行
*           PWM环境下的正式注入校准。
* @输入参数: 无
* @输出参数: 无
* @返 回 值: 无
***************************************************************************************************/
void BspAdcPreOffset(void)
{
    uint64_t u64Sum[BSP_ADC_INJECTED_CHANNELS] = {0u};   /* 各通道累计和 */
    HAL_StatusTypeDef Status;
    uint32_t SampleIndex;
    uint16_t Raw;
    uint8_t i;
    uint32_t value_rank1 = 0;
    uint32_t value_rank2 = 0;
    uint32_t value_rank3 = 0;
    ADC_InjectionConfTypeDef sConfigInjected = {0};      /* 注入组配置结构体 */

    /* 停止ADC1注入中断采样 */
    Status = HAL_ADCEx_InjectedStop_IT(&hadc1);
    if (Status != HAL_OK)
    {
        SEGGER_RTT_printf(0, "ADC1 stop failed 1\r\n");
        Error_Handler();
    }
    // BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);   /* 输出0矢量（零电压） */
    BspPwmStartPowerOutputs();
    HAL_Delay(100);
    /** Configure Injected Channel
    */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_1;          /* 注入通道1: Ia */
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;   /* 采样时间12.5周期 */
    sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;    /* 单端输入 */
    sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;   /* 不使用硬件偏移 */
    sConfigInjected.InjectedOffset = 0;
    sConfigInjected.InjectedNbrOfConversion = 3;              /* 注入组共3个通道 */
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;  /* 禁止不连续转换模式 */
    sConfigInjected.AutoInjectedConv = DISABLE;               /* 禁止自动注入 */
    sConfigInjected.QueueInjectedContext = DISABLE;           /* 禁止上下文队列 */
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;   /* 预采样使用软件触发 */
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_NONE;
    sConfigInjected.InjecOversamplingMode = DISABLE;          /* 禁止过采样 */
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Injected Channel
    */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_2;          /* 注入通道2: Ib */
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Injected Channel
    */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_3;          /* 注入通道3: Ic */
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }

    for (SampleIndex = 0u; SampleIndex < BSP_ADC_PRE_OFFSET_SAMPLE_COUNT; SampleIndex++)
    {
        /* 1. 启动注入组转换（非中断模式） */
        if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK)
        {
            /* 启动失败处理 */
            SEGGER_RTT_printf(0, "ADC1 start failed 2\r\n");
            Error_Handler();
        }

        /* 2. 轮询等待转换完成，超时时间设为100ms */
        if (HAL_ADCEx_InjectedPollForConversion(&hadc1, 100) == HAL_OK)
        {
            /* 3. 转换完成，读取各通道数据 */
            value_rank1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
            value_rank2 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
            value_rank3 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
        }
        else
        {
            /* 转换超时或出错处理 */
            SEGGER_RTT_printf(0, "ADC1 poll failed 3\r\n");
            Error_Handler();
        }
        u64Sum[0] += value_rank1;   /* 累计A相原始值 */
        u64Sum[1] += value_rank2;   /* 累计B相原始值 */
        u64Sum[2] += value_rank3;   /* 累计C相原始值 */
    }

    for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
    {
        u16CurrentOffset[i] = (uint16_t)(u64Sum[i] / BSP_ADC_PRE_OFFSET_SAMPLE_COUNT);   /* 求平均作为零电流偏置 */
        u16InjectedRaw[i] = u16CurrentOffset[i];   /* 原始值缓冲同步为偏置 */
        s32CurrentCode[i] = 0;                     /* 电流码清零 */
    }

    /* 偏置范围检查：任一通道超出 20%~80% 满量程则标记无效，不卡程序 */
    u8PreOffsetValid = 1u;
    for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
    {
        if ((u16CurrentOffset[i] < BSP_ADC_PRE_OFFSET_MIN) ||
            (u16CurrentOffset[i] > BSP_ADC_PRE_OFFSET_MAX))
        {
            u8PreOffsetValid = 0u;
            SEGGER_RTT_printf(0, "PreOffset CH%u out of range: %u\r\n",
                              (unsigned)i, (unsigned)u16CurrentOffset[i]);
        }
    }

    SEGGER_RTT_printf(0, "ADC1 pre-offset: %d %d %d valid=%u\r\n", u16CurrentOffset[0],
                      u16CurrentOffset[1], u16CurrentOffset[2], (unsigned)u8PreOffsetValid);
    u8SampleValid = 0u;   /* 采样窗口有效标志清零 */

    /* 恢复硬件触发：必须按 RANK_1 → RANK_2 → RANK_3 顺序重新配置全部 3 个 rank。
     * HAL 的上下文队列机制要求 InjectedNbrOfConversion 次调用构成一个完整上下文，
     * 缺少任一 rank 会使对应 JSQx 槽位保持为 0（即 ADC_CHANNEL_0），导致采样错误通道。 */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_1;   /* 注入通道1: Ia */
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;           /* 恢复TIM1硬件触发 */
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING;   /* 下降沿触发 */
    sConfigInjected.InjecOversamplingMode = DISABLE;   /* 禁止过采样 */
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Injected Channel
    */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_2;   /* 注入通道2: Ib */
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Injected Channel
    */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_3;   /* 注入通道3: Ic */
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        Error_Handler();
    }

    // Status = HAL_ADCEx_InjectedStart_IT(&hadc1);
    // if (Status != HAL_OK)
    // {
    //     Error_Handler();
    // }
}
