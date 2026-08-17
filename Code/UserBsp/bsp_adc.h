/**
 * @file    bsp_adc.h
 * @brief   ADC 底层驱动头文件 — 注入模式三相电流采样 + 偏置校准状态机
 *******************************************************************************
 * @note    ADC1 工作在注入扫描模式，3 通道分别采样 Ia、Ib、Ic。
 *          采样由 TIM1_CH4 → TRGO2 → ADC 注入组硬件触发。
 *          偏置校准采用非阻塞状态机：
 *          SETTLE → DISCARD → ACCUMULATE → CALCULATE → READY。
 *          所有时间和计数参数集中定义，禁止散落魔法数字 (doc §20)。
 *******************************************************************************
 */

#ifndef __BSP_ADC_H__
#define __BSP_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

/* ===================================================================== *
 *  集中参数定义 (doc §20)
 * ===================================================================== */

/* ---- TIM1 / PWM (doc §3, §23) ---- */
#define CS_TIM1_CLOCK_HZ           (170000000u)  /* TIM1 计数时钟 (Hz)，须与 CubeMX Clock Configuration 一致 */
#define CS_PWM_FREQUENCY_HZ        (20000u)      /* PWM 开关频率 (Hz) */
#define CS_PWM_PRESCALER           (0u)          /* TIM1 预分频 */
#define CS_PWM_ARR                 (4249u)       /* TIM1 自动重装值 ARR = fTIM / (2 * fPWM * (PSC+1)) - 1 */

/* ---- ADC (doc §4, §23) ---- */
#define CS_ADC_CLOCK_HZ            (42500000u)   /* ADC 时钟 (Hz)，170 MHz / 4 = 42.5 MHz */
#define CS_ADC_SAMPLE_TICKS        (300u)         /* ADC 采样保持时间对应的 TIM1 计数数 (12.5 cycles @ 42.5 MHz = 50 ticks) */
#define CS_ADC_TRIGGER_CCR         (250u)         /* ADC 触发点 CCR4，谷点后延时 (CNT=0 向上计数到此值时触发) */

/* ---- 消隐 / 安全裕量 (doc §5) ---- */
#define CS_BLANK_TICKS             (150u)        /* 总消隐时间对应的 TIM1 计数数 (死区 + 驱动延迟 + MOS开关 + 振铃 + 运放建立) */
#define CS_CCR_MIN                 (CS_ADC_TRIGGER_CCR + CS_ADC_SAMPLE_TICKS + CS_BLANK_TICKS) /* CCR 下限 = CCR4 + 采样保持 + 消隐，保证采样窗口有效 */

/* ---- 偏置校准 (由 BspAdcPreOffset 阻塞式完成) ---- */
/* ---- 通道数 ---- */
#define BSP_ADC_INJECTED_CHANNELS  (3u)          /* ADC1 注入通道数 (Ia, Ib, Ic) */
#define BSP_ADC2_REGULAR_CHANNELS  (5u)          /* ADC2 常规通道数 (SHA, SHB, SHC, POT, VBUS) */

/* ---- 启动前普通轮询零偏采样 ---- */
#define BSP_ADC_PRE_OFFSET_SAMPLE_COUNT    (1024u) /* 启动前普通轮询零偏采样次数 */
#define BSP_ADC_PRE_OFFSET_POLL_TIMEOUT_MS (2u)    /* 启动前普通轮询单次转换超时 (ms) */
#define BSP_ADC_PRE_OFFSET_MIN             (819u)  /* 偏置下限 (20% * 4095) */
#define BSP_ADC_PRE_OFFSET_MAX             (3276u) /* 偏置上限 (80% * 4095) */

#if (BSP_ADC_PRE_OFFSET_SAMPLE_COUNT == 0u)
#error "BSP_ADC_PRE_OFFSET_SAMPLE_COUNT must be greater than 0"
#endif

/* ===================================================================== *
*  校准状态 (仅 IDLE / READY，由 BspAdcPreOffset 设置)
* ===================================================================== */

typedef enum
{
    E_BSP_ADC_CAL_IDLE = 0,        /**< 空闲，未校准 */
    E_BSP_ADC_CAL_READY            /**< 校准完成，可进入 FOC */
} eBspAdcCalStateDef;

/* ---- ADC2 通道枚举 ---- */
typedef enum
{
    E_BSP_ADC2_SHA = 0,   /**< PC0: SHA 通道 */
    E_BSP_ADC2_SHB,       /**< PC1: SHB 通道 */
    E_BSP_ADC2_SHC,       /**< PC2: SHC 通道 */
    E_BSP_ADC2_POT,       /**< PC4: 电位器 */
    E_BSP_ADC2_VBUS,      /**< PC5: 母线电压 */
} eBspAdc2ChannelDef;

/* ===================================================================== *
 *  接口函数
 * ===================================================================== */

/**
 * @brief  启动 ADC1 注入组中断采样
 * @note   偏置已由 BspAdcPreOffset 在调用前完成，此处仅启动中断。
 */
HAL_StatusTypeDef BspAdcStartInjected(void);
void BspAdcPreOffset(void);

/**
 * @brief  查询偏置校准是否完成
 * @retval 1  校准完成 (E_BSP_ADC_CAL_READY)
 * @retval 0  尚未完成
 */
uint8_t BspAdcIsCurrentOffsetReady(void);

/**
 * @brief  查询预采样偏置是否有效
 * @retval 1  所有通道偏置在有效范围内
 * @retval 0  任一通道偏置超出范围
 */
uint8_t BspAdcIsPreOffsetValid(void);

/**
 * @brief  查询当前校准状态
 */
eBspAdcCalStateDef BspAdcGetCalState(void);

/**
 * @brief  查询本周期采样窗口是否有效
 * @retval 1  采样有效
 * @retval 0  采样窗口无效，应跳过积分更新
 * @note   检查条件：CCR4 + ADC_SAMPLE_TICKS + BLANK_TICKS <= min(CCR1, CCR2, CCR3)
 */
uint8_t BspAdcIsSampleValid(void);

/**
 * @brief  获取指定注入通道的原始 ADC 值
 * @param[in] u8Index  通道索引（0=Ia, 1=Ib, 2=Ic）
 * @return 原始 ADC 值（0~4095），越界返回 0
 */
uint16_t BspAdcGetInjectedRaw(uint8_t u8Index);

/**
 * @brief  获取零电流偏置原始值
 * @param[in] u8Index  通道索引（0=Ia, 1=Ib, 2=Ic）
 * @return 偏置平均值（ADC counts），越界返回 0
 */
uint16_t BspAdcGetCurrentOffsetRaw(uint8_t u8Index);

/**
 * @brief  获取零电流偏置电压
 * @param[in] u8Index  通道索引（0=Ia, 1=Ib, 2=Ic）
 * @return 偏置电压（V）
 */
float BspAdcGetCurrentOffsetVoltage(uint8_t u8Index);

/**
 * @brief  获取有符号电流码 (raw - offset)
 * @param[in] u8Index  通道索引（0=Ia, 1=Ib, 2=Ic）
 * @return 有符号电流码，正电流为正
 * @note   使用 int32_t 避免无符号减法下溢 (doc §16)
 */
int32_t BspAdcGetCurrentCode(uint8_t u8Index);

/* ---- 三相电流（A）---- */
float BspAdcGetIa(void);
float BspAdcGetIb(void);
float BspAdcGetIc(void);

/* ---- ADC2 接口 ---- */
HAL_StatusTypeDef BspAdc2UpdateAll(void);
uint16_t BspAdc2GetRaw(eBspAdc2ChannelDef eChannel);
float BspAdc2GetVoltage(eBspAdc2ChannelDef eChannel);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H__ */
