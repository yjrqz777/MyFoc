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
/** @brief TIM1 计数时钟 (Hz)，须与 CubeMX Clock Configuration 一致 */
#define CS_TIM1_CLOCK_HZ           170000000u

/** @brief PWM 开关频率 (Hz) */
#define CS_PWM_FREQUENCY_HZ        20000u

/** @brief TIM1 预分频 */
#define CS_PWM_PRESCALER           0u

/** @brief TIM1 自动重装值 ARR = fTIM / (2 * fPWM * (PSC+1)) - 1 */
#define CS_PWM_ARR                 4249u

/* ---- ADC (doc §4, §23) ---- */
/** @brief ADC 时钟 (Hz)，170 MHz / 4 = 42.5 MHz */
#define CS_ADC_CLOCK_HZ            42500000u

/** @brief ADC 采样保持时间对应的 TIM1 计数数 (12.5 cycles @ 42.5 MHz = 50 ticks) */
#define CS_ADC_SAMPLE_TICKS        50u

/** @brief ADC 触发点 CCR4，谷点后延时 (CNT=0 向上计数到此值时触发) */
#define CS_ADC_TRIGGER_CCR         50u

/* ---- 消隐 / 安全裕量 (doc §5) ---- */
/** @brief 总消隐时间对应的 TIM1 计数数 (死区 + 驱动延迟 + MOS开关 + 振铃 + 运放建立) */
#define CS_BLANK_TICKS             150u

/** @brief CCR 下限 = CCR4 + 采样保持 + 消隐，保证采样窗口有效 */
#define CS_CCR_MIN                 (CS_ADC_TRIGGER_CCR + CS_ADC_SAMPLE_TICKS + CS_BLANK_TICKS)

/* ---- 偏置校准 (doc §11~§15, §23) ---- */
/** @brief 模拟链路稳定等待时间 (ms) */
#define CS_CAL_SETTLE_TIME_MS      10u

/** @brief 丢弃启动阶段样本数 */
#define CS_CAL_DISCARD_COUNT       64u

/** @brief 累计偏置样本数 (2 的幂，便于移位除法) */
#define CS_CAL_SAMPLE_COUNT        512u

/** @brief 校准时单通道噪声跨度最大值 (LSB) */
#define CS_CAL_MAX_SPAN            40u

/** @brief 偏置下限 (20% * 4095) */
#define CS_CAL_OFFSET_MIN          819u

/** @brief 偏置上限 (80% * 4095) */
#define CS_CAL_OFFSET_MAX          3276u

/** @brief 前后半段均值漂移最大值 (LSB) */
#define CS_CAL_DRIFT_LIMIT         8u

/** @brief 校准失败最大重试次数 */
#define CS_CAL_MAX_RETRY           3u

/* ---- 通道数 ---- */
/** @brief ADC1 注入通道数 (Ia, Ib, Ic) */
#define BSP_ADC_INJECTED_CHANNELS  3u

/** @brief ADC2 常规通道数 (SHA, SHB, SHC, POT, VBUS) */
#define BSP_ADC2_REGULAR_CHANNELS  5u

/* ===================================================================== *
 *  校准状态机 (doc §9)
 * ===================================================================== */

typedef enum
{
    CS_CAL_IDLE = 0,        /**< 空闲，未开始校准 */
    CS_CAL_SETTLE,          /**< 等待模拟链路稳定 */
    CS_CAL_DISCARD,         /**< 丢弃启动阶段样本 */
    CS_CAL_ACCUMULATE,      /**< 累计偏置样本 */
    CS_CAL_CALCULATE,       /**< 等待主循环计算偏置 */
    CS_CAL_READY,           /**< 校准完成，可进入 FOC */
    CS_CAL_ERROR            /**< 校准失败，禁止启动电机 */
} BspAdc_CalState_t;

/* ---- ADC2 通道枚举 ---- */
typedef enum
{
    BSP_ADC2_SHA = 0,   /**< PC0: SHA 通道 */
    BSP_ADC2_SHB,       /**< PC1: SHB 通道 */
    BSP_ADC2_SHC,       /**< PC2: SHC 通道 */
    BSP_ADC2_POT,       /**< PC4: 电位器 */
    BSP_ADC2_VBUS,      /**< PC5: 母线电压 */
} BspAdc2Channel_t;

/* ---- 校准调试信息 (doc §22.15) ---- */
typedef struct
{
    uint16_t min_raw[BSP_ADC_INJECTED_CHANNELS];  /**< 校准期间各通道最小值 */
    uint16_t max_raw[BSP_ADC_INJECTED_CHANNELS];  /**< 校准期间各通道最大值 */
    uint16_t span[BSP_ADC_INJECTED_CHANNELS];     /**< 噪声跨度 = max - min */
    uint16_t offset[BSP_ADC_INJECTED_CHANNELS];   /**< 计算得到的偏置 */
    int16_t drift[BSP_ADC_INJECTED_CHANNELS];     /**< 前后半段均值差 */
    uint8_t retry_count;                          /**< 已重试次数 */
    BspAdc_CalState_t state;                      /**< 当前状态 */
} BspAdc_CalDebug_t;

/* ===================================================================== *
 *  接口函数 (doc §19)
 * ===================================================================== */

/**
 * @brief  初始化采样模块并启动 ADC1 注入组（含内部自校准）
 * @retval HAL_OK       启动成功
 * @retval HAL_ERROR    ADC 自校准失败
 * @retval HAL_BUSY     外设忙
 * @note   清除校准状态，设置偏置为默认值 2048。之后需调用 BspAdc_CalibrationStart() 开始校准。
 */
HAL_StatusTypeDef BspAdc_StartInjected(void);

/**
 * @brief  启动偏置校准状态机
 * @note   复位累计值、min/max、前后半段累计值，进入 SETTLE 状态。
 *         调用前必须确保：驱动器 EN 关闭、功率 PWM 未启动、TIM1_CH4 触发已运行。
 */
void BspAdc_CalibrationStart(void);

/**
 * @brief  偏置校准状态机处理（由主循环调用）
 * @note   负责：稳定等待计时、偏置平均值计算、范围/跨度/漂移检查、失败重试、状态切换。
 *         中断中仅做累计，除法和检查在此函数中执行 (doc §13)。
 */
void BspAdc_Process(void);

/**
 * @brief  查询偏置校准是否完成
 * @retval 1  校准成功 (CS_CAL_READY)
 * @retval 0  尚未完成或失败
 */
uint8_t BspAdc_IsCurrentOffsetReady(void);

/**
 * @brief  查询当前校准状态
 */
BspAdc_CalState_t BspAdc_GetCalState(void);

/**
 * @brief  查询本周期采样窗口是否有效
 * @retval 1  采样有效
 * @retval 0  采样窗口无效，应跳过积分更新
 * @note   检查条件：CCR4 + ADC_SAMPLE_TICKS + BLANK_TICKS <= min(CCR1, CCR2, CCR3)
 */
uint8_t BspAdc_IsSampleValid(void);

/**
 * @brief  获取指定注入通道的原始 ADC 值
 * @param[in] index  通道索引（0=Ia, 1=Ib, 2=Ic）
 * @return 原始 ADC 值（0~4095），越界返回 0
 */
uint16_t BspAdc_GetInjectedRaw(uint8_t index);

/**
 * @brief  获取零电流偏置原始值
 * @param[in] index  通道索引（0=Ia, 1=Ib, 2=Ic）
 * @return 偏置平均值（ADC counts），越界返回 0
 */
uint16_t BspAdc_GetCurrentOffsetRaw(uint8_t index);

/**
 * @brief  获取零电流偏置电压
 * @param[in] index  通道索引（0=Ia, 1=Ib, 2=Ic）
 * @return 偏置电压（V）
 */
float BspAdc_GetCurrentOffsetVoltage(uint8_t index);

/**
 * @brief  获取有符号电流码 (raw - offset)
 * @param[in] index  通道索引（0=Ia, 1=Ib, 2=Ic）
 * @return 有符号电流码，正电流为正
 * @note   使用 int32_t 避免无符号减法下溢 (doc §16)
 */
int32_t BspAdc_GetCurrentCode(uint8_t index);

/* ---- 三相电流（A）---- */
float BspAdc_GetIa(void);
float BspAdc_GetIb(void);
float BspAdc_GetIc(void);

/* ---- 校准调试信息 ---- */
void BspAdc_GetCalDebug(BspAdc_CalDebug_t *info);

/* ---- ADC2 接口（保持不变）---- */
HAL_StatusTypeDef BspAdc2_UpdateAll(void);
uint16_t BspAdc2_GetRaw(BspAdc2Channel_t channel);
float BspAdc2_GetVoltage(BspAdc2Channel_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H__ */
