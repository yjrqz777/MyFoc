/**
 * @file    user_foc.h
 * @brief   FOC 电流闭环控制头文件
 *******************************************************************************
 * @note    声明 FOC 核心数据结构和对外接口。
 *          FOC 流程：Clarke → Park → PI 调节 → 逆 Park → 逆 Clarke
 *          控制周期 10kHz（100us），由 ADC1 注入转换完成中断驱动。
 *******************************************************************************
 */

#ifndef __USERFOC_H__
#define __USERFOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

/** @brief 三相调制电压，范围与 BspPwm_SetVoltageABC 一致（-100 ~ 100） */
typedef struct
{
    float ua;
    float ub;
    float uc;
} ThreePhaseVoltage_t;

/** @brief dq 轴电流，单位 A */
typedef struct
{
    float d;
    float q;
} DQCurrent_t;

/** @brief 单次电流环输入 */
typedef struct
{
    float ia;
    float ib;
    float ic;
    float theta;
    float id_ref;
    float iq_ref;
} FOC_CurrentLoopInput_t;

/** @brief 单次电流环输出 */
typedef struct
{
    ThreePhaseVoltage_t voltage;
    DQCurrent_t current;
} FOC_CurrentLoopOutput_t;

/**
 * @brief 复位 FOC 动态状态和电流参考值
 * @note  应在启动 ADC 注入转换前调用，避免与快速环中断并发
 */
void FOC_Reset(void);

/**
 * @brief 执行一次 FOC 电流闭环
 * @param[in] input 三相电流、电角度和 dq 电流参考值
 * @param[out] output 三相调制电压和实际 dq 电流，可传 NULL
 * @note 由 TIM1 CH4 触发的 ADC1 注入转换完成中断调用，标称频率 10 kHz
 */
void FOC_RunCurrentLoop(const FOC_CurrentLoopInput_t *input, FOC_CurrentLoopOutput_t *output);

/** @brief 兼容接口：使用已设置的参考值执行一次电流环 */
void FOC_CurrentLoop(float ia, float ib, float ic, float theta);

/** @brief 设置兼容接口使用的 dq 电流参考值 */
void FOC_SetCurrentReference(float id_ref, float iq_ref);

/** @brief 获取最近一次闭环计算的三相调制电压 */
ThreePhaseVoltage_t FOC_GetThreePhaseVoltage(void);

/** @brief 获取最近一次闭环计算的 dq 轴电流 */
DQCurrent_t FOC_GetDQCurrent(void);

#ifdef __cplusplus
}
#endif

#endif /* __USERFOC_H__ */
