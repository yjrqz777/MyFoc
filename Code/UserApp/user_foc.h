/**
 * @file    user_foc.h
 * @brief   FOC 电流闭环控制头文件
 *******************************************************************************
 * @note    声明 FOC 核心数据结构和对外接口。
 *          FOC 流程：Clarke → Park → PI 调节 → 逆 Park → 逆 Clarke
 *          控制周期 20kHz（50us），由 ADC1 注入转换完成中断驱动。
 *******************************************************************************
 */

#ifndef __USER_FOC_H__
#define __USER_FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

/** @brief 三相调制电压，范围与 BspPwmSetVoltageAbc 一致（-100 ~ 100） */
typedef struct tThreePhaseVoltageDef
{
    float f32Ua;
    float f32Ub;
    float f32Uc;
} tThreePhaseVoltageDef;

/** @brief dq 轴电流，单位 A */
typedef struct tDqCurrentDef
{
    float f32D;
    float f32Q;
} tDqCurrentDef;

/** @brief 单次电流环输入 */
typedef struct tFocCurrentLoopInputDef
{
    float f32Ia;
    float f32Ib;
    float f32Ic;
    float f32Theta;
    float f32IdReference;
    float f32IqReference;
} tFocCurrentLoopInputDef;

/** @brief 单次电流环输出 */
typedef struct tFocCurrentLoopOutputDef
{
    tThreePhaseVoltageDef tVoltage;
    tDqCurrentDef tCurrent;
} tFocCurrentLoopOutputDef;

/**
 * @brief 复位 FOC 动态状态和电流参考值
 * @note  应在启动 ADC 注入转换前调用，避免与快速环中断并发
 */
void UsrFocReset(void);

/**
 * @brief 执行一次 FOC 电流闭环
 * @param[in] ptInput 三相电流、电角度和 dq 电流参考值
 * @param[out] ptOutput 三相调制电压和实际 dq 电流，可传 NULL
 * @note 由 TIM1 CH4 触发的 ADC1 注入转换完成中断调用，标称频率 20 kHz
 */
void UsrFocRunCurrentLoop(const tFocCurrentLoopInputDef * ptInput, tFocCurrentLoopOutputDef * ptOutput);

/** @brief 兼容接口：使用已设置的参考值执行一次电流环 */
void UsrFocCurrentLoop(float f32Ia, float f32Ib, float f32Ic, float f32Theta);

/** @brief 设置兼容接口使用的 dq 电流参考值 */
void UsrFocSetCurrentReference(float f32IdReference, float f32IqReference);

/** @brief 获取最近一次闭环计算的三相调制电压 */
tThreePhaseVoltageDef UsrFocGetThreePhaseVoltage(void);

/** @brief 获取最近一次闭环计算的 dq 轴电流 */
tDqCurrentDef UsrFocGetDqCurrent(void);

#ifdef __cplusplus
}
#endif

#endif /* __USER_FOC_H__ */
