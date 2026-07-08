/***************************************************************************************************
 * Author: yjrqz777 3210551161@qq.com
 * Date: 2026-03-19
 * Description: FOC电流闭环控制头文件
 * FilePath: \Myfoc2\Code\UserApp\user_foc.h
 * @YJRQZ777
***************************************************************************************************/

#ifndef __USERFOC_H__
#define __USERFOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

/***************************************************************************************************
 * 函数声明
***************************************************************************************************/

/**
 * @brief FOC电流闭环主控制函数
 * @param ia, ib, ic - 采样的三相电流（单位：A）
 * @param theta - 转子位置角（单位：弧度，0~2π）
 * @return none
 * @note 调用频率：10kHz（100µs），应在TIM6中断中调用
 */
void FOC_CurrentLoop(float ia, float ib, float ic, float theta);

/**
 * @brief 获取FOC输出的三相电压
 * @param none
 * @return 三相电压结构体（单位：V或PWM占空比）
 */
typedef struct {
    float ua;
    float ub;
    float uc;
} ThreePhaseVoltage_t;

ThreePhaseVoltage_t FOC_GetThreePhaseVoltage(void);

/**
 * @brief 设置FOC目标电流
 * @param id_ref - 目标直轴电流（通常为0A）
 * @param iq_ref - 目标交轴电流（控制转矩，单位：A）
 * @return none
 */
void FOC_SetCurrentReference(float id_ref, float iq_ref);

/**
 * @brief 获取FOC实际的dq轴电流
 * @param none
 * @return dq轴电流结构体
 */
typedef struct {
    float d;
    float q;
} DQCurrent_t;

DQCurrent_t FOC_GetDQCurrent(void);

#ifdef __cplusplus
}
#endif

#endif /* __USERFOC_H__ */
