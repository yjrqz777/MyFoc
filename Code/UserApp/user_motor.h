/**
 * @file    user_motor.h
 * @brief   电机控制应用层头文件 — 开 V/F 启动 + FOC 电流环
 *******************************************************************************
 * @note    提供电机初始化、启动、快速控制循环接口
 *          开环角度以斜坡方式逐步加速到目标转速
 *******************************************************************************
 */

#ifndef __USER_MOTOR_H__
#define __USER_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

void UserMotor_Init(void);
HAL_StatusTypeDef UserMotor_Start(void);
void UserMotor_FastLoop(void);
float UserMotor_GetOpenLoopTheta(void);
float UserMotor_GetIqRef(void);
float UserMotor_GetIqRefTarget(void);
uint8_t UserMotor_IsOverCurrentFault(void);
float UserMotor_GetFaultIa(void);
float UserMotor_GetFaultIb(void);
float UserMotor_GetFaultIc(void);

#ifdef __cplusplus
}
#endif

#endif /* __USER_MOTOR_H__ */
