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

void UsrMotorInit(void);
HAL_StatusTypeDef UsrMotorStart(void);
void UsrMotorFastLoop(void);
float UsrMotorGetOpenLoopTheta(void);
float UsrMotorGetIqRef(void);
float UsrMotorGetIqRefTarget(void);
uint8_t UsrMotorIsOverCurrentFault(void);
float UsrMotorGetFaultIa(void);
float UsrMotorGetFaultIb(void);
float UsrMotorGetFaultIc(void);

#ifdef __cplusplus
}
#endif

#endif /* __USER_MOTOR_H__ */
