/**
 * @file    user_motor_hall_vf.h
 * @brief   Hall feedback and V/F open-loop startup support.
 */

#ifndef __USER_MOTOR_HALL_VF_H__
#define __USER_MOTOR_HALL_VF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

typedef struct tUserMotorHallVfStateDef
{
    float f32OpenLoopTheta;
    float f32OpenLoopStep;
    float f32OpenVoltage;
    float f32HallCheckLastSpeed;
    uint32_t u32HallFocLockCounter;
} tUserMotorHallVfStateDef;

void UserMotorHallVfInit(tUserMotorHallVfStateDef *ptState);
void UserMotorHallVfSetVoltageVector(float f32Theta, float f32Voltage);
void UserMotorHallVfRunStartupOpenLoop(tUserMotorHallVfStateDef *ptState,
                                       float f32SpeedRef,
                                       uint32_t u32FastLoopHz,
                                       float f32SpeedReferenceMax);
uint8_t UserMotorHallVfIsReadyForFoc(tUserMotorHallVfStateDef *ptState,
                                     float f32HallSpeed,
                                     uint32_t u32FastLoopHz);
uint8_t UserMotorHallVfShouldExitFoc(float f32SpeedRef,
                                     float f32HallSpeed);

#ifdef __cplusplus
}
#endif

#endif /* __USER_MOTOR_HALL_VF_H__ */
