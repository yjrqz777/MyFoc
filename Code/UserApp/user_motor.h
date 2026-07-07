#ifndef __USER_MOTOR_H__
#define __USER_MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void UserMotor_Init(void);
HAL_StatusTypeDef UserMotor_Start(void);
void UserMotor_FastLoop(void);
float UserMotor_GetOpenLoopTheta(void);

#ifdef __cplusplus
}
#endif

#endif
