#ifndef __BSP_PWM_FOC_H__
#define __BSP_PWM_FOC_H__

#include "user_global.h"

#define BSP_PWM_DUTY_Q10_MAX (1024u)

void BspPwmSetDutyQ10(uint16_t u16DutyA,
                      uint16_t u16DutyB,
                      uint16_t u16DutyC);

#endif
